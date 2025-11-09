#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_mac.h"

// BLE (NimBLE) Includes
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "COUNTER_PROJECT";

// *** 1. 引脚定义 (根据你的方案) ***

// 7段数码管段位 (A-G)
#define SEG_A_PIN   GPIO_NUM_3
#define SEG_B_PIN   GPIO_NUM_7
#define SEG_C_PIN   GPIO_NUM_6
#define SEG_D_PIN   GPIO_NUM_4
#define SEG_E_PIN   GPIO_NUM_2
#define SEG_F_PIN   GPIO_NUM_5
#define SEG_G_PIN   GPIO_NUM_8

// 3位共阳数码管位选 (DIG1-3)
#define DIG_1_PIN   GPIO_NUM_10 // 百位
#define DIG_2_PIN   GPIO_NUM_19 // 十位
#define DIG_3_PIN   GPIO_NUM_18 // 个位

// 输入引脚
#define KEY_SWITCH_PIN  GPIO_NUM_0 // 键轴 (PCNT)
#define ZERO_SWITCH_PIN GPIO_NUM_1 // 置零 (ISR)


// *** 2. 全局变量 ***

// 计数器 (volatile 保证多任务安全访问)
static volatile uint32_t g_counter = 0;
// 计数器变化标志，通知BLE任务
static volatile bool g_counter_updated = false;

// 用于ISR通知主任务
static TaskHandle_t g_main_task_handle = NULL;
// 用于BLE连接
static uint16_t g_ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;

// GPIO1持续低电平计时（用于5秒清零检测）
static volatile TickType_t g_zero_press_start_tick = 0;
#define ZERO_PRESS_DURATION_MS 5000  // 5秒

// BLE 句柄和地址类型变量
static uint16_t g_counter_val_handle;
static uint8_t g_ble_addr_type;

// 键轴脉冲计数（使用 GPIO 中断实现）
static volatile uint32_t g_pulse_count = 0;
// 用于保护脉冲计数的互斥锁
static portMUX_TYPE g_pulse_mux = portMUX_INITIALIZER_UNLOCKED;
// 按键防抖：记录上次处理时间（tick计数）
static volatile TickType_t g_last_key_process_tick = 0;
#define KEY_DEBOUNCE_TICKS pdMS_TO_TICKS(100)  // 防抖时间100ms，适合一秒5-6次按键
// 按键状态机：用于确保一次按键只计数一次
static volatile bool g_key_press_processed = false;  // 标记当前按键是否已处理

// *** 声明函数原型 ***
void start_ble_advertising(void);


// *** 3. 数码管显示任务 ***

// 7段数码管编码 (0-9)
// 编码格式：g,f,e,d,c,b,a (bit 6-0)，1=熄灭，0=点亮
const uint8_t segment_map[10] = {
    (1 << 6),                               // 0: a,b,c,d,e,f (熄灭g)
    (1 << 0) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6), // 1: b,c (熄灭a,d,e,f,g)
    (1 << 2) | (1 << 5),                    // 2: a,b,d,e,g (熄灭c,f)
    (1 << 4) | (1 << 5),                    // 3: a,b,c,d,g (熄灭e,f)
    (1 << 0) | (1 << 3) | (1 << 4),         // 4: b,c,f,g (熄灭a,d,e)
    (1 << 1) | (1 << 4),                    // 5: a,c,d,f,g (熄灭b,e)
    (1 << 1),                               // 6: a,c,d,e,f,g (熄灭b)
    (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6), // 7: a,b,c (熄灭d,e,f,g)
    0,                                      // 8: a,b,c,d,e,f,g (全部点亮)
    (1 << 4)                                // 9: a,b,c,d,f,g (熄灭e，修复：a段应该点亮)
};

// 根据数字设置7个段位引脚
void set_segments(uint8_t num) {
    if (num > 9) num = 0; // 仅显示0-9
    uint8_t mapping = segment_map[num];

    // 1 = 熄灭, 0 = 点亮
    gpio_set_level(SEG_A_PIN, (mapping >> 0) & 1);
    gpio_set_level(SEG_B_PIN, (mapping >> 1) & 1);
    gpio_set_level(SEG_C_PIN, (mapping >> 2) & 1);
    gpio_set_level(SEG_D_PIN, (mapping >> 3) & 1);
    gpio_set_level(SEG_E_PIN, (mapping >> 4) & 1);
    gpio_set_level(SEG_F_PIN, (mapping >> 5) & 1);
    gpio_set_level(SEG_G_PIN, (mapping >> 6) & 1);
}

// 初始化数码管所有GPIO
void init_display_gpio(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SEG_A_PIN) | (1ULL << SEG_B_PIN) | (1ULL << SEG_C_PIN) |
                        (1ULL << SEG_D_PIN) | (1ULL << SEG_E_PIN) | (1ULL << SEG_F_PIN) |
                        (1ULL << SEG_G_PIN) | (1ULL << DIG_1_PIN) | (1ULL << DIG_2_PIN) |
                        (1ULL << DIG_3_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

// 数码管动态扫描任务
void display_task(void *pvParameters) {
    uint32_t count;
    uint8_t digit1, digit2, digit3;

    // 初始关闭所有位选 (共阳，高电平关闭)
    gpio_set_level(DIG_1_PIN, 1);
    gpio_set_level(DIG_2_PIN, 1);
    gpio_set_level(DIG_3_PIN, 1);

    while (1) {
        count = g_counter % 1000; // 只显示0-999

        digit1 = count % 10;        // 个位
        digit2 = (count / 10) % 10;  // 十位
        digit3 = (count / 100) % 10; // 百位

        // --- 显示百位 (DIG1) ---
        set_segments(digit3);
        gpio_set_level(DIG_1_PIN, 0); // 打开百位
        // 分多次延迟，确保tick中断有机会运行
        vTaskDelay(pdMS_TO_TICKS(2));
        vTaskDelay(pdMS_TO_TICKS(2));  // 总共4ms显示时间，提高亮度
        gpio_set_level(DIG_1_PIN, 1); // 关闭百位
        vTaskDelay(pdMS_TO_TICKS(1));  // 位切换间隔

        // --- 显示十位 (DIG2) ---
        set_segments(digit2);
        gpio_set_level(DIG_2_PIN, 0); // 打开十位
        vTaskDelay(pdMS_TO_TICKS(2));
        vTaskDelay(pdMS_TO_TICKS(2));  // 总共4ms显示时间，提高亮度
        gpio_set_level(DIG_2_PIN, 1); // 关闭十位
        vTaskDelay(pdMS_TO_TICKS(1));  // 位切换间隔

        // --- 显示个位 (DIG3) ---
        set_segments(digit1);
        gpio_set_level(DIG_3_PIN, 0); // 打开个位
        vTaskDelay(pdMS_TO_TICKS(2));
        vTaskDelay(pdMS_TO_TICKS(2));  // 总共4ms显示时间，提高亮度
        gpio_set_level(DIG_3_PIN, 1); // 关闭个位
        
        // 总循环时间约 4+1+4+1+4+1+15 = 30ms，刷新率约33Hz
        // 分多次延迟，确保tick中断能够及时运行
        vTaskDelay(pdMS_TO_TICKS(5));
        vTaskDelay(pdMS_TO_TICKS(5));
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}


// *** 4. 输入处理 (PCNT 和 ISR) ***

// 键轴脉冲计数 ISR（下降沿触发）
static void IRAM_ATTR key_switch_isr(void *arg) {
    // 下降沿触发：从高电平到低电平（按下按键，IO0变为低电平）
    // 每次IO0变为低电平时，计数加一
    portENTER_CRITICAL_ISR(&g_pulse_mux);
    g_pulse_count++;
    portEXIT_CRITICAL_ISR(&g_pulse_mux);
}

// 置零按钮 ISR（已改为在主循环中检测持续5秒低电平，不再使用中断）
// 保留此函数以防需要，但不再注册中断处理
static void IRAM_ATTR zero_switch_isr(void *arg) {
    // 不再使用中断方式，改为在主循环中检测持续5秒低电平
}

// 使用 GPIO 中断实现脉冲计数
void init_inputs(void) {
    // 1. 配置 键轴 (GPIO 中断实现脉冲计数)
    ESP_LOGI(TAG, "Initializing GPIO interrupt for pulse counting on GPIO%d...", KEY_SWITCH_PIN);
    gpio_config_t io_conf_key = {
        .pin_bit_mask = (1ULL << KEY_SWITCH_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, // 启用内部上拉，确保初始为高电平
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE // 下降沿触发：按下时从高到低
    };
    esp_err_t ret = gpio_config(&io_conf_key);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to config key switch GPIO: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Key switch GPIO configured successfully");
    }
    
    // 等待一小段时间，确保上拉电阻生效
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // 验证GPIO初始电平（多次读取以确保稳定）
    int initial_level = 0;
    int high_count = 0;
    for (int i = 0; i < 10; i++) {
        if (gpio_get_level(KEY_SWITCH_PIN) == 1) {
            high_count++;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    initial_level = (high_count >= 5) ? 1 : 0;
    ESP_LOGI(TAG, "Key switch GPIO%d initial level: %d (1=high, 0=low), high_count=%d/10", 
             KEY_SWITCH_PIN, initial_level, high_count);
    if (initial_level == 0) {
        ESP_LOGW(TAG, "警告: GPIO%d 初始为低电平，可能无法检测下降沿！", KEY_SWITCH_PIN);
        ESP_LOGW(TAG, "请检查硬件连接：1) 按键是否短路 2) 上拉电阻是否正常工作");
    }

    // 2. 配置 置零按钮 (GPIO1，不使用中断，在主循环中检测持续5秒低电平)
    gpio_config_t io_conf_zero = {
        .pin_bit_mask = (1ULL << ZERO_SWITCH_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, // 内部上拉，确保初始电平为1
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE // 不使用中断，在主循环中轮询检测
    };
    ret = gpio_config(&io_conf_zero);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to config zero switch GPIO: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Zero switch GPIO configured successfully");
    }
    
    // 等待一小段时间，确保上拉电阻生效
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // 验证GPIO1初始电平（多次读取以确保稳定）
    int zero_initial_level = 0;
    int zero_high_count = 0;
    for (int i = 0; i < 10; i++) {
        if (gpio_get_level(ZERO_SWITCH_PIN) == 1) {
            zero_high_count++;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    zero_initial_level = (zero_high_count >= 5) ? 1 : 0;
    ESP_LOGI(TAG, "Zero switch GPIO%d initial level: %d (1=high, 0=low), high_count=%d/10", 
             ZERO_SWITCH_PIN, zero_initial_level, zero_high_count);
    if (zero_initial_level == 0) {
        ESP_LOGW(TAG, "警告: GPIO%d 初始为低电平，可能无法正常工作！", ZERO_SWITCH_PIN);
    }
    
    // 初始化GPIO1持续低电平计时
    g_zero_press_start_tick = 0;
    
    // 安装 GPIO ISR 服务（仅用于KEY_SWITCH_PIN）
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) { // ESP_ERR_INVALID_STATE 表示已经安装过了
        ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(ret));
    }
    
    ret = gpio_isr_handler_add(KEY_SWITCH_PIN, key_switch_isr, (void *)KEY_SWITCH_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add key switch ISR handler: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Key switch ISR handler added successfully");
    }
    
    // 初始化脉冲计数
    g_pulse_count = 0;
    // 将上次处理时间设置为0，确保第一次按键不会被防抖过滤
    g_last_key_process_tick = 0;
    // 初始化按键处理标志
    g_key_press_processed = false;
    
    // 验证GPIO初始状态（应该为高电平）
    int initial_key_level = gpio_get_level(KEY_SWITCH_PIN);
    if (initial_key_level == 1) {
        ESP_LOGI(TAG, "IO0初始化完成: GPIO%d 为高电平 (准备检测下降沿)", KEY_SWITCH_PIN);
        ESP_LOGI(TAG, "工作模式: 使用中断检测下降沿，防抖时间200ms");
    } else {
        ESP_LOGW(TAG, "警告: IO0初始化时GPIO%d为低电平，等待变为高电平...", KEY_SWITCH_PIN);
        // 如果初始是低电平，等待它变为高电平后再开始检测
        int wait_count = 0;
        while (gpio_get_level(KEY_SWITCH_PIN) == 0 && wait_count < 100) {
            vTaskDelay(pdMS_TO_TICKS(10));
            wait_count++;
        }
        if (gpio_get_level(KEY_SWITCH_PIN) == 1) {
            ESP_LOGI(TAG, "IO0已变为高电平，可以正常检测下降沿");
        } else {
            ESP_LOGW(TAG, "警告: IO0持续为低电平，请检查硬件连接");
        }
    }
}


// *** 5. BLE (NimBLE) 通信 ***

// 定义 UUID
static const ble_uuid128_t gatt_svc_uuid =
    BLE_UUID128_INIT(0x28, 0x91, 0xae, 0x8d, 0x3d, 0x45, 0x4f, 0xde,
                     0x81, 0x4a, 0x51, 0x69, 0xd0, 0x18, 0x50, 0x01);
static const ble_uuid128_t gatt_chr_uuid =
    BLE_UUID128_INIT(0x28, 0x91, 0xae, 0x8d, 0x3d, 0x45, 0x4f, 0xde,
                     0x81, 0x4a, 0x51, 0x69, 0xd0, 0x18, 0x50, 0x02);

// GATT 事件处理
static int gatt_svc_access(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    // 客户端读取计数值时调用
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        // 修正：创建 g_counter 的本地副本以移除 volatile 限定符
        uint32_t count_copy = g_counter;
        int rc = os_mbuf_append(ctxt->om, &count_copy, sizeof(count_copy));
        return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    
    // 客户端写入 CCCD (启用/禁用 Notify)
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        ESP_LOGW(TAG, "客户端尝试写入计数值，此操作未实现");
    }

    return 0;
}

// 定义GATT服务
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = &gatt_chr_uuid.u,
                .access_cb = gatt_svc_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &g_counter_val_handle,
            },
            {0} // 结束
        },
    },
    {0} // 结束
};

// BLE 通知任务
void ble_notify_counter(void) {
    // 协议栈会自动检查谁订阅了通知。我们只需要检查连接句柄是否有效。
    if (g_ble_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        // 修正：创建 g_counter 的本地副本以移除 volatile 限定符
        uint32_t count_copy = g_counter;
        struct os_mbuf *om = ble_hs_mbuf_from_flat(&count_copy, sizeof(count_copy));
        if (om) {
            // NimBLE 会自动只发送给已订阅此特征的客户端
            ble_gatts_notify_custom(g_ble_conn_handle, g_counter_val_handle, om);
        }
    }
}

// BLE GAP 事件处理
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "BLE Connected");
        g_ble_conn_handle = event->connect.conn_handle;
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "BLE Disconnected");
        g_ble_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        
        // 修正：调用我们自己的函数来重启广播，而不是错误的 ble_gap_adv_start
        start_ble_advertising();
        
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "BLE Advertising Complete");
        break;
    }
    return 0;
}

// 启动 BLE 广播
void start_ble_advertising(void)
{
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    // 注意：不需要调用 ble_gap_adv_set_fields，因为 ble_svc_gap_device_name_set 已经设置了设备名称
    // GAP服务会自动处理广播字段
    int rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
    if (rc == 0) {
        ESP_LOGI(TAG, "BLE Advertising started");
    } else {
        ESP_LOGE(TAG, "Failed to start BLE advertising: %d", rc);
    }
}

// BLE 协议栈同步回调
void ble_on_sync(void)
{
    ble_hs_id_infer_auto(0, &g_ble_addr_type); 
    start_ble_advertising();
}

// BLE 主任务
void ble_host_task(void *param)
{
    nimble_port_run(); // 此函数不会返回
    nimble_port_freertos_deinit();
}

// *** 6. 主程序 (app_main) ***

void app_main(void)
{
    esp_err_t ret;

    // 初始化 NVS (BLE 必需)
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    g_main_task_handle = xTaskGetCurrentTaskHandle();

    // 1. 初始化硬件
    init_inputs();
    init_display_gpio();

    // 2. 启动数码管显示任务（降低优先级到1，避免阻塞IDLE任务导致看门狗触发）
    // 优先级1是最低优先级（IDLE任务优先级为0），确保不会阻塞系统
    // 注意：显示任务需要频繁运行，但必须让出CPU给tick中断
    xTaskCreate(display_task, "display_task", 2048, NULL, 1, NULL);

    // 3. 初始化 BLE
    nimble_port_init();
    
    // 获取MAC地址后4位作为设备标识，支持多设备识别
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    char device_name[32];
    snprintf(device_name, sizeof(device_name), "Counter-%02X%02X", mac[4], mac[5]);
    ble_svc_gap_device_name_set(device_name); // 设置BLE设备名称（使用MAC地址后4位）
    ESP_LOGI(TAG, "BLE Device Name: %s", device_name);
    
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);
    ble_hs_cfg.sync_cb = ble_on_sync;
    nimble_port_freertos_init(ble_host_task);

    ESP_LOGI(TAG, "ESP32 Counter Initialized");
    ESP_LOGI(TAG, "Current counter value: %lu", g_counter);

    // 4. 主循环
    static uint32_t loop_count = 0;
    while (1) {
        loop_count++;
        TickType_t current_tick = xTaskGetTickCount();
        
        // 简化逻辑：检查来自GPIO中断的脉冲计数（下降沿触发）
        if (g_pulse_count > 0) {
            // 检查是否在防抖时间内
            TickType_t time_since_last = (g_last_key_process_tick == 0) ? 
                                        KEY_DEBOUNCE_TICKS + 1 : 
                                        (current_tick - g_last_key_process_tick);
            
            if (time_since_last >= KEY_DEBOUNCE_TICKS && !g_key_press_processed) {
                // 防抖时间已过，执行计数
                uint32_t pulse_val = 0;
                taskENTER_CRITICAL(&g_pulse_mux);
                pulse_val = g_pulse_count;
                g_pulse_count = 0;
                taskEXIT_CRITICAL(&g_pulse_mux);
                
                if (pulse_val > 0) {
                    g_last_key_process_tick = current_tick;
                    g_key_press_processed = true;
                    // 计数到999后直接回到0
                    g_counter = (g_counter + 1) % 1000;
                    g_counter_updated = true;
                    // 减少日志输出
                    if (g_counter % 10 == 0 || g_counter <= 5) {
                        ESP_LOGI(TAG, "按键按下! 计数: %lu", g_counter);
                    }
                }
            } else {
                // 防抖期间，清零脉冲计数
                taskENTER_CRITICAL(&g_pulse_mux);
                g_pulse_count = 0;
                taskEXIT_CRITICAL(&g_pulse_mux);
            }
        }
        
        // 检查按键是否释放（重置处理标志）
        if (gpio_get_level(KEY_SWITCH_PIN) == 1 && g_key_press_processed) {
            g_key_press_processed = false;
        }

        // 简化逻辑：检测GPIO1持续5秒低电平（清零功能）
        if (gpio_get_level(ZERO_SWITCH_PIN) == 0) {
            if (g_zero_press_start_tick == 0) {
                g_zero_press_start_tick = current_tick;
            } else if ((current_tick - g_zero_press_start_tick) >= pdMS_TO_TICKS(ZERO_PRESS_DURATION_MS)) {
                // 持续5秒，执行清零
                ESP_LOGI(TAG, "GPIO1持续按下5秒，计数清零");
                g_counter = 0;
                g_counter_updated = true;
                taskENTER_CRITICAL(&g_pulse_mux);
                g_pulse_count = 0;
                taskEXIT_CRITICAL(&g_pulse_mux);
                g_zero_press_start_tick = 0;
            }
        } else if (g_zero_press_start_tick != 0) {
            g_zero_press_start_tick = 0;
        }

        // 简化逻辑：BLE通知
        if (g_counter_updated) {
            ble_notify_counter();
            g_counter_updated = false;
        }

        // 减少日志输出
        if (loop_count % 200 == 0) {
            ESP_LOGI(TAG, "状态: 计数器=%lu", g_counter);
        }

        // 简化主循环延迟
        vTaskDelay(pdMS_TO_TICKS(50)); // 50ms 轮询一次
    }
}
