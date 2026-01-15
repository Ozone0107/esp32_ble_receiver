#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

// 引用新的整合 API
#include "bt_receiver.h"

static const char *TAG = "APP_MAIN";

#define ACTION_GPIO 2

static void IRAM_ATTR state_action_callback(uint8_t cmd) {
    
    // 直接使用 bt_receiver.h 裡定義的 Enum
    switch (cmd) {
        case BT_CMD_RESET:
            ESP_LOGI(TAG, ">>> [收到指令] RESET");
            break;
        case BT_CMD_READY:
            ESP_LOGI(TAG, ">>> [收到指令] READY");
            break;
        case BT_CMD_PLAY:
            ESP_LOGI(TAG, ">>> [收到指令] PLAY");
            break;
        case BT_CMD_TEST:
            ESP_LOGI(TAG, ">>> [收到指令] TEST");
            break;
        case BT_CMD_PAUSE:
            ESP_LOGI(TAG, ">>> [收到指令] PAUSE");
            break;
        default:
            ESP_LOGE(TAG, ">>> [未知指令] ID: 0x%02X", cmd);
            break;
    }
}
// 測試用的 Callback：閃爍 LED
static void IRAM_ATTR led_blink_action(uint8_t cmd) {
    static int level = 0;
    gpio_set_level(ACTION_GPIO, level = !level);
    // 這裡通常是 Player Task 的通知入口
    // 例如：xTaskNotifyFromISR(player_task_handle, ...);
}

void app_main(void) {
    nvs_flash_init();
    
    // 初始化 GPIO (測試 Action 觸發用)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ACTION_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = 0, .pull_up_en = 0, .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(ACTION_GPIO, 0);

    // 1. 初始化接收器
    bt_receiver_config_t rx_cfg = {
        .feedback_gpio_num = -1,    // 關閉底層 GPIO (若想用示波器看收包訊號可填入 GPIO 號碼)
        .manufacturer_id = 0xFFFF,  // 目標廠商 ID
        .my_player_id = 1,          
        .sync_window_us = 500000,    // 500ms
        .queue_size = 20
    };
    bt_receiver_init(&rx_cfg);

    // 2. 啟動
    bt_receiver_start();
    ESP_LOGI(TAG, "System Started. I am Player #%d.", rx_cfg.my_player_id);

    // 3. 註冊動作
    // 設定當 Target Time 到達時，要執行什麼動作
    //bt_receiver_register_callback(state_action_callback);
    bt_receiver_register_callback(led_blink_action);
    // 讓 Main Loop 保持活躍
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}