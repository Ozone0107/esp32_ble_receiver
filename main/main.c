#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

#include "bt_receiver.h"

static const char *TAG = "APP_MAIN";

#define ACTION_GPIO 2

static void IRAM_ATTR state_action_callback(uint8_t cmd_raw, bt_trigger_t trigger, uint32_t val) {
    bt_cmd_t cmd = (bt_cmd_t)cmd_raw;

    //剛收到封包 PLAY 亮準備燈
    if (trigger == BT_TRIG_IMMEDIATE) {
        if (cmd == BT_CMD_PLAY) {
            ESP_LOGI(TAG, ">>> [IMMEDIATE] LED ON (Prep: %lu us)", val);
            gpio_set_level(ACTION_GPIO, 1);
        }
        else if (cmd == BT_CMD_RESET) {
            ESP_LOGI(TAG, ">>> [IMMEDIATE] RESET received, LED OFF");
            gpio_set_level(ACTION_GPIO, 0);
        }
    }
    // 關準備燈
    else if (trigger == BT_TRIG_PREP_END) {
        if (cmd == BT_CMD_PLAY) {
            ESP_LOGI(TAG, ">>> [PREP END] LED OFF");
            gpio_set_level(ACTION_GPIO, 0);
        }
    }

    // 同步時間到 (SYNC)
    else if (trigger == BT_TRIG_SYNC) {
        switch (cmd) {
            case BT_CMD_PLAY:
                ESP_LOGW(TAG, ">>> [SYNC ACTION] PLAY");
                // start_music();
                break;
            case BT_CMD_RESET:
                ESP_LOGW(TAG, ">>> [SYNC ACTION] RESET");
                break;
            case BT_CMD_PAUSE:
                ESP_LOGW(TAG, ">>> [SYNC ACTION] PAUSE");
                break;
            case BT_CMD_TEST:
                ESP_LOGW(TAG, ">>> [SYNC ACTION] TEST");
                break;
            case BT_CMD_READY:
                ESP_LOGW(TAG, ">>> [SYNC ACTION] READY");
                break;           
            default:
                break;
        }
    }
}
// 測試用：閃爍 LED
static void IRAM_ATTR led_blink_action(uint8_t cmd_raw, bt_trigger_t trigger, uint32_t val) {
    static int level = 0;
    gpio_set_level(ACTION_GPIO, level = !level);

}

void app_main(void) {

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
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
        .feedback_gpio_num = -1,    // 關閉底層 GPIO
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
    bt_receiver_register_callback(state_action_callback);
    //bt_receiver_register_callback(led_blink_action);
    // 讓 Main Loop 保持活躍
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}