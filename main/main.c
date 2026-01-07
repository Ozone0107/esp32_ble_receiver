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

static void IRAM_ATTR led_blink_action(void) {
    static int level = 0;
    gpio_set_level(ACTION_GPIO, level = !level);
    // 這裡可以加上 player 的其他邏輯
}

void app_main(void) {
    nvs_flash_init();
    
    // 初始化 GPIO (測試用)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ACTION_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = 0, .pull_up_en = 0, .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(ACTION_GPIO, 0);

    // 1. 初始化接收器
    bt_receiver_config_t rx_cfg = {
        .feedback_gpio_num = -1,    // 關閉底層 Debug GPIO
        .manufacturer_id = 0xFFFF,  // 目標廠商 ID
        .sync_window_us = 1000000,    // us
        .queue_size = 20
    };
    bt_receiver_init(&rx_cfg);

    // 2. 啟動
    bt_receiver_start();
    ESP_LOGI(TAG, "System Started. Waiting for signals...");

    // 3. 註冊動作
    bt_receiver_register_callback(led_blink_action);

    // 讓 Main Loop 活著
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}