/*
 * bt_receiver.h
 * 合併了原有的 ble_fast_rx, ble_sync_core 與 bt_hci_common 的功能
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// 定義 Callback 函式型別 (時間到時要執行的動作)
typedef void (*bt_receiver_callback_t)(void);

// 整合後的設定結構
typedef struct {
    int feedback_gpio_num;      // 用於示波器除錯的 GPIO (-1 為不使用)
    uint16_t manufacturer_id;   // 要監聽的廠商 ID (如 0xFFFF)
    uint32_t sync_window_us;    // 收集樣本的時間窗口 (建議 50000 us)
    uint32_t queue_size;        // 內部緩衝區大小 (建議 20)
} bt_receiver_config_t;

/**
 * @brief 初始化藍牙接收器 (GPIO, Queue, BT Controller, Timer)
 */
esp_err_t bt_receiver_init(const bt_receiver_config_t *config);

/**
 * @brief 開始接收並執行同步任務
 */
esp_err_t bt_receiver_start(void);

/**
 * @brief 停止接收
 */
esp_err_t bt_receiver_stop(void);

/**
 * @brief 動態註冊/更換時間到要執行的動作
 * @param callback 函式指標，傳入 NULL 則取消動作
 */
void bt_receiver_register_callback(bt_receiver_callback_t callback);

#ifdef __cplusplus
}
#endif