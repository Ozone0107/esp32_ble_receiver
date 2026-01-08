#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*bt_receiver_callback_t)(void);

// --- [修改] 接收到的封包結構 ---
typedef struct {
    uint8_t cmd_type;
    uint64_t target_mask; // 新增：8 bytes 的玩家遮罩
    uint32_t delay_val;
    int8_t rssi;
    int64_t rx_time_us; 
} ble_rx_packet_t;

// --- [修改] 初始化設定 ---
typedef struct {
    int feedback_gpio_num;      
    uint16_t manufacturer_id;   
    int my_player_id;           // 新增：設定這台接收器的 ID (0-63)。設為 -1 代表接收所有 Mask
    uint32_t sync_window_us;    
    uint32_t queue_size;        
} bt_receiver_config_t;

esp_err_t bt_receiver_init(const bt_receiver_config_t *config);
esp_err_t bt_receiver_start(void);
esp_err_t bt_receiver_stop(void);
void bt_receiver_register_callback(bt_receiver_callback_t callback);

#ifdef __cplusplus
}
#endif