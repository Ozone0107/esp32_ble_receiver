#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BT_CMD_RESET = 0x01,
    BT_CMD_READY = 0x02,
    BT_CMD_TEST  = 0x03,
    BT_CMD_PLAY  = 0xA0,
    BT_CMD_PAUSE = 0xA1,
    BT_CMD_UNKNOWN = 0xFF
} bt_cmd_t;

typedef enum {
    BT_TRIG_IMMEDIATE, // 剛收到封包 (亮準備燈)
    BT_TRIG_PREP_END,  // 預備時間結束 (關準備燈)
    BT_TRIG_SYNC       // 同步時間到達 (執行主要動作)
} bt_trigger_t;

// 定義 Callback 
typedef void (*bt_receiver_callback_t)(uint8_t cmd, bt_trigger_t trigger, uint32_t val);

// 定義封包結構
typedef struct {
    bt_cmd_t cmd_type;     // 指令類型
    uint64_t target_mask;  // 目標遮罩
    uint32_t delay_val;    // 延遲執行時間
    uint32_t prep_led_us;  // 預備燈號時間
    int8_t rssi;           // 訊號強度
    int64_t rx_time_us;    // 接收時間戳
} ble_rx_packet_t;

// 初始化設定
typedef struct {
    int feedback_gpio_num;      
    uint16_t manufacturer_id;   
    int my_player_id;           
    uint32_t sync_window_us;    
    uint32_t queue_size;        
} bt_receiver_config_t;

// 函式宣告
esp_err_t bt_receiver_init(const bt_receiver_config_t *config);
esp_err_t bt_receiver_start(void);
esp_err_t bt_receiver_stop(void);
void bt_receiver_register_callback(bt_receiver_callback_t callback);

#ifdef __cplusplus
}
#endif