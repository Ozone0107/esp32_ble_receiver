
#include "bt_receiver.h"
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_bt.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "esp_timer.h"

// --- HCI Macros & Constants (Internal) ---
#define HCI_H4_CMD_PREAMBLE_SIZE           (4)
#define HCI_GRP_HOST_CONT_BASEBAND_CMDS    (0x03 << 10)
#define HCI_GRP_BLE_CMDS                   (0x08 << 10)
#define HCI_RESET                          (0x0003 | HCI_GRP_HOST_CONT_BASEBAND_CMDS)
#define HCI_SET_EVT_MASK                   (0x0001 | HCI_GRP_HOST_CONT_BASEBAND_CMDS)
#define HCI_BLE_WRITE_SCAN_PARAM           (0x000B | HCI_GRP_BLE_CMDS)
#define HCI_BLE_WRITE_SCAN_ENABLE          (0x000C | HCI_GRP_BLE_CMDS)
#define HCIC_PARAM_SIZE_SET_EVENT_MASK         (8)
#define HCIC_PARAM_SIZE_BLE_WRITE_SCAN_PARAM   (7)
#define HCIC_PARAM_SIZE_BLE_WRITE_SCAN_ENABLE  (2)

#define UINT16_TO_STREAM(p, u16) {*(p)++ = (uint8_t)(u16); *(p)++ = (uint8_t)((u16) >> 8);}
#define UINT8_TO_STREAM(p, u8)   {*(p)++ = (uint8_t)(u8);}
#define ARRAY_TO_STREAM(p, a, len) {int ijk; for (ijk = 0; ijk < len; ijk++) *(p)++ = (uint8_t) a[ijk];}

enum { H4_TYPE_COMMAND = 1, H4_TYPE_ACL = 2, H4_TYPE_SCO = 3, H4_TYPE_EVENT = 4 };


static const char *TAG = "BT_RECEIVER";

static bt_receiver_config_t s_config;
static QueueHandle_t s_adv_queue = NULL;
static esp_timer_handle_t s_action_timer = NULL;
static esp_timer_handle_t s_prep_timer = NULL;   // 負責關準備燈
static TaskHandle_t s_task_handle = NULL;
static bool s_is_running = false;
static uint8_t hci_cmd_buf[128];
static volatile bt_receiver_callback_t s_registered_callback = NULL;
static volatile uint8_t s_pending_cmd = 0;

// 指令代碼轉成字串(輸出用)
static const char* cmd_to_str(bt_cmd_t cmd) {
    switch (cmd) {
        case BT_CMD_RESET: return "RESET"; 
        case BT_CMD_READY: return "READY";
        case BT_CMD_TEST:  return "TEST";
        case BT_CMD_PLAY:  return "PLAY";
        case BT_CMD_PAUSE: return "PAUSE";
        default:           return "UNKNOWN";
    }
}

// ==========================================
// Part 1: HCI Helper Functions (Private)
// ==========================================
static uint16_t make_cmd_reset(uint8_t *buf) {
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_RESET);
    UINT8_TO_STREAM (buf, 0);
    return HCI_H4_CMD_PREAMBLE_SIZE;
}

static uint16_t make_cmd_set_evt_mask (uint8_t *buf, uint8_t *evt_mask) {
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_SET_EVT_MASK);
    UINT8_TO_STREAM  (buf, HCIC_PARAM_SIZE_SET_EVENT_MASK);
    ARRAY_TO_STREAM(buf, evt_mask, HCIC_PARAM_SIZE_SET_EVENT_MASK);
    return HCI_H4_CMD_PREAMBLE_SIZE + HCIC_PARAM_SIZE_SET_EVENT_MASK;
}

static uint16_t make_cmd_ble_set_scan_params (uint8_t *buf, uint8_t scan_type,
                                       uint16_t scan_interval, uint16_t scan_window,
                                       uint8_t own_addr_type, uint8_t filter_policy) {
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_BLE_WRITE_SCAN_PARAM);
    UINT8_TO_STREAM  (buf, HCIC_PARAM_SIZE_BLE_WRITE_SCAN_PARAM);
    UINT8_TO_STREAM (buf, scan_type);
    UINT16_TO_STREAM (buf, scan_interval);
    UINT16_TO_STREAM (buf, scan_window);
    UINT8_TO_STREAM (buf, own_addr_type);
    UINT8_TO_STREAM (buf, filter_policy);
    return HCI_H4_CMD_PREAMBLE_SIZE + HCIC_PARAM_SIZE_BLE_WRITE_SCAN_PARAM;
}

static uint16_t make_cmd_ble_set_scan_enable (uint8_t *buf, uint8_t scan_enable, uint8_t filter_duplicates) {
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_BLE_WRITE_SCAN_ENABLE);
    UINT8_TO_STREAM  (buf, HCIC_PARAM_SIZE_BLE_WRITE_SCAN_ENABLE);
    UINT8_TO_STREAM (buf, scan_enable);
    UINT8_TO_STREAM (buf, filter_duplicates);
    return HCI_H4_CMD_PREAMBLE_SIZE + HCIC_PARAM_SIZE_BLE_WRITE_SCAN_ENABLE;
}

// ==========================================
// Part 2: Fast RX Driver & ISR
// ==========================================

//確認與解析封包
static void IRAM_ATTR fast_parse_and_trigger(uint8_t *data, uint16_t len) {
    int64_t now_us = esp_timer_get_time();

    // Check Header: Event(0x04) -> LE Meta(0x3E) -> Adv Report(0x02)
    if (data[0] != 0x04 || data[1] != 0x3E || data[3] != 0x02) return;

    uint8_t num_reports = data[4];
    uint8_t *payload = &data[5];

    for (int i = 0; i < num_reports; i++) {
        uint8_t data_len = payload[8];
        uint8_t *adv_data = &payload[9];
        int8_t rssi = payload[9 + data_len];

        uint8_t offset = 0;
while (offset < data_len) {
            uint8_t ad_len = adv_data[offset++];
            if (ad_len == 0) break;
            uint8_t ad_type = adv_data[offset++];

            // Type(1) + ID(2) + CMD(1) + Mask(8) + Delay(4) + Prep(4) = 20 bytes
            if (ad_type == 0xFF && ad_len >= 20) {
                uint16_t target_id = s_config.manufacturer_id;
                
                // 檢查 Manufacturer ID (Offset 0, 1)
                if (adv_data[offset] == (target_id & 0xFF) && 
                    adv_data[offset+1] == ((target_id >> 8) & 0xFF)) {
                    
                    // 解析 Mask (Offset 3 ~ 10)
                    uint64_t rcv_mask = 0;
                    for(int k=0; k<8; k++) {
                        rcv_mask |= ((uint64_t)adv_data[offset + 3 + k] << (k*8));
                    }

                    // 檢查 Target ID (邏輯不變)
                    bool is_target = true;
                    if (s_config.my_player_id >= 0) {
                        if (!((rcv_mask >> s_config.my_player_id) & 1ULL)) {
                            is_target = false;
                        }
                    }

                    if (is_target) {
                        // GPIO Trigger 
                        if (s_config.feedback_gpio_num >= 0) {
                            gpio_set_level(s_config.feedback_gpio_num, 1);
                            esp_rom_delay_us(1);
                            gpio_set_level(s_config.feedback_gpio_num, 0);
                        }

                        // 解析 CMD (Offset 2)
                        uint8_t rcv_cmd = adv_data[offset+2];

                        // 解析 Delay (Offset 11 ~ 14) - Big Endian
                        uint32_t rcv_delay = (adv_data[offset+11] << 24) |
                                             (adv_data[offset+12] << 16) |
                                             (adv_data[offset+13] << 8)  |
                                             (adv_data[offset+14]);

                        // [新增] 解析 Prep LED Time (Offset 15 ~ 18) - Big Endian
                        // Sender 寫入順序: MSB 在前 (>>24 先寫入)
                        uint32_t rcv_prep = (adv_data[offset+15] << 24) |
                                            (adv_data[offset+16] << 16) |
                                            (adv_data[offset+17] << 8)  |
                                            (adv_data[offset+18]);

                        // 填入結構
                        ble_rx_packet_t pkt;
                        pkt.cmd_type = (bt_cmd_t)rcv_cmd; 
                        pkt.target_mask = rcv_mask;
                        pkt.delay_val = rcv_delay;
                        pkt.prep_led_us = rcv_prep; 
                        pkt.rssi = rssi;
                        pkt.rx_time_us = now_us;

                        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                        xQueueSendFromISR(s_adv_queue, &pkt, &xHigherPriorityTaskWoken);
                        
                        if (xHigherPriorityTaskWoken) {
                            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                        }
                        
                        return; 
                    }
                }
            }
            offset += (ad_len - 1);
        }
        payload += (10 + data_len + 1);
    }
}

static int host_rcv_pkt(uint8_t *data, uint16_t len) {
    fast_parse_and_trigger(data, len);
    return ESP_OK;
}
static void controller_rcv_pkt_ready(void) {}
static esp_vhci_host_callback_t vhci_host_cb = { controller_rcv_pkt_ready, host_rcv_pkt };


// ==========================================
// Part 3: Sync Logic & Timer
// ==========================================

static void IRAM_ATTR timer_timeout_cb(void *arg) {
    if (s_registered_callback) {
        s_registered_callback(s_pending_cmd, BT_TRIG_SYNC, 0);
    }
}

// Prep Timer Callback (關準備燈)
static void IRAM_ATTR prep_timer_timeout_cb(void *arg) {
    if (s_registered_callback) {
        // 傳遞 PREP_END 觸發
        s_registered_callback(s_pending_cmd, BT_TRIG_PREP_END, 0);
    }
}

//以收到的封包算出開始時間
static void sync_process_task(void *arg) {
    ble_rx_packet_t pkt;
    bt_cmd_t current_cmd = BT_CMD_UNKNOWN;
    bt_cmd_t processed_cmd = BT_CMD_UNKNOWN;
    
    uint32_t current_prep_us = 0; // 暫存 Prep 時間

    int64_t sum_target = 0;
    int count = 0;
    bool collecting = false;
    int64_t window_start_time = 0;

    ESP_LOGI(TAG, "Sync Task Running...");

    while (s_is_running) {
        if (xQueueReceive(s_adv_queue, &pkt, pdMS_TO_TICKS(10)) == pdTRUE) {
            int64_t now = esp_timer_get_time();

            if (pkt.cmd_type != processed_cmd) {
                // 偵測新 Burst (開始收集)
                if (!collecting || pkt.cmd_type != current_cmd) {
                    
                    // 收到新指令，先把兩個計時器都停掉 (避免舊的干擾)
                    if (s_action_timer) esp_timer_stop(s_action_timer);
                    if (s_prep_timer) esp_timer_stop(s_prep_timer);

                    collecting = true;
                    current_cmd = (bt_cmd_t)pkt.cmd_type;
                    current_prep_us = pkt.prep_led_us; // 抓取 Prep 時間
                    
                    sum_target = 0;
                    count = 0;
                    window_start_time = now;

                    // 立即觸發 IMMEDIATE (開準備燈)
                    if (s_registered_callback) {
                        s_registered_callback((uint8_t)current_cmd, BT_TRIG_IMMEDIATE, current_prep_us);
                    }
                }

                // 收集封包
                if (collecting && now < (window_start_time + s_config.sync_window_us)) {
                    sum_target += (pkt.rx_time_us + pkt.delay_val);
                    count++;
                }
            }
        }

        // 結算
        if (collecting) {
            int64_t now = esp_timer_get_time();
            if (now >= (window_start_time + s_config.sync_window_us)) {
                collecting = false; 

                if (count > 0) {
                    int64_t final_target = sum_target / count;
                    int64_t wait_for_action = final_target - now;

                    // window_start_time 近似於發送時間
                    int64_t wait_for_prep = (window_start_time + current_prep_us) - now;

                    s_pending_cmd = (uint8_t)current_cmd;
                    processed_cmd = current_cmd; 

                    // 1. 設定 Action Timer (主動作)
                    if (wait_for_action > 500) {
                        esp_timer_start_once(s_action_timer, wait_for_action);
                        ESP_LOGI(TAG, "CMD: [%s] Action in: %lld us", cmd_to_str(current_cmd), wait_for_action);
                    } else {
                        ESP_LOGW(TAG, "CMD Missed! [%s] Latency too high.", cmd_to_str(current_cmd));
                    }

                    // 2. 設定 Prep Timer (關燈)
                    if (current_cmd == BT_CMD_PLAY &&     
                        current_prep_us > 0 && 
                        wait_for_prep > 1000 && 
                        wait_for_prep < wait_for_action) {
                        
                        esp_timer_start_once(s_prep_timer, wait_for_prep);
                        ESP_LOGI(TAG, "Prep Timer Set: %lld us", wait_for_prep);
                    }
                }
            }
        }
    }
    vTaskDelete(NULL);
}

// ==========================================
// Part 4: Public APIs
// ==========================================

esp_err_t bt_receiver_init(const bt_receiver_config_t *config) {
    if (!config) return ESP_ERR_INVALID_ARG;
    s_config = *config;

   // 1. Create Queue
    s_adv_queue = xQueueCreate(s_config.queue_size, sizeof(ble_rx_packet_t));
    if (!s_adv_queue) return ESP_ERR_NO_MEM;

    // 2. Setup GPIO (if used)
    if (s_config.feedback_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << s_config.feedback_gpio_num),
            .mode = GPIO_MODE_OUTPUT,
            .pull_down_en = 0, .pull_up_en = 0, .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&io_conf);
        gpio_set_level(s_config.feedback_gpio_num, 0);
    }

    // 3. Init BT Controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_vhci_host_register_callback(&vhci_host_cb);

    // 4. Create Timers
    // Action Timer
    esp_timer_create_args_t action_timer_args = {
        .callback = &timer_timeout_cb,
        .name = "bt_action_timer"
    };
    esp_timer_create(&action_timer_args, &s_action_timer);

    // Prep Timer
    esp_timer_create_args_t prep_timer_args = {
        .callback = &prep_timer_timeout_cb,
        .name = "bt_prep_timer"
    };
    esp_timer_create(&prep_timer_args, &s_prep_timer);

    return ESP_OK;
}

esp_err_t bt_receiver_start(void) {
    if (s_is_running) return ESP_OK;

    // Send HCI Commands
    uint16_t sz = make_cmd_reset(hci_cmd_buf);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
    vTaskDelay(pdMS_TO_TICKS(20));
    
    uint8_t mask[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20};
    sz = make_cmd_set_evt_mask(hci_cmd_buf, mask);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
    vTaskDelay(pdMS_TO_TICKS(20));

    sz = make_cmd_ble_set_scan_params(hci_cmd_buf, 0x00, 0x0F, 0x0F, 0x00, 0x00);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
    vTaskDelay(pdMS_TO_TICKS(20));

    sz = make_cmd_ble_set_scan_enable(hci_cmd_buf, 1, 0);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);

    // Start Logic Task
    s_is_running = true;
    xTaskCreatePinnedToCore(sync_process_task, "bt_rx_task", 4096, NULL, 5, &s_task_handle, 1);
    
    ESP_LOGI(TAG, "Receiver Started");
    return ESP_OK;
}

esp_err_t bt_receiver_stop(void) {
    s_is_running = false;
    
    // Stop Scanning
    uint16_t sz = make_cmd_ble_set_scan_enable(hci_cmd_buf, 0, 0);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
    
    // Stop Timer
    if (s_action_timer) esp_timer_stop(s_action_timer);
    if (s_prep_timer) esp_timer_stop(s_prep_timer); 
    
    return ESP_OK;
}

void bt_receiver_register_callback(bt_receiver_callback_t callback) {
    s_registered_callback = callback;
    if (callback) {
        ESP_LOGI(TAG, "Callback Registered");
    } else {
        ESP_LOGI(TAG, "Callback Cleared");
    }
}