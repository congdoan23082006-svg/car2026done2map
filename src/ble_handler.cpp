#include "ble_handler.h"
#include "map_manager.h"
#include "sensor.h"
#include "motor.h"
#include "pid_controller.h"
#include "encoder.h"

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

static BLEServer *pServer = NULL;
static BLECharacteristic *pTxCharacteristic = NULL;
static SemaphoreHandle_t s_ble_tx_mutex = NULL; // Khóa Mutex chống xung đột đa luồng BLE
static bool deviceConnected = false;
static bool oldDeviceConnected = false;
static CarRunMode_t s_car_mode = CAR_MODE_STOP;
static int16_t s_scan_speed = 120; // Tốc độ quét mặc định chuẩn thi đấu 120

void car_set_run_mode(CarRunMode_t mode) {
    s_car_mode = mode;
    if (mode == CAR_MODE_STOP) {
        pid_set_auto_mode(false);
        pid_set_sector_action(ACTION_NORMAL);
        motor_stop();
    } else if (mode == CAR_MODE_ALIGN) {
        pid_set_auto_mode(false); // Ở chế độ căn tâm, controlTask trực tiếp điều khiển lực xoay nhẹ quanh tâm
        pid_set_sector_action(ACTION_NORMAL);
    } else {
        // KHÔNG reset xung ở đây! Xung chỉ reset khi người dùng bấm nút CMD:RESET_ENC hoặc khi xuất phát
        pid_set_auto_mode(true);
        pid_set_sector_action(ACTION_NORMAL);
    }
}

CarRunMode_t car_get_run_mode(void) {
    return s_car_mode;
}

void car_set_scan_speed(int16_t speed) {
    s_scan_speed = constrain(speed, 0, 255);
}

int16_t car_get_scan_speed(void) {
    return s_scan_speed;
}

// Callbacks xử lý kết nối BLE
class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) override {
        deviceConnected = true;
        // Kích hoạt ngay chế độ độ trễ siêu thấp (Low Latency: 7.5ms - 15ms)
        pServer->updateConnParams(param->connect.remote_bda, 6, 12, 0, 100);
        Serial.println("\n>> 🔵 [BLE] ĐÃ KẾT NỐI VỚI WEB (Đã kích hoạt Low-Latency 7.5ms)!");
    }
    void onDisconnect(BLEServer* pServer) override {
        deviceConnected = false;
        Serial.println("\n>> 🔴 [BLE] ĐÃ NGẮT KẾT NỐI KHỎI WEB!");
        // Bật lại quảng bá để Web có thể quét kết nối lại bất cứ lúc nào
        pServer->startAdvertising();
    }
};

// Callbacks nhận lệnh từ Web BLE
class RxCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        String rxValue = pCharacteristic->getValue().c_str();
        if (rxValue.length() == 0) return;

        rxValue.trim();
        Serial.printf(">> 📥 [WEB LỆNH]: %s\n", rxValue.c_str());

        // 1. Lệnh điều khiển chạy
        if (rxValue == "CMD:SCAN") {
            // LƯỢT 1: Quét map chậm (bò tốc độ an toàn 70 để lấy mốc)
            car_set_run_mode(CAR_MODE_SCAN);
            ble_send_string("ACK:SCAN_MODE_STARTED");
        } else if (rxValue == "CMD:RACE" || rxValue == "CMD:START") {
            // LƯỢT 2: Bung tốc độ đua theo sa bàn
            car_set_run_mode(CAR_MODE_RACE);
            ble_send_string("ACK:RACE_MODE_STARTED");
        } else if (rxValue == "CMD:STOP") {
            car_set_run_mode(CAR_MODE_STOP);
            ble_send_string("ACK:STOPPED");
        } else if (rxValue == "CMD:CALIB") {
            car_set_run_mode(CAR_MODE_STOP);
            sensor_start_sweep_calibration(4000);
            ble_send_string("ACK:CALIB_STARTED");
        }
        else if (rxValue.startsWith("CMD:SCAN_SPD:")) {
            int16_t spd = rxValue.substring(13).toInt();
            car_set_scan_speed(spd);
            pid_set_base_speed(spd);
            ble_send_string("ACK:SCAN_SPD_SET");
        }
        else if (rxValue == "CMD:RESET_ENC") {
            encoder_reset();
            pid_set_sector_action(ACTION_NORMAL);
            ble_send_string("ACK:ENC_RESET");
        }
        else if (rxValue.startsWith("CMD:PID:")) {
            // Cú pháp: CMD:PID:<kp>:<ki>:<kd>:<spd>[:<turn>]
            // Tự động chuẩn hóa dấu phẩy thành dấu chấm tránh lỗi bàn phím tiếng Việt
            rxValue.replace(',', '.');
            int p1 = rxValue.indexOf(':', 8);
            int p2 = rxValue.indexOf(':', p1 + 1);
            int p3 = rxValue.indexOf(':', p2 + 1);
            int p4 = rxValue.indexOf(':', p3 + 1);
            if (p1 > 0 && p2 > 0 && p3 > 0) {
                float kp = rxValue.substring(8, p1).toFloat();
                float ki = rxValue.substring(p1 + 1, p2).toFloat();
                float kd = rxValue.substring(p2 + 1, p3).toFloat();
                int16_t spd = (p4 > 0) ? rxValue.substring(p3 + 1, p4).toInt() : rxValue.substring(p3 + 1).toInt();
                pid_set_tunings(kp, ki, kd);
                pid_set_base_speed(spd);
                car_set_scan_speed(spd);
                if (p4 > 0) {
                    int16_t turn = rxValue.substring(p4 + 1).toInt();
                    pid_set_turn_speed(turn);
                }
                ble_send_string("ACK:PID_UPDATED");
            }
        }
        else if (rxValue.startsWith("CMD:TURN:")) {
            int16_t turn = rxValue.substring(9).toInt();
            pid_set_turn_speed(turn);
            ble_send_string("ACK:TURN_UPDATED");
        }
        // 2. Chọn map & làn
        else if (rxValue.startsWith("CMD:MAP:")) {
            uint8_t m = rxValue.substring(8).toInt();
            map_select(m);
            ble_send_map_data(m);
        } else if (rxValue.startsWith("CMD:LANE:")) {
            bool right = (rxValue.substring(9).toInt() == 1);
            map_set_right_lane(right);
            ble_send_string(right ? "ACK:LANE_RIGHT" : "ACK:LANE_LEFT");
        }
        // 3. Quản lý Waypoint
        else if (rxValue.startsWith("CMD:WP_SET:")) {
            // Cú pháp: CMD:WP_SET:<map>:<idx>:<ticks>:<speed>:<action>
            int p1 = rxValue.indexOf(':', 11);
            int p2 = rxValue.indexOf(':', p1 + 1);
            int p3 = rxValue.indexOf(':', p2 + 1);
            int p4 = rxValue.indexOf(':', p3 + 1);

            if (p1 > 0 && p2 > 0 && p3 > 0 && p4 > 0) {
                uint8_t m = rxValue.substring(11, p1).toInt();
                uint8_t idx = rxValue.substring(p1 + 1, p2).toInt();
                uint32_t ticks = rxValue.substring(p2 + 1, p3).toInt();
                int16_t spd = rxValue.substring(p3 + 1, p4).toInt();
                uint8_t act = rxValue.substring(p4 + 1).toInt();

                map_set_waypoint(m, idx, ticks, spd, act);
                ble_send_string("ACK:WP_SET_OK");
            }
        } else if (rxValue.startsWith("CMD:WP_CLEAR:")) {
            uint8_t m = rxValue.substring(13).toInt();
            map_clear_waypoints(m);
            ble_send_string("ACK:WP_CLEARED");
        } else if (rxValue == "CMD:SAVE") {
            map_save_to_flash();
            ble_send_string("ACK:SAVED_FLASH");
        } else if (rxValue == "CMD:RESET_DEF") {
            map_load_defaults();
            map_save_to_flash();
            ble_send_map_data(map_get_selected());
            ble_send_string("ACK:DEFAULTS_RESTORED");
        } else if (rxValue.startsWith("CMD:GET_MAP:")) {
            uint8_t m = rxValue.substring(12).toInt();
            ble_send_map_data(m);
        }
        else if (rxValue == "CMD:POLARITY:TOGGLE") {
            bool cur = sensor_get_polarity();
            sensor_set_polarity(!cur);
            ble_send_string(!cur ? "ACK:POLARITY:HIGHER" : "ACK:POLARITY:LOWER");
        }
        else if (rxValue == "CMD:POLARITY:GET") {
            ble_send_string(sensor_get_polarity() ? "ACK:POLARITY:HIGHER" : "ACK:POLARITY:LOWER");
        }
    }
};

void ble_init(const char *device_name) {
    if (!s_ble_tx_mutex) {
        s_ble_tx_mutex = xSemaphoreCreateMutex();
    }
    BLEDevice::init(device_name);
    BLEDevice::setMTU(256);

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    // TX Characteristic (Notify to Web)
    pTxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_TX,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pTxCharacteristic->addDescriptor(new BLE2902());

    // RX Characteristic (Write from Web)
    BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
    );
    pRxCharacteristic->setCallbacks(new RxCallbacks());

    pService->start();

    // Bắt đầu quảng bá (Advertising)
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); // Tăng tốc độ nhận diện
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
}

bool ble_is_connected(void) {
    return deviceConnected;
}

void ble_send_string(const char *str) {
    if (deviceConnected && pTxCharacteristic) {
        if (s_ble_tx_mutex && xSemaphoreTake(s_ble_tx_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            pTxCharacteristic->setValue((uint8_t*)str, strlen(str));
            pTxCharacteristic->notify();
            xSemaphoreGive(s_ble_tx_mutex);
        }
    }
    Serial.printf(">> 📤 [BLE PHẢN HỒI]: %s\n", str);
}

void ble_send_telemetry(int32_t ticks_l, int32_t ticks_r, float line_pos, 
                        int16_t pwm_l, int16_t pwm_r, 
                        uint8_t map_idx, uint8_t wp_idx, uint8_t car_state,
                        uint16_t sensor_mask) {
    if (!deviceConnected || !pTxCharacteristic) return;

    // Định dạng gói tin CSV chuẩn Web:
    // T:<ticks_L>,<ticks_R>,<pos*10>,<pwm_L>,<pwm_R>,<map>,<wp>,<state>,<mask_int>
    char buf[96];
    int16_t pos_scaled = (int16_t)roundf(line_pos * 10.0f);
    snprintf(buf, sizeof(buf), "T:%ld,%ld,%d,%d,%d,%u,%u,%u,%u",
             (long)ticks_l, (long)ticks_r, pos_scaled,
             pwm_l, pwm_r, map_idx, wp_idx, car_state, (unsigned int)sensor_mask);

    if (s_ble_tx_mutex && xSemaphoreTake(s_ble_tx_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        pTxCharacteristic->setValue((uint8_t*)buf, strlen(buf));
        pTxCharacteristic->notify();
        xSemaphoreGive(s_ble_tx_mutex);
    }

    // In Serial mỗi 500ms để kiểm tra tận mắt dữ liệu gửi lên Web
    static uint32_t s_last_dbg = 0;
    if (millis() - s_last_dbg > 500) {
        s_last_dbg = millis();
        Serial.printf(">> 📡 [BLE GỬI]: %s\n", buf);
    }
}

void ble_send_map_data(uint8_t map_idx) {
    if (!deviceConnected || !pTxCharacteristic) return;

    uint8_t count = map_get_waypoint_count(map_idx);
    char buf[64];
    snprintf(buf, sizeof(buf), "MAP_INFO:%u,%u,%u", map_idx, count, map_is_right_lane() ? 1 : 0);
    ble_send_string(buf);
    delay(10);

    for (uint8_t i = 0; i < count; i++) {
        const Waypoint_t *wp = map_get_waypoint(map_idx, i);
        if (wp) {
            snprintf(buf, sizeof(buf), "WP:%u,%u,%lu,%d,%u",
                     map_idx, i, (unsigned long)wp->start_ticks, wp->target_speed, wp->action);
            ble_send_string(buf);
            delay(15); // Khoảng nghỉ nhỏ để tránh tràn bộ đệm BLE Notify
        }
    }
}
