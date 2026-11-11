#include <Arduino.h>
#include "config.h"
#include "sensor.h"
#include "motor.h"
#include "encoder.h"
#include "pid_controller.h"
#include "map_manager.h"
#include "ble_handler.h"

// Biến chia sẻ giữa 2 Core
static volatile bool s_run_requested = false;
static volatile float s_live_line_pos = 0.0f; // Vị trí tim line thời gian thực (luôn cập nhật 500Hz)
static uint8_t s_active_wp_idx = 0;
static uint8_t s_active_action = 0;
static int16_t s_active_speed = 120;

// Task handle
static TaskHandle_t s_control_task_handle = NULL;
static TaskHandle_t s_ble_task_handle = NULL;

// =========================================================================
// CORE 1: TASK ĐIỀU KHIỂN ĐUA THỜI GIAN THỰC (Tần số 500Hz = 2ms/chu kỳ)
// Tuyệt đối không bị ảnh hưởng bởi BLE, WiFi, hay các tác vụ mạng!
// =========================================================================
void controlTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(2); // 2ms = 500Hz chuẩn thi đấu

    for (;;) {
        // LUÔN CẬP NHẬT ENCODER ĐỊNH KỲ (2ms) - DÙ XE ĐANG CHẠY HAY ĐỨNG YÊN!
        encoder_update(0.002f);

        // 1. Quá trình Calib cảm biến
        static bool was_calibrating = false;
        if (sensor_is_calibrating()) {
            was_calibrating = true;
            sensor_update_calibration_loop();
            motor_stop();
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            continue;
        } else if (was_calibrating) {
            was_calibrating = false;
            ble_send_string("ACK:CALIB_DONE");
            Serial.println(">> ✅ [CALIB XONG] Đã lưu ngưỡng và tự động vào chế độ CĂN TÂM (V_base = 0)!");
            car_set_run_mode(CAR_MODE_ALIGN);
        }

        // LUÔN ĐỌC VÀ TÍNH TOÁN CẢM BIẾN REAL-TIME - KỂ CẢ KHI XE ĐANG ĐỨNG YÊN!
        // Giúp Web luôn nhận diện 16 mắt bật/tắt và thước ngắm tọa độ sống động
        float cur_pos = 0.0f;
        uint8_t cur_black_cnt = 0;
        pid_calculate_position(&cur_pos, &cur_black_cnt);
        s_live_line_pos = cur_pos; // CẬP NHẬT TỌA ĐỘ VẠCH REAL-TIME LIÊN TỤC 500HZ!

        CarRunMode_t mode = car_get_run_mode();
        static CarRunMode_t s_prev_mode = CAR_MODE_STOP;
        static uint64_t s_race_start_us = 0;
        static uint32_t s_align_start = 0;
        static uint8_t s_finish_debounce = 0;

        // 2. CHẾ ĐỘ CĂN TÂM TẠI CHỖ (Zero-Base Speed Pivot Alignment)
        if (mode == CAR_MODE_ALIGN && calibrationReady) {
            if (s_align_start == 0) s_align_start = millis();

            // An toàn 1: Hẹn giờ 20 giây tự chuyển về STOP để bảo vệ pin và động cơ
            if (millis() - s_align_start > 20000) {
                car_set_run_mode(CAR_MODE_STOP);
                s_align_start = 0;
                motor_coast();
                vTaskDelayUntil(&xLastWakeTime, xFrequency);
                continue;
            }

            // An toàn 2: Nhấc bổng xe (0 mắt đen) -> Thả trôi motor tự do (PWM = 0), không quay tít trên không!
            if (cur_black_cnt == 0) {
                motor_coast();
                vTaskDelayUntil(&xLastWakeTime, xFrequency);
                continue;
            }

            // An toàn 3: Đã ở tim vạch (|cur_pos| < 6.0) -> Thả trôi, kéo tiến/lùi dọc vạch hoàn toàn tự do!
            if (fabsf(cur_pos) < 6.0f) {
                motor_coast();
                vTaskDelayUntil(&xLastWakeTime, xFrequency);
                continue;
            }

            // Lệch góc (|cur_pos| >= 6.0): Tự động xoay vặn đầu xe về tim vạch
            // Dải PWM mạnh mẽ (Min 78 - Max 110, gấp đôi bản cũ) để thắng hoàn toàn ma sát tĩnh của lốp xe
            float err = fabsf(cur_pos);
            int16_t pwm_mag = 78 + (int16_t)((err - 6.0f) * 0.9f);
            if (pwm_mag > 110) pwm_mag = 110;

            // Hướng xoay:
            // cur_pos > 0: Vạch ở bên phải -> Bánh trái tiến (+), bánh phải lùi (-) để xoay phải
            // cur_pos < 0: Vạch ở bên trái -> Bánh trái lùi (-), bánh phải tiến (+) để xoay trái
            if (cur_pos > 0.0f) {
                motor_set_speeds(pwm_mag, -pwm_mag);
            } else {
                motor_set_speeds(-pwm_mag, pwm_mag);
            }

            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            continue;
        } else {
            s_align_start = 0;
        }

        // 3. Quản lý chuyển Mode an toàn (State Transition gác cổng):
        // Bất cứ khi nào đổi mode (STOP -> CHẠY, 1 -> 2, 2 -> 1, hoặc CHẠY -> STOP):
        // Luôn dọn dẹp biến, reset xung và đồng bộ trạng thái sạch sẽ tuyệt đối
        if (mode != s_prev_mode) {
            s_prev_mode = mode;
            encoder_reset();
            pid_set_sector_action(ACTION_NORMAL);
            s_active_wp_idx = 0;
            s_active_action = ACTION_NORMAL;
            s_finish_debounce = 0;

            if (mode == CAR_MODE_SCAN || mode == CAR_MODE_RACE) {
                pid_set_auto_mode(true);
                s_race_start_us = esp_timer_get_time();
                ble_send_string("LOG:AUTO_START");
            } else {
                pid_set_auto_mode(false);
                motor_stop();
            }
        }

        // 4. Chế độ bám line tự động (Quét map hoặc Chạy đua)
        if ((mode == CAR_MODE_SCAN || mode == CAR_MODE_RACE) && calibrationReady) {
            // Lấy số xung trung bình 2 bánh và quãng đường để định vị trên sa bàn
            int64_t ticks_l = encoder_get_ticks_L();
            int64_t ticks_r = encoder_get_ticks_R();
            uint32_t avg_ticks = (uint32_t)((labs(ticks_l) + labs(ticks_r)) / 2);
            float dist_avg_mm = encoder_get_dist_avg_mm();
            uint8_t total_black = sensor_get_black_count();

            // Nhận diện vạch đích an toàn 3 lớp (Tương thích thanh cảm biến hơi cong nhẹ ở rìa):
            // 1. Phải chạy tối thiểu 50cm (500mm / ~515 xung) rời vạch xuất phát
            // 2. Có >= 12 mắt đen (75% dàn mắt - chống trượt khi mép băng dính bị nhăn/hở)
            // 3. Cả 2 cánh ngoài cùng bên Trái (mắt 0..2) và bên Phải (mắt 13..15) đều chạm đen
            bool dist_ok = (dist_avg_mm >= 500.0f || avg_ticks >= 515);
            bool wing_left = (sensor_is_black(0) || sensor_is_black(1) || sensor_is_black(2));
            bool wing_right = (sensor_is_black(13) || sensor_is_black(14) || sensor_is_black(15));
            bool is_crossline_candidate = dist_ok && (total_black >= 12) && wing_left && wing_right;

            int16_t tgt_spd = car_get_scan_speed(); // Đồng bộ tốc độ mặc định 120 (người dùng tự tune trên BLE)
            uint8_t act = 0;
            uint8_t wp_idx = 0;

            if (mode == CAR_MODE_SCAN) {
                // LƯỢT 1: Quét sa bàn chậm, bám line PID cơ bản an toàn tuyệt đối (tốc độ cài đặt)
                tgt_spd = car_get_scan_speed();
                act = ACTION_NORMAL;
                wp_idx = 0;

                // Lọc Debounce vạch đích 5 chu kỳ liên tiếp (10ms) - bắt dính vạch kể cả khi phi tốc độ cao
                if (is_crossline_candidate) {
                    s_finish_debounce++;
                } else {
                    s_finish_debounce = 0;
                }

                if (s_finish_debounce >= 5) {
                    s_finish_debounce = 0;
                    float elapsed = (float)(esp_timer_get_time() - s_race_start_us) / 1000000.0f;
                    char lapMsg[48];
                    snprintf(lapMsg, sizeof(lapMsg), "LOG:RACE_FINISH:%.3f", elapsed);
                    ble_send_string(lapMsg);

                    pid_set_sector_action(ACTION_NORMAL);
                    s_active_wp_idx = 0;
                    s_active_action = ACTION_NORMAL;
                    motor_stop();
                    car_set_run_mode(CAR_MODE_STOP);
                    s_prev_mode = CAR_MODE_STOP;
                    ble_send_string("ACK:SCAN_FINISHED");
                    Serial.printf(">> 🏁 [VẠCH ĐÍCH] Quét map hoàn tất tại %.1f mm (%u xung, %.3fs)\n", dist_avg_mm, avg_ticks, elapsed);
                    vTaskDelayUntil(&xLastWakeTime, xFrequency);
                    continue;
                }
            } else {
                // LƯỢT 2: Chạy đua bung tốc độ theo bảng Waypoint sa bàn
                map_get_current_action(avg_ticks, &tgt_spd, &act, &wp_idx);

                uint8_t total_wps = map_get_waypoint_count(map_get_selected());
                bool is_race_finish_candidate = (act == ACTION_BRAKE_STOP) || 
                    (is_crossline_candidate && (total_wps == 0 || wp_idx >= total_wps - 1));

                if (is_race_finish_candidate) {
                    s_finish_debounce++;
                } else {
                    s_finish_debounce = 0;
                }

                if (s_finish_debounce >= 5) {
                    s_finish_debounce = 0;
                    float elapsed = (float)(esp_timer_get_time() - s_race_start_us) / 1000000.0f;
                    char lapMsg[48];
                    snprintf(lapMsg, sizeof(lapMsg), "LOG:RACE_FINISH:%.3f", elapsed);
                    ble_send_string(lapMsg);

                    pid_set_sector_action(ACTION_NORMAL);
                    s_active_wp_idx = 0;
                    s_active_action = ACTION_NORMAL;
                    motor_stop();
                    car_set_run_mode(CAR_MODE_STOP);
                    s_prev_mode = CAR_MODE_STOP;
                    ble_send_string("ACK:FINISH_STOPPED");
                    Serial.printf(">> 🏁 [VẠCH ĐÍCH] Phanh dừng tại mốc %u (%.1f mm, %u xung, %.3fs)\n", wp_idx, dist_avg_mm, avg_ticks, elapsed);
                    vTaskDelayUntil(&xLastWakeTime, xFrequency);
                    continue;
                }
            }

            s_active_speed = tgt_spd;
            s_active_action = act;
            s_active_wp_idx = wp_idx;

            // Nạp thông số vào bộ điều khiển
            pid_set_base_speed(tgt_spd);
            pid_set_sector_action(act);

            // Thực thi 1 bước PID
            pid_line_follow_step();
        } else {
            // Xe đang dừng: Khóa cứng động cơ nhưng CẢM BIẾN VÀ ENCODER VẪN HOẠT ĐỘNG!
            s_finish_debounce = 0;
            pid_set_sector_action(ACTION_NORMAL);
            s_active_wp_idx = 0;
            s_active_action = ACTION_NORMAL;
            motor_stop();
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ================================================================
// CORE 0: TASK BLE & TRUYỀN THÔNG WEB (Chạy ngầm ở Core 0)
// ================================================================
void bleTask(void *pvParameters) {
    static uint32_t s_last_wait_log = 0;
    for (;;) {
        // Bắn dữ liệu Telemetry về Web mỗi 150ms (chuẩn nhịp mượt theo bản tham khảo)
        if (ble_is_connected()) {
            int32_t tl = (int32_t)encoder_get_ticks_L();
            int32_t tr = (int32_t)encoder_get_ticks_R();
            float pos = s_live_line_pos; // TRUYỀN TỌA ĐỘ REAL-TIME ĐỘNG (KỂ CẢ KHI DỪNG XE)
            int16_t ml = motor_get_left_speed();
            int16_t mr = motor_get_right_speed();
            uint8_t m_idx = map_get_selected();
            uint8_t state = (uint8_t)car_get_run_mode();
            uint16_t mask = sensor_get_line_bits();

            // Gói duy nhất: Tọa độ, Xung, PWM và Mask 16 mắt (T:)
            ble_send_telemetry(tl, tr, pos, ml, mr, m_idx, s_active_wp_idx, state, mask);
        } else {
            if (millis() - s_last_wait_log > 2000) {
                s_last_wait_log = millis();
                Serial.println(">> ⏳ [BLE] Đang chờ Web kết nối... (Tên BLE: RACECAR_16SEN)");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(40)); // 40ms/lần (25Hz) - Phản hồi siêu mượt, tức thời và không lag
    }
}

void setup() {
    Serial.begin(115200);
    delay(400);

    // 1. Cấu hình Nút nhấn chân 35 & Đèn LED báo
    pinMode(PIN_BUTTON_35, INPUT_PULLUP);
    pinMode(PIN_LED_BOARD, OUTPUT);
    digitalWrite(PIN_LED_BOARD, LOW);

    // 2. Khởi tạo ngoại vi phần cứng
    motor_init();
    sensor_init();
    encoder_init();
    map_manager_init();

    // 3. Khởi tạo bộ điều khiển PID chuẩn người dùng đã test (Kp=2.0, Ki=0.0, Kd=20.0, baseSpeed=110)
    pid_init(2.0f, 0.0f, 20.0f, 110);

    // 4. Khởi tạo BLE Server với tên nhận diện
    ble_init("RACECAR_16SEN");

    Serial.println("\n=======================================================");
    Serial.println("  🏎️  XE DÒ LINE 16 MẮT 2026 - DUAL-CORE FREERTOS + BLE");
    Serial.println("=======================================================");
    Serial.println(" ✅ CORE 1: Đang chạy Task điều khiển PID & Động cơ (500Hz)");
    Serial.println(" ✅ CORE 0: Đang chạy Task Bluetooth BLE & Web Dashboard");
    Serial.println(" 📡 Tên thiết bị BLE: RACECAR_16SEN");
    Serial.println(" 🌐 Mở Web HTML trên điện thoại để kết nối và điều khiển!");
    Serial.println("=======================================================\n");

    // 5. Tạo Task Core 1 (Ưu tiên cao nhất: 10)
    xTaskCreatePinnedToCore(
        controlTask,
        "ControlTask",
        8192,
        NULL,
        10,
        &s_control_task_handle,
        1 // Ghim vào CORE 1
    );

    // 6. Tạo Task Core 0 (Ưu tiên thông thường: 2)
    xTaskCreatePinnedToCore(
        bleTask,
        "BleTask",
        4096,
        NULL,
        2,
        &s_ble_task_handle,
        0 // Ghim vào CORE 0
    );
}

void loop() {
    // -------------------------------------------------------------
    // NÚT BẤM VẬT LÝ CHÂN 35 (Dự phòng khẩn cấp tại sân đấu)
    // -------------------------------------------------------------
    static int last_btn_state = HIGH;
    static unsigned long last_btn_time = 0;

    int btn_read = digitalRead(PIN_BUTTON_35);
    if (btn_read != last_btn_state && (millis() - last_btn_time > 50)) {
        last_btn_state = btn_read;
        last_btn_time = millis();

        if (btn_read == LOW) {
            if (!calibrationReady) {
                // Nhấn lần 1 chưa calib -> Quét calib 4s
                Serial.println(">> [NÚT 35] Bắt đầu Calib quét 4s...");
                sensor_start_sweep_calibration(4000);
            } else {
                // Đã calib -> Bật / Tắt chạy đua
                CarRunMode_t cur = car_get_run_mode();
                if (cur == CAR_MODE_STOP || cur == CAR_MODE_ALIGN) {
                    car_set_run_mode(CAR_MODE_RACE);
                    Serial.println(">> [NÚT 35] BUNG TỐC ĐỘ ĐUA: BẬT <<");
                } else {
                    car_set_run_mode(CAR_MODE_STOP);
                    Serial.println(">> [NÚT 35] DỪNG XE <<");
                }
            }
        }
    }

    // Đèn LED báo trạng thái:
    // - Nhấp nháy: Đang Calib
    // - Sáng đứng yên: Sẵn sàng / Đang chạy đua
    // - Tắt: Đang dừng
    if (sensor_is_calibrating()) {
        digitalWrite(PIN_LED_BOARD, (millis() / 100) % 2);
    } else if (pid_is_auto_mode()) {
        digitalWrite(PIN_LED_BOARD, HIGH);
    } else {
        digitalWrite(PIN_LED_BOARD, ble_is_connected() ? HIGH : LOW);
    }

    delay(20);
}
