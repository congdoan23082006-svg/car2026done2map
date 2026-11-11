#include "pid_controller.h"
#include "sensor.h"
#include "motor.h"
#include "encoder.h"
#include <Arduino.h>

// =================== CẤU HÌNH TRỌNG SỐ CHO 16 MẮT ===================
// Tâm vạch lý tưởng nằm chính giữa mắt 7 và 8 (Error = 0.0)
// Lệch sang trái: Error < 0 | Lệch sang phải: Error > 0
static const float SENSOR_WEIGHTS[N_SENSORS] = {
    -75.0f, -65.0f, -55.0f, -45.0f, -35.0f, -25.0f, -15.0f, -5.0f, // 8 mắt trái (0..7)
      5.0f,  15.0f,  25.0f,  35.0f,  45.0f,  55.0f,  65.0f,  75.0f  // 8 mắt phải (8..15)
};

// Cấu hình PID chuẩn người dùng đã test chạy ngon: Kp=2.0, Ki=0.0, Kd=20.0, baseSpeed=110, turnSpeed=130
static PID_Config_t pidConfig = {
    .Kp = 2.0f,
    .Ki = 0.0f,
    .Kd = 20.0f,
    .baseSpeed = 110,
    .maxSpeed = 255,
    .minSpeed = -25,   // Khóa đảo chiều gắt: Bánh trong chỉ hãm nhẹ (-25), giữ dính lốp sàn, triệt tiêu 100% lắc đuôi
    .turnSpeed = 130
};

static bool autoMode = false;
static LineCase_t currentCase = LINE_CASE_STOPPED;
static float lastError = 0.0f;
static float integralError = 0.0f;
static float lastCorrection = 0.0f;
static unsigned long lastComputeTime = 0;
static float s_ramped_base = 0.0f; // Tốc độ nền tăng tốc mềm (Soft-start Ramp)
static float s_filtered_d = 0.0f;  // Bộ lọc thông thấp EMA khử rung rơ khâu vi phân D

#define DASHED_LINE_TIMEOUT_MS   60  // Thời gian cho phép lướt qua nét đứt (ms) - chuẩn hóa cho motor GA25 370 RPM (hở 10-12cm vẫn lướt êm)
static unsigned long lostLineStartTime = 0;
static bool isLineCurrentlyLost = false;
static int8_t lastKnownDirection = 0; // -1: Trái, +1: Phải, 0: Giữa tâm
static float lastKnownPosition = 0.0f;// Vị trí vạch cuối cùng còn nhìn thấy

// Cơ chế chốt cua góc vuông 90° (Active 90° Corner Latch)
static uint8_t s_corner_latch = 0; // 0: None, 1: Turn Left, 2: Turn Right
static unsigned long s_corner_latch_time = 0;
static bool s_has_parallel_line = false; // Cờ phát hiện line song song để triệt tiêu bắt nhầm góc vuông

// Cấu trúc mô tả một cụm vạch đen liên tục (Line Cluster)
typedef struct {
    uint8_t start_idx;
    uint8_t end_idx;
    uint8_t black_count;
    uint32_t sumCalibrated;
    float sumWeights;
    float centroid;
} SensorCluster_t;

static uint8_t s_sector_action = 0; // 0: Normal, 1: Force Left, 2: Force Right, 3: Lock Straight, 4: Brake

static inline int16_t clamp_int16(int16_t v, int16_t min_v, int16_t max_v) {
    if (v < min_v) return min_v;
    if (v > max_v) return max_v;
    return v;
}

void pid_init(float kp, float ki, float kd, int16_t baseSpeed) {
    pidConfig.Kp = kp;
    pidConfig.Ki = ki;
    pidConfig.Kd = kd;
    pidConfig.baseSpeed = baseSpeed;
    lastError = 0.0f;
    integralError = 0.0f;
    lastComputeTime = millis();
    autoMode = false;
    currentCase = LINE_CASE_STOPPED;
    isLineCurrentlyLost = false;
    lastKnownDirection = 0;
    lastKnownPosition = 0.0f;
    s_corner_latch = 0;
    s_corner_latch_time = 0;
    s_has_parallel_line = false;
    s_ramped_base = 0.0f;
    s_filtered_d = 0.0f;
}

void pid_set_tunings(float kp, float ki, float kd) {
    pidConfig.Kp = kp;
    pidConfig.Ki = ki;
    pidConfig.Kd = kd;
}

void pid_set_base_speed(int16_t speed) {
    pidConfig.baseSpeed = clamp_int16(speed, 0, 255);
}

void pid_set_turn_speed(int16_t speed) {
    pidConfig.turnSpeed = clamp_int16(speed, 60, 220);
}

int16_t pid_get_turn_speed(void) {
    return pidConfig.turnSpeed;
}

void pid_set_sector_action(uint8_t action) {
    s_sector_action = action;
}

uint8_t pid_get_sector_action(void) {
    return s_sector_action;
}

void pid_set_auto_mode(bool enable) {
    autoMode = enable;
    isLineCurrentlyLost = false;
    lastKnownDirection = 0;
    lastKnownPosition = 0.0f;
    s_corner_latch = 0;
    s_corner_latch_time = 0;
    s_sector_action = 0;
    s_has_parallel_line = false;
    s_filtered_d = 0.0f;
    if (!autoMode) {
        currentCase = LINE_CASE_STOPPED;
        s_ramped_base = 0.0f;
        motor_stop();
    } else {
        integralError = 0.0f;
        lastError = 0.0f;
        lastComputeTime = millis();
    }
}

bool pid_is_auto_mode(void) {
    return autoMode;
}

bool pid_calculate_position(float *outPosition, uint8_t *outBlackCount) {
    sensor_read_all(); // Cập nhật đọc giá trị mới nhất từ 16 mắt ADC

    // BƯỚC 1: Phân đoạn các mắt đen thành các cụm liên tục (Clustering)
    // Giúp phân biệt rạch ròi giữa line xe đang bám và line song song lân cận
    #define MAX_LINE_CLUSTERS 4
    SensorCluster_t clusters[MAX_LINE_CLUSTERS];
    uint8_t cluster_cnt = 0;

    bool in_cluster = false;
    uint8_t c_start = 0;

    for (int i = 0; i < N_SENSORS; i++) {
        uint16_t cal = sensor_get_calibrated(i);
        // Một mắt được tính là thuộc cụm vạch đen nếu tín hiệu >= 180
        bool is_black = (cal >= 180);

        if (is_black) {
            if (!in_cluster) {
                in_cluster = true;
                c_start = i;
            }
        } else {
            if (in_cluster) {
                in_cluster = false;
                if (cluster_cnt < MAX_LINE_CLUSTERS) {
                    clusters[cluster_cnt].start_idx = c_start;
                    clusters[cluster_cnt].end_idx = i - 1;
                    cluster_cnt++;
                }
            }
        }
    }
    if (in_cluster && cluster_cnt < MAX_LINE_CLUSTERS) {
        clusters[cluster_cnt].start_idx = c_start;
        clusters[cluster_cnt].end_idx = N_SENSORS - 1;
        cluster_cnt++;
    }

    // BƯỚC 2: Tính toán trọng tâm cho từng cụm và chọn lọc các cụm thực sự hợp lệ
    SensorCluster_t valid_clusters[MAX_LINE_CLUSTERS];
    uint8_t valid_cnt = 0;

    for (uint8_t c = 0; c < cluster_cnt; c++) {
        uint32_t sumCal = 0;
        float sumW = 0.0f;
        uint8_t b_cnt = 0;
        uint16_t max_val = 0;

        for (int i = clusters[c].start_idx; i <= clusters[c].end_idx; i++) {
            uint16_t cal = sensor_get_calibrated(i);
            sumCal += cal;
            sumW += (float)cal * SENSOR_WEIGHTS[i];
            if (cal > max_val) max_val = cal;
            if (cal >= 500) b_cnt++;
        }

        // Tiêu chuẩn một cụm vạch thật: Tổng tín hiệu >= 300 và đỉnh độ đen >= 350
        if (sumCal >= 300 && max_val >= 350) {
            valid_clusters[valid_cnt].start_idx = clusters[c].start_idx;
            valid_clusters[valid_cnt].end_idx = clusters[c].end_idx;
            valid_clusters[valid_cnt].black_count = b_cnt;
            valid_clusters[valid_cnt].sumCalibrated = sumCal;
            valid_clusters[valid_cnt].sumWeights = sumW;
            valid_clusters[valid_cnt].centroid = sumW / (float)sumCal;
            valid_cnt++;
        }
    }

    // Phát hiện có line song song: Có từ 2 cụm vạch thực thụ tách biệt nhau
    s_has_parallel_line = (valid_cnt >= 2);

    // BƯỚC 3: Nếu không tìm thấy cụm vạch hợp lệ nào -> Mất vạch
    if (valid_cnt == 0) {
        *outBlackCount = 0;
        *outPosition = lastKnownPosition;
        return false;
    }

    // BƯỚC 4: Lựa chọn cụm mục tiêu theo chiến lược Bám Dính Liên Tục (Line Continuity Tracking)
    uint8_t chosen_idx = 0;

    if (s_sector_action == 1) {
        // Ép rẽ trái: Luôn chọn cụm vạch nằm lệch sang trái nhất (centroid nhỏ nhất)
        float min_c = valid_clusters[0].centroid;
        for (uint8_t i = 1; i < valid_cnt; i++) {
            if (valid_clusters[i].centroid < min_c) {
                min_c = valid_clusters[i].centroid;
                chosen_idx = i;
            }
        }
    } else if (s_sector_action == 2) {
        // Ép rẽ phải: Luôn chọn cụm vạch nằm lệch sang phải nhất (centroid lớn nhất)
        float max_c = valid_clusters[0].centroid;
        for (uint8_t i = 1; i < valid_cnt; i++) {
            if (valid_clusters[i].centroid > max_c) {
                max_c = valid_clusters[i].centroid;
                chosen_idx = i;
            }
        }
    } else {
        // Bình thường: Chọn cụm vạch nằm gần tọa độ vạch trước đó (lastKnownPosition) nhất!
        // Triệt tiêu hoàn toàn hiện tượng xe bị "hút" sang đường song song bên cạnh
        float min_dist = fabsf(valid_clusters[0].centroid - lastKnownPosition);
        for (uint8_t i = 1; i < valid_cnt; i++) {
            float dist = fabsf(valid_clusters[i].centroid - lastKnownPosition);
            if (dist < min_dist) {
                min_dist = dist;
                chosen_idx = i;
            }
        }
    }

    // Xuất kết quả của cụm vạch hợp lệ duy nhất được chọn
    float pos = valid_clusters[chosen_idx].centroid;
    *outPosition = pos;
    *outBlackCount = valid_clusters[chosen_idx].black_count;

    // Cập nhật bộ nhớ vị trí và hướng vạch
    lastKnownPosition = pos;
    if (pos < -15.0f) {
        lastKnownDirection = -1; // Vạch ở bên trái
    } else if (pos > 15.0f) {
        lastKnownDirection = 1;  // Vạch ở bên phải
    } else {
        lastKnownDirection = 0;  // Vạch ở chính giữa tâm
    }

    return true;
}

LineCase_t pid_detect_case(float position, uint8_t blackCount) {
    if (!calibrationReady) {
        return LINE_CASE_STOPPED;
    }

    // Nếu đang trong trạng thái chốt cua góc vuông -> Giữ nguyên trạng thái này
    if (s_corner_latch == 1) return LINE_CASE_CORNER_LEFT;
    if (s_corner_latch == 2) return LINE_CASE_CORNER_RIGHT;

    // Kiểm tra vạch ngang / ngã tư / vạch đích / Diode (từ 10 mắt đen trở lên)
    uint16_t mask = sensor_get_line_bits();
    uint8_t totalBlack = __builtin_popcount(mask);
    if (totalBlack >= 10) {
        // NẾU MỐC SA BÀN ĐANG LÀ ÉP TRÁI (BẪY DIODE RẼ TRÁI / NGÃ 3 CHỮ T):
        // CẢM BIẾN CHẠM VÀO VẠCH NGANG -> KÍCH HOẠT BẺ VUÔNG TRÁI NGAY!
        if (s_sector_action == 1) {
            s_corner_latch = 1;
            s_corner_latch_time = millis();
            return LINE_CASE_CORNER_LEFT;
        }
        // NẾU MỐC SA BÀN ĐANG LÀ ÉP PHẢI (BẪY DIODE RẼ PHẢI / NGÃ 3 CHỮ T):
        // CẢM BIẾN CHẠM VÀO VẠCH NGANG -> KÍCH HOẠT BẺ VUÔNG PHẢI NGAY!
        if (s_sector_action == 2) {
            s_corner_latch = 2;
            s_corner_latch_time = millis();
            return LINE_CASE_CORNER_RIGHT;
        }

        // Mặc định: Ngã tư chữ thập -> Chạy thẳng vượt qua
        return LINE_CASE_CROSS_LINE;
    }

    // NHẬN DIỆN GÓC VUÔNG 90 ĐỘ CHỦ ĐỘNG (ACTIVE 90° CORNER DETECTION)
    // Áp dụng khi:
    // 1. Tuyệt đối KHÔNG có line song song (s_has_parallel_line == false)
    // 2. Không khóa chạy thẳng (ACTION_LOCK_STRAIGHT) và không phanh đích (ACTION_BRAKE_STOP)
    // 3. Chỉ kích hoạt khi gặp góc vuông 90° thực sự (vạch ngang rộng, tâm không còn vạch chạy thẳng)
    //    Không kích hoạt nhầm tại các góc nhọn/tù của đoạn zíc-zắc răng cưa
    if (!s_has_parallel_line && s_sector_action != 3 && s_sector_action != 4) {
        uint8_t leftOuter = __builtin_popcount(mask & 0x000F);   // Cụm mắt 0,1,2,3 cực trái
        uint8_t rightOuter = __builtin_popcount(mask & 0xF000);  // Cụm mắt 12,13,14,15 cực phải
        uint8_t centerBits = __builtin_popcount(mask & 0x03C0);  // Cụm mắt tâm 6,7,8,9

        // 1. Góc vuông Trái: Cụm mắt ngoài cùng bên trái nhìn thấy vạch ngang rộng (>= 3 mắt),
        // cánh phải không quá 1 mắt nhiễu (rightOuter <= 1), tâm không còn vạch tiến tới (centerBits <= 1)
        // và vạch trước đó xuất phát từ tâm thẳng (fabsf(lastKnownPosition) <= 25.0f), loại trừ hoàn toàn đoạn zíc-zắc
        if (leftOuter >= 3 && rightOuter <= 1 && (mask & 0x0003) != 0 && totalBlack <= 9 && centerBits <= 1 && fabsf(lastKnownPosition) <= 25.0f) {
            s_corner_latch = 1;
            s_corner_latch_time = millis();
            return LINE_CASE_CORNER_LEFT;
        }

        // 2. Góc vuông Phải: Cụm mắt ngoài cùng bên phải nhìn thấy vạch ngang rộng (>= 3 mắt),
        // cánh trái không quá 1 mắt nhiễu (leftOuter <= 1), tâm không còn vạch tiến tới (centerBits <= 1)
        // và vạch trước đó xuất phát từ tâm thẳng (fabsf(lastKnownPosition) <= 25.0f), loại trừ hoàn toàn đoạn zíc-zắc
        if (rightOuter >= 3 && leftOuter <= 1 && (mask & 0xC000) != 0 && totalBlack <= 9 && centerBits <= 1 && fabsf(lastKnownPosition) <= 25.0f) {
            s_corner_latch = 2;
            s_corner_latch_time = millis();
            return LINE_CASE_CORNER_RIGHT;
        }
    }

    // Kiểm tra mất vạch hoàn toàn
    if (blackCount == 0) {
        if (!isLineCurrentlyLost) {
            isLineCurrentlyLost = true;
            lostLineStartTime = millis();
        }

        unsigned long lostElapsed = millis() - lostLineStartTime;

        // Nếu vạch trước đó nằm trong dải trung tâm (|lastKnownPosition| <= 35.0f)
        // và mất vạch trong thời gian lướt nét đứt (<= 350ms) -> Đoạn nét đứt!
        if (fabsf(lastKnownPosition) <= 35.0f && lostElapsed <= DASHED_LINE_TIMEOUT_MS) {
            return LINE_CASE_DASHED_LINE;
        }

        return LINE_CASE_LOST_LINE;
    }

    isLineCurrentlyLost = false;
    return LINE_CASE_NORMAL_PID;
}

void pid_line_follow_step(void) {
    if (!autoMode) {
        return;
    }

    // Nếu sa bàn yêu cầu khóa thẳng phóng qua vạch đứt
    if (s_sector_action == 3) {
        motor_set_speeds(pidConfig.baseSpeed, pidConfig.baseSpeed);
        lastComputeTime = millis();
        lastError = 0.0f;
        integralError = 0.0f;
        return;
    }
    // Nếu sa bàn yêu cầu phanh chốt đích
    if (s_sector_action == 4) {
        motor_stop();
        return;
    }

    // =========================================================================
    // XỬ LÝ CHỐT CUA GÓC VUÔNG 90 ĐỘ (PIVOT SPIN KIỂM SOÁT TỐC ĐỘ, KHÔNG GIẬT LẮC)
    // =========================================================================
    if (s_corner_latch != 0) {
        unsigned long latchElapsed = millis() - s_corner_latch_time;
        uint16_t mask = sensor_get_line_bits();
        // Mắt trung tâm (5, 6, 7, 8, 9, 10) khóa lại line mới
        bool center_locked = ((mask & 0x07E0) != 0);

        if ((latchElapsed >= 35 && center_locked) || latchElapsed > 350) {
            s_corner_latch = 0;
            integralError = 0.0f;
            lastError = 0.0f;
        } else {
            int16_t t_fwd = pidConfig.turnSpeed;
            // Bánh nghịch: Giảm tỷ lệ đảo chiều xuống 20% (hãm nhẹ -26 PWM) để xoay compa cực êm, không trượt lốp
            int16_t t_rev = - (int16_t)(pidConfig.turnSpeed * 20 / 100);
            if (s_corner_latch == 1) {
                // Xoay góc vuông Trái: Bánh trái hãm lùi nhẹ, bánh phải phóng tới
                motor_set_speeds(t_rev, t_fwd);
            } else {
                // Xoay góc vuông Phải: Bánh trái phóng tới, bánh phải hãm lùi nhẹ
                motor_set_speeds(t_fwd, t_rev);
            }
            return;
        }
    }

    float position = 0.0f;
    uint8_t blackCount = 0;
    pid_calculate_position(&position, &blackCount);

    // Xác định trạng thái
    currentCase = pid_detect_case(position, blackCount);

    int16_t t_fwd = pidConfig.turnSpeed;
    int16_t t_rev = - (int16_t)(pidConfig.turnSpeed * 20 / 100);

    // Nếu vừa kích hoạt chốt cua góc vuông trong pid_detect_case -> Thực thi ngay
    if (s_corner_latch != 0) {
        if (s_corner_latch == 1) {
            motor_set_speeds(t_rev, t_fwd);
        } else {
            motor_set_speeds(t_fwd, t_rev);
        }
        return;
    }

    unsigned long now = millis();
    float dt = (float)(now - lastComputeTime) / 1000.0f;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;
    lastComputeTime = now;

    switch (currentCase) {

        // =====================================================================
        // CASE 1: BÁM LINE PID CHÍNH (Đường thẳng, cua mềm, cua gắt, cua chữ S)
        // =====================================================================
        case LINE_CASE_NORMAL_PID: {
            float error = position; // -75.0 đến +75.0 (0.0 là chuẩn giữa tâm)
            float absErr = fabsf(error);

            // 1. Khâu P phi tuyến mềm mại (Non-linear P):
            // Lệch ít: lái êm ru; Lệch nhiều: tăng vừa phải (0.005 thay vì 0.012) để không giật lắc
            float pGain = pidConfig.Kp * (1.0f + 0.005f * absErr);
            float pTerm = pGain * error;

            // 2. Khâu I có Anti-windup (kẹp trong khoảng [-150, 150])
            integralError += error * dt;
            if (integralError > 150.0f) integralError = 150.0f;
            if (integralError < -150.0f) integralError = -150.0f;
            float iTerm = pidConfig.Ki * integralError;

            // 3. Khâu D giảm chấn với bộ lọc thông thấp EMA (triệt tiêu rung rơ vi sai & derivative kick)
            float deltaErr = error - lastError;
            lastError = error;
            float raw_d = pidConfig.Kd * deltaErr;
            s_filtered_d = 0.65f * s_filtered_d + 0.35f * raw_d;
            float dTerm = s_filtered_d;

            // Tổng giá trị bù lái (Correction)
            lastCorrection = pTerm + iTerm + dTerm;

            // 4. Phân phối tốc độ ra 2 bánh:
            int16_t leftSpeed = 0;
            int16_t rightSpeed = 0;

            if (pidConfig.baseSpeed > 0) {
                // 1. KHỞI ĐỘNG MỀM (Soft-Start Ramp-up): Tăng dần tốc độ từ 38 lên baseSpeed trong ~200ms
                if (s_ramped_base < 38.0f) {
                    s_ramped_base = 38.0f;
                }
                if (s_ramped_base < (float)pidConfig.baseSpeed) {
                    s_ramped_base += 1.0f; // Mỗi chu kỳ 2ms tăng 1 PWM
                    if (s_ramped_base > (float)pidConfig.baseSpeed) s_ramped_base = (float)pidConfig.baseSpeed;
                } else if (s_ramped_base > (float)pidConfig.baseSpeed) {
                    s_ramped_base -= 2.0f;
                    if (s_ramped_base < (float)pidConfig.baseSpeed) s_ramped_base = (float)pidConfig.baseSpeed;
                }

                // Giảm tốc ôm cua chủ động (Dynamic Corner Drag)
                int16_t currentBase = (int16_t)s_ramped_base - (int16_t)(absErr * 0.55f);
                if (currentBase < 40) currentBase = 40;

                // Tắt hoàn toàn auto_boost để xe giữ đúng tốc độ, không tự nhồi ga khi ôm cua làm văng xe
                leftSpeed  = currentBase + (int16_t)lastCorrection;
                rightSpeed = currentBase - (int16_t)lastCorrection;

                // Giới hạn trong dải an toàn
                leftSpeed  = clamp_int16(leftSpeed, pidConfig.minSpeed, pidConfig.maxSpeed);
                rightSpeed = clamp_int16(rightSpeed, pidConfig.minSpeed, pidConfig.maxSpeed);
            } else {
                // CHẾ ĐỘ TEST CĂN TÂM TĨNH TRÊN BÀN (baseSpeed = 0):
                // Lệch trái -> bánh quay đưa về phải; Lệch phải -> bánh quay đưa về trái; Chuẩn tâm -> đứng yên!
                leftSpeed  = clamp_int16((int16_t)lastCorrection, -120, 120);
                rightSpeed = clamp_int16(-(int16_t)lastCorrection, -120, 120);
            }

            motor_set_speeds(leftSpeed, rightSpeed);
            break;
        }

        // =====================================================================
        // CASE: GÓC VUÔNG 90 ĐỘ TRÁI / PHẢI
        // =====================================================================
        case LINE_CASE_CORNER_LEFT: {
            motor_set_speeds(t_rev, t_fwd);
            break;
        }

        case LINE_CASE_CORNER_RIGHT: {
            motor_set_speeds(t_fwd, t_rev);
            break;
        }

        // =====================================================================
        // CASE 2: MẤT LINE (Dùng Line Memory xoay mạnh mẽ tìm lại)
        // =====================================================================
        case LINE_CASE_LOST_LINE: {
            unsigned long lostElapsed = millis() - lostLineStartTime;

            // Mất vạch quá 1.5 giây -> Phanh dừng xe an toàn
            if (lostElapsed > 1500) {
                motor_stop();
                break;
            }

            // Xoay tại chỗ theo hướng nhìn thấy vạch lần cuối với lực mạnh
            if (lastKnownDirection < 0 || lastKnownPosition < -5.0f || lastError < -5.0f) {
                motor_set_speeds(t_rev, t_fwd); // Giật xoay mạnh tìm vạch bên trái
            } else if (lastKnownDirection > 0 || lastKnownPosition > 5.0f || lastError > 5.0f) {
                motor_set_speeds(t_fwd, t_rev); // Giật xoay mạnh tìm vạch bên phải
            } else {
                // Nếu hướng = 0 (trước đó đi thẳng), dùng lastKnownPosition để quyết định xoay
                if (lastKnownPosition <= 0.0f) {
                    motor_set_speeds(t_rev, t_fwd);
                } else {
                    motor_set_speeds(t_fwd, t_rev);
                }
            }
            break;
        }

        // =====================================================================
        // CASE 3: VẠCH NGANG / NGÃ TƯ
        // =====================================================================
        case LINE_CASE_CROSS_LINE: {
            motor_set_speeds(pidConfig.baseSpeed, pidConfig.baseSpeed);
            break;
        }

        // =====================================================================
        // CASE 4: ĐOẠN NÉT ĐỨT (DASHED LINE)
        // =====================================================================
        case LINE_CASE_DASHED_LINE: {
            motor_set_speeds(pidConfig.baseSpeed, pidConfig.baseSpeed);
            break;
        }

        // =====================================================================
        // CASE 5: DỪNG XE
        // =====================================================================
        case LINE_CASE_STOPPED:
        default: {
            motor_stop();
            break;
        }
    }
}

LineCase_t pid_get_current_case(void) {
    return currentCase;
}

const char* pid_get_case_name(LineCase_t c) {
    switch (c) {
        case LINE_CASE_NORMAL_PID:   return "NORMAL_PID";
        case LINE_CASE_LOST_LINE:    return "LOST_LINE";
        case LINE_CASE_CROSS_LINE:   return "CROSS_LINE";
        case LINE_CASE_DASHED_LINE:  return "DASHED_LINE";
        case LINE_CASE_CORNER_LEFT:  return "CORNER_LEFT";
        case LINE_CASE_CORNER_RIGHT: return "CORNER_RIGHT";
        case LINE_CASE_STOPPED:      return "STOPPED";
        default: return "UNKNOWN";
    }
}

float pid_get_last_error(void) {
    return lastError;
}

float pid_get_correction(void) {
    return lastCorrection;
}

float pid_get_kp(void) {
    return pidConfig.Kp;
}

float pid_get_ki(void) {
    return pidConfig.Ki;
}

float pid_get_kd(void) {
    return pidConfig.Kd;
}

int16_t pid_get_base_speed(void) {
    return pidConfig.baseSpeed;
}
