#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// =================== CÁC TRẠNG THÁI LINE (LINE CASES) ===================
typedef enum {
    LINE_CASE_NORMAL_PID = 0,  // Bám line PID trơn tru (đường thẳng, cua mềm, cua gắt)
    LINE_CASE_LOST_LINE,       // Mất vạch (dùng Line Memory xoay tìm lại)
    LINE_CASE_CROSS_LINE,      // Vạch ngang / ngã tư / vạch đích (chạy thẳng vượt qua)
    LINE_CASE_DASHED_LINE,     // Vạch nét đứt (chạy thẳng vượt qua)
    LINE_CASE_CORNER_LEFT,     // Góc vuông 90° Trái (Chốt cua gắt xoay tại chỗ)
    LINE_CASE_CORNER_RIGHT,    // Góc vuông 90° Phải (Chốt cua gắt xoay tại chỗ)
    LINE_CASE_STOPPED          // Dừng xe
} LineCase_t;

// =================== CẤU TRÚC THÔNG SỐ PID ===================
typedef struct {
    float   Kp;
    float   Ki;
    float   Kd;
    int16_t baseSpeed;
    int16_t maxSpeed;
    int16_t minSpeed;
    int16_t turnSpeed;   // Lực bẻ cua 90° / Diode (PWM: 90 - 180)
} PID_Config_t;

// =================== HÀM GIAO TIẾP (API) ===================

/**
 * @brief Khởi tạo thông số bộ điều khiển PID
 */
void pid_init(float kp, float ki, float kd, int16_t baseSpeed);

/**
 * @brief Cập nhật thông số Kp, Ki, Kd
 */
void pid_set_tunings(float kp, float ki, float kd);

/**
 * @brief Cập nhật tốc độ nền baseSpeed (0 - 255)
 */
void pid_set_base_speed(int16_t speed);

/**
 * @brief Cập nhật lực bẻ cua 90° / Diode (90 - 200)
 */
void pid_set_turn_speed(int16_t speed);
int16_t pid_get_turn_speed(void);

/**
 * @brief Bật/Tắt chế độ tự động chạy PID
 */
void pid_set_auto_mode(bool enable);

/**
 * @brief Cập nhật hành động ép kịch bản sa bàn (Normal, Force Left, Force Right, Lock Straight, Stop)
 */
void pid_set_sector_action(uint8_t action);
uint8_t pid_get_sector_action(void);

/**
 * @brief Kiểm tra xem xe có đang ở chế độ tự động không
 */
bool pid_is_auto_mode(void);

/**
 * @brief Tính vị trí tim vạch bằng thuật toán Smooth Centroid (-75.0 đến +75.0)
 * @param outPosition Con trỏ nhận giá trị vị trí (-75.0: lệch cực trái, +75.0: cực phải, 0.0: tâm)
 * @param outBlackCount Con trỏ nhận số lượng mắt nhìn thấy vạch đen
 * @return true nếu nhìn thấy vạch, false nếu mất vạch hoàn toàn
 */
bool pid_calculate_position(float *outPosition, uint8_t *outBlackCount);

/**
 * @brief Nhận diện trạng thái sa bàn (Case)
 */
LineCase_t pid_detect_case(float position, uint8_t blackCount);

/**
 * @brief Thực thi 1 chu kỳ điều khiển PID (tính sai số và xuất xung PWM ra 2 motor)
 */
void pid_line_follow_step(void);

/**
 * @brief Lấy trạng thái hiện tại của xe
 */
LineCase_t pid_get_current_case(void);
const char* pid_get_case_name(LineCase_t c);

/**
 * @brief Lấy các thông số giám sát (Monitoring)
 */
float pid_get_last_error(void);
float pid_get_correction(void);
float pid_get_kp(void);
float pid_get_ki(void);
float pid_get_kd(void);
int16_t pid_get_base_speed(void);

#ifdef __cplusplus
}
#endif

#endif // PID_CONTROLLER_H
