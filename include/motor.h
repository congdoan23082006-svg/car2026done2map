#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Khởi tạo các kênh PWM tần số cao (20kHz) cho 2 động cơ cầu H
 */
void motor_init(void);

/**
 * @brief Dừng cả hai động cơ ngay lập tức và khóa cứng (Active Short Brake)
 */
void motor_stop(void);

/**
 * @brief Thả trôi tự do cả hai động cơ (Coast mode / Freewheel, bánh xe lăn trơn)
 */
void motor_coast(void);

/**
 * @brief Điều khiển tốc độ độc lập cho từng động cơ
 * @param left Tốc độ động cơ trái (-255 đến 255, Dương: Tiến, Âm: Lùi, 0: Phanh)
 * @param right Tốc độ động cơ phải (-255 đến 255, Dương: Tiến, Âm: Lùi, 0: Phanh)
 */
void motor_set_speeds(int16_t left, int16_t right);

/**
 * @brief Điều khiển động cơ trái
 * @param speed Tốc độ (-255 đến 255)
 */
void motor_set_left(int16_t speed);

/**
 * @brief Điều khiển động cơ phải
 * @param speed Tốc độ (-255 đến 255)
 */
void motor_set_right(int16_t speed);

/**
 * @brief Lấy tốc độ PWM hiện tại của động cơ trái
 */
int16_t motor_get_left_speed(void);

/**
 * @brief Lấy tốc độ PWM hiện tại của động cơ phải
 */
int16_t motor_get_right_speed(void);

/**
 * @brief Đổi chiều quay quy ước cho động cơ (1 hoặc -1)
 */
void motor_set_dir(int8_t dir_left, int8_t dir_right);

/**
 * @brief Lấy chiều quay động cơ trái hiện tại
 */
int8_t motor_get_dir_left(void);

/**
 * @brief Lấy chiều quay động cơ phải hiện tại
 */
int8_t motor_get_dir_right(void);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_H
