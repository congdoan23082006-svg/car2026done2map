#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include <stdbool.h>
#include <esp_attr.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Khởi tạo ngoại vi PCNT phần cứng Quadrature 4X và bộ lọc Glitch filter cho 2 bánh
 */
void encoder_init(void);

/**
 * @brief Reset toàn bộ số xung tích lũy và quãng đường về 0
 */
void encoder_reset(void);

/**
 * @brief Cập nhật Odometry định kỳ (ghim vào IRAM để chạy tốc độ tối đa không trễ)
 * @param dt_sec Khoảng thời gian chu kỳ tính bằng giây (ví dụ: 0.010 = 10ms)
 */
void encoder_update(float dt_sec);

/**
 * @brief Lấy tổng số xung tích lũy 64-bit bánh Trái
 */
int64_t encoder_get_ticks_L(void);

/**
 * @brief Lấy tổng số xung tích lũy 64-bit bánh Phải
 */
int64_t encoder_get_ticks_R(void);

/**
 * @brief Lấy quãng đường di chuyển bánh Trái (mm)
 */
float encoder_get_dist_L_mm(void);

/**
 * @brief Lấy quãng đường di chuyển bánh Phải (mm)
 */
float encoder_get_dist_R_mm(void);

/**
 * @brief Lấy quãng đường trung bình xe đã đi (mm)
 */
float encoder_get_dist_avg_mm(void);

/**
 * @brief Lấy vận tốc tức thời bánh Trái (mm/s)
 */
float encoder_get_speed_L_mms(void);

/**
 * @brief Lấy vận tốc tức thời bánh Phải (mm/s)
 */
float encoder_get_speed_R_mms(void);

/**
 * @brief Lấy vận tốc vòng quay bánh Trái (RPM)
 */
float encoder_get_rpm_L(void);

/**
 * @brief Lấy vận tốc vòng quay bánh Phải (RPM)
 */
float encoder_get_rpm_R(void);

/**
 * @brief Cài đặt hệ số số xung trên 1 vòng bánh (PPWR)
 */
void encoder_set_ppr(float ppr_L, float ppr_R);

/**
 * @brief Lấy hệ số xung hiện tại
 */
void encoder_get_ppr(float *ppr_L, float *ppr_R);

/**
 * @brief Đổi chiều đếm xung (1 hoặc -1)
 */
void encoder_set_dir(int8_t dir_L, int8_t dir_R);

/**
 * @brief Lấy chiều đếm xung bánh Trái
 */
int8_t encoder_get_dir_L(void);

/**
 * @brief Lấy chiều đếm xung bánh Phải
 */
int8_t encoder_get_dir_R(void);

#ifdef __cplusplus
}
#endif

#endif // ENCODER_H
