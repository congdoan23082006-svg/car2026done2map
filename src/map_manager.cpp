#include "map_manager.h"
#include "ble_handler.h"
#include "pid_controller.h"
#include <Arduino.h>
#include <Preferences.h>

static Preferences prefs;
static MapProfile_t s_maps[NUM_MAPS];
static uint8_t s_selected_map = 0;
static bool s_is_right_lane = false;

void map_load_defaults(void) {
    // -------------------------------------------------------------
    // KHỞI TẠO MẶC ĐỊNH CHO 3 BẢN ĐỒ
    // MAP 1: HÌNH THOI (5 mốc cấu hình thực tế)
    // MAP 2: LÀN TRÁI  (Mặc định ép trái vào nhánh sóng)
    // MAP 3: LÀN PHẢI  (Đối xứng: ép phải vào nhánh sóng)
    // -------------------------------------------------------------
    // 1. MAP 1 (HÌNH THOI)
    s_maps[0].count = 5;
    s_maps[0].wps[0] = { .start_ticks = 0,    .target_speed = 120, .action = ACTION_NORMAL };
    s_maps[0].wps[1] = { .start_ticks = 400,  .target_speed = 110, .action = ACTION_FORCE_LEFT };
    s_maps[0].wps[2] = { .start_ticks = 500,  .target_speed = 110, .action = ACTION_NORMAL };
    s_maps[0].wps[3] = { .start_ticks = 2324, .target_speed = 110, .action = ACTION_FORCE_RIGHT };
    s_maps[0].wps[4] = { .start_ticks = 3324, .target_speed = 110, .action = ACTION_NORMAL };

    // 2. MAP 2 (PHẢI - DIODE: 13 mốc người dùng đã tune thực tế)
    s_maps[1].count = 13;
    s_maps[1].wps[0]  = { .start_ticks = 0,     .target_speed = 80,  .action = ACTION_NORMAL };
    s_maps[1].wps[1]  = { .start_ticks = 420,   .target_speed = 80,  .action = ACTION_FORCE_RIGHT };
    s_maps[1].wps[2]  = { .start_ticks = 1745,  .target_speed = 110, .action = ACTION_NORMAL };
    s_maps[1].wps[3]  = { .start_ticks = 5631,  .target_speed = 70,  .action = ACTION_FORCE_LEFT };
    s_maps[1].wps[4]  = { .start_ticks = 5807,  .target_speed = 80,  .action = ACTION_NORMAL };
    s_maps[1].wps[5]  = { .start_ticks = 6300,  .target_speed = 110, .action = ACTION_FORCE_RIGHT };
    s_maps[1].wps[6]  = { .start_ticks = 6471,  .target_speed = 110, .action = ACTION_FORCE_RIGHT };
    s_maps[1].wps[7]  = { .start_ticks = 6564,  .target_speed = 110, .action = ACTION_FORCE_RIGHT };
    s_maps[1].wps[8]  = { .start_ticks = 6600,  .target_speed = 110, .action = ACTION_FORCE_RIGHT };
    s_maps[1].wps[9]  = { .start_ticks = 6707,  .target_speed = 110, .action = ACTION_FORCE_RIGHT };
    s_maps[1].wps[10] = { .start_ticks = 7000,  .target_speed = 120, .action = ACTION_NORMAL };
    s_maps[1].wps[11] = { .start_ticks = 7852,  .target_speed = 140, .action = ACTION_NORMAL };
    s_maps[1].wps[12] = { .start_ticks = 11000, .target_speed = 200, .action = ACTION_NORMAL };

    // 3. MAP 3 (TRÁI - DIODE: 13 mốc đối xứng của Map 2)
    s_maps[2].count = 13;
    s_maps[2].wps[0]  = { .start_ticks = 0,     .target_speed = 80,  .action = ACTION_NORMAL };
    s_maps[2].wps[1]  = { .start_ticks = 420,   .target_speed = 80,  .action = ACTION_FORCE_LEFT };
    s_maps[2].wps[2]  = { .start_ticks = 1745,  .target_speed = 110, .action = ACTION_NORMAL };
    s_maps[2].wps[3]  = { .start_ticks = 5631,  .target_speed = 70,  .action = ACTION_FORCE_RIGHT };
    s_maps[2].wps[4]  = { .start_ticks = 5807,  .target_speed = 80,  .action = ACTION_NORMAL };
    s_maps[2].wps[5]  = { .start_ticks = 6300,  .target_speed = 110, .action = ACTION_FORCE_LEFT };
    s_maps[2].wps[6]  = { .start_ticks = 6471,  .target_speed = 110, .action = ACTION_FORCE_LEFT };
    s_maps[2].wps[7]  = { .start_ticks = 6564,  .target_speed = 110, .action = ACTION_FORCE_LEFT };
    s_maps[2].wps[8]  = { .start_ticks = 6600,  .target_speed = 110, .action = ACTION_FORCE_LEFT };
    s_maps[2].wps[9]  = { .start_ticks = 6707,  .target_speed = 110, .action = ACTION_FORCE_LEFT };
    s_maps[2].wps[10] = { .start_ticks = 7000,  .target_speed = 120, .action = ACTION_NORMAL };
    s_maps[2].wps[11] = { .start_ticks = 7852,  .target_speed = 140, .action = ACTION_NORMAL };
    s_maps[2].wps[12] = { .start_ticks = 11000, .target_speed = 200, .action = ACTION_NORMAL };
}

void map_manager_init(void) {
    if (!map_load_from_flash()) {
        map_load_defaults();
        map_save_to_flash();
    } else {
        // Nếu Flash chưa có mốc nào hoặc Map 2, Map 3 chưa cập nhật đủ 13 mốc -> Nạp mới và lưu
        if (s_maps[0].count == 0 || s_maps[1].count < 13 || s_maps[2].count < 13) {
            map_load_defaults();
            map_save_to_flash();
        } else {
            for (uint8_t m = 0; m < NUM_MAPS; m++) {
                map_sort_waypoints(m);
            }
        }
    }
}

void map_sort_waypoints(uint8_t map_idx) {
    if (map_idx >= NUM_MAPS) return;
    uint8_t n = s_maps[map_idx].count;
    if (n <= 1) return;
    // Sắp xếp chèn (Insertion Sort) đảm bảo start_ticks luôn tăng dần
    for (uint8_t i = 1; i < n; i++) {
        Waypoint_t key = s_maps[map_idx].wps[i];
        int8_t j = (int8_t)i - 1;
        while (j >= 0 && s_maps[map_idx].wps[j].start_ticks > key.start_ticks) {
            s_maps[map_idx].wps[j + 1] = s_maps[map_idx].wps[j];
            j--;
        }
        s_maps[map_idx].wps[j + 1] = key;
    }
}

void map_save_to_flash(void) {
    for (uint8_t m = 0; m < NUM_MAPS; m++) {
        map_sort_waypoints(m);
    }
    prefs.begin("race_maps_v2", false);
    prefs.putBytes("map_data", (const void*)s_maps, sizeof(s_maps));
    prefs.putUChar("sel_map", s_selected_map);
    prefs.putBool("is_right", s_is_right_lane);
    prefs.end();
}

bool map_load_from_flash(void) {
    prefs.begin("race_maps_v2", true);
    if (!prefs.isKey("map_data")) {
        prefs.end();
        return false;
    }
    size_t len = prefs.getBytes("map_data", (void*)s_maps, sizeof(s_maps));
    if (len != sizeof(s_maps)) {
        prefs.end();
        return false;
    }
    s_selected_map = prefs.getUChar("sel_map", 0);
    if (s_selected_map >= NUM_MAPS) s_selected_map = 0;
    s_is_right_lane = prefs.getBool("is_right", false);
    prefs.end();
    return true;
}

void map_select(uint8_t map_idx) {
    if (map_idx < NUM_MAPS) {
        s_selected_map = map_idx;
    }
}

uint8_t map_get_selected(void) {
    return s_selected_map;
}

void map_set_right_lane(bool is_right) {
    s_is_right_lane = is_right;
}

bool map_is_right_lane(void) {
    return s_is_right_lane;
}

bool map_set_waypoint(uint8_t map_idx, uint8_t wp_idx, uint32_t ticks, int16_t speed, uint8_t action) {
    if (map_idx >= NUM_MAPS || wp_idx >= MAX_WAYPOINTS) return false;

    s_maps[map_idx].wps[wp_idx].start_ticks = ticks;
    s_maps[map_idx].wps[wp_idx].target_speed = speed;
    s_maps[map_idx].wps[wp_idx].action = action;

    if (wp_idx >= s_maps[map_idx].count) {
        s_maps[map_idx].count = wp_idx + 1;
    }
    return true;
}

void map_clear_waypoints(uint8_t map_idx) {
    if (map_idx < NUM_MAPS) {
        s_maps[map_idx].count = 0;
    }
}

uint8_t map_get_waypoint_count(uint8_t map_idx) {
    if (map_idx < NUM_MAPS) return s_maps[map_idx].count;
    return 0;
}

const Waypoint_t* map_get_waypoint(uint8_t map_idx, uint8_t wp_idx) {
    if (map_idx < NUM_MAPS && wp_idx < s_maps[map_idx].count) {
        return &s_maps[map_idx].wps[wp_idx];
    }
    return NULL;
}

void map_get_current_action(uint32_t current_ticks, int16_t *out_speed, uint8_t *out_action, uint8_t *out_wp_idx) {
    MapProfile_t *prof = &s_maps[s_selected_map];

    // 1. Nếu không có mốc nào: Chạy PID thuần với tốc độ đua cài đặt
    if (prof->count == 0) {
        *out_speed = pid_get_base_speed();
        *out_action = ACTION_NORMAL;
        *out_wp_idx = 0;
        return;
    }

    // 2. Nếu xe chưa chạy tới Mốc Bắt Đầu đầu tiên (current_ticks < mốc 0):
    // Xe xuất phát chạy PID bình thường theo tốc độ mốc đầu (hoặc base speed)
    if (current_ticks < prof->wps[0].start_ticks) {
        *out_speed = prof->wps[0].target_speed > 0 ? prof->wps[0].target_speed : pid_get_base_speed();
        *out_action = ACTION_NORMAL;
        *out_wp_idx = 0;
        return;
    }

    // 3. Tìm mốc BẮT ĐẦU đang có hiệu lực (quét ngược từ mốc lớn nhất về mốc 0):
    // Mốc nào có start_ticks <= current_ticks đầu tiên chính là mốc đang thực thi!
    for (int8_t i = (int8_t)(prof->count - 1); i >= 0; i--) {
        if (current_ticks >= prof->wps[i].start_ticks) {
            *out_speed = prof->wps[i].target_speed;
            uint8_t act = prof->wps[i].action;

            // Nếu xe đặt ở Làn Phải (Mirror Track): Đảo ngược hướng ép Trái <-> Phải
            if (s_is_right_lane) {
                if (act == ACTION_FORCE_LEFT) act = ACTION_FORCE_RIGHT;
                else if (act == ACTION_FORCE_RIGHT) act = ACTION_FORCE_LEFT;
            }

            *out_action = act;
            *out_wp_idx = (uint8_t)i;
            return;
        }
    }

    *out_speed = pid_get_base_speed();
    *out_action = ACTION_NORMAL;
    *out_wp_idx = 0;
}
