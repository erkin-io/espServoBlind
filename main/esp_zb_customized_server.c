#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_ota_ops.h"
#include "esp_app_desc.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/temperature_sensor.h"
#include "esp_zigbee_core.h"
#include "ha/esp_zigbee_ha_standard.h"

#define TAG "ESP_SERVO_BLIND"

#define FW_VERSION_STR "\x05""1.1.0"
#define FW_APP_VERSION 2
// OTA file version, one byte each: major.minor.patch.build. Must grow with every release.
#define FW_FILE_VERSION 0x01010000

// -- OTA --
#define OTA_IMAGE_TYPE          0x5342   // "SB": espServoBlind
#define OTA_HW_VERSION          1
#define OTA_MAX_DATA_SIZE       223      // block size Espressif recommends
#define OTA_QUERY_INTERVAL_MIN  (24 * 60)
#define OTA_ELEMENT_HEADER_LEN  6        // sub-element tag id (2) + length (4)
#define OTA_TAG_UPGRADE_IMAGE   0x0000
// New firmware is kept only after it has stayed on the network this long; a crash before
// that restarts into the old firmware
#define OTA_CONFIRM_DELAY_MS    60000
// New firmware that hasn't joined by then rolls back on its own, even if it never crashes
#define OTA_JOIN_TIMEOUT_MS     (10 * 60 * 1000)

// -- HARDWARE CONFIG (ESP32-C6 SuperMini) --
#define BUTTON_PIN GPIO_NUM_9   // BOOT button
#define SERVO_PIN  GPIO_NUM_2
#define LED_PIN    GPIO_NUM_15  // status LED (the WS2812 on GPIO8 is not used)

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          SERVO_PIN
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_14_BIT
#define LEDC_DUTY_MAX           (1 << 14)
#define LEDC_FREQUENCY          50
#define SERVO_PERIOD_US         (1000000 / LEDC_FREQUENCY)

// A servo reads one pulse per period, so the ramp updates once per period
#define MOVE_FRAME_MS           (1000 / LEDC_FREQUENCY)

// Same range the Z2M converter allows, so a bad value written over the air can't slam the servo
#define SERVO_US_LIMIT_MIN      500
#define SERVO_US_LIMIT_MAX      2500
#define MOVE_SPEED_LIMIT_MS     1000

// -- BUTTON --
#define BUTTON_POLL_MS          10
#define BUTTON_DEBOUNCE_MS      30
#define BUTTON_SHORT_MAX_MS     2000   // shorter = Connect; 2-10 s = ignored on purpose
#define BUTTON_RESET_MS         10000  // held this long = Zigbee factory reset on release

// -- NETWORK RETRY (only while not on any network) --
#define RETRY_MIN_MS            10000
#define RETRY_MAX_MS            300000
#define PROBE_TIMEOUT_MS        10000
#define CONNECT_MAX_MS          60000
#define SCAN_DURATION           3      // ((1 << 3) + 1) beacon intervals per channel, ~2.2 s for all 16

// -- ZIGBEE IDENTITY --
#define ENDPOINT 1
#define COORDINATOR_ADDR 0x0000
#define COORDINATOR_ENDPOINT 1
#define ESPRESSIF_MANUFACTURER_CODE 0x131B

#define CUSTOM_CLUSTER_ID 0xFF00
#define ATTR_SERVO_MIN_US_ID     0x0001
#define ATTR_SERVO_MAX_US_ID     0x0002
#define ATTR_MOVE_SPEED_MS_ID    0x0003
#define ATTR_IDLE_TIMEOUT_MS_ID  0x0004
#define ATTR_RESET_REASON_ID     0x0010
#define ATTR_BOOT_COUNT_ID       0x0011
#define ATTR_CHIP_TEMP_ID        0x0012   // int16, 0.01 C, 0x8000 = not known

// -- CHIP TEMPERATURE --
#define CHIP_TEMP_UNKNOWN       INT16_MIN
#define CHIP_TEMP_PERIOD_MS     (5 * 60 * 1000)
#define CHIP_TEMP_RANGE_MIN_C   20    // the sensor's 20..100 C range: this is here to catch heat
#define CHIP_TEMP_RANGE_MAX_C   100

#define NVS_NAMESPACE "storage"
#define ZB_ROLE_ROUTER 1

// The device is DC powered, so it joins as a Router: no parent to lose, Z2M pings it directly,
// it restores its network from flash at boot, and it matches CONFIG_ZB_ZCZR.
#define ESP_ZB_ZR_CONFIG()                                          \
    {                                                               \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER,                   \
        .install_code_policy = false,                               \
        .nwk_cfg.zczr_cfg = {                                       \
            .max_children = 10,                                     \
        },                                                          \
    }

#ifndef ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID
#define ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID 0x0008
#endif
#ifndef ESP_ZB_ZCL_ATTR_WINDOW_COVERING_WINDOW_COVERING_TYPE_ID
#define ESP_ZB_ZCL_ATTR_WINDOW_COVERING_WINDOW_COVERING_TYPE_ID 0x0000
#endif

// -- STATE STRUCTS --
typedef struct {
    uint16_t servo_min_us;
    uint16_t servo_max_us;
    uint16_t move_speed_ms;
    uint16_t idle_timeout_ms;

    uint8_t current_percent;
    uint8_t target_percent;
    bool position_trusted;   // false: unknown where the servo is (first boot, or power lost mid-move)
} blind_config_t;

static blind_config_t s_cfg = {
    .servo_min_us = 1000,
    .servo_max_us = 2000,
    .move_speed_ms = 20,
    .idle_timeout_ms = 1000,

    .current_percent = 50,
    .target_percent = 50,
    .position_trusted = false,
};

typedef enum {
    LED_SIGNAL_NONE,
    LED_SIGNAL_OK,           // 1 long flash: coordinator answered
    LED_SIGNAL_UNREACHABLE,  // 3 slow flashes: coordinator silent, staying on this network
    LED_SIGNAL_SWITCHING,    // fast burst: leaving to join another open network
} led_signal_t;

static SemaphoreHandle_t s_cfg_mutex;
static TaskHandle_t s_move_task;
static volatile bool s_network_joined = false;
static volatile bool s_identify_active = false;
static volatile bool s_zb_started = false;
static volatile bool s_connect_busy = false;
static volatile bool s_button_reset_armed = false;
static volatile led_signal_t s_led_signal = LED_SIGNAL_NONE;

static uint8_t s_reset_reason;
static uint16_t s_boot_count;
static int16_t s_chip_temp = CHIP_TEMP_UNKNOWN;
static temperature_sensor_handle_t s_temp_sensor;

static uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// -- NVS SAVING --
static void save_config_to_nvs(void) {
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
        xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
        nvs_set_u16(handle, "min_us", s_cfg.servo_min_us);
        nvs_set_u16(handle, "max_us", s_cfg.servo_max_us);
        nvs_set_u16(handle, "spd_ms", s_cfg.move_speed_ms);
        nvs_set_u16(handle, "idto_ms", s_cfg.idle_timeout_ms);
        xSemaphoreGive(s_cfg_mutex);
        nvs_commit(handle);
        nvs_close(handle);
    }
}

// "moving" is set before a move and cleared with the final position, so a power cut
// mid-move is detected at the next boot
static void save_motion_state_to_nvs(bool moving, uint8_t pos) {
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
        if (!moving) nvs_set_u8(handle, "pos", pos);
        nvs_set_u8(handle, "moving", moving ? 1 : 0);
        nvs_commit(handle);
        nvs_close(handle);
    }
}

static void load_config_from_nvs(void) {
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
        xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
        nvs_get_u16(handle, "min_us", &s_cfg.servo_min_us);
        nvs_get_u16(handle, "max_us", &s_cfg.servo_max_us);
        nvs_get_u16(handle, "spd_ms", &s_cfg.move_speed_ms);
        nvs_get_u16(handle, "idto_ms", &s_cfg.idle_timeout_ms);
        uint8_t pos, moving = 0;
        nvs_get_u8(handle, "moving", &moving);
        if (nvs_get_u8(handle, "pos", &pos) == ESP_OK && pos <= 100) {
            s_cfg.current_percent = pos;
            s_cfg.target_percent = pos;
            s_cfg.position_trusted = !moving;
        }
        s_cfg.servo_min_us = clamp_u16(s_cfg.servo_min_us, SERVO_US_LIMIT_MIN, SERVO_US_LIMIT_MAX);
        s_cfg.servo_max_us = clamp_u16(s_cfg.servo_max_us, SERVO_US_LIMIT_MIN, SERVO_US_LIMIT_MAX);
        s_cfg.move_speed_ms = clamp_u16(s_cfg.move_speed_ms, 0, MOVE_SPEED_LIMIT_MS);
        xSemaphoreGive(s_cfg_mutex);
        nvs_close(handle);
    }
}

static void update_boot_diagnostics(void) {
    s_reset_reason = (uint8_t)esp_reset_reason();
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
        nvs_get_u16(handle, "boots", &s_boot_count);
        s_boot_count++;
        nvs_set_u16(handle, "boots", s_boot_count);
        nvs_commit(handle);
        nvs_close(handle);
    }
    ESP_LOGI(TAG, "Boot #%u, reset reason %u", s_boot_count, s_reset_reason);
}

// 1.0.0 ran as an End Device. Its saved network can't be reused by a Router, so wipe the
// Zigbee storage once (calibration and position live in NVS and are kept).
static bool needs_role_migration(void) {
    nvs_handle_t handle;
    uint8_t role = 0;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
        nvs_get_u8(handle, "zb_role", &role);
        nvs_close(handle);
    }
    return role != ZB_ROLE_ROUTER;
}

static void save_role(void) {
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
        nvs_set_u8(handle, "zb_role", ZB_ROLE_ROUTER);
        nvs_commit(handle);
        nvs_close(handle);
    }
}

// -- NATIVE IDENTIFY CALLBACK --
static void identify_cb(uint8_t identify_on) {
    ESP_LOGI(TAG, "Identify %s", identify_on ? "started" : "stopped");
    s_identify_active = identify_on;
}

// -- STATUS LED TASK --
static void led_blink(uint32_t on_ms, uint32_t off_ms) {
    gpio_set_level(LED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(on_ms));
    gpio_set_level(LED_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(off_ms));
}

static void led_task(void *pvParameters) {
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    while (1) {
        led_signal_t signal = s_led_signal;
        if (s_button_reset_armed) {
            // Held long enough: solid on until released
            gpio_set_level(LED_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(50));
        } else if (signal != LED_SIGNAL_NONE) {
            s_led_signal = LED_SIGNAL_NONE;
            if (signal == LED_SIGNAL_OK) {
                led_blink(1000, 500);
            } else if (signal == LED_SIGNAL_UNREACHABLE) {
                for (int i = 0; i < 3; i++) led_blink(300, 300);
            } else {
                for (int i = 0; i < 10; i++) led_blink(50, 50);
            }
        } else if (s_connect_busy || s_identify_active) {
            led_blink(100, 100);
        } else if (!s_network_joined) {
            led_blink(100, 1000);
        } else {
            gpio_set_level(LED_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// -- SERVO OUTPUT --
// Position in hundredths of a percent (0..10000), so the ramp is not limited to 1% (~10 us) steps
static void servo_write(int32_t pos_h, uint16_t min_us, uint16_t max_us) {
    // Signed math so min > max (inverted direction) works too
    int32_t us = (int32_t)min_us + (((int32_t)max_us - (int32_t)min_us) * pos_h) / 10000;
    uint32_t duty = ((uint32_t)us * LEDC_DUTY_MAX) / SERVO_PERIOD_US;
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

static void servo_off(void) {
    // Applied at the next period, so the last pulse is never cut short. With no pulses the
    // servo stops holding torque: no buzz, no jitter, no stall current.
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

// The stack asserts (and restarts the chip) when asked to report an attribute that has no entry
// in its reporting table. That table is restored from flash at boot, so after a firmware update
// it can come from the old firmware and miss attributes added since. Make sure the entry exists.
// Must be called with the Zigbee lock held (or from the Zigbee task)
static bool ensure_reporting_entry(uint16_t cluster, uint16_t attr) {
    esp_zb_zcl_attr_location_info_t loc = {
        .endpoint_id = ENDPOINT,
        .cluster_id = cluster,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        .attr_id = attr,
    };
    if (esp_zb_zcl_find_reporting_info(loc)) return true;

    ESP_LOGW(TAG, "No reporting entry for 0x%04x/0x%04x, adding one", cluster, attr);
    // No periodic reports (max interval 0); Z2M can still configure its own
    esp_zb_zcl_reporting_info_t info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = ENDPOINT,
        .cluster_id = cluster,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = attr,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 1,
        .u.send_info.max_interval = 0,
        .u.send_info.def_min_interval = 1,
        .u.send_info.def_max_interval = 0,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    esp_err_t err = esp_zb_zcl_update_reporting_info(&info);
    if (err == ESP_OK && esp_zb_zcl_find_reporting_info(loc)) return true;
    ESP_LOGE(TAG, "Could not add a reporting entry for 0x%04x/0x%04x (%s), not reporting it", cluster, attr,
             esp_err_to_name(err));
    return false;
}

// Must be called with the Zigbee lock held (or from the Zigbee task)
static void send_report_to_coordinator(uint16_t cluster, uint16_t attr) {
    if (!ensure_reporting_entry(cluster, attr)) return;
    // Sent straight to the coordinator, so it works even before Z2M has re-bound the device
    esp_zb_zcl_report_attr_cmd_t report = {
        .zcl_basic_cmd = {
            .dst_addr_u.addr_short = COORDINATOR_ADDR,
            .dst_endpoint = COORDINATOR_ENDPOINT,
            .src_endpoint = ENDPOINT,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = cluster,
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
        .attributeID = attr,
    };
    esp_zb_zcl_report_attr_cmd_req(&report);
}

// Pushes the calibration saved in flash to the cluster and to Z2M, so Z2M always shows the values
// the device really uses (including after clamping), not just what it last tried to write.
// Must be called with the Zigbee lock held (or from the Zigbee task)
static void report_config(void) {
    xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
    uint16_t values[] = {s_cfg.servo_min_us, s_cfg.servo_max_us, s_cfg.move_speed_ms, s_cfg.idle_timeout_ms};
    xSemaphoreGive(s_cfg_mutex);
    const uint16_t ids[] = {ATTR_SERVO_MIN_US_ID, ATTR_SERVO_MAX_US_ID, ATTR_MOVE_SPEED_MS_ID, ATTR_IDLE_TIMEOUT_MS_ID};
    for (int i = 0; i < 4; i++) {
        esp_zb_zcl_set_attribute_val(ENDPOINT, CUSTOM_CLUSTER_ID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ids[i], &values[i], false);
        if (s_network_joined) send_report_to_coordinator(CUSTOM_CLUSTER_ID, ids[i]);
    }
}

// Runs in the Zigbee task shortly after a write, once the stack has stored the written value
static void report_config_cb(uint8_t param) {
    report_config();
}

// Must be called with the Zigbee lock held (or from the Zigbee task)
static void report_diagnostics(void) {
    if (!s_network_joined) return;
    send_report_to_coordinator(CUSTOM_CLUSTER_ID, ATTR_RESET_REASON_ID);
    send_report_to_coordinator(CUSTOM_CLUSTER_ID, ATTR_BOOT_COUNT_ID);
    send_report_to_coordinator(CUSTOM_CLUSTER_ID, ATTR_CHIP_TEMP_ID);
}

// -- CHIP TEMPERATURE --
// Outside the sensor's range this returns false and the last value is kept
static bool chip_temp_read(void) {
    float celsius;
    if (temperature_sensor_get_celsius(s_temp_sensor, &celsius) != ESP_OK) return false;
    s_chip_temp = (int16_t)(celsius * 100);
    ESP_LOGI(TAG, "Chip temperature %.1f C", celsius);
    return true;
}

// Runs before the Zigbee task starts, so the first reading becomes the attribute's starting value
static void chip_temp_init(void) {
    temperature_sensor_config_t cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(CHIP_TEMP_RANGE_MIN_C, CHIP_TEMP_RANGE_MAX_C);
    esp_err_t err = temperature_sensor_install(&cfg, &s_temp_sensor);
    if (err == ESP_OK) err = temperature_sensor_enable(s_temp_sensor);
    if (err != ESP_OK) {
        // Only a diagnostic: the cover works without it
        ESP_LOGE(TAG, "Temperature sensor not available: %s", esp_err_to_name(err));
        s_temp_sensor = NULL;
        return;
    }
    chip_temp_read();
}

static void chip_temp_task(void *pvParameters) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(CHIP_TEMP_PERIOD_MS));
        if (!s_zb_started || !chip_temp_read()) continue;
        esp_zb_lock_acquire(portMAX_DELAY);
        esp_zb_zcl_set_attribute_val(ENDPOINT, CUSTOM_CLUSTER_ID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_CHIP_TEMP_ID,
                                     &s_chip_temp, false);
        if (s_network_joined) send_report_to_coordinator(CUSTOM_CLUSTER_ID, ATTR_CHIP_TEMP_ID);
        esp_zb_lock_release();
    }
}

// Must be called with the Zigbee lock held (or from the Zigbee task)
static void report_position_locked(uint8_t pos, bool force_report) {
    esp_zb_zcl_set_attribute_val(ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID, &pos, false);
    // An unchanged value doesn't trigger a report on its own, so send one explicitly:
    // the hub always gets a confirmation, even when the same percentage is re-sent.
    if (force_report && s_network_joined) {
        send_report_to_coordinator(ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING,
                                   ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID);
    }
}

static void report_position(uint8_t pos, bool force_report) {
    esp_zb_lock_acquire(portMAX_DELAY);
    report_position_locked(pos, force_report);
    esp_zb_lock_release();
}

static uint8_t current_position(void) {
    xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
    uint8_t cur = s_cfg.current_percent;
    xSemaphoreGive(s_cfg_mutex);
    return cur;
}

// Every command (re)drives the servo, even when the target equals the remembered position,
// so a servo that was pushed by hand or lost power mid-move is put back where it belongs.
static void request_move(uint8_t target) {
    if (target > 100) target = 100;
    xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
    s_cfg.target_percent = target;
    xSemaphoreGive(s_cfg_mutex);
    xTaskNotifyGive(s_move_task);
}

static void request_stop(void) {
    // Read and set under one lock so the servo doesn't step back after stopping
    xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
    s_cfg.target_percent = s_cfg.current_percent;
    xSemaphoreGive(s_cfg_mutex);
    xTaskNotifyGive(s_move_task);
}

// -- FREERTOS MOVEMENT TASK --
static void move_task(void *pvParameters) {
    bool pwm_on = false;

    xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
    int32_t pos_h = s_cfg.current_percent * 100;
    uint8_t saved_pos = s_cfg.current_percent;
    // NVS doesn't hold a clean final position yet (first boot or power lost mid-move)
    bool nvs_dirty = !s_cfg.position_trusted;
    xSemaphoreGive(s_cfg_mutex);

    while (1) {
        xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
        uint16_t idle_to = s_cfg.idle_timeout_ms;
        xSemaphoreGive(s_cfg_mutex);

        // Sleep until a command arrives. While the servo is powered, wake after the idle
        // timeout to cut the PWM (0 = keep holding forever).
        TickType_t wait = (pwm_on && idle_to > 0) ? pdMS_TO_TICKS(idle_to) : portMAX_DELAY;
        if (ulTaskNotifyTake(pdTRUE, wait) == 0) {
            servo_off();
            pwm_on = false;
            ESP_LOGI(TAG, "Idle: PWM off");
            continue;
        }

        xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
        int32_t tgt_h = s_cfg.target_percent * 100;
        if (!s_cfg.position_trusted) {
            // Unknown start: go straight to the target at the servo's own speed. A ramp from a
            // stale position would first jump there and could swing through an unwanted spot.
            pos_h = tgt_h;
            s_cfg.position_trusted = true;
        }
        xSemaphoreGive(s_cfg_mutex);

        if (pos_h != tgt_h) {
            save_motion_state_to_nvs(true, 0);
            nvs_dirty = true;
        }

        int32_t start_h = pos_h;
        int32_t ramp_tgt_h = tgt_h;
        int64_t t0 = esp_timer_get_time();
        while (1) {
            xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
            tgt_h = s_cfg.target_percent * 100;
            uint16_t spd = s_cfg.move_speed_ms;
            uint16_t min_us = s_cfg.servo_min_us;
            uint16_t max_us = s_cfg.servo_max_us;
            xSemaphoreGive(s_cfg_mutex);

            if (tgt_h != ramp_tgt_h) {
                // New target mid-move: continue from where we are
                start_h = pos_h;
                ramp_tgt_h = tgt_h;
                t0 = esp_timer_get_time();
            }

            int32_t prev_pct = (pos_h + 50) / 100;
            if (spd == 0) {
                pos_h = tgt_h;
            } else {
                // move_speed_ms per 1% = 100 hundredths, independent of the RTOS tick
                int64_t travelled = ((esp_timer_get_time() - t0) / 10) / spd;
                int32_t dist = tgt_h > start_h ? tgt_h - start_h : start_h - tgt_h;
                if (travelled >= dist) pos_h = tgt_h;
                else pos_h = tgt_h > start_h ? start_h + (int32_t)travelled : start_h - (int32_t)travelled;
            }
            int32_t pct = (pos_h + 50) / 100;

            servo_write(pos_h, min_us, max_us);
            pwm_on = true;

            xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
            s_cfg.current_percent = (uint8_t)pct;
            xSemaphoreGive(s_cfg_mutex);

            if (pos_h == tgt_h) break;
            // Progress every 5% (a fast ramp can cover several percent per frame)
            if (pct / 5 != prev_pct / 5) report_position((uint8_t)pct, false);
            vTaskDelay(pdMS_TO_TICKS(MOVE_FRAME_MS));
        }

        uint8_t cur = current_position();
        report_position(cur, true);
        if (nvs_dirty || cur != saved_pos) {
            save_motion_state_to_nvs(false, cur);
            saved_pos = cur;
            nvs_dirty = false;
        }
        ESP_LOGI(TAG, "At %d%%", cur);
    }
}

// -- ZIGBEE OTA CLIENT --
// Z2M is the OTA server. The image is written into the spare app slot and only booted when it is
// complete, valid and built from this project. With rollback enabled in the bootloader, the new
// image must rejoin the network before it is kept; a crash or restart before that goes back.
static const esp_partition_t *s_ota_partition = NULL;
static esp_ota_handle_t s_ota_handle = 0;
static bool s_ota_active = false;
static bool s_ota_tag_received = false;
static uint16_t s_ota_tag_id = 0;
static uint32_t s_ota_offset = 0;
static uint32_t s_ota_total = 0;
static uint8_t s_ota_last_decile = 0;
static int64_t s_ota_start_us = 0;

static void ota_reset(void) {
    if (s_ota_active) esp_ota_abort(s_ota_handle);
    s_ota_active = false;
    s_ota_tag_received = false;
    s_ota_offset = 0;
    s_ota_total = 0;
    s_ota_last_decile = 0;
}

// The first block starts with the OTA sub-element header (tag id + length); only the upgrade
// image element is accepted
static esp_err_t ota_element_data(uint32_t total_size, const uint8_t *payload, uint16_t payload_size,
                                  const uint8_t **outbuf, uint16_t *outlen) {
    if (!s_ota_tag_received) {
        if (!payload || payload_size <= OTA_ELEMENT_HEADER_LEN) {
            ESP_LOGE(TAG, "OTA: invalid element header");
            return ESP_ERR_INVALID_ARG;
        }
        uint32_t length;
        memcpy(&s_ota_tag_id, payload, sizeof(s_ota_tag_id));
        memcpy(&length, payload + sizeof(s_ota_tag_id), sizeof(length));
        if (length + OTA_ELEMENT_HEADER_LEN != total_size) {
            ESP_LOGE(TAG, "OTA: element length %lu does not match image size %lu", (unsigned long)length,
                     (unsigned long)total_size);
            return ESP_ERR_INVALID_ARG;
        }
        s_ota_tag_received = true;
        payload += OTA_ELEMENT_HEADER_LEN;
        payload_size -= OTA_ELEMENT_HEADER_LEN;
    }
    if (s_ota_tag_id != OTA_TAG_UPGRADE_IMAGE) {
        ESP_LOGE(TAG, "OTA: unsupported element tag 0x%04x", s_ota_tag_id);
        return ESP_ERR_INVALID_ARG;
    }
    *outbuf = payload;
    *outlen = payload_size;
    return ESP_OK;
}

static esp_err_t ota_finish(void) {
    esp_err_t err = esp_ota_end(s_ota_handle);  // verifies the image checksum and hash
    s_ota_active = false;
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA: image is not valid: %s", esp_err_to_name(err));
        return err;
    }
    // Refuse anything that isn't this firmware, even if it carries our manufacturer code and type
    esp_app_desc_t desc;
    err = esp_ota_get_partition_description(s_ota_partition, &desc);
    if (err != ESP_OK || strncmp(desc.project_name, esp_app_get_description()->project_name, sizeof(desc.project_name)) != 0) {
        ESP_LOGE(TAG, "OTA: image is not %s, refusing it", esp_app_get_description()->project_name);
        return ESP_ERR_INVALID_VERSION;
    }
    err = esp_ota_set_boot_partition(s_ota_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA: could not select the new image: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGW(TAG, "OTA: version %s received in %lld s, restarting into it", desc.version,
             (esp_timer_get_time() - s_ota_start_us) / 1000000);
    servo_off();
    esp_restart();
    return ESP_OK;
}

static esp_err_t ota_status_handler(const esp_zb_zcl_ota_upgrade_value_message_t *msg) {
    if (msg->info.status != ESP_ZB_ZCL_STATUS_SUCCESS) return ESP_OK;

    esp_err_t err = ESP_OK;
    switch (msg->upgrade_status) {
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_START:
            ota_reset();
            s_ota_partition = esp_ota_get_next_update_partition(NULL);
            if (!s_ota_partition) {
                ESP_LOGE(TAG, "OTA: no spare app slot");
                return ESP_FAIL;
            }
            // Erase sector by sector while writing, so the Zigbee task is never blocked for long
            err = esp_ota_begin(s_ota_partition, OTA_WITH_SEQUENTIAL_WRITES, &s_ota_handle);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "OTA: begin failed: %s", esp_err_to_name(err));
                return err;
            }
            s_ota_active = true;
            s_ota_start_us = esp_timer_get_time();
            ESP_LOGW(TAG, "OTA: download started into %s", s_ota_partition->label);
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_RECEIVE:
            if (!s_ota_active) return ESP_FAIL;
            s_ota_total = msg->ota_header.image_size;
            s_ota_offset += msg->payload_size;
            if (msg->payload_size && msg->payload) {
                const uint8_t *data = NULL;
                uint16_t len = 0;
                err = ota_element_data(s_ota_total, msg->payload, msg->payload_size, &data, &len);
                if (err == ESP_OK) err = esp_ota_write(s_ota_handle, data, len);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "OTA: write failed: %s", esp_err_to_name(err));
                    ota_reset();
                    return err;
                }
            }
            if (s_ota_total) {
                uint8_t decile = (uint8_t)((uint64_t)s_ota_offset * 10 / s_ota_total);
                if (decile != s_ota_last_decile) {
                    s_ota_last_decile = decile;
                    ESP_LOGI(TAG, "OTA: %u%%", decile * 10);
                }
            }
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_APPLY:
            ESP_LOGI(TAG, "OTA: apply");
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_CHECK:
            err = (s_ota_active && s_ota_offset == s_ota_total) ? ESP_OK : ESP_FAIL;
            ESP_LOGI(TAG, "OTA: download check %s (%lu/%lu bytes)", err == ESP_OK ? "passed" : "failed",
                     (unsigned long)s_ota_offset, (unsigned long)s_ota_total);
            if (err != ESP_OK) ota_reset();
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_FINISH:
            if (!s_ota_active) return ESP_FAIL;
            err = ota_finish();
            if (err != ESP_OK) ota_reset();
            break;
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_ABORT:
        case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_ERROR:
            ESP_LOGW(TAG, "OTA: aborted (status %d)", msg->upgrade_status);
            ota_reset();
            break;
        default:
            ESP_LOGI(TAG, "OTA: status %d", msg->upgrade_status);
            break;
    }
    return err;
}

static esp_err_t ota_query_image_resp_handler(const esp_zb_zcl_ota_upgrade_query_image_resp_message_t *msg) {
    if (msg->info.status != ESP_ZB_ZCL_STATUS_SUCCESS) return ESP_OK;
    // No image on offer arrives as an all-zero "success". Declining it is the normal case, not an error.
    if (msg->image_size == 0) {
        ESP_LOGI(TAG, "OTA: no update on offer");
        return ESP_ERR_INVALID_ARG;
    }
    ESP_LOGW(TAG, "OTA: server offers version 0x%08lx (%lu bytes), running 0x%08lx", (unsigned long)msg->file_version,
             (unsigned long)msg->image_size, (unsigned long)FW_FILE_VERSION);
    if (msg->manufacturer_code != ESPRESSIF_MANUFACTURER_CODE || msg->image_type != OTA_IMAGE_TYPE) {
        ESP_LOGE(TAG, "OTA: image is for another product (0x%04x/0x%04x), rejecting", msg->manufacturer_code,
                 msg->image_type);
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

// A new image runs in "pending verify" until it proves it works: it must rejoin the network and
// then stay up for OTA_CONFIRM_DELAY_MS. Any restart before that boots the old image again, and
// an image that never joins rolls itself back after OTA_JOIN_TIMEOUT_MS.
static esp_timer_handle_t s_ota_join_timer;

static bool ota_running_image_pending(void) {
    esp_ota_img_states_t state;
    return esp_ota_get_state_partition(esp_ota_get_running_partition(), &state) == ESP_OK &&
           state == ESP_OTA_IMG_PENDING_VERIFY;
}

static void ota_join_timeout_cb(void *arg) {
    if (!ota_running_image_pending()) return;
    ESP_LOGE(TAG, "OTA: new firmware did not prove itself in %d min, going back to the old one",
             OTA_JOIN_TIMEOUT_MS / 60000);
    servo_off();
    esp_ota_mark_app_invalid_rollback_and_reboot();
}

// Called at boot, before the Zigbee stack starts
static void ota_start_join_watchdog(void) {
    if (!ota_running_image_pending()) return;
    ESP_LOGW(TAG, "OTA: running new firmware on trial, kept only once it has been on the network for %d s",
             OTA_CONFIRM_DELAY_MS / 1000);
    const esp_timer_create_args_t args = {.callback = ota_join_timeout_cb, .name = "ota_join"};
    if (esp_timer_create(&args, &s_ota_join_timer) == ESP_OK) {
        esp_timer_start_once(s_ota_join_timer, (uint64_t)OTA_JOIN_TIMEOUT_MS * 1000);
    }
}

// Runs in the Zigbee task, OTA_CONFIRM_DELAY_MS after the network came up
static void ota_confirm_cb(uint8_t param) {
    if (!s_network_joined || !ota_running_image_pending()) return;
    if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) {
        if (s_ota_join_timer) esp_timer_stop(s_ota_join_timer);
        ESP_LOGW(TAG, "OTA: new firmware confirmed, rollback cancelled");
    }
}

static void ota_schedule_confirm(void) {
    if (!ota_running_image_pending()) return;
    esp_zb_scheduler_alarm_cancel(ota_confirm_cb, 0);
    esp_zb_scheduler_alarm(ota_confirm_cb, 0, OTA_CONFIRM_DELAY_MS);
}

static void ota_server_found_cb(esp_zb_zdp_status_t zdo_status, uint16_t addr, uint8_t endpoint, void *user_ctx) {
    if (zdo_status != ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "OTA: no OTA server found");
        return;
    }
    // Ask once now and then daily; Z2M answers but only installs when you press Update
    esp_zb_ota_upgrade_client_query_interval_set(ENDPOINT, OTA_QUERY_INTERVAL_MIN);
    esp_zb_ota_upgrade_client_query_image_req(addr, endpoint);
    ESP_LOGI(TAG, "OTA: server at 0x%04hx endpoint %d", addr, endpoint);
}

static void ota_discover_server(void) {
    static uint16_t cluster_list[] = {ESP_ZB_ZCL_CLUSTER_ID_OTA_UPGRADE};
    esp_zb_zdo_match_desc_req_param_t req = {
        .dst_nwk_addr = COORDINATOR_ADDR,
        .addr_of_interest = COORDINATOR_ADDR,
        .profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .num_in_clusters = 1,
        .num_out_clusters = 0,
        .cluster_list = cluster_list,
    };
    esp_zb_zdo_match_cluster(&req, ota_server_found_cb, NULL);
}

// -- ZIGBEE CALLBACK HANDLERS --
static esp_err_t cb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message) {
    if (callback_id == ESP_ZB_CORE_OTA_UPGRADE_VALUE_CB_ID) {
        return message ? ota_status_handler((const esp_zb_zcl_ota_upgrade_value_message_t *)message) : ESP_FAIL;
    }
    if (callback_id == ESP_ZB_CORE_OTA_UPGRADE_QUERY_IMAGE_RESP_CB_ID) {
        return message ? ota_query_image_resp_handler((const esp_zb_zcl_ota_upgrade_query_image_resp_message_t *)message) : ESP_FAIL;
    }

    if (callback_id == ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID) {
        const esp_zb_zcl_set_attr_value_message_t *msg = (const esp_zb_zcl_set_attr_value_message_t *)message;
        if (!msg || msg->info.dst_endpoint != ENDPOINT || msg->info.cluster != CUSTOM_CLUSTER_ID) return ESP_OK;
        if (!msg->attribute.data.value || msg->attribute.data.type != ESP_ZB_ZCL_ATTR_TYPE_U16) {
            return ESP_ERR_INVALID_ARG;
        }

        uint16_t value = *(uint16_t *)msg->attribute.data.value;
        ESP_LOGI(TAG, "Config attr 0x%04x = %u", msg->attribute.id, value);

        // New calibration is only used from the next command on: changing it never moves the servo
        xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
        switch (msg->attribute.id) {
            case ATTR_SERVO_MIN_US_ID:    s_cfg.servo_min_us = clamp_u16(value, SERVO_US_LIMIT_MIN, SERVO_US_LIMIT_MAX); break;
            case ATTR_SERVO_MAX_US_ID:    s_cfg.servo_max_us = clamp_u16(value, SERVO_US_LIMIT_MIN, SERVO_US_LIMIT_MAX); break;
            case ATTR_MOVE_SPEED_MS_ID:   s_cfg.move_speed_ms = clamp_u16(value, 0, MOVE_SPEED_LIMIT_MS); break;
            case ATTR_IDLE_TIMEOUT_MS_ID: s_cfg.idle_timeout_ms = value; break;
            default: break;
        }
        xSemaphoreGive(s_cfg_mutex);
        save_config_to_nvs();
        // Echo what was actually saved (a value out of range is clamped) back to Z2M
        esp_zb_scheduler_alarm_cancel(report_config_cb, 0);
        esp_zb_scheduler_alarm(report_config_cb, 0, 200);
    } else if (callback_id == ESP_ZB_CORE_WINDOW_COVERING_MOVEMENT_CB_ID) {
        const esp_zb_zcl_window_covering_movement_message_t *msg = (const esp_zb_zcl_window_covering_movement_message_t *)message;
        if (!msg) return ESP_OK;

        ESP_LOGI(TAG, "Window covering cmd 0x%02x", msg->command);
        switch (msg->command) {
            case ESP_ZB_ZCL_CMD_WINDOW_COVERING_UP_OPEN:    request_move(0); break;
            case ESP_ZB_ZCL_CMD_WINDOW_COVERING_DOWN_CLOSE: request_move(100); break;
            case ESP_ZB_ZCL_CMD_WINDOW_COVERING_STOP:       request_stop(); break;
            case ESP_ZB_ZCL_CMD_WINDOW_COVERING_GO_TO_LIFT_PERCENTAGE:
                request_move(msg->payload.percentage_lift_value);
                break;
            default: break;
        }
    }
    return ESP_OK;
}

// -- NETWORK COMMISSIONING --
static uint32_t s_retry_delay_ms = RETRY_MIN_MS;

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask) {
    esp_err_t err = esp_zb_bdb_start_top_level_commissioning(mode_mask);
    if (err != ESP_OK) ESP_LOGW(TAG, "Commissioning (mode 0x%x) not started: %s", mode_mask, esp_err_to_name(err));
}

static void cancel_commissioning_retries(void) {
    esp_zb_scheduler_alarm_cancel((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_INITIALIZATION);
    esp_zb_scheduler_alarm_cancel((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING);
}

// Never give up on the network, but don't hammer the air either: 10 s doubling up to 5 min
static void schedule_commissioning_retry(uint8_t mode) {
    ESP_LOGW(TAG, "Retrying commissioning in %lu s", (unsigned long)(s_retry_delay_ms / 1000));
    cancel_commissioning_retries();
    esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, mode, s_retry_delay_ms);
    s_retry_delay_ms *= 2;
    if (s_retry_delay_ms > RETRY_MAX_MS) s_retry_delay_ms = RETRY_MAX_MS;
}

static void on_network_up(void) {
    s_network_joined = true;
    s_retry_delay_ms = RETRY_MIN_MS;
    // Not confirmed here: if a report below crashed a freshly confirmed image, it would restart
    // forever with no way back. Confirm only after it has stayed up for a while.
    ota_schedule_confirm();
    report_position_locked(current_position(), true);
    report_config();
    report_diagnostics();
    ota_discover_server();
}

// -- CONNECT BUTTON (runs in the Zigbee task) --
// One press, one meaning: join the network that's inviting me; otherwise keep the one I have
// and show I'm alive. A working network is never left.
static bool s_probe_pending = false;

static void connect_done(led_signal_t signal) {
    s_led_signal = signal;
    s_connect_busy = false;
}

static void factory_reset_cb(uint8_t param) {
    // Wipes the Zigbee network only (calibration and position live in NVS) and restarts
    esp_zb_factory_reset();
}

static void scan_complete_cb(esp_zb_zdp_status_t zdo_status, uint8_t count, esp_zb_network_descriptor_t *nwk_descriptor) {
    esp_zb_ieee_addr_t own_ext_pan;
    esp_zb_get_extended_pan_id(own_ext_pan);

    bool other_open = false;
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS && nwk_descriptor) {
        for (uint8_t i = 0; i < count; i++) {
            const esp_zb_network_descriptor_t *nwk = &nwk_descriptor[i];
            if (nwk->permit_joining && nwk->router_capacity &&
                memcmp(nwk->extended_pan_id, own_ext_pan, sizeof(own_ext_pan)) != 0) {
                ESP_LOGI(TAG, "Open network found: PAN 0x%04hx, channel %d", nwk->short_pan_id, nwk->logic_channel);
                other_open = true;
                break;
            }
        }
    }

    if (other_open) {
        // Our coordinator is gone and another network invites us: move there
        ESP_LOGW(TAG, "Coordinator unreachable, joining the open network");
        s_led_signal = LED_SIGNAL_SWITCHING;
        esp_zb_scheduler_alarm(factory_reset_cb, 0, 1500);
    } else {
        ESP_LOGW(TAG, "Coordinator unreachable and no other open network, staying");
        connect_done(LED_SIGNAL_UNREACHABLE);
    }
}

static void connect_probe_failed(void) {
    // Only look elsewhere when our own coordinator doesn't answer, so a healthy device never
    // jumps to a neighbour's network that happens to be open for pairing
    esp_zb_zdo_active_scan_request(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK, SCAN_DURATION, scan_complete_cb);
}

static void probe_timeout_cb(uint8_t param) {
    if (!s_probe_pending) return;
    s_probe_pending = false;
    connect_probe_failed();
}

static void probe_rsp_cb(esp_zb_zdp_status_t zdo_status, esp_zb_zdo_ieee_addr_rsp_t *resp, void *user_ctx) {
    if (!s_probe_pending) return;
    s_probe_pending = false;
    esp_zb_scheduler_alarm_cancel(probe_timeout_cb, 0);
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Coordinator answered");
        connect_done(LED_SIGNAL_OK);
    } else {
        connect_probe_failed();
    }
}

static void connect_cb(uint8_t param) {
    if (!esp_zb_bdb_dev_joined()) {
        ESP_LOGI(TAG, "Connect: not on a network, searching now");
        cancel_commissioning_retries();
        s_retry_delay_ms = RETRY_MIN_MS;
        bdb_start_top_level_commissioning_cb(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        connect_done(LED_SIGNAL_NONE);
        return;
    }

    ESP_LOGI(TAG, "Connect: announcing and checking the coordinator");
    // Z2M marks the device online on any message, and re-adds a deleted device on announce
    esp_zb_zdo_device_announcement_req();
    report_position_locked(current_position(), true);
    report_diagnostics();

    esp_zb_zdo_ieee_addr_req_param_t req = {
        .dst_nwk_addr = COORDINATOR_ADDR,
        .addr_of_interest = COORDINATOR_ADDR,
        .request_type = 0,
        .start_index = 0,
    };
    s_probe_pending = true;
    esp_zb_scheduler_alarm(probe_timeout_cb, 0, PROBE_TIMEOUT_MS);
    esp_zb_zdo_ieee_addr_req(&req, probe_rsp_cb, NULL);
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type) {
        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
            bdb_start_top_level_commissioning_cb(ESP_ZB_BDB_MODE_INITIALIZATION);
            break;
        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
            if (err_status == ESP_OK) {
                if (esp_zb_bdb_is_factory_new()) {
                    ESP_LOGI(TAG, "Factory new, searching for a network");
                    bdb_start_top_level_commissioning_cb(ESP_ZB_BDB_MODE_NETWORK_STEERING);
                } else {
                    ESP_LOGI(TAG, "Back on the saved network");
                    on_network_up();
                }
            } else {
                s_network_joined = false;
                schedule_commissioning_retry(ESP_ZB_BDB_MODE_INITIALIZATION);
            }
            break;
        case ESP_ZB_BDB_SIGNAL_STEERING:
            if (err_status == ESP_OK) {
                ESP_LOGI(TAG, "Joined network (PAN 0x%04hx, channel %d)", esp_zb_get_pan_id(), esp_zb_get_current_channel());
                on_network_up();
            } else if (!esp_zb_bdb_dev_joined()) {
                s_network_joined = false;
                schedule_commissioning_retry(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            }
            break;
        case ESP_ZB_ZDO_SIGNAL_LEAVE: {
            esp_zb_zdo_signal_leave_params_t *leave = (esp_zb_zdo_signal_leave_params_t *)esp_zb_app_signal_get_params(p_sg_p);
            if (leave && leave->leave_type == ESP_ZB_NWK_LEAVE_TYPE_RESET) {
                ESP_LOGW(TAG, "Removed from the network, searching for one");
                s_network_joined = false;
                s_retry_delay_ms = RETRY_MIN_MS;
                schedule_commissioning_retry(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            }
            break;
        }
        default:
            ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                     esp_err_to_name(err_status));
            break;
    }
}

// -- BUTTON MONITORING TASK --
static void button_task(void *pvParameters) {
    gpio_reset_pin(BUTTON_PIN);
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_PIN, GPIO_PULLUP_ONLY);

    TickType_t connect_started = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
        // Safety net: a check that never finishes must not lock the button out
        if (s_connect_busy && xTaskGetTickCount() - connect_started > pdMS_TO_TICKS(CONNECT_MAX_MS)) {
            ESP_LOGW(TAG, "Connect check timed out");
            s_connect_busy = false;
        }
        if (gpio_get_level(BUTTON_PIN) != 0) continue;

        // Debounce: a glitch shorter than this is not a press
        uint32_t held_ms = 0;
        while (gpio_get_level(BUTTON_PIN) == 0) {
            vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
            held_ms += BUTTON_POLL_MS;
            if (held_ms >= BUTTON_RESET_MS) s_button_reset_armed = true;
        }
        s_button_reset_armed = false;
        if (held_ms < BUTTON_DEBOUNCE_MS || !s_zb_started) continue;

        if (held_ms >= BUTTON_RESET_MS) {
            ESP_LOGW(TAG, "Button: Zigbee factory reset (calibration kept)");
            esp_zb_lock_acquire(portMAX_DELAY);
            esp_zb_factory_reset();
            esp_zb_lock_release();
        } else if (held_ms < BUTTON_SHORT_MAX_MS && !s_connect_busy) {
            s_connect_busy = true;
            connect_started = xTaskGetTickCount();
            esp_zb_lock_acquire(portMAX_DELAY);
            esp_zb_scheduler_alarm(connect_cb, 0, 0);
            esp_zb_lock_release();
        }
    }
}

// -- ZIGBEE INITIALIZATION TASK --
static void esp_zb_task(void *pvParameters) {
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZR_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    bool migrate = needs_role_migration();
    if (migrate) {
        ESP_LOGW(TAG, "First start as Router: clearing the old Zigbee network data");
        esp_zb_nvram_erase_at_start(true);
    }

    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();
    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();

    // Core Basic Setup
    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
    uint8_t zcl_ver = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE, app_ver = FW_APP_VERSION, stack_ver = 1, hw_ver = 1, pwr_src = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE;

    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &zcl_ver);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_APPLICATION_VERSION_ID, &app_ver);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_STACK_VERSION_ID, &stack_ver);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_HW_VERSION_ID, &hw_ver);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &pwr_src);

    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *)"\x08""erkin.io");
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (void *)"\x0D""espServoBlind");
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_PRODUCT_URL_ID, (void *)"\x29""https://github.com/erkin-io/espServoBlind");

    // Populates "Firmware Version" on the Zigbee2MQTT dashboard instead of Unknown
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_DATE_CODE_ID, (void *)FW_VERSION_STR);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_SW_BUILD_ID, (void *)FW_VERSION_STR);

    esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // Core Identify Setup
    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
    uint16_t identify_time = 0;
    esp_zb_identify_cluster_add_attr(esp_zb_identify_cluster, ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID, &identify_time);
    esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
    uint8_t current_lift = s_cfg.current_percent;
    uint16_t v1 = s_cfg.servo_min_us, v2 = s_cfg.servo_max_us, v3 = s_cfg.move_speed_ms, v4 = s_cfg.idle_timeout_ms;
    xSemaphoreGive(s_cfg_mutex);

    // Window covering, starting from the last position remembered across reboots
    esp_zb_attribute_list_t *esp_zb_wc_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING);
    uint8_t wc_type = 0;
    esp_zb_window_covering_cluster_add_attr(esp_zb_wc_cluster, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_WINDOW_COVERING_TYPE_ID, &wc_type);
    esp_zb_window_covering_cluster_add_attr(esp_zb_wc_cluster, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID, &current_lift);
    esp_zb_cluster_list_add_window_covering_cluster(esp_zb_cluster_list, esp_zb_wc_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    // OTA client: the server only offers images with this manufacturer code and image type
    esp_zb_ota_cluster_cfg_t ota_cluster_cfg = {
        .ota_upgrade_file_version = FW_FILE_VERSION,
        .ota_upgrade_downloaded_file_ver = ESP_ZB_ZCL_OTA_UPGRADE_FILE_VERSION_DEF_VALUE,
        .ota_upgrade_manufacturer = ESPRESSIF_MANUFACTURER_CODE,
        .ota_upgrade_image_type = OTA_IMAGE_TYPE,
    };
    esp_zb_attribute_list_t *esp_zb_ota_client_cluster = esp_zb_ota_cluster_create(&ota_cluster_cfg);
    esp_zb_zcl_ota_upgrade_client_variable_t ota_client_cfg = {
        .timer_query = ESP_ZB_ZCL_OTA_UPGRADE_QUERY_TIMER_COUNT_DEF,
        .hw_version = OTA_HW_VERSION,
        .max_data_size = OTA_MAX_DATA_SIZE,
    };
    uint16_t ota_server_addr = 0xffff;
    uint8_t ota_server_ep = 0xff;
    esp_zb_ota_cluster_add_attr(esp_zb_ota_client_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_CLIENT_DATA_ID, &ota_client_cfg);
    esp_zb_ota_cluster_add_attr(esp_zb_ota_client_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_SERVER_ADDR_ID, &ota_server_addr);
    esp_zb_ota_cluster_add_attr(esp_zb_ota_client_cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_SERVER_ENDPOINT_ID, &ota_server_ep);
    esp_zb_cluster_list_add_ota_cluster(esp_zb_cluster_list, esp_zb_ota_client_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

    // Our Custom Boundary Configuration Setup, plus read-only boot diagnostics
    esp_zb_attribute_list_t *custom_cluster = esp_zb_zcl_attr_list_create(CUSTOM_CLUSTER_ID);
    // Calibration: writable from Z2M, and reported back so Z2M shows the saved values
    const uint8_t cfg_access = ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING;
    esp_zb_custom_cluster_add_custom_attr(custom_cluster, ATTR_SERVO_MIN_US_ID, ESP_ZB_ZCL_ATTR_TYPE_U16, cfg_access, &v1);
    esp_zb_custom_cluster_add_custom_attr(custom_cluster, ATTR_SERVO_MAX_US_ID, ESP_ZB_ZCL_ATTR_TYPE_U16, cfg_access, &v2);
    esp_zb_custom_cluster_add_custom_attr(custom_cluster, ATTR_MOVE_SPEED_MS_ID, ESP_ZB_ZCL_ATTR_TYPE_U16, cfg_access, &v3);
    esp_zb_custom_cluster_add_custom_attr(custom_cluster, ATTR_IDLE_TIMEOUT_MS_ID, ESP_ZB_ZCL_ATTR_TYPE_U16, cfg_access, &v4);
    esp_zb_custom_cluster_add_custom_attr(custom_cluster, ATTR_RESET_REASON_ID, ESP_ZB_ZCL_ATTR_TYPE_U8,
                                          ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &s_reset_reason);
    esp_zb_custom_cluster_add_custom_attr(custom_cluster, ATTR_BOOT_COUNT_ID, ESP_ZB_ZCL_ATTR_TYPE_U16,
                                          ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &s_boot_count);
    esp_zb_custom_cluster_add_custom_attr(custom_cluster, ATTR_CHIP_TEMP_ID, ESP_ZB_ZCL_ATTR_TYPE_S16,
                                          ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &s_chip_temp);
    esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, custom_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_WINDOW_COVERING_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, endpoint_config);
    esp_zb_device_register(esp_zb_ep_list);

    // Bind Handlers
    esp_zb_core_action_handler_register(cb_action_handler);
    esp_zb_identify_notify_handler_register(ENDPOINT, identify_cb);
    esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);

    ESP_ERROR_CHECK(esp_zb_start(false));
    if (migrate) save_role();
    s_zb_started = true;
    esp_zb_stack_main_loop();
}

void app_main(void) {
    esp_zb_platform_config_t config = {
        .radio_config = {
            .radio_mode = ZB_RADIO_MODE_NATIVE,
        },
        .host_config = {
            .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,
        },
    };

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    s_cfg_mutex = xSemaphoreCreateMutex();
    load_config_from_nvs();
    update_boot_diagnostics();
    ota_start_join_watchdog();

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Starts with the output low: the servo is not driven at boot, so it doesn't twitch.
    // It moves only when a command arrives.
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    xTaskCreate(move_task, "move_task", 4096, NULL, 5, &s_move_task);
    xTaskCreate(button_task, "button_task", 3072, NULL, 4, NULL);
    xTaskCreate(led_task, "led_task", 2048, NULL, 3, NULL);
    chip_temp_init();
    if (s_temp_sensor) xTaskCreate(chip_temp_task, "chip_temp_task", 3072, NULL, 2, NULL);
    xTaskCreate(esp_zb_task, "esp_zb_task", 8192, NULL, 5, NULL);
}
