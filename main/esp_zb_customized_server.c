#include <stdio.h>
#include <stdlib.h>
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
#include "driver/usb_serial_jtag.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_rom_sys.h"
#include "esp_zigbee_core.h"
#include "ha/esp_zigbee_ha_standard.h"

#define TAG "ESP_SERVO_BLIND"

#define FW_VERSION_STR "\x05""1.3.0"
#define FW_APP_VERSION 3
// OTA file version, one byte each: major.minor.patch.build. Must grow with every release.
#define FW_FILE_VERSION 0x01030001

// -- OTA --
#define OTA_IMAGE_TYPE          0x5342   // "SB": espServoBlind
#define OTA_HW_VERSION          1
#define OTA_MAX_DATA_SIZE       223      // block size Espressif recommends
#define OTA_QUERY_INTERVAL_MIN  (24 * 60)
#define OTA_ELEMENT_HEADER_LEN  6        // sub-element tag id (2) + length (4)
#define OTA_TAG_UPGRADE_IMAGE   0x0000
// New firmware is kept only after it has stayed up this long AND the coordinator has answered
// it; a crash before that restarts into the old firmware
#define OTA_CONFIRM_DELAY_MS    60000
// The coordinator is asked again this often until it answers
#define OTA_CONFIRM_RETRY_MS    30000
// New firmware that hasn't reached the coordinator by then rolls back on its own, even if it
// never crashes
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
// A shorter idle timeout could cut the signal before the servo has arrived (0 = hold forever)
#define IDLE_TIMEOUT_MIN_MS     500
// Without feedback and without a ramp (speed 0) the servo needs time to get there: about 0.3 ms
// per microsecond of pulse change, at most this long
#define JUMP_SETTLE_MAX_MS      800

// -- POSITION FEEDBACK (optional: a servo with an analog feedback wire) --
// The wire carries the voltage of the servo's own position potentiometer. Off by default, so a
// servo without the wire works exactly as before.
#define FB_PIN                  GPIO_NUM_4
#define FB_PERIOD_MS            50
#define FB_SAMPLES              9       // median of these, taken about 0.3 ms apart
#define FB_RANGE_MARGIN_MV      300     // a reading this far outside the calibration = no signal
#define FB_REPORT_MIN_MS        500     // feedback values are reported at most this often
#define FB_REPORT_MV            40      // ...and only after a change of this size (noise is a few mV)
#define FB_WIRE_CHECK_MS        2000    // with the motor off, the wire is tested this often
#define FB_WIRE_SETTLE_MS       20      // time for the pin to follow the ESP's weak pull resistor
#define FB_WIRE_MAX_MV          600     // a connected servo holds the pin within this; a loose wire can't
#define FB_HAND_PCT             3       // moved this far with the motor off = moved by hand
#define FB_HAND_STABLE_N        10      // ...for this many readings in a row (0.5 s)
#define FB_BLOCK_LAG_US         150     // this far from the commanded pulse...
#define FB_BLOCK_MS             700     // ...with no movement for this long = blocked
#define FB_STILL_MV             12      // less change than this is "no movement"
#define FB_ARRIVE_US            60      // within this of the commanded pulse = arrived
#define FB_SETTLE_MS            1500    // time allowed to arrive after the ramp ends

// -- CALIBRATION (finds the two ends of the servo's travel from the feedback) --
#define CAL_US_MIN              400
#define CAL_US_MAX              2600
#define CAL_SPEED_US_S          120
#define CAL_FRAME_MS            50
#define CAL_STALL_US            80      // the feedback must change while the pulse moves this far...
#define CAL_STALL_MV            20      // ...by at least this much, or the end is reached
#define CAL_MATCH_MV            6
#define CAL_HIST                48
#define CAL_MIN_SPAN_US         500     // a smaller result means the wire is not connected
#define CAL_MIN_SPAN_MV         500
#define CAL_RETURN_US_S         300

// -- TEST PULSE (drive the servo with a raw pulse width, from Z2M or the USB page) --
#define RAW_SPEED_DEFAULT_US_S  200
#define RAW_SPEED_MAX_US_S      500
#define RAW_HOLD_MAX_MS         (5 * 60 * 1000)   // a forgotten test pulse is switched off

// -- HEARTBEAT --
#define HEARTBEAT_MS            60000
#define BOOT_REPORT_FIRST_MS    1000    // the reports after joining are spread out, not one burst
#define BOOT_REPORT_STEP_MS     400

// -- USB (the control page in tools/) --
#define USB_PERIOD_MS           50

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
// Added in 1.3.0. Older converters ignore them; older firmware answers "unsupported attribute".
#define ATTR_FEEDBACK_ENABLE_ID  0x0005   // u8, 0/1, saved
#define ATTR_COMMAND_ID          0x0006   // u8, write-only action (see command_t), reads 0
#define ATTR_TEST_PULSE_ID       0x0007   // u16, 0 = off, else drive this pulse width (us)
#define ATTR_STATUS_ID           0x0013   // u8, see status_t
#define ATTR_FEEDBACK_MV_ID      0x0014   // u16, feedback voltage in mV
#define ATTR_FEEDBACK_POS_ID     0x0015   // u8, position from the feedback in %, 0xFF = not known
#define ATTR_CAL_LO_US_ID        0x0016   // u16, calibration: low end of the travel, pulse...
#define ATTR_CAL_LO_MV_ID        0x0017   // u16, ...and feedback voltage there
#define ATTR_CAL_HI_US_ID        0x0018   // u16, high end, pulse
#define ATTR_CAL_HI_MV_ID        0x0019   // u16, high end, voltage

// Reported as "last reset reason" when the settings storage had to be erased at boot
#define RESET_REASON_STORAGE_ERASED 0x80

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

    // Position feedback. Calibration all zero = not calibrated.
    uint8_t fb_enable;
    uint16_t cal_lo_us, cal_lo_mv, cal_hi_us, cal_hi_mv;
} blind_config_t;

typedef enum {
    STATUS_OK = 0,
    STATUS_MOVING = 1,
    STATUS_BLOCKED = 2,        // feedback: the servo did not reach the commanded position
    STATUS_NO_FEEDBACK = 3,    // feedback is on but there is no usable signal on the wire
    STATUS_MOVED_BY_HAND = 4,  // feedback: the position changed with the motor off
    STATUS_CALIBRATING = 5,
    STATUS_TEST = 6,           // driven by a test pulse
} status_t;

typedef enum {
    COMMAND_NONE = 0,
    COMMAND_CALIBRATE = 1,
    COMMAND_SET_OPEN_HERE = 2,    // take the current (hand-set) position as 0 %
    COMMAND_SET_CLOSED_HERE = 3,  // take the current (hand-set) position as 100 %
} command_t;

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
static bool s_storage_erased = false;

// Servo output and feedback, shared between the tasks (single values, written by one task each)
static volatile int s_pulse_us = 0;          // pulse being sent now, 0 = PWM off
static volatile int s_pulse_target_us = 0;   // where the current move ends (for the USB page)
static volatile int s_fb_mv = 0;             // latest feedback voltage
static volatile uint8_t s_fb_pct = 0xFF;     // latest position from the feedback, 0xFF = not known
static volatile uint8_t s_status = STATUS_OK;
static volatile bool s_move_busy = false;    // the move task is driving the servo
static volatile bool s_abort = false;        // stop a calibration
static volatile bool s_cal_request = false;
static volatile bool s_move_request = false;  // a position command is waiting for the move task
static volatile bool s_raw_active = false;   // test pulse mode
static volatile int s_raw_target_us = 0;
static volatile int s_raw_speed_us_s = RAW_SPEED_DEFAULT_US_S;
static volatile bool s_coordinator_seen = false;
static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_adc_cali;
static adc_channel_t s_adc_channel;
static bool s_adc_ok = false;
static volatile bool s_fb_wire_ok = true;    // false: the last wire test found nothing on the pin

static uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static int clamp_int(int v, int lo, int hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// 0 = hold forever; anything else must leave the servo time to arrive
static uint16_t clamp_idle_timeout(uint16_t v) {
    return (v != 0 && v < IDLE_TIMEOUT_MIN_MS) ? IDLE_TIMEOUT_MIN_MS : v;
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
        nvs_set_u8(handle, "fb_en", s_cfg.fb_enable);
        xSemaphoreGive(s_cfg_mutex);
        nvs_commit(handle);
        nvs_close(handle);
    }
}

// Written only when a calibration succeeds
static void save_calibration_to_nvs(void) {
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
        xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
        nvs_set_u16(handle, "cal_lo_us", s_cfg.cal_lo_us);
        nvs_set_u16(handle, "cal_lo_mv", s_cfg.cal_lo_mv);
        nvs_set_u16(handle, "cal_hi_us", s_cfg.cal_hi_us);
        nvs_set_u16(handle, "cal_hi_mv", s_cfg.cal_hi_mv);
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
        // Keys added in 1.3.0: missing after an update from an older version, so the defaults
        // (feedback off, not calibrated) apply
        nvs_get_u8(handle, "fb_en", &s_cfg.fb_enable);
        nvs_get_u16(handle, "cal_lo_us", &s_cfg.cal_lo_us);
        nvs_get_u16(handle, "cal_lo_mv", &s_cfg.cal_lo_mv);
        nvs_get_u16(handle, "cal_hi_us", &s_cfg.cal_hi_us);
        nvs_get_u16(handle, "cal_hi_mv", &s_cfg.cal_hi_mv);
        s_cfg.fb_enable = s_cfg.fb_enable ? 1 : 0;
        s_cfg.idle_timeout_ms = clamp_idle_timeout(s_cfg.idle_timeout_ms);
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
    s_reset_reason = s_storage_erased ? RESET_REASON_STORAGE_ERASED : (uint8_t)esp_reset_reason();
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
    // The settings were erased at this boot, so the marker is gone too. The Zigbee network data
    // is in its own partition and still good: keep it, the device stays paired.
    if (s_storage_erased) return false;
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
static void servo_write_us(int us) {
    us = clamp_int(us, CAL_US_MIN, CAL_US_MAX);
    uint32_t duty = ((uint32_t)us * LEDC_DUTY_MAX) / SERVO_PERIOD_US;
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    s_pulse_us = us;
}

static int percent_to_us(int32_t pos_h, uint16_t min_us, uint16_t max_us) {
    // Signed math so min > max (inverted direction) works too
    return (int)((int32_t)min_us + (((int32_t)max_us - (int32_t)min_us) * pos_h) / 10000);
}

static void servo_write(int32_t pos_h, uint16_t min_us, uint16_t max_us) {
    servo_write_us(percent_to_us(pos_h, min_us, max_us));
}

static void servo_off(void) {
    // Applied at the next period, so the last pulse is never cut short. With no pulses the
    // servo stops holding torque: no buzz, no jitter, no stall current.
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    s_pulse_us = 0;
    s_pulse_target_us = 0;
}

// -- POSITION FEEDBACK --
static blind_config_t config_snapshot(void) {
    xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
    blind_config_t cfg = s_cfg;
    xSemaphoreGive(s_cfg_mutex);
    return cfg;
}

static bool cal_valid(const blind_config_t *cfg) {
    return (int)cfg->cal_hi_us - (int)cfg->cal_lo_us >= CAL_MIN_SPAN_US &&
           abs((int)cfg->cal_hi_mv - (int)cfg->cal_lo_mv) >= CAL_MIN_SPAN_MV;
}

// A reading outside the calibrated voltages (with some margin) is not a position: no wire, or a
// broken one
static bool fb_mv_in_range(const blind_config_t *cfg, int mv) {
    int lo = cfg->cal_lo_mv < cfg->cal_hi_mv ? cfg->cal_lo_mv : cfg->cal_hi_mv;
    int hi = cfg->cal_lo_mv < cfg->cal_hi_mv ? cfg->cal_hi_mv : cfg->cal_lo_mv;
    return mv >= lo - FB_RANGE_MARGIN_MV && mv <= hi + FB_RANGE_MARGIN_MV;
}

// The pulse width that puts the servo where the feedback says it is now
static int fb_mv_to_us(const blind_config_t *cfg, int mv) {
    int us = (int)cfg->cal_lo_us + (mv - (int)cfg->cal_lo_mv) * ((int)cfg->cal_hi_us - (int)cfg->cal_lo_us) /
                                       ((int)cfg->cal_hi_mv - (int)cfg->cal_lo_mv);
    return clamp_int(us, cfg->cal_lo_us, cfg->cal_hi_us);
}

// Position in hundredths of a percent for a pulse width, limited to 0..100 %
static int32_t us_to_pos_h(const blind_config_t *cfg, int us) {
    int span = (int)cfg->servo_max_us - (int)cfg->servo_min_us;
    if (span == 0) return 0;
    int32_t pos_h = (int32_t)(((int64_t)(us - (int)cfg->servo_min_us) * 10000) / span);
    return pos_h < 0 ? 0 : (pos_h > 10000 ? 10000 : pos_h);
}

// True when the feedback can be used for positions right now
static bool fb_usable(const blind_config_t *cfg) {
    return s_adc_ok && s_fb_wire_ok && cfg->fb_enable && cal_valid(cfg) && fb_mv_in_range(cfg, s_fb_mv);
}

static int cmp_int(const void *a, const void *b) {
    return *(const int *)a - *(const int *)b;
}

// Median of a few samples: ignores the short spikes the servo motor puts on the wire
static int fb_read_mv(void) {
    if (!s_adc_ok) return 0;
    int raw[FB_SAMPLES];
    for (int i = 0; i < FB_SAMPLES; i++) {
        if (adc_oneshot_read(s_adc, s_adc_channel, &raw[i]) != ESP_OK) raw[i] = 0;
        esp_rom_delay_us(300);
    }
    qsort(raw, FB_SAMPLES, sizeof(int), cmp_int);
    int mv = 0;
    adc_cali_raw_to_voltage(s_adc_cali, raw[FB_SAMPLES / 2], &mv);
    return mv;
}

// Only a missing feature if it fails: the cover works without it
static void fb_init(void) {
    adc_unit_t unit;
    esp_err_t err = adc_oneshot_io_to_channel(FB_PIN, &unit, &s_adc_channel);
    if (err == ESP_OK) {
        adc_oneshot_unit_init_cfg_t unit_cfg = {.unit_id = unit};
        err = adc_oneshot_new_unit(&unit_cfg, &s_adc);
    }
    if (err == ESP_OK) {
        adc_oneshot_chan_cfg_t chan_cfg = {.atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT};
        err = adc_oneshot_config_channel(s_adc, s_adc_channel, &chan_cfg);
    }
    if (err == ESP_OK) {
        adc_cali_curve_fitting_config_t cali_cfg = {
            .unit_id = unit, .chan = s_adc_channel, .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        err = adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_adc_cali);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Feedback input not available: %s", esp_err_to_name(err));
        return;
    }
    s_adc_ok = true;
    s_fb_mv = fb_read_mv();
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
static void set_custom_attr_locked(uint16_t attr, void *value, bool report) {
    esp_zb_zcl_set_attribute_val(ENDPOINT, CUSTOM_CLUSTER_ID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, attr, value, false);
    if (report && s_network_joined) send_report_to_coordinator(CUSTOM_CLUSTER_ID, attr);
}

static void report_config(void) {
    blind_config_t cfg = config_snapshot();
    uint16_t values[] = {cfg.servo_min_us, cfg.servo_max_us, cfg.move_speed_ms, cfg.idle_timeout_ms};
    const uint16_t ids[] = {ATTR_SERVO_MIN_US_ID, ATTR_SERVO_MAX_US_ID, ATTR_MOVE_SPEED_MS_ID, ATTR_IDLE_TIMEOUT_MS_ID};
    for (int i = 0; i < 4; i++) set_custom_attr_locked(ids[i], &values[i], true);
    set_custom_attr_locked(ATTR_FEEDBACK_ENABLE_ID, &cfg.fb_enable, true);
    // The command attribute is an action, not a value: it always reads 0 again
    uint8_t no_command = COMMAND_NONE;
    set_custom_attr_locked(ATTR_COMMAND_ID, &no_command, false);
    uint16_t test_pulse = s_raw_active ? (uint16_t)s_raw_target_us : 0;
    set_custom_attr_locked(ATTR_TEST_PULSE_ID, &test_pulse, true);
}

// Must be called with the Zigbee lock held (or from the Zigbee task)
static void report_calibration_locked(void) {
    blind_config_t cfg = config_snapshot();
    set_custom_attr_locked(ATTR_CAL_LO_US_ID, &cfg.cal_lo_us, true);
    set_custom_attr_locked(ATTR_CAL_LO_MV_ID, &cfg.cal_lo_mv, true);
    set_custom_attr_locked(ATTR_CAL_HI_US_ID, &cfg.cal_hi_us, true);
    set_custom_attr_locked(ATTR_CAL_HI_MV_ID, &cfg.cal_hi_mv, true);
}

// Must be called with the Zigbee lock held (or from the Zigbee task)
static void report_status_locked(void) {
    uint8_t status = s_status;
    set_custom_attr_locked(ATTR_STATUS_ID, &status, true);
}

// Must be called with the Zigbee lock held (or from the Zigbee task)
static void report_feedback_locked(void) {
    uint16_t mv = (uint16_t)s_fb_mv;
    uint8_t pct = s_fb_pct;
    set_custom_attr_locked(ATTR_FEEDBACK_MV_ID, &mv, true);
    set_custom_attr_locked(ATTR_FEEDBACK_POS_ID, &pct, true);
}

// For tasks other than the Zigbee task
static void set_status(status_t status) {
    if (s_status == status) return;
    s_status = status;
    if (!s_zb_started) return;
    esp_zb_lock_acquire(portMAX_DELAY);
    report_status_locked();
    esp_zb_lock_release();
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
    // A position command ends a test pulse and a calibration
    s_raw_target_us = 0;
    s_abort = true;
    s_move_request = true;
    xTaskNotifyGive(s_move_task);
}

static void request_test_pulse(int us);

static void request_stop(void) {
    // Stop ends a test pulse or a calibration, and nothing else then
    if (s_raw_active) {
        request_test_pulse(0);
        return;
    }
    if (s_status == STATUS_CALIBRATING) {
        s_abort = true;
        return;
    }
    // Read and set under one lock so the servo doesn't step back after stopping
    xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
    s_cfg.target_percent = s_cfg.current_percent;
    xSemaphoreGive(s_cfg_mutex);
    s_move_request = true;
    xTaskNotifyGive(s_move_task);
}

static void request_calibration(void) {
    s_raw_target_us = 0;
    s_abort = false;
    s_cal_request = true;
    xTaskNotifyGive(s_move_task);
}

// 0 switches the motor off and leaves the test mode
static void request_test_pulse(int us) {
    s_raw_target_us = us > 0 ? clamp_int(us, CAL_US_MIN, CAL_US_MAX) : 0;
    if (us > 0) s_raw_active = true;
    xTaskNotifyGive(s_move_task);
}

// Hand teaching: the cover was put where "open" (0 %) or "closed" (100 %) must be, with the motor
// off. The feedback voltage there gives the pulse width for that end.
// Must be called with the Zigbee lock held (or from the Zigbee task)
static bool teach_position_locked(bool closed) {
    blind_config_t cfg = config_snapshot();
    if (!fb_usable(&cfg) || s_move_busy) {
        ESP_LOGW(TAG, "Set position here: needs working, calibrated feedback and a servo at rest");
        return false;
    }
    uint16_t us = clamp_u16((uint16_t)fb_mv_to_us(&cfg, s_fb_mv), SERVO_US_LIMIT_MIN, SERVO_US_LIMIT_MAX);
    uint8_t pct = closed ? 100 : 0;
    xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
    if (closed) s_cfg.servo_max_us = us;
    else s_cfg.servo_min_us = us;
    s_cfg.current_percent = pct;
    s_cfg.target_percent = pct;
    s_cfg.position_trusted = true;
    xSemaphoreGive(s_cfg_mutex);
    save_config_to_nvs();
    ESP_LOGI(TAG, "%s position set to %u us", closed ? "Closed" : "Open", us);
    s_status = STATUS_OK;
    report_status_locked();
    report_position_locked(pct, true);
    return true;
}

// -- CALIBRATION (runs in the move task) --
// The pulse is moved slowly to one side until the feedback stops following, then to the other
// side. A servo ignores pulses outside its range, so the feedback goes flat exactly at each end.
static float s_cal_hist_us[CAL_HIST];
static int s_cal_hist_mv[CAL_HIST];
static int s_cal_hist_n;

static void cal_hist_push(float us, int mv) {
    if (s_cal_hist_n == CAL_HIST) {
        memmove(s_cal_hist_us, s_cal_hist_us + 1, (CAL_HIST - 1) * sizeof(float));
        memmove(s_cal_hist_mv, s_cal_hist_mv + 1, (CAL_HIST - 1) * sizeof(int));
        s_cal_hist_n--;
    }
    s_cal_hist_us[s_cal_hist_n] = us;
    s_cal_hist_mv[s_cal_hist_n] = mv;
    s_cal_hist_n++;
}

static int cal_hist_avg(int last) {
    return (s_cal_hist_mv[last] + s_cal_hist_mv[last - 1] + s_cal_hist_mv[last - 2]) / 3;
}

// True when the pulse moved a full window and the feedback did not follow
static bool cal_stalled(void) {
    if (s_cal_hist_n < 6) return false;
    float now_us = s_cal_hist_us[s_cal_hist_n - 1];
    for (int i = s_cal_hist_n - 4; i >= 2; i--) {
        float d = now_us - s_cal_hist_us[i];
        if (d < 0) d = -d;
        if (d >= CAL_STALL_US) return abs(cal_hist_avg(s_cal_hist_n - 1) - cal_hist_avg(i)) < CAL_STALL_MV;
    }
    return false;
}

// One side of the calibration. Returns false when it was stopped.
static bool cal_find_end(int dir, float *us, uint16_t *end_us, uint16_t *end_mv) {
    float target = dir < 0 ? CAL_US_MIN : CAL_US_MAX;
    s_cal_hist_n = 0;
    while (1) {
        if (s_abort) return false;
        *us += dir * CAL_SPEED_US_S * CAL_FRAME_MS / 1000.0f;
        if ((dir < 0 && *us < target) || (dir > 0 && *us > target)) *us = target;
        servo_write_us((int)(*us + 0.5f));
        vTaskDelay(pdMS_TO_TICKS(CAL_FRAME_MS));
        int mv = fb_read_mv();
        s_fb_mv = mv;
        cal_hist_push(*us, mv);
        if (*us == target || cal_stalled()) break;
    }
    // The end is the first pulse at which the feedback already had its final value
    int final_mv = s_cal_hist_n >= 3 ? cal_hist_avg(s_cal_hist_n - 1) : s_cal_hist_mv[s_cal_hist_n - 1];
    float found = s_cal_hist_us[s_cal_hist_n - 1];
    for (int i = 0; i < s_cal_hist_n; i++) {
        if (abs(s_cal_hist_mv[i] - final_mv) <= CAL_MATCH_MV) {
            found = s_cal_hist_us[i];
            break;
        }
    }
    *end_us = (uint16_t)(((int)(found + 0.5f) + 2) / 5 * 5);
    *end_mv = (uint16_t)final_mv;
    *us = *end_us;
    servo_write_us(*end_us);
    return true;
}

// Returns with the PWM on (the caller's idle timeout switches it off)
static void run_calibration(void) {
    if (!s_adc_ok) {
        set_status(STATUS_NO_FEEDBACK);
        return;
    }
    ESP_LOGW(TAG, "Calibration started");
    set_status(STATUS_CALIBRATING);
    blind_config_t cfg = config_snapshot();
    // Start where the servo is, if that is known; otherwise from the centre (the servo jumps there)
    float us = 1500;
    if (s_pulse_us > 0) us = s_pulse_us;
    else if (cal_valid(&cfg) && fb_mv_in_range(&cfg, s_fb_mv)) us = fb_mv_to_us(&cfg, s_fb_mv);
    float start_us = us;
    s_pulse_target_us = CAL_US_MIN;

    uint16_t lo_us = 0, lo_mv = 0, hi_us = 0, hi_mv = 0;
    bool done = cal_find_end(-1, &us, &lo_us, &lo_mv);
    if (done) {
        s_pulse_target_us = CAL_US_MAX;
        done = cal_find_end(1, &us, &hi_us, &hi_mv);
    }

    status_t result = STATUS_OK;
    if (!done) {
        ESP_LOGW(TAG, "Calibration stopped");
    } else if ((int)hi_us - (int)lo_us < CAL_MIN_SPAN_US || abs((int)hi_mv - (int)lo_mv) < CAL_MIN_SPAN_MV) {
        // The voltage did not follow the servo: no feedback wire on this pin
        ESP_LOGE(TAG, "Calibration failed: %u us = %u mV, %u us = %u mV", lo_us, lo_mv, hi_us, hi_mv);
        result = STATUS_NO_FEEDBACK;
    } else {
        xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
        s_cfg.cal_lo_us = lo_us;
        s_cfg.cal_lo_mv = lo_mv;
        s_cfg.cal_hi_us = hi_us;
        s_cfg.cal_hi_mv = hi_mv;
        xSemaphoreGive(s_cfg_mutex);
        save_calibration_to_nvs();
        ESP_LOGW(TAG, "Calibration: %u us = %u mV, %u us = %u mV", lo_us, lo_mv, hi_us, hi_mv);
        esp_zb_lock_acquire(portMAX_DELAY);
        report_calibration_locked();
        esp_zb_lock_release();
        // Go back to where the servo was before
        float back = start_us < lo_us ? lo_us : (start_us > hi_us ? hi_us : start_us);
        s_pulse_target_us = (int)back;
        while (us != back && !s_abort) {
            float step = CAL_RETURN_US_S * MOVE_FRAME_MS / 1000.0f;
            if (us > back) us = us - step < back ? back : us - step;
            else us = us + step > back ? back : us + step;
            servo_write_us((int)(us + 0.5f));
            vTaskDelay(pdMS_TO_TICKS(MOVE_FRAME_MS));
        }
        vTaskDelay(pdMS_TO_TICKS(300));
    }

    // The position in percent now comes from the feedback (when it is usable)
    s_fb_mv = fb_read_mv();
    cfg = config_snapshot();
    if (result == STATUS_OK && fb_usable(&cfg)) {
        uint8_t pct = (uint8_t)((us_to_pos_h(&cfg, fb_mv_to_us(&cfg, s_fb_mv)) + 50) / 100);
        xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
        s_cfg.current_percent = pct;
        s_cfg.target_percent = pct;
        s_cfg.position_trusted = true;
        xSemaphoreGive(s_cfg_mutex);
    } else {
        xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
        s_cfg.position_trusted = false;
        xSemaphoreGive(s_cfg_mutex);
    }
    s_abort = false;
    set_status(result);
}

// -- TEST PULSE (runs in the move task) --
// Ramps the raw pulse to s_raw_target_us and follows new targets until the motor is switched off
// (target 0) or the target is reached. Returns true while the pulse must be held.
static bool run_test_pulse(void) {
    blind_config_t cfg = config_snapshot();
    float us;
    if (s_pulse_us > 0) us = s_pulse_us;
    else if (cal_valid(&cfg) && cfg.fb_enable && fb_mv_in_range(&cfg, s_fb_mv)) us = fb_mv_to_us(&cfg, s_fb_mv);
    else us = s_raw_target_us;   // position not known: the servo jumps to the first pulse
    set_status(STATUS_TEST);
    int64_t last = esp_timer_get_time();
    while (1) {
        int target = s_raw_target_us;
        if (target == 0) return false;
        s_pulse_target_us = target;
        int64_t now = esp_timer_get_time();
        float step = s_raw_speed_us_s * (now - last) / 1e6f;
        last = now;
        if (us > target) us = us - step < target ? target : us - step;
        else us = us + step > target ? target : us + step;
        servo_write_us((int)(us + 0.5f));
        if (us == target) return true;
        vTaskDelay(pdMS_TO_TICKS(MOVE_FRAME_MS));
    }
}

// -- FREERTOS MOVEMENT TASK --
// Feedback during a move: the servo is "blocked" when it is far from the commanded pulse and the
// feedback has not moved for a while. A servo that is still travelling is never blocked.
typedef struct {
    int ref_mv;
    int64_t ref_time;
} fb_watch_t;

static void fb_watch_start(fb_watch_t *w) {
    w->ref_mv = s_fb_mv;
    w->ref_time = esp_timer_get_time();
}

// Distance in microseconds of pulse between where the servo is and where it was told to go
static int fb_lag_us(const blind_config_t *cfg, int cmd_us) {
    cmd_us = clamp_int(cmd_us, cfg->cal_lo_us, cfg->cal_hi_us);
    return abs(fb_mv_to_us(cfg, s_fb_mv) - cmd_us);
}

static bool fb_watch_blocked(fb_watch_t *w, const blind_config_t *cfg, int cmd_us) {
    int64_t now = esp_timer_get_time();
    if (abs(s_fb_mv - w->ref_mv) >= FB_STILL_MV) {
        w->ref_mv = s_fb_mv;
        w->ref_time = now;
        return false;
    }
    return fb_lag_us(cfg, cmd_us) > FB_BLOCK_LAG_US && now - w->ref_time > (int64_t)FB_BLOCK_MS * 1000;
}

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
        // timeout to cut the PWM (0 = keep holding forever). A test pulse is held until it is
        // switched off, but not forever.
        TickType_t wait = portMAX_DELAY;
        if (pwm_on && s_raw_active) wait = pdMS_TO_TICKS(RAW_HOLD_MAX_MS);
        else if (pwm_on && idle_to > 0) wait = pdMS_TO_TICKS(idle_to);
        s_move_busy = false;
        if (ulTaskNotifyTake(pdTRUE, wait) == 0) {
            servo_off();
            pwm_on = false;
            if (s_raw_active) {
                s_raw_active = false;
                s_raw_target_us = 0;
                set_status(STATUS_OK);
            }
            ESP_LOGI(TAG, "Idle: PWM off");
            continue;
        }
        s_move_busy = true;

        if (s_cal_request) {
            s_cal_request = false;
            s_raw_active = false;
            run_calibration();
            pwm_on = s_pulse_us > 0;
            // The servo is somewhere else now: don't ramp from the old position
            xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
            pos_h = s_cfg.current_percent * 100;
            xSemaphoreGive(s_cfg_mutex);
            nvs_dirty = true;
            continue;
        }

        if (s_raw_active) {
            if (s_raw_target_us > 0 && !s_move_request) {
                run_test_pulse();
                pwm_on = s_pulse_us > 0;
                // Reached: hold it and wait for the next command
                if (s_raw_target_us > 0 && !s_move_request) continue;
            }
            // Test mode ended (motor off, or a position command took over)
            s_raw_active = false;
            xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
            s_cfg.position_trusted = false;
            xSemaphoreGive(s_cfg_mutex);
            nvs_dirty = true;
            if (!s_move_request) {
                servo_off();
                pwm_on = false;
                set_status(STATUS_OK);
                continue;
            }
        } else if (!s_move_request) {
            // "Motor off" with nothing running
            servo_off();
            pwm_on = false;
            continue;
        }
        s_move_request = false;

        blind_config_t cfg = config_snapshot();
        s_fb_mv = fb_read_mv();
        bool fb = fb_usable(&cfg);
        int32_t tgt_h = cfg.target_percent * 100;
        if (fb && !pwm_on) {
            // The real position is known: ramp from there, wherever the cover was left
            pos_h = us_to_pos_h(&cfg, fb_mv_to_us(&cfg, s_fb_mv));
        } else if (!cfg.position_trusted) {
            // Unknown start: go straight to the target at the servo's own speed. A ramp from a
            // stale position would first jump there and could swing through an unwanted spot.
            pos_h = tgt_h;
        }
        xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
        s_cfg.position_trusted = true;
        xSemaphoreGive(s_cfg_mutex);

        // With feedback the position is read from the servo, so nothing is written to flash
        if (pos_h != tgt_h && !fb) {
            save_motion_state_to_nvs(true, 0);
            nvs_dirty = true;
        }
        set_status(STATUS_MOVING);

        int32_t start_h = pos_h;
        int32_t ramp_tgt_h = tgt_h;
        int64_t t0 = esp_timer_get_time();
        int jump_us = 0;
        bool blocked = false;
        fb_watch_t watch;
        fb_watch_start(&watch);
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
                jump_us = abs(percent_to_us(tgt_h, min_us, max_us) - percent_to_us(pos_h, min_us, max_us));
                pos_h = tgt_h;
            } else {
                // move_speed_ms per 1% = 100 hundredths, independent of the RTOS tick
                int64_t travelled = ((esp_timer_get_time() - t0) / 10) / spd;
                int32_t dist = tgt_h > start_h ? tgt_h - start_h : start_h - tgt_h;
                if (travelled >= dist) pos_h = tgt_h;
                else pos_h = tgt_h > start_h ? start_h + (int32_t)travelled : start_h - (int32_t)travelled;
            }
            int32_t pct = (pos_h + 50) / 100;

            s_pulse_target_us = percent_to_us(tgt_h, min_us, max_us);
            servo_write(pos_h, min_us, max_us);
            pwm_on = true;

            xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
            s_cfg.current_percent = (uint8_t)pct;
            xSemaphoreGive(s_cfg_mutex);

            if (pos_h == tgt_h) break;
            if (fb) {
                s_fb_mv = fb_read_mv();
                if (fb_watch_blocked(&watch, &cfg, s_pulse_us)) {
                    blocked = true;
                    break;
                }
            }
            // Progress every 5% (a fast ramp can cover several percent per frame)
            if (pct / 5 != prev_pct / 5) report_position((uint8_t)pct, false);
            vTaskDelay(pdMS_TO_TICKS(MOVE_FRAME_MS));
        }

        if (fb && !blocked) {
            // The ramp has ended: give the servo time to arrive, and see that it does
            int64_t deadline = esp_timer_get_time() + (int64_t)FB_SETTLE_MS * 1000;
            while (esp_timer_get_time() < deadline) {
                vTaskDelay(pdMS_TO_TICKS(MOVE_FRAME_MS));
                s_fb_mv = fb_read_mv();
                if (fb_lag_us(&cfg, s_pulse_us) <= FB_ARRIVE_US) break;
                if (fb_watch_blocked(&watch, &cfg, s_pulse_us)) {
                    blocked = true;
                    break;
                }
                xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
                bool retarget = s_cfg.target_percent * 100 != tgt_h;
                xSemaphoreGive(s_cfg_mutex);
                if (retarget) break;   // a new command is waiting: the outer loop takes it
            }
        } else if (!fb && jump_us > 0) {
            // No ramp and no feedback: wait for the servo before the position counts as reached
            int settle = jump_us * 3 / 10;
            vTaskDelay(pdMS_TO_TICKS(settle > JUMP_SETTLE_MAX_MS ? JUMP_SETTLE_MAX_MS : settle));
        }

        if (blocked) {
            // Something holds the cover: stop pushing, and report where it really is
            servo_off();
            pwm_on = false;
            uint8_t real = (uint8_t)((us_to_pos_h(&cfg, fb_mv_to_us(&cfg, s_fb_mv)) + 50) / 100);
            xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
            s_cfg.current_percent = real;
            s_cfg.target_percent = real;
            xSemaphoreGive(s_cfg_mutex);
            pos_h = real * 100;
            ESP_LOGW(TAG, "Blocked at %d%%", real);
            s_status = STATUS_BLOCKED;
            esp_zb_lock_acquire(portMAX_DELAY);
            report_status_locked();
            report_position_locked(real, true);
            esp_zb_lock_release();
            continue;
        }

        uint8_t cur = current_position();
        s_status = STATUS_OK;
        esp_zb_lock_acquire(portMAX_DELAY);
        report_status_locked();
        report_position_locked(cur, true);
        esp_zb_lock_release();
        if (!fb && (nvs_dirty || cur != saved_pos)) {
            save_motion_state_to_nvs(false, cur);
            saved_pos = cur;
            nvs_dirty = false;
        }
        ESP_LOGI(TAG, "At %d%%", cur);
    }
}

// -- FEEDBACK TASK --
// Is there a servo on the feedback pin? The ESP's weak pull-up and pull-down resistors (about
// 45 kOhm) are switched on in turn. A servo's potentiometer holds its voltage against them; a
// pin with no wire, or a cut wire, follows them all the way. Only done with the motor off: the
// servo's own controller reads the same point, and it is not driving anything then.
static bool fb_wire_present(void) {
    gpio_set_pull_mode(FB_PIN, GPIO_PULLUP_ONLY);
    vTaskDelay(pdMS_TO_TICKS(FB_WIRE_SETTLE_MS));
    int up_mv = fb_read_mv();
    gpio_set_pull_mode(FB_PIN, GPIO_PULLDOWN_ONLY);
    vTaskDelay(pdMS_TO_TICKS(FB_WIRE_SETTLE_MS));
    int down_mv = fb_read_mv();
    gpio_set_pull_mode(FB_PIN, GPIO_FLOATING);
    vTaskDelay(pdMS_TO_TICKS(FB_WIRE_SETTLE_MS));
    return up_mv - down_mv < FB_WIRE_MAX_MV;
}

// Reads the feedback wire, keeps the reported values fresh, and notices a cover that was moved
// by hand while the motor is off.
static void fb_task(void *pvParameters) {
    int stable = 0;
    bool synced = false;
    int reported_mv = -1000;
    uint8_t reported_pct = 0xFE;
    int64_t last_report = 0;
    int64_t last_wire_check = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(FB_PERIOD_MS));
        blind_config_t cfg = config_snapshot();
        if (!cfg.fb_enable) {
            s_fb_pct = 0xFF;
            s_fb_wire_ok = true;
            stable = 0;
            synced = false;
            continue;
        }
        bool idle = s_pulse_us == 0 && !s_move_busy;

        // A loose wire gives voltages that wander like a moving cover. Test the wire regularly,
        // and always before something is called "moved by hand".
        int64_t now = esp_timer_get_time();
        bool hand_pending = stable >= FB_HAND_STABLE_N - 1;
        if (idle && (hand_pending || now - last_wire_check >= (int64_t)FB_WIRE_CHECK_MS * 1000)) {
            last_wire_check = now;
            bool present = fb_wire_present();
            if (present != s_fb_wire_ok && !s_move_busy) {
                s_fb_wire_ok = present;
                if (!present) {
                    ESP_LOGW(TAG, "Feedback wire: no servo on the pin");
                    // Where the cover is can't be known any more
                    xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
                    s_cfg.position_trusted = false;
                    xSemaphoreGive(s_cfg_mutex);
                    set_status(STATUS_NO_FEEDBACK);
                } else {
                    ESP_LOGW(TAG, "Feedback wire: connected again");
                    set_status(STATUS_OK);
                }
            }
        }

        // During a move the move task reads the wire itself
        if (!s_move_busy) s_fb_mv = fb_read_mv();

        uint8_t pct = 0xFF;
        if (s_fb_wire_ok && cal_valid(&cfg)) {
            if (fb_mv_in_range(&cfg, s_fb_mv)) {
                pct = (uint8_t)((us_to_pos_h(&cfg, fb_mv_to_us(&cfg, s_fb_mv)) + 50) / 100);
                if (s_status == STATUS_NO_FEEDBACK && !s_move_busy) set_status(STATUS_OK);
            } else if (!s_move_busy && s_status != STATUS_NO_FEEDBACK) {
                ESP_LOGW(TAG, "Feedback voltage %d mV is outside the calibration: no signal", s_fb_mv);
                set_status(STATUS_NO_FEEDBACK);
            }
        }
        s_fb_pct = pct;

        // First good reading (after boot, or after the feedback was switched on or calibrated):
        // the position is what the servo says, not what was remembered
        if (pct == 0xFF) {
            synced = false;
        } else if (!synced && idle) {
            synced = true;
            xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
            s_cfg.current_percent = pct;
            s_cfg.target_percent = pct;
            s_cfg.position_trusted = true;
            xSemaphoreGive(s_cfg_mutex);
            cfg.current_percent = pct;
            cfg.position_trusted = true;
            if (s_zb_started) report_position(pct, true);
        }

        // Moved by hand: the motor is off and the position is somewhere else, and stays there
        if (synced && idle && cfg.position_trusted && abs((int)pct - (int)cfg.current_percent) >= FB_HAND_PCT) {
            if (++stable >= FB_HAND_STABLE_N) {
                stable = 0;
                xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
                s_cfg.current_percent = pct;
                s_cfg.target_percent = pct;
                xSemaphoreGive(s_cfg_mutex);
                ESP_LOGW(TAG, "Moved by hand to %d%%", pct);
                s_status = STATUS_MOVED_BY_HAND;
                if (s_zb_started) {
                    esp_zb_lock_acquire(portMAX_DELAY);
                    report_status_locked();
                    report_position_locked(pct, true);
                    esp_zb_lock_release();
                }
            }
        } else {
            stable = 0;
        }

        // Reported only when it really changed: the few millivolts of noise at rest, or the
        // wandering voltage of a loose wire, must not fill the hub's log
        bool known = pct != 0xFF, was_known = reported_pct != 0xFF;
        now = esp_timer_get_time();
        if (s_zb_started && now - last_report >= (int64_t)FB_REPORT_MIN_MS * 1000 &&
            (known != was_known || reported_pct == 0xFE || (known && abs(s_fb_mv - reported_mv) >= FB_REPORT_MV))) {
            last_report = now;
            reported_mv = s_fb_mv;
            reported_pct = pct;
            esp_zb_lock_acquire(portMAX_DELAY);
            report_feedback_locked();
            esp_zb_lock_release();
        }
    }
}

// -- HEARTBEAT TASK --
// One small report a minute, so the hub knows the device is alive and what state it is in
static void heartbeat_task(void *pvParameters) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_MS));
        if (!s_zb_started || !s_network_joined) continue;
        esp_zb_lock_acquire(portMAX_DELAY);
        report_status_locked();
        if (!s_move_busy) report_position_locked(current_position(), true);
        esp_zb_lock_release();
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

// A new image runs in "pending verify" until it proves it works: it must stay up for
// OTA_CONFIRM_DELAY_MS and get an answer from the coordinator over the air. Being "joined" is not
// enough: a router restores its network from flash without any radio contact, so an image with a
// broken radio would look fine. Any restart before that boots the old image again, and an image
// that never reaches the coordinator rolls itself back after OTA_JOIN_TIMEOUT_MS.
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
    ESP_LOGW(TAG, "OTA: running new firmware on trial, kept only once it has run for %d s and the coordinator answers",
             OTA_CONFIRM_DELAY_MS / 1000);
    const esp_timer_create_args_t args = {.callback = ota_join_timeout_cb, .name = "ota_join"};
    if (esp_timer_create(&args, &s_ota_join_timer) == ESP_OK) {
        esp_timer_start_once(s_ota_join_timer, (uint64_t)OTA_JOIN_TIMEOUT_MS * 1000);
    }
}

static void ota_confirm_cb(uint8_t param);

static void ota_contact_rsp_cb(esp_zb_zdp_status_t zdo_status, esp_zb_zdo_ieee_addr_rsp_t *resp, void *user_ctx) {
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) s_coordinator_seen = true;
    if (!ota_running_image_pending()) return;
    if (!s_coordinator_seen) {
        ESP_LOGW(TAG, "OTA: no answer from the coordinator yet, asking again");
        return;
    }
    if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) {
        if (s_ota_join_timer) esp_timer_stop(s_ota_join_timer);
        esp_zb_scheduler_alarm_cancel(ota_confirm_cb, 0);
        ESP_LOGW(TAG, "OTA: new firmware confirmed, rollback cancelled");
    }
}

// Runs in the Zigbee task, OTA_CONFIRM_DELAY_MS after the network came up, and then again until
// the coordinator has answered
static void ota_confirm_cb(uint8_t param) {
    if (!ota_running_image_pending()) return;
    esp_zb_scheduler_alarm(ota_confirm_cb, 0, OTA_CONFIRM_RETRY_MS);
    if (!s_network_joined) return;
    esp_zb_zdo_ieee_addr_req_param_t req = {
        .dst_nwk_addr = COORDINATOR_ADDR,
        .addr_of_interest = COORDINATOR_ADDR,
        .request_type = 0,
        .start_index = 0,
    };
    esp_zb_zdo_ieee_addr_req(&req, ota_contact_rsp_cb, NULL);
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
        if (!msg->attribute.data.value) return ESP_ERR_INVALID_ARG;

        uint16_t value;
        if (msg->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) value = *(uint16_t *)msg->attribute.data.value;
        else if (msg->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8) value = *(uint8_t *)msg->attribute.data.value;
        else return ESP_ERR_INVALID_ARG;
        ESP_LOGI(TAG, "Config attr 0x%04x = %u", msg->attribute.id, value);

        if (msg->attribute.id == ATTR_COMMAND_ID) {
            // An action, nothing is saved
            if (value == COMMAND_CALIBRATE) request_calibration();
            else if (value == COMMAND_SET_OPEN_HERE) teach_position_locked(false);
            else if (value == COMMAND_SET_CLOSED_HERE) teach_position_locked(true);
        } else if (msg->attribute.id == ATTR_TEST_PULSE_ID) {
            request_test_pulse(value);
        } else {
            // New calibration is only used from the next command on: changing it never moves the servo
            xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
            switch (msg->attribute.id) {
                case ATTR_SERVO_MIN_US_ID:    s_cfg.servo_min_us = clamp_u16(value, SERVO_US_LIMIT_MIN, SERVO_US_LIMIT_MAX); break;
                case ATTR_SERVO_MAX_US_ID:    s_cfg.servo_max_us = clamp_u16(value, SERVO_US_LIMIT_MIN, SERVO_US_LIMIT_MAX); break;
                case ATTR_MOVE_SPEED_MS_ID:   s_cfg.move_speed_ms = clamp_u16(value, 0, MOVE_SPEED_LIMIT_MS); break;
                case ATTR_IDLE_TIMEOUT_MS_ID: s_cfg.idle_timeout_ms = clamp_idle_timeout(value); break;
                case ATTR_FEEDBACK_ENABLE_ID: s_cfg.fb_enable = value ? 1 : 0; break;
                default: break;
            }
            xSemaphoreGive(s_cfg_mutex);
            save_config_to_nvs();
            if (msg->attribute.id == ATTR_FEEDBACK_ENABLE_ID && s_status != STATUS_MOVING) {
                // A stale "no feedback" or "moved by hand" must not outlive the setting
                s_status = STATUS_OK;
                report_status_locked();
            }
        }
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

// The values the hub shows are sent in small groups with pauses between them. Sent as one burst
// right after the network comes up, some were lost before the route to the coordinator was ready.
static void boot_report_cb(uint8_t step) {
    if (!s_network_joined) return;
    switch (step) {
        case 0: report_position_locked(current_position(), true); report_status_locked(); break;
        case 1: report_config(); break;
        case 2: report_diagnostics(); break;
        // Once at start, also with the feedback off: a reading to look back at when something is odd
        case 3: report_feedback_locked(); break;
        case 4: report_calibration_locked(); break;
        case 5: ota_discover_server(); break;
        default: return;
    }
    esp_zb_scheduler_alarm(boot_report_cb, step + 1, BOOT_REPORT_STEP_MS);
}

static void on_network_up(void) {
    s_network_joined = true;
    s_retry_delay_ms = RETRY_MIN_MS;
    // Not confirmed here: if a report below crashed a freshly confirmed image, it would restart
    // forever with no way back. Confirm only after it has stayed up for a while.
    ota_schedule_confirm();
    for (uint8_t step = 0; step <= 5; step++) esp_zb_scheduler_alarm_cancel(boot_report_cb, step);
    esp_zb_scheduler_alarm(boot_report_cb, 0, BOOT_REPORT_FIRST_MS);
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
        s_coordinator_seen = true;
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
    report_status_locked();
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

// -- USB TASK --
// Text commands over the board's USB port, for the control page in tools/servo_live.html. The
// page needs no special firmware: it talks to this one. Nothing here is needed for normal use.
//
// From the board, about 20 lines a second once a command has been received:
//   T <pulse_us> <target_us> <feedback_mv> <lo_us> <hi_us>     pulse 0 = motor off
// and on events:
//   C <lo_us> <lo_mv> <hi_us> <hi_mv>     a calibration has finished
//   I <key> <value>                       answer to "info", ended by "I end"
// Commands, one per line:
//   on            test pulse at the present position      t <us>     test pulse target
//   off           motor off                               v <us/s>   test pulse speed
//   cal           calibrate                               pos <pct>  normal move, 0..100 %
//   fb <0|1>      feedback off/on (saved)                 seto/setc  set open/closed position here
//   min/max <us>  open/closed pulse (saved)               spd/idle <ms>  speed, motor-off delay (saved)
//   calset <lo_us> <lo_mv> <hi_us> <hi_mv>   restore an exported calibration (saved)
//   info          print settings and state
static bool s_usb_stream = false;

static void usb_send(const char *text) {
    // Never waits: without a listening computer the text is dropped
    if (usb_serial_jtag_is_connected()) usb_serial_jtag_write_bytes(text, strlen(text), 0);
}

static void usb_info(void) {
    static const char *const status_names[] = {"ok", "moving", "blocked", "no_feedback", "moved_by_hand", "calibrating", "test"};
    blind_config_t cfg = config_snapshot();
    char line[96];
    // The network key and other secrets are never printed
    snprintf(line, sizeof(line), "I firmware %s\n", &FW_VERSION_STR[1]);
    usb_send(line);
    snprintf(line, sizeof(line), "I status %s\n", s_status < 7 ? status_names[s_status] : "?");
    usb_send(line);
    snprintf(line, sizeof(line), "I position_percent %u\nI target_percent %u\n", cfg.current_percent, cfg.target_percent);
    usb_send(line);
    snprintf(line, sizeof(line), "I servo_min_us %u\nI servo_max_us %u\n", cfg.servo_min_us, cfg.servo_max_us);
    usb_send(line);
    snprintf(line, sizeof(line), "I move_speed_ms %u\nI idle_timeout_ms %u\n", cfg.move_speed_ms, cfg.idle_timeout_ms);
    usb_send(line);
    snprintf(line, sizeof(line), "I feedback %s\nI feedback_mv %d\n", cfg.fb_enable ? "on" : "off", s_fb_mv);
    usb_send(line);
    if (s_fb_pct == 0xFF) snprintf(line, sizeof(line), "I feedback_position unknown\n");
    else snprintf(line, sizeof(line), "I feedback_position %u\n", s_fb_pct);
    usb_send(line);
    snprintf(line, sizeof(line), "I calibration %s\n", cal_valid(&cfg) ? "valid" : "none");
    usb_send(line);
    snprintf(line, sizeof(line), "I cal_low %u_us_%u_mV\nI cal_high %u_us_%u_mV\n", cfg.cal_lo_us, cfg.cal_lo_mv, cfg.cal_hi_us,
             cfg.cal_hi_mv);
    usb_send(line);
    vTaskDelay(pdMS_TO_TICKS(20));   // let the first half leave the USB buffer
    snprintf(line, sizeof(line), "I boot_count %u\nI reset_reason %u\n", s_boot_count, s_reset_reason);
    usb_send(line);
    if (s_chip_temp == CHIP_TEMP_UNKNOWN) snprintf(line, sizeof(line), "I chip_temperature unknown\n");
    else snprintf(line, sizeof(line), "I chip_temperature %d.%d_C\n", s_chip_temp / 100, abs(s_chip_temp % 100) / 10);
    usb_send(line);
    snprintf(line, sizeof(line), "I uptime_s %lld\n", esp_timer_get_time() / 1000000);
    usb_send(line);
    snprintf(line, sizeof(line), "I update_on_trial %s\n", ota_running_image_pending() ? "yes" : "no");
    usb_send(line);
    snprintf(line, sizeof(line), "I zigbee_joined %s\nI coordinator_answered %s\n", s_network_joined ? "yes" : "no",
             s_coordinator_seen ? "yes" : "not_checked");
    usb_send(line);
    if (s_zb_started && s_network_joined) {
        esp_zb_lock_acquire(portMAX_DELAY);
        uint8_t channel = esp_zb_get_current_channel();
        uint16_t pan = esp_zb_get_pan_id(), addr = esp_zb_get_short_address();
        esp_zb_lock_release();
        snprintf(line, sizeof(line), "I zigbee_channel %u\nI zigbee_pan_id 0x%04x\nI zigbee_address 0x%04x\n", channel, pan, addr);
        usb_send(line);
    }
    usb_send("I end\n");
}

static void usb_set_feedback(bool on) {
    xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
    s_cfg.fb_enable = on ? 1 : 0;
    xSemaphoreGive(s_cfg_mutex);
    save_config_to_nvs();
    if (!s_zb_started) return;
    esp_zb_lock_acquire(portMAX_DELAY);
    if (s_status != STATUS_MOVING) {
        s_status = STATUS_OK;
        report_status_locked();
    }
    report_config();
    esp_zb_lock_release();
}

// The four settings that Zigbee2MQTT can write, for a hub that has no converter for them.
// Same limits as over Zigbee, saved the same way, and the hub is told the new value.
static void usb_set_setting(uint16_t attr_id, int val) {
    uint16_t value = (uint16_t)clamp_int(val, 0, UINT16_MAX);
    xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
    switch (attr_id) {
        case ATTR_SERVO_MIN_US_ID:    s_cfg.servo_min_us = clamp_u16(value, SERVO_US_LIMIT_MIN, SERVO_US_LIMIT_MAX); break;
        case ATTR_SERVO_MAX_US_ID:    s_cfg.servo_max_us = clamp_u16(value, SERVO_US_LIMIT_MIN, SERVO_US_LIMIT_MAX); break;
        case ATTR_MOVE_SPEED_MS_ID:   s_cfg.move_speed_ms = clamp_u16(value, 0, MOVE_SPEED_LIMIT_MS); break;
        case ATTR_IDLE_TIMEOUT_MS_ID: s_cfg.idle_timeout_ms = clamp_idle_timeout(value); break;
        default: break;
    }
    xSemaphoreGive(s_cfg_mutex);
    save_config_to_nvs();
    if (!s_zb_started) return;
    esp_zb_lock_acquire(portMAX_DELAY);
    report_config();
    esp_zb_lock_release();
}

// Restores a calibration that was exported before. Only for the same servo: the voltages are
// different for each servo. A calibration that is not plausible is ignored.
static void usb_set_calibration(const char *args) {
    unsigned lo_us, lo_mv, hi_us, hi_mv;
    if (sscanf(args, "%u %u %u %u", &lo_us, &lo_mv, &hi_us, &hi_mv) != 4) return;
    if (lo_us < CAL_US_MIN || hi_us > CAL_US_MAX || lo_mv > 3300 || hi_mv > 3300) return;
    blind_config_t test = {.cal_lo_us = lo_us, .cal_lo_mv = lo_mv, .cal_hi_us = hi_us, .cal_hi_mv = hi_mv};
    if (!cal_valid(&test) || s_status == STATUS_CALIBRATING) return;
    xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
    s_cfg.cal_lo_us = lo_us;
    s_cfg.cal_lo_mv = lo_mv;
    s_cfg.cal_hi_us = hi_us;
    s_cfg.cal_hi_mv = hi_mv;
    xSemaphoreGive(s_cfg_mutex);
    save_calibration_to_nvs();
    if (!s_zb_started) return;
    esp_zb_lock_acquire(portMAX_DELAY);
    report_calibration_locked();
    esp_zb_lock_release();
}

static void usb_teach(bool closed) {
    if (!s_zb_started) return;
    esp_zb_lock_acquire(portMAX_DELAY);
    if (teach_position_locked(closed)) report_config();
    esp_zb_lock_release();
}

static void usb_command(char *line) {
    char *arg = strchr(line, ' ');
    int val = arg ? atoi(arg + 1) : 0;
    if (strcmp(line, "on") == 0) {
        // Start at the present position: from the feedback when it is known, else the centre
        blind_config_t cfg = config_snapshot();
        int us = 1500;
        if (s_pulse_us > 0) us = s_pulse_us;
        else if (cal_valid(&cfg) && fb_mv_in_range(&cfg, s_fb_mv)) us = fb_mv_to_us(&cfg, s_fb_mv);
        request_test_pulse(us);
    } else if (strcmp(line, "off") == 0) {
        s_abort = true;
        request_test_pulse(0);
    } else if (strncmp(line, "t ", 2) == 0) {
        if (s_raw_active && val > 0) request_test_pulse(val);
    } else if (strncmp(line, "v ", 2) == 0) {
        s_raw_speed_us_s = clamp_int(val, 20, RAW_SPEED_MAX_US_S);
    } else if (strcmp(line, "cal") == 0) {
        request_calibration();
    } else if (strncmp(line, "pos ", 4) == 0) {
        request_move((uint8_t)clamp_int(val, 0, 100));
    } else if (strncmp(line, "fb ", 3) == 0) {
        usb_set_feedback(val != 0);
    } else if (strncmp(line, "min ", 4) == 0) {
        usb_set_setting(ATTR_SERVO_MIN_US_ID, val);
    } else if (strncmp(line, "max ", 4) == 0) {
        usb_set_setting(ATTR_SERVO_MAX_US_ID, val);
    } else if (strncmp(line, "spd ", 4) == 0) {
        usb_set_setting(ATTR_MOVE_SPEED_MS_ID, val);
    } else if (strncmp(line, "idle ", 5) == 0) {
        usb_set_setting(ATTR_IDLE_TIMEOUT_MS_ID, val);
    } else if (strncmp(line, "calset ", 7) == 0) {
        usb_set_calibration(line + 7);
    } else if (strcmp(line, "seto") == 0) {
        usb_teach(false);
    } else if (strcmp(line, "setc") == 0) {
        usb_teach(true);
    } else if (strcmp(line, "info") == 0) {
        usb_info();
    }
}

static void usb_task(void *pvParameters) {
    usb_serial_jtag_driver_config_t usb_cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    if (usb_serial_jtag_driver_install(&usb_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "USB commands not available");
        vTaskDelete(NULL);
        return;
    }
    char line[48];
    int n = 0;
    bool was_calibrating = false;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(USB_PERIOD_MS));
        uint8_t c;
        while (usb_serial_jtag_read_bytes(&c, 1, 0) > 0) {
            if (c == '\r') continue;
            if (c == '\n') {
                line[n] = 0;
                n = 0;
                // Old log text can still sit in the USB buffer: start on a clean line
                if (!s_usb_stream) usb_send("\n");
                s_usb_stream = true;
                usb_command(line);
                continue;
            }
            if (n < (int)sizeof(line) - 1) line[n++] = (char)c;
        }
        if (!s_usb_stream) continue;
        if (!usb_serial_jtag_is_connected()) {
            // The computer is gone: stop talking, and don't leave a test pulse running
            s_usb_stream = false;
            if (s_raw_active) request_test_pulse(0);
            continue;
        }

        blind_config_t cfg = config_snapshot();
        char out[96];
        bool calibrating = s_status == STATUS_CALIBRATING;
        if (was_calibrating && !calibrating && cal_valid(&cfg)) {
            snprintf(out, sizeof(out), "C %u %u %u %u\n", cfg.cal_lo_us, cfg.cal_lo_mv, cfg.cal_hi_us, cfg.cal_hi_mv);
            usb_send(out);
        }
        was_calibrating = calibrating;
        int lo = cal_valid(&cfg) ? cfg.cal_lo_us : CAL_US_MIN;
        int hi = cal_valid(&cfg) ? cfg.cal_hi_us : CAL_US_MAX;
        snprintf(out, sizeof(out), "T %d %d %d %d %d\n", s_pulse_us, s_pulse_target_us, s_fb_mv, lo, hi);
        usb_send(out);
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
    uint8_t fb_enable = s_cfg.fb_enable;
    uint16_t cal_lo_us = s_cfg.cal_lo_us, cal_lo_mv = s_cfg.cal_lo_mv, cal_hi_us = s_cfg.cal_hi_us, cal_hi_mv = s_cfg.cal_hi_mv;
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
    // Position feedback (1.3.0): setting, actions, status and calibration
    const uint8_t ro_access = ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING;
    uint8_t command = COMMAND_NONE, status = s_status, fb_pct = 0xFF;
    uint16_t test_pulse = 0, fb_mv = (uint16_t)s_fb_mv;
    esp_zb_custom_cluster_add_custom_attr(custom_cluster, ATTR_FEEDBACK_ENABLE_ID, ESP_ZB_ZCL_ATTR_TYPE_U8, cfg_access, &fb_enable);
    esp_zb_custom_cluster_add_custom_attr(custom_cluster, ATTR_COMMAND_ID, ESP_ZB_ZCL_ATTR_TYPE_U8,
                                          ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &command);
    esp_zb_custom_cluster_add_custom_attr(custom_cluster, ATTR_TEST_PULSE_ID, ESP_ZB_ZCL_ATTR_TYPE_U16, cfg_access, &test_pulse);
    esp_zb_custom_cluster_add_custom_attr(custom_cluster, ATTR_STATUS_ID, ESP_ZB_ZCL_ATTR_TYPE_U8, ro_access, &status);
    esp_zb_custom_cluster_add_custom_attr(custom_cluster, ATTR_FEEDBACK_MV_ID, ESP_ZB_ZCL_ATTR_TYPE_U16, ro_access, &fb_mv);
    esp_zb_custom_cluster_add_custom_attr(custom_cluster, ATTR_FEEDBACK_POS_ID, ESP_ZB_ZCL_ATTR_TYPE_U8, ro_access, &fb_pct);
    esp_zb_custom_cluster_add_custom_attr(custom_cluster, ATTR_CAL_LO_US_ID, ESP_ZB_ZCL_ATTR_TYPE_U16, ro_access, &cal_lo_us);
    esp_zb_custom_cluster_add_custom_attr(custom_cluster, ATTR_CAL_LO_MV_ID, ESP_ZB_ZCL_ATTR_TYPE_U16, ro_access, &cal_lo_mv);
    esp_zb_custom_cluster_add_custom_attr(custom_cluster, ATTR_CAL_HI_US_ID, ESP_ZB_ZCL_ATTR_TYPE_U16, ro_access, &cal_hi_us);
    esp_zb_custom_cluster_add_custom_attr(custom_cluster, ATTR_CAL_HI_MV_ID, ESP_ZB_ZCL_ATTR_TYPE_U16, ro_access, &cal_hi_mv);
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
    if (migrate || s_storage_erased) save_role();
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
        // The settings storage can't be used as it is. Erasing it is the only way on; the hub is
        // told through the "last reset reason", so lost settings are not a mystery.
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
        s_storage_erased = true;
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

    fb_init();
    xTaskCreate(move_task, "move_task", 4096, NULL, 5, &s_move_task);
    if (s_adc_ok) xTaskCreate(fb_task, "fb_task", 3072, NULL, 4, NULL);
    xTaskCreate(heartbeat_task, "heartbeat_task", 3072, NULL, 2, NULL);
    xTaskCreate(usb_task, "usb_task", 4096, NULL, 2, NULL);
    xTaskCreate(button_task, "button_task", 3072, NULL, 4, NULL);
    xTaskCreate(led_task, "led_task", 2048, NULL, 3, NULL);
    chip_temp_init();
    if (s_temp_sensor) xTaskCreate(chip_temp_task, "chip_temp_task", 3072, NULL, 2, NULL);
    xTaskCreate(esp_zb_task, "esp_zb_task", 8192, NULL, 5, NULL);
}
