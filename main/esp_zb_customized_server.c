#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_zigbee_core.h"
#include "ha/esp_zigbee_ha_standard.h"

#define TAG "ESP_SERVO_BLIND"

// -- HARDWARE CONFIG --
#define BUTTON_PIN GPIO_NUM_9
#define SERVO_PIN  GPIO_NUM_2
#define LED_PIN    GPIO_NUM_15

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          SERVO_PIN
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_14_BIT 
#define LEDC_FREQUENCY          50

// -- ZIGBEE IDENTITY --
#define ENDPOINT 1

#define CUSTOM_CLUSTER_ID 0xFF00
#define ATTR_SERVO_MIN_US_ID     0x0001
#define ATTR_SERVO_MAX_US_ID     0x0002
#define ATTR_MOVE_SPEED_MS_ID    0x0003
#define ATTR_IDLE_TIMEOUT_MS_ID  0x0004

#define NVS_NAMESPACE "storage"

#define ESP_ZB_ZED_CONFIG()                                         \
    {                                                               \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,                       \
        .install_code_policy = false,                               \
        .nwk_cfg.zed_cfg = {                                        \
            .ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_64MIN,            \
            .keep_alive = 3000,                                     \
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
    bool is_known;
} blind_config_t;

static blind_config_t s_cfg = {
    .servo_min_us = 1000,
    .servo_max_us = 2000,
    .move_speed_ms = 20,
    .idle_timeout_ms = 1000,
    
    .current_percent = 50,
    .target_percent = 50,
    .is_known = false
};

static SemaphoreHandle_t s_cfg_mutex;
static bool s_network_joined = false;
static bool s_identify_active = false;

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

static void load_config_from_nvs(void) {
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
        xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
        nvs_get_u16(handle, "min_us", &s_cfg.servo_min_us);
        nvs_get_u16(handle, "max_us", &s_cfg.servo_max_us);
        nvs_get_u16(handle, "spd_ms", &s_cfg.move_speed_ms);
        nvs_get_u16(handle, "idto_ms", &s_cfg.idle_timeout_ms);
        xSemaphoreGive(s_cfg_mutex);
        nvs_close(handle);
    }
}

// -- NATIVE IDENTIFY CALLBACK --
static void identify_cb(uint8_t identify_on) {
    if (identify_on) {
        ESP_LOGI(TAG, "Identify Triggered! LED Blinking...");
        s_identify_active = true;
    } else {
        ESP_LOGI(TAG, "Identify Stopped.");
        s_identify_active = false;
    }
}

// -- STATUS LED TASK --
static void led_task(void *pvParameters) {
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    while (1) {
        if (s_identify_active) {
            gpio_set_level(LED_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(LED_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        } else if (!s_network_joined) {
            gpio_set_level(LED_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(LED_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(1000));
        } else {
            gpio_set_level(LED_PIN, 0); 
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

// -- FREERTOS MOVEMENT TASK --
static void move_task(void *pvParameters) {
    uint32_t idle_timer = 0;
    while (1) {
        xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
        bool known = s_cfg.is_known;
        uint8_t cur = s_cfg.current_percent;
        uint8_t tgt = s_cfg.target_percent;
        uint16_t spd = s_cfg.move_speed_ms;
        uint16_t max_us = s_cfg.servo_max_us;
        uint16_t min_us = s_cfg.servo_min_us;
        uint16_t idle_to = s_cfg.idle_timeout_ms;
        xSemaphoreGive(s_cfg_mutex);

        if (!known) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (cur != tgt) {
            if (cur < tgt) cur++;
            else cur--;
            
            xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
            s_cfg.current_percent = cur;
            xSemaphoreGive(s_cfg_mutex);

            uint32_t current_us = min_us + ((max_us - min_us) * cur) / 100;
            uint32_t duty = (current_us * 16383) / 20000;
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            
            idle_timer = 0;
            
            if (cur == tgt || cur % 5 == 0) {
                esp_zb_lock_acquire(portMAX_DELAY);
                // ALWAYS report standard 'cur' back so hub UI doesn't visually break
                esp_zb_zcl_set_attribute_val(ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID, &cur, false);
                esp_zb_lock_release();
            }

            if (spd > 0) vTaskDelay(pdMS_TO_TICKS(spd));
        } else {
            if (idle_to > 0 && idle_timer < idle_to) {
                vTaskDelay(pdMS_TO_TICKS(10));
                idle_timer += 10;
                if (idle_timer >= idle_to) {
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0); 
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                }
            } else {
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        }
    }
}

// -- ZIGBEE CALLBACK HANDLERS --
static esp_err_t cb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message) {
    ESP_LOGI(TAG, "=> ZIGBEE ACTION CALLBACK TRIGGERED: ID %d", callback_id);

    if (callback_id == ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID) {
        esp_zb_zcl_set_attr_value_message_t *msg = (esp_zb_zcl_set_attr_value_message_t *)message;
        
        ESP_LOGW(TAG, "   [SET_ATTR RECVD] Cluster: 0x%04x | Attr: 0x%04x", msg->info.cluster, msg->attribute.id);

        if (msg->info.cluster == CUSTOM_CLUSTER_ID) {
            xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
            if (msg->attribute.id == ATTR_SERVO_MIN_US_ID) s_cfg.servo_min_us = *(uint16_t*)msg->attribute.data.value;
            else if (msg->attribute.id == ATTR_SERVO_MAX_US_ID) s_cfg.servo_max_us = *(uint16_t*)msg->attribute.data.value;
            else if (msg->attribute.id == ATTR_MOVE_SPEED_MS_ID) s_cfg.move_speed_ms = *(uint16_t*)msg->attribute.data.value;
            else if (msg->attribute.id == ATTR_IDLE_TIMEOUT_MS_ID) s_cfg.idle_timeout_ms = *(uint16_t*)msg->attribute.data.value;
            xSemaphoreGive(s_cfg_mutex);
            save_config_to_nvs();
            ESP_LOGI(TAG, "   => Custom NVS Setting Updated!");
        }
    } else if (callback_id == ESP_ZB_CORE_WINDOW_COVERING_MOVEMENT_CB_ID) {
        esp_zb_zcl_window_covering_movement_message_t *msg = (esp_zb_zcl_window_covering_movement_message_t *)message;
        
        ESP_LOGW(TAG, "   [WINDOW COVERING CMD RECVD] Cmd ID: 0x%02x", msg->command);

        xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
        uint8_t cmd_id = msg->command;
        
        if (cmd_id == 0x00) { s_cfg.target_percent = 0; s_cfg.is_known = true; } // Up / Open
        else if (cmd_id == 0x01) { s_cfg.target_percent = 100; s_cfg.is_known = true; } // Down / Close 
        else if (cmd_id == 0x02) { s_cfg.target_percent = s_cfg.current_percent; s_cfg.is_known = true; } // Stop
        else if (cmd_id == 0x05) { // GoToLiftPercentage
            s_cfg.target_percent = msg->payload.percentage_lift_value;
            if (s_cfg.target_percent > 100) s_cfg.target_percent = 100;
            s_cfg.is_known = true;
        }
        xSemaphoreGive(s_cfg_mutex);
        
        ESP_LOGI(TAG, "   => Action parsed. Servo Target updated to %d%%", s_cfg.target_percent);
        return ESP_OK; 
    }
    return ESP_OK;
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask) {
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    static uint32_t backoff_delay = 1000;

    switch (sig_type) {
        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
            break;
        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
            if (err_status == ESP_OK) {
                if (esp_zb_bdb_is_factory_new()) esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
                else s_network_joined = true;
            }
            break;
        case ESP_ZB_BDB_SIGNAL_STEERING:
            if (err_status == ESP_OK) {
                s_network_joined = true; 
                backoff_delay = 1000; 
            } else {
                s_network_joined = false;
                esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, backoff_delay);
                backoff_delay *= 2;
                if (backoff_delay > 60000) backoff_delay = 60000; 
            }
            break;
        default: break;
    }
}

// -- BUTTON MONITORING TASK --
static void button_task(void *pvParameters) {
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_PIN, GPIO_PULLUP_ONLY);

    while(1) {
        if (gpio_get_level(BUTTON_PIN) == 0) { 
            uint32_t press_time = 0;
            while(gpio_get_level(BUTTON_PIN) == 0) {
                vTaskDelay(pdMS_TO_TICKS(100));
                press_time += 100;
            }
            if (press_time >= 3000) {
                esp_zb_factory_reset();
            } else if (press_time >= 50) {
                s_network_joined = false;
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// -- ZIGBEE INITIALIZATION TASK --
static void esp_zb_task(void *pvParameters) {
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);
    
    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();
    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
    
    // Core Basic Setup (FIRMWARE ID UPDATE)
    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
    uint8_t zcl_ver = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE, app_ver = 1, stack_ver = 1, hw_ver = 1, pwr_src = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE;
    
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &zcl_ver);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_APPLICATION_VERSION_ID, &app_ver);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_STACK_VERSION_ID, &stack_ver);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_HW_VERSION_ID, &hw_ver);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &pwr_src);
    
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *)"\x08""erkin.io");
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (void *)"\x0D""espServoBlind");
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_PRODUCT_URL_ID, (void *)"\x29""https://github.com/erkin-io/espServoBlind");
    
    // Natively Populates "Firmware Version: 1.0.0" on Zigbee2MQTT Dashboard instead of Unknown
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_DATE_CODE_ID, (void *)"\x05""1.0.0");
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_SW_BUILD_ID, (void *)"\x05""1.0.0");
    
    esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    
    // Core Identify Setup
    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
    uint16_t identify_time = 0;
    esp_zb_identify_cluster_add_attr(esp_zb_identify_cluster, ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID, &identify_time);
    esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_attribute_list_t *esp_zb_wc_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_WINDOW_COVERING);
    uint8_t wc_type = 0, current_lift = 50;  
    esp_zb_window_covering_cluster_add_attr(esp_zb_wc_cluster, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_WINDOW_COVERING_TYPE_ID, &wc_type);
    esp_zb_window_covering_cluster_add_attr(esp_zb_wc_cluster, ESP_ZB_ZCL_ATTR_WINDOW_COVERING_CURRENT_POSITION_LIFT_PERCENTAGE_ID, &current_lift);
    esp_zb_cluster_list_add_window_covering_cluster(esp_zb_cluster_list, esp_zb_wc_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_ota_cluster_cfg_t ota_cluster_cfg = {
        .ota_upgrade_downloaded_file_ver = 1,
        .ota_upgrade_manufacturer = 0x100B,
        .ota_upgrade_image_type = 0x0100,
    };
    esp_zb_attribute_list_t *esp_zb_ota_client_cluster = esp_zb_ota_cluster_create(&ota_cluster_cfg);
    esp_zb_cluster_list_add_ota_cluster(esp_zb_cluster_list, esp_zb_ota_client_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

    // Our Custom Boundary Configuration Setup
    esp_zb_attribute_list_t *custom_cluster = esp_zb_zcl_attr_list_create(CUSTOM_CLUSTER_ID);
    xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
    uint16_t v1 = s_cfg.servo_min_us, v2 = s_cfg.servo_max_us, v3 = s_cfg.move_speed_ms, v4 = s_cfg.idle_timeout_ms;
    xSemaphoreGive(s_cfg_mutex);
    
    esp_zb_custom_cluster_add_custom_attr(custom_cluster, ATTR_SERVO_MIN_US_ID, ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &v1);
    esp_zb_custom_cluster_add_custom_attr(custom_cluster, ATTR_SERVO_MAX_US_ID, ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &v2);
    esp_zb_custom_cluster_add_custom_attr(custom_cluster, ATTR_MOVE_SPEED_MS_ID, ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &v3);
    esp_zb_custom_cluster_add_custom_attr(custom_cluster, ATTR_IDLE_TIMEOUT_MS_ID, ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &v4);
    
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
    
    esp_zb_set_trace_level_mask(ESP_ZB_TRACE_LEVEL_VERBOSE, ESP_ZB_TRACE_SUBSYSTEM_ZCL);
    
    ESP_ERROR_CHECK(esp_zb_start(false));
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
    
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    
    s_cfg_mutex = xSemaphoreCreateMutex();
    load_config_from_nvs();

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

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

    xTaskCreate(move_task, "move_task", 4096, NULL, 5, NULL);
    xTaskCreate(button_task, "button_task", 2048, NULL, 5, NULL);
    xTaskCreate(led_task, "led_task", 2048, NULL, 5, NULL);
    xTaskCreate(esp_zb_task, "esp_zb_task", 4096, NULL, 5, NULL);
}
