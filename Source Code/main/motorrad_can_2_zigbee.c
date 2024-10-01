/*
 * SPDX-FileCopyrightText: 2010-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/*
 * The following example demonstrates a Listen Only node in a TWAI network. The
 * Listen Only node will not take part in any TWAI bus activity (no acknowledgments
 * and no error frames). This example will execute multiple iterations, with each
 * iteration the Listen Only node will do the following:
 * 1) Listen for ping and ping response
 * 2) Listen for start command
 * 3) Listen for data messages
 * 4) Listen for stop and stop response
 */
#include <stdio.h>
#include <stdlib.h>
#include "motorrad_can_2_zigbee.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_check.h"
#include "notification_led.h"

/*Zigbee Stuff*/
#include "ha/esp_zigbee_ha_standard.h"
#include "switch_driver.h"

/* --------------------- Definitions and static variables ------------------ */
#define MAIN_TAG "MotorradCan2Zigbee"
#define CAN_TAG "CAN"
#define ZIGBEE_TAG "ZIGBEE"
#define LONG_PRESS_TIME_SECONDS 2
void process_message(twai_message_t rx_msg);
/* --------------------- Zigbee Stuff --------------------- */
int prev_action = NONE;

static switch_func_pair_t button_func_pair[] = {
    {GPIO_INPUT_IO_TOGGLE_SWITCH, SWITCH_ONOFF_TOGGLE_CONTROL}};

static void report_button_action()
{
    /* Send report attributes command */
    esp_zb_zcl_report_attr_cmd_t report_attr_cmd;
    report_attr_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
    report_attr_cmd.attributeID = ESP_ZB_ZCL_ATTR_MULTI_VALUE_PRESENT_VALUE_ID;
    report_attr_cmd.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
    report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_MULTI_VALUE;
    report_attr_cmd.zcl_basic_cmd.src_endpoint = HA_ESP_ENDPOINT_BUTTONS;

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd);
    esp_zb_lock_release();
    ESP_EARLY_LOGI(ZIGBEE_TAG, "Send 'report attributes' command");
    led_zigbee_data_sent();
}

static void esp_app_buttons_handler(switch_func_pair_t *button_func_pair)
{
    if (button_func_pair->func == SWITCH_ONOFF_TOGGLE_CONTROL)
    {
        ESP_EARLY_LOGI(ZIGBEE_TAG, "BRN PRESSED!");
        esp_app_temp_sensor_handler(LONG_UP);
    }
}

static void esp_app_temp_sensor_handler(short int state)
{
    ESP_EARLY_LOGI(ZIGBEE_TAG, "Sending value");

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(HA_ESP_ENDPOINT_BUTTONS,
                                 ESP_ZB_ZCL_CLUSTER_ID_MULTI_VALUE, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_MULTI_VALUE_PRESENT_VALUE_ID, &state, false);
    esp_zb_lock_release();
    if (state != prev_action)
    {
        // Report Change
        report_button_action();
        prev_action = state;
    }
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , ZIGBEE_TAG, "Failed to start Zigbee bdb commissioning");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type)
    {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(ZIGBEE_TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK)
        {
            // ESP_LOGI(ZIGBEE_TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");´
            switch_driver_init(button_func_pair, PAIR_SIZE(button_func_pair), esp_app_buttons_handler);
            ESP_LOGI(ZIGBEE_TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
            if (esp_zb_bdb_is_factory_new())
            {
                ESP_LOGI(ZIGBEE_TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            }
            else
            {
                ESP_LOGI(ZIGBEE_TAG, "Device rebooted");
                led_ok();
            }
        }
        else
        {
            /* commissioning failed */
            ESP_LOGW(ZIGBEE_TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
            led_zigbee_error();
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK)
        {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(ZIGBEE_TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            led_ok();
        }
        else
        {
            ESP_LOGI(ZIGBEE_TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    default:
        ESP_LOGI(ZIGBEE_TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

static esp_zb_cluster_list_t *custom_actions_clusters_create(esp_zb_multistate_value_cluster_cfg_t *actions)
{

    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_basic_cluster_cfg_t basic_cfg =
        {
            .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
            .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE,
        };
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&basic_cfg);

    esp_zb_identify_cluster_cfg_t identify_cfg = {
        .identify_time = ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE,
    };

    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, MANUFACTURER_NAME));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, MODEL_IDENTIFIER));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(&identify_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_multistate_value_cluster(cluster_list, esp_zb_multistate_value_cluster_create(&actions), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    // Enable reporting on current value cluster
    esp_zb_attribute_list_t *multi_value_cluster =
        esp_zb_cluster_list_get_cluster(cluster_list, ESP_ZB_ZCL_CLUSTER_ID_MULTI_VALUE, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_attribute_list_t *attr = multi_value_cluster;

    while (attr)
    {
        if (attr->attribute.id == ESP_ZB_ZCL_ATTR_MULTI_VALUE_PRESENT_VALUE_ID)
        {
            attr->attribute.access = multi_value_cluster->attribute.access | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING;
            break;
        }
        attr = attr->next;
    }
    return cluster_list;
}

static esp_zb_ep_list_t *custom_actions_ep_create(uint8_t endpoint_id, esp_zb_multistate_value_cluster_cfg_t *actions)
{
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = endpoint_id,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
        .app_device_version = 0};
    esp_zb_ep_list_add_ep(ep_list, custom_actions_clusters_create(actions), endpoint_config);
    return ep_list;
}

static void esp_zb_task(void *pvParameters)
{
    led_zigbee_connecting();
    /* Initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);
    esp_zb_secur_network_min_join_lqi_set(0);
    esp_zb_multistate_value_cluster_cfg_t actions_cfg = {
        .number_of_states = 4,
        .out_of_service = false,
        .present_value = 0,
        .status_flags = ESP_ZB_ZCL_STATUS_SUCCESS,
    };

    esp_zb_ep_list_t *esp_zb_sensor_ep = custom_actions_ep_create(HA_ESP_ENDPOINT_BUTTONS, &actions_cfg);

    /* Register the device */
    esp_zb_device_register(esp_zb_sensor_ep);

    /* Config the reporting info  */
    esp_zb_zcl_reporting_info_t reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_ENDPOINT_BUTTONS,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_MULTI_VALUE, // Chanigng this kills it
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 1,
        .u.send_info.max_interval = 0,
        .u.send_info.def_min_interval = 1,
        .u.send_info.def_max_interval = 0,
        .u.send_info.delta.u16 = 100,
        .attr_id = ESP_ZB_ZCL_ATTR_MULTI_VALUE_PRESENT_VALUE_ID, // This does work
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };

    esp_zb_zcl_update_reporting_info(&reporting_info);
    ESP_EARLY_LOGI(ZIGBEE_TAG, "Configured Reporting info");

    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));

    esp_zb_stack_main_loop();
}

/* --------------------- Canbus Stuff --------------------- */

static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
// Set TX queue length to 0 due to listen only mode
static const twai_general_config_t g_config = {.mode = TWAI_MODE_NORMAL,
                                               .tx_io = GPIO_NUM_3,
                                               .rx_io = GPIO_NUM_2,
                                               .clkout_io = TWAI_IO_UNUSED,
                                               .bus_off_io = TWAI_IO_UNUSED,
                                               .tx_queue_len = 0,
                                               .rx_queue_len = 5,
                                               .alerts_enabled = TWAI_ALERT_NONE,
                                               .clkout_divider = 0};

static void twai_receive_task(void *arg)
{
    int previous_state = 0;
    int prev_time = 0;
    ESP_LOGI(CAN_TAG, "Ready to receive messages.");
    while (true)
    {
        twai_message_t rx_msg;
        twai_receive(&rx_msg, portMAX_DELAY);
        led_can_data_rx();
        // TODO: Make this more readable
        switch (rx_msg.identifier)
        {
        case 0x2a0:
            /* code */
            switch (rx_msg.data[2])
            {
            case 0x00:
                previous_state = 0;
                prev_action = NONE;
                // esp_app_temp_sensor_handler(NONE); // Report to HA
                break;
            case 0x20:
                if (previous_state == 1)
                {
                    if (esp_timer_get_time() - prev_time > LONG_PRESS_TIME_SECONDS * 1000000)
                    {
                        ESP_LOGI(CAN_TAG, "LONG UP");
                        esp_app_temp_sensor_handler(LONG_UP); // Report to HA
                    }
                    else
                    {
                        ESP_LOGI(CAN_TAG, "UP");
                        esp_app_temp_sensor_handler(UP); // Report to HA
                    }
                }
                else
                {
                    previous_state = 1;
                    prev_time = esp_timer_get_time();
                }

                break;
            case 0x10:
                if (previous_state == 2)
                {
                    if (esp_timer_get_time() - prev_time > LONG_PRESS_TIME_SECONDS * 1000000)
                    {
                        ESP_LOGI(CAN_TAG, "LONG DOWN");
                        esp_app_temp_sensor_handler(LONG_DOWN); // Report to HA
                    }

                    else
                    {
                        ESP_LOGI(CAN_TAG, "DOWN");
                        esp_app_temp_sensor_handler(DOWN); // Report to HA
                    }
                }
                else
                {

                    prev_time = esp_timer_get_time();
                    previous_state = 2;
                }
                break;
            default:
                break;
            }
        default:
            break;
        }
    }

    vTaskDelete(NULL);
}

/* --------------------- End Of Canbus Stuff --------------------- */

void app_main(void)
{
    configure_led();
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Install and start TWAI driver
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(CAN_TAG, "Driver installed");
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(CAN_TAG, "Driver started");
    xTaskCreate(twai_receive_task, "TWAI_rx", 4096, NULL, 1, NULL); // Start CAN RX task

    // Start Zigbee drivers
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL); // Start Zigbee stack task

    ESP_LOGI(MAIN_TAG, "Device ready!");
    vTaskDelay(pdMS_TO_TICKS(100));
}