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
#include "can_driver.h"
#include "notification_led.h"

void (*handle_menu_buttons)(int);

static void twai_receive_task(void *arg)
{
    int previous_state = NONE;
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
            // Left Hand Switches
        case 0x2a0:
            switch (rx_msg.data[2]) // D2 Byte
            {
            case 0x00:
                previous_state = NONE;
                break;
            case 0x20: // Menu Up Button
                if (previous_state == UP || previous_state == LONG_UP)
                {
                    if (esp_timer_get_time() - prev_time > LONG_PRESS_TIME_SECONDS * 1000000 && previous_state != LONG_UP)
                    {
                        ESP_LOGI(CAN_TAG, "LONG UP");
                        handle_menu_buttons(LONG_UP); // Report to HA
                        previous_state = LONG_UP;
                    }
                }
                else
                {
                    previous_state = UP;
                    prev_time = esp_timer_get_time();
                    ESP_LOGI(CAN_TAG, "UP");
                    handle_menu_buttons(UP); // Report to HA
                }

                break;
            case 0x10: // Menu Down Button
                if (previous_state == DOWN || previous_state == LONG_DOWN)
                {
                    if (esp_timer_get_time() - prev_time > LONG_PRESS_TIME_SECONDS * 1000000 && previous_state != LONG_DOWN)
                    {
                        ESP_LOGI(CAN_TAG, "LONG DOWN");
                        handle_menu_buttons(LONG_DOWN); // Report to HA
                        previous_state = LONG_DOWN;
                    }
                }
                else
                {
                    prev_time = esp_timer_get_time();
                    previous_state = DOWN;
                    ESP_LOGI(CAN_TAG, "DOWN");
                    handle_menu_buttons(DOWN); // Report to HA
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

void configure_can(void (*handle_menu_buttons_callback)(int))
{
    // Install and start TWAI driver
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(CAN_TAG, "Driver installed");
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(CAN_TAG, "Driver started");
    xTaskCreate(twai_receive_task, "TWAI_rx", 4096, NULL, 1, NULL); // Start CAN RX task
    handle_menu_buttons = handle_menu_buttons_callback;
}