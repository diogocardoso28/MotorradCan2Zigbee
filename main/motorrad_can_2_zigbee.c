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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "esp_timer.h"

/* --------------------- Definitions and static variables ------------------ */
#define RX_TASK_PRIO 9

#define MAIN_TAG "MotorradCan2Zigbee"
#define CAN_TAG "CAN"

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

static SemaphoreHandle_t rx_sem;

/* --------------------------- Tasks and Functions -------------------------- */

static void twai_receive_task(void *arg)
{
    xSemaphoreTake(rx_sem, portMAX_DELAY);
    int previous_state = 0;
    int prev_time = 0;
    ESP_LOGI(CAN_TAG, "Ready to receive messages.");
    while (true)
    {
        twai_message_t rx_msg;
        twai_receive(&rx_msg, portMAX_DELAY);
        // TODO: Make this more readable
        switch (rx_msg.identifier)
        {
        case 0x2a0:
            /* code */
            switch (rx_msg.data[2])
            {
            case 0x00:
                previous_state = 0;
                break;
            case 0x20:
                if (previous_state == 1)
                {
                    if (esp_timer_get_time() - prev_time > 2000000)
                        ESP_LOGI(CAN_TAG, "LONG UP");
                    else
                    {

                        ESP_LOGI(CAN_TAG, "UP");
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
                    if (esp_timer_get_time() - prev_time > 2000000)
                        ESP_LOGI(CAN_TAG, "LONG DOWN");
                    else
                        ESP_LOGI(CAN_TAG, "DOWN");
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

    xSemaphoreGive(rx_sem);
    vTaskDelete(NULL);
}

void app_main(void)
{
    rx_sem = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, RX_TASK_PRIO, NULL, tskNO_AFFINITY);

    // Install and start TWAI driver
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(CAN_TAG, "Driver installed");
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(CAN_TAG, "Driver started");
    ESP_LOGI(MAIN_TAG, "Device ready!");
    xSemaphoreGive(rx_sem); // Start RX task
    vTaskDelay(pdMS_TO_TICKS(100));
    xSemaphoreTake(rx_sem, portMAX_DELAY); // Wait for RX task to complete

    // Stop and uninstall TWAI driver
    ESP_ERROR_CHECK(twai_stop());
    ESP_LOGI(CAN_TAG, "Driver stopped");
    ESP_ERROR_CHECK(twai_driver_uninstall());
    ESP_LOGI(CAN_TAG, "Driver uninstalled");

    // Cleanup
    vSemaphoreDelete(rx_sem);
}