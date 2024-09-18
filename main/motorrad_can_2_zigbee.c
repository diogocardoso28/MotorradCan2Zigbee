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
//Example Configuration
#define NO_OF_ITERS                     3
#define RX_TASK_PRIO                    9

#define MAIN_TAG                     "MotorradCan2Zigbee"
#define CAN_TAG                      "CAN"

#define ID_MASTER_STOP_CMD              0x0A0
#define ID_MASTER_START_CMD             0x0A1
#define ID_MASTER_PING                  0x0A2
#define ID_SLAVE_STOP_RESP              0x0B0
#define ID_SLAVE_DATA                   0x0B1
#define ID_SLAVE_PING_RESP              0x0B2

static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
//Set TX queue length to 0 due to listen only mode
static const twai_general_config_t g_config = {.mode = TWAI_MODE_NORMAL,
                                               .tx_io = GPIO_NUM_3, .rx_io = GPIO_NUM_2,
                                               .clkout_io = TWAI_IO_UNUSED, .bus_off_io = TWAI_IO_UNUSED,
                                               .tx_queue_len = 5, .rx_queue_len = 5,
                                               .alerts_enabled = TWAI_ALERT_NONE,
                                               .clkout_divider = 0
                                              };

static SemaphoreHandle_t rx_sem;

/* --------------------------- Tasks and Functions -------------------------- */

static void twai_receive_task(void *arg)
{
    xSemaphoreTake(rx_sem, portMAX_DELAY);   
    int previous_state = 0;
    int prev_time = 0;      
    while (true)
    {
        twai_message_t rx_msg;
        twai_receive(&rx_msg, portMAX_DELAY);
        // ESP_LOG_BUFFER_HEX(CAN_TAG,rx_msg.data,8);
        //Decode Message
        // ESP_LOG_BUFFER_HEX(CAN_TAG,rx_msg.identifier,4);
        // ESP_LOG_BUFFER_CHAR(CAN_TAG, rx_msg.,1);
        // printf("%lx",rx_msg.identifier);
        switch (rx_msg.identifier)
        {
        case 0x2a0:
            /* code */
            switch (rx_msg.data[2])
            {
                case 0x00:
                // ESP_LOGI(CAN_TAG,"inactive");
                    previous_state=0;
                    break;
                case 0x20:
                    
                    if (previous_state == 1 )
                    {
                        if(esp_timer_get_time()-prev_time>2000000)
                            ESP_LOGI(CAN_TAG,"LONG UP"); else {

                    ESP_LOGI(CAN_TAG,"UP");    
                            }   
                    } else {
                        previous_state=1;
                        prev_time = esp_timer_get_time();
                    }

                    break;
                case 0x10:

                    if (previous_state == 2)
                    {
                        if(esp_timer_get_time()-prev_time >2000000)
                            ESP_LOGI(CAN_TAG,"LONG DOWN");    
                        else
                             ESP_LOGI(CAN_TAG,"DOWN");

                    } else {
                   
                        prev_time = esp_timer_get_time();
                        previous_state=2;
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

    //Install and start TWAI driver
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(MAIN_TAG, "Driver installed");
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(MAIN_TAG, "Driver started");

    // ESP_LOGI(MAIN_TAG, "Sending Message!");
    
    // // Configure message to transmit
    // twai_message_t message = {
    //     // Message type and format settings
    //     .extd = 1,              // Standard vs extended format
    //     .rtr = 0,               // Data vs RTR frame
    //     .ss = 0,                // Whether the message is single shot (i.e., does not repeat on error)
    //     .self = 0,              // Whether the message is a self reception request (loopback)
    //     .dlc_non_comp = 0,      // DLC is less than 8
    //     // Message ID and payload
    //     .identifier = 0xAAAA,
    //     .data_length_code = 4,
    //     .data = {0, 1, 2, 3},
    // };

    // //Queue message for transmission
    // if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
    //     ESP_LOGI(EXAMPLE_TAG,"Message queued for transmission\n");
    // } else {
    //     ESP_LOGI(EXAMPLE_TAG,"Failed to queue message for transmission\n");
    // }

    xSemaphoreGive(rx_sem);                     //Start RX task
    vTaskDelay(pdMS_TO_TICKS(100));
    xSemaphoreTake(rx_sem, portMAX_DELAY);      //Wait for RX task to complete

    //Stop and uninstall TWAI driver
    ESP_ERROR_CHECK(twai_stop());
    ESP_LOGI(MAIN_TAG, "Driver stopped");
    ESP_ERROR_CHECK(twai_driver_uninstall());
    ESP_LOGI(MAIN_TAG, "Driver uninstalled");

    //Cleanup
    vSemaphoreDelete(rx_sem);
}