#include "notification_led.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "led_strip.h"

static led_strip_handle_t led_strip;
TaskHandle_t xHandle = NULL;

void kill_task(void)
{
    if (xHandle != NULL)
    {
        vTaskDelete(xHandle);
        xHandle = NULL;
    }
}

void configure_led(void)
{
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = GPIO_NUM_8,
        .max_leds = 1, // at least one LED on board
    };

    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
    led_strip_clear(led_strip);
    ESP_EARLY_LOGI(LED_TAG, "Led Configured Succesfully");
}

void led_ok(void)
{
    kill_task();
    // START LED
    led_strip_set_pixel(led_strip, 0, 0, 255, 0);
    /* Refresh the strip to send data */
    led_strip_refresh(led_strip);
}

void led_blink_yellow()
{
    while (1)
    {

        led_strip_set_pixel(led_strip, 0, 255, 255, 0);
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(500));
        led_strip_set_pixel(led_strip, 0, 0, 0, 0);
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void led_zigbee_error(void)
{
    kill_task();
    led_strip_set_pixel(led_strip, 0, 255, 0, 0);
    led_strip_refresh(led_strip);
}

void led_zigbee_connecting(void)
{

    xTaskCreate(led_blink_yellow, "LED", 1024, NULL, 1, &xHandle);
}