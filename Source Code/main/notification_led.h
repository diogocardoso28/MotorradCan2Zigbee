#pragma once

#define LED_TAG "LED"

void configure_led(void);
void led_ok(void);
void led_zigbee_connecting(void);
void led_zigbee_error(void);
void led_zigbee_data_sent(void);
void led_can_data_rx(void);
