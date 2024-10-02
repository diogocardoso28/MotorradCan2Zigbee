#pragma once

#define CAN_TAG "CAN"
#define LONG_PRESS_TIME_SECONDS 2

void configure_can();

// Data Structures Declaration

enum ButtonActions
{
    UP,
    LONG_UP,
    DOWN,
    LONG_DOWN,
    NONE,
};