#pragma once

#include "ros.h"

#if defined(STM32F4xx)
#include "STM32Hardware.h"
namespace ros
{
    //NodeHandle for STM32s with tons of space
    typedef NodeHandle_<STM32Hardware, 40, 40> NUNodeHandle;
}
#else
namespace ros
{
    //NodeHandle for limmited space of arduino
    typedef NodeHandle_<ArduinoHardware, 5, 5, 128, 128> NUNodeHandle;
}
#endif
