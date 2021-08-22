#pragma once

#include "ros.h"

#if defined(STM32F4xx)
#include "STM32Hardware.h"
namespace ros
{
    typedef NodeHandle_<STM32Hardware, 32, 32> NUNodeHandle;
}
#else
namespace ros
{
    //NodeHandle for limmited space of arduino
    typedef NodeHandle_<ArduinoHardware, 1, 5, 128, 128> NUNodeHandle;
}
#endif
