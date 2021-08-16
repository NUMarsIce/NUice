#pragma once

#include "ros.h"

#if defined(STM32F4xx)
#include "STM32Hardware.h"
namespace ros
{
    typedef NodeHandle_<STM32Hardware> NUNodeHandle;
}
#else
namespace ros
{
    //NodeHandle for limmited space of arduino
    typedef NodeHandle_<ArduinoHardware, 6, 6, 150, 150> NUNodeHandle;
}
#endif
