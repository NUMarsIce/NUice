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
typedef NodeHandle_<ArduinoHardware> NUNodeHandle;
}
#endif