#include "driver_encode.h"
DriverOutput driver_encode(int16_t speed, int8_t sign, uint16_t period)
{
    DriverOutput output = {0u, false};
    int32_t value = speed;
    uint32_t magnitude, active;
    if (period == 0u || (sign != 1 && sign != -1)) return output;
    if (value > 1000) value = 1000;
    if (value < -1000) value = -1000;
    value *= sign;
    magnitude = (uint32_t)(value < 0 ? -value : value);
    active = (magnitude * period + 500u) / 1000u;
    if (active == 0u) return output;
    output.in2_high = value < 0;
    output.compare = (uint16_t)(value < 0 ? (uint32_t)period - active : active);
    return output;
}