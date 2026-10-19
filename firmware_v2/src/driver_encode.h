#ifndef DRIVER_ENCODE_H
#define DRIVER_ENCODE_H
#include <stdbool.h>
#include <stdint.h>
typedef struct { uint16_t compare; bool in2_high; } DriverOutput;
/* DRV8833 双输入方式，不是独立 PH/EN 接口。
 * IN1 接 PWM，IN2 接方向GPIO：00滑行，10正转，01反转，11制动。
 * 因此反转时 IN1 的高电平占空比必须取反。
 */
DriverOutput driver_encode(int16_t speed, int8_t sign, uint16_t period);
#endif