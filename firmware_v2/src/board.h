#ifndef BOARD_H
#define BOARD_H
#include <stdbool.h>
#include <stdint.h>
bool board_init(void);
uint32_t board_millis(void);
void board_delay_us(uint32_t microseconds);
void board_motors_apply(const int16_t wheel[4]);
void board_motors_stop(void);
bool board_local_stop(void);
void board_led(bool on);
void board_watchdog_feed(void);
void SysTick_Handler(void);
#endif