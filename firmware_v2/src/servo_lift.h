#ifndef SERVO_LIFT_H
#define SERVO_LIFT_H
#include <stdbool.h>
#include <stdint.h>

/* 车二的上下摆臂（挑杆）舵机：TIM2_CH3 / PA2，50Hz 标准舵机信号。
 *
 * 车二用：BOARD_SERVO_ENABLE=1（tools/build_v2.py --servo）
 * 车一用：BOARD_SERVO_ENABLE=0 → 下面全部编译成空函数，PA2 一个电平都不输出，
 *        车一的行为与加这个模块之前完全一致。
 */
void servo_lift_init(uint32_t now_ms);
void servo_lift_set_us(uint16_t pulse_us);
uint16_t servo_lift_get_us(void);
/* 按住升 / 按住降，松开保持；按钮信号由调用方从 PS2 帧里取。
 * 舵机只在收到有效 PS2 帧且未急停时动作，失联时保持当前角度不动。
 */
void servo_lift_step(bool up, bool down, uint32_t now_ms);

#endif
