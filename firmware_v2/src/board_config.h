#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H
#include <stdint.h>

/* 根据用户的 interface_baseline.json 保留 TIM4 和全部电机引脚。
 * 本工程直接访问寄存器，不需要 HAL 的 htim4 / hspi1 句柄。
 * STM32F103C8T6、DRV8833 红板：参赛手册 PDF 第22页物资表。
 */
#define BOARD_CORE_HZ          64000000u /* 使用内部 HSI/2*16，不依赖外部晶振。 */
#define BOARD_MOTOR_PERIOD     6400u     /* TIM4 64MHz/6400 = 10kHz。 */
#define BOARD_CONTROL_MS       10u
#define BOARD_MAIN_STALL_MS    100u
/* IWDG：主循环停滞后的第二层复位保护。
 * ⚠ 2026-10-19 实测本车这块 F103C8 关闭：
 *    写 IWDG_PR/RLR 后 IWDG_SR 的 PVU/RVU 永远不清零（手动写也一样），
 *    board_init() 卡在该等待循环 → 超时 return false → main 死循环，
 *    表现就是"上电后完全没反应"。先用 0 保证能跑，看门狗问题另行解决。
 */
#define BOARD_WATCHDOG_ENABLE  0
#define BOARD_LOCAL_STOP_ENABLE 0       /* 未确认 PB5 按键实物，默认不启用。 */

/* PS2：CS 来自用户基线，其余三根沿用旧工程 SPI1 的管脚。
 * 虽然管脚与 SPI1 相同，本程序采用 GPIO 模拟通信，LSB first，空闲 CLK=1。
 * CLK=PA5，DAT=PA6（接收器->STM32），CMD=PA7（STM32->接收器），CS=PB0。
 */
#define PS2_CLK_PIN 5u
#define PS2_DAT_PIN 6u
#define PS2_CMD_PIN 7u
#define PS2_CS_PIN  0u
#define PS2_HALF_BIT_US 4u

/* ===== 车二：上下摆动的挑杆舵机（TIM2_CH4 / PA3）=====
 * 引脚沿用 docs/硬件接线与焊接指南.md 的冻结表：PA0 夹爪 / PA1 臂升降 /
 * PA2 前铲 / PA3 备用·挑杆，四路都在 TIM2 上。车二没有前铲，用第 4 路挑杆。
 * 车一没有这个机构，保持 0：PA3 一个电平都不输出，车一与原固件行为一致。
 * 车二编译时加 -DBOARD_SERVO_ENABLE=1（tools/build_v2.py --servo）。
 * 机械装好后先空载试：把 MIN/MAX 收窄到机构真正的行程端点，别让舵机顶死。
 */
#ifndef BOARD_SERVO_ENABLE
#define BOARD_SERVO_ENABLE 0
#endif
#define BOARD_SERVO_PORT_PIN   3u       /* PA3 = TIM2_CH4 */
#define BOARD_SERVO_MIN_US     600u     /* 最低位（收车姿态） */
#define BOARD_SERVO_MAX_US     2400u    /* 最高位 */
#define BOARD_SERVO_INIT_US    1500u    /* 上电中位，别让机构一上电就甩到底 */
#define BOARD_SERVO_RATE_US_S  900u     /* 按住时每秒走 900us，全程约 2 秒 */

/* 电机顺序：LF左前，RF右前，LB左后，RB右后。
 * TIM4_CH1/PB6 -> LF IN1；PB12 -> LF IN2。
 * TIM4_CH2/PB7 -> RF IN1；PB13 -> RF IN2。
 * TIM4_CH3/PB8 -> LB IN1；PB14 -> LB IN2。
 * TIM4_CH4/PB9 -> RB IN1；PB15 -> RB IN2。
 * 板上 AIN1/AIN2 或 BIN1/BIN2 都是一个 H 桥的一对输入；每个车轮用一对。
 * nSLEEP 若模块引出，运行时需拉高；VM/GND 为驱动电源，不能接到信号脚。
 */
#define MOTOR_LF_SIGN 1
#define MOTOR_RF_SIGN 1
/* ★ 2026-10-19 实测：推"前进"时前两轮正常、后两轮反转。
 *   已用寄存器证实 TIM4_CCR1..4 与 PB12~15 四路完全同值，
 *   即单片机给四个驱动的信号一模一样 ⇒ 差异来自后轮马达线序/装配。
 *   这里用软件补偿：后两轮取反。等效于把后轮两根马达线对调。
 *   若以后重新焊线，记得把这两行改回 1，否则后轮又会反。 */
#define MOTOR_LB_SIGN -1
#define MOTOR_RB_SIGN -1
/* 上电架空测试：哪个轮子在“前进”时反转，就把它的 SIGN 从 1 改为 -1。
 * 这里只改变电机正负方向，不改变 GPIO 接线或麦轮解算公式。
 */
#if ((MOTOR_LF_SIGN != 1) && (MOTOR_LF_SIGN != -1)) || \
    ((MOTOR_RF_SIGN != 1) && (MOTOR_RF_SIGN != -1)) || \
    ((MOTOR_LB_SIGN != 1) && (MOTOR_LB_SIGN != -1)) || \
    ((MOTOR_RB_SIGN != 1) && (MOTOR_RB_SIGN != -1))
#error "Each MOTOR_xx_SIGN must be +1 or -1"
#endif

/* 基线中其余资源保持预留，本版不驱动舵机、蜂鸣器、ADC或限位机构。
 * TIM2 PA0..PA3 / TIM1_CH4 PA11：舵机；PB10/PB11：限位；PB1：电池ADC；
 * PA8：蜂鸣器；PB5：本地按键；PC13：状态LED（本版使用）。
 * 不修改 SWD 的 PA13/PA14，ST-Link 仍可连接。
 */
#endif