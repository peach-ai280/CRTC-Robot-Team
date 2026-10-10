#ifndef __MAIN_H
#define __MAIN_H

/* ============================================================================
 * main.h —— 全工程唯一的「HAL + 句柄」入口
 *
 * 为什么单独建这个头文件：
 *   每个模块的 .h 都 #include "main.h"，这样它们不用各自去 include HAL，
 *   也不会出现「某个模块换了引脚就要改十个文件」的情况。
 *   CubeIDE 新建工程时会自动生成一个 main.h —— 用本文件覆盖它即可。
 * ==========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * 两套底层，二选一：
 *   默认：用自带的 hal_shim.h（纯寄存器实现，不依赖官方库，装上编译器就能编）
 *   如果在 CubeIDE 里已经生成了官方 HAL，编译时加上 -DUSE_ST_HAL 切过去，
 *   业务代码（motor.c / servo.c / ps2.c ...）一个字都不用改。
 * -------------------------------------------------------------------------*/
#ifdef USE_ST_HAL
  #include "stm32f1xx_hal.h"
#else
  #include "hal_shim.h"
#endif

/* --- 外设句柄（定义在 main.c）------------------------------------------- */
extern TIM_HandleTypeDef  htim1;    /* ★ 第 5 路舵机 PWM，50Hz（PA11 = TIM1_CH4，回转 SG90） */
extern TIM_HandleTypeDef  htim2;    /* 舵机 PWM，50Hz（PA0~PA3） */
extern TIM_HandleTypeDef  htim4;    /* 电机 PWM，10kHz（PB6~PB9） */
extern SPI_HandleTypeDef  hspi1;    /* PS2 手柄（PA5/6/7） */
extern UART_HandleTypeDef huart1;   /* 调试串口 + 视觉模块（PA9/PA10, 115200） */
extern ADC_HandleTypeDef  hadc1;    /* 电池电压（PB1, ADC1_IN9） */

/* 串口接收单字节缓冲：中断里收到一个字节就喂给视觉状态机 */
extern volatile uint8_t g_uart1_rx;

void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
