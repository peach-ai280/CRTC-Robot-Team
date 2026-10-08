#ifndef __APP_H
#define __APP_H

#include "main.h"

/* ============================================================================
 * app.h —— 应用层总调度（唯一需要被 main.c 调用的两个函数）
 *
 * 之所以把逻辑全部放在 app.c 而尽量少动 main.c，是因为 CubeMX 每次生成代码
 * 都会重写 main.c。放在 USER CODE BEGIN 保护块里虽然安全，但新手最容易在这里
 * 踩坑。所以 main.c 里只保留两行：
 *     App_Init();               // 在 while(1) 之前
 *     while (1) { App_Tick(); } // 在 while(1) 里
 * ==========================================================================*/

void App_Init(void);
void App_Tick(void);      /* 非阻塞，内部按 config.h 的周期分频 */

/* 串口收到一个字节时喂给应用层（视觉帧解析用）。
   由 main.c 的 HAL_UART_RxCpltCallback 调用。 */
void App_UartRxCallback(UART_HandleTypeDef *huart, uint8_t byte);

#endif /* __APP_H */
