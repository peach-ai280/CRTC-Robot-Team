/* ============================================================================
 * main.c —— CRTC2026 完整固件入口
 *
 * 这个文件只做四件事，别的东西都在各自模块里：
 *   1. HAL_Init + 系统时钟        -> bsp.c
 *   2. 外设初始化                  -> bsp.c
 *   3. App_Init()                  -> app.c
 *   4. while(1) { App_Tick(); }    -> app.c
 *
 * 【两个上电小功能】
 *   · 按住板载按键 PB5 再上电 -> 舵机中位标定模式（四个舵机锁定 90°，装舵盘用）
 *   · 上电串口会打印 SYSCLK，若不是 72000000 说明晶振没起振
 * ==========================================================================*/

#include "main.h"
#include "bsp.h"
#include "board.h"
#include "config.h"
#include "app.h"
#include "servo.h"
#include <stdio.h>

/* ============================================================================
 * 串口接收回调 —— 视觉数据入口
 *   每收到 1 字节 -> 喂给 vision.c -> 立刻再开下一次接收（这句不能漏）
 *   USE_VISION=0 时那些字节被直接丢弃，不影响任何东西。
 * ==========================================================================*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        App_UartRxCallback(huart, (uint8_t)g_uart1_rx);
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&g_uart1_rx, 1);
    }
}

/* ============================================================================
 * 舵机中位标定模式
 *   装舵盘时必须知道机械中位在哪。按住 PB5 上电，四个舵机停在 90°，
 *   这时装舵盘/摆杆最准。装好后断电重启即恢复正常。
 * ==========================================================================*/
static void servo_calib_mode(void)
{
    printf("\r\n[SERVO CALIB] all servos forced to 90 deg.\r\n");
    printf("[SERVO CALIB] install horns now, then power-cycle.\r\n");

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1500);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1500);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1500);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 1500);

    while (1)
    {
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        HAL_Delay(100);
    }
}

/* ========================================================================== */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    BSP_InitAll();

    printf("\r\n==== CRTC2026 firmware ====\r\n");
    printf("SYSCLK = %lu Hz\r\n", SystemCoreClock);
    if (SystemCoreClock != 72000000UL)
        printf("[WARN] HSE failed, running on HSI. Fix the crystal!\r\n");

    /* 等电源稳定：电池接上时 5V 降压有上升沿，立刻动舵机会掉压复位 */
    HAL_Delay(200);

    if (HAL_GPIO_ReadPin(KEY_PORT, KEY_PIN) == GPIO_PIN_RESET)
        servo_calib_mode();

    App_Init();

    while (1)
    {
        App_Tick();
    }
}
