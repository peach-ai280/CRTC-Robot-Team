/* ============================================================================
 * hal_mcu.cpp —— 硬件抽象层的「真机版」
 *
 * 这里是把 crtc:: 的抽象调用落到 STM32F103 的寄存器上。
 * 只在这个文件里 include 硬件头文件，其余 C++ 代码完全不碰寄存器，
 * 这样同一份逻辑换个文件就能在模拟器里跑。
 * ==========================================================================*/

#include "crtc.hpp"

extern "C" {
#include "main.h"
#include "board.h"
#include "bsp.h"
}

#include <stdio.h>
#include <stdarg.h>

namespace crtc {
namespace hal {

static const uint32_t kMotorCh[4] = {
    MOTOR_TIM_CH_LF, MOTOR_TIM_CH_RF, MOTOR_TIM_CH_LB, MOTOR_TIM_CH_RB
};
static const uint32_t kServoCh[4] = {
    SERVO_TIM_CH_GRIP, SERVO_TIM_CH_ARM, SERVO_TIM_CH_SCOOP, SERVO_TIM_CH_EXTRA
};
static const uint16_t kDirPin[4] = {
    MOTOR_LF_DIR_PIN, MOTOR_RF_DIR_PIN, MOTOR_LB_DIR_PIN, MOTOR_RB_DIR_PIN
};

void motorCompare(int idx, uint16_t cmp)
{
    if (idx < 0 || idx > 3) return;
    __HAL_TIM_SET_COMPARE(&MOTOR_PWM_TIM, kMotorCh[idx], cmp);
}

void motorDir(int idx, bool high)
{
    if (idx < 0 || idx > 3) return;
    HAL_GPIO_WritePin(MOTOR_DIR_PORT, kDirPin[idx],
                      high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void servoPulse(int ch, uint16_t us)
{
    if (ch < 0 || ch > 3) return;
    __HAL_TIM_SET_COMPARE(&SERVO_TIM, kServoCh[ch], us);
}

uint32_t millis(void) { return HAL_GetTick(); }

/* 轮询方式收一个字节，没有就返回 -1。
 * 不用中断是为了让示例尽量简单——控制周期 5ms，115200bps 下
 * 每 5ms 最多来 0.6 个字节，轮询完全够，不会丢。 */
int uartRead(void)
{
    USART_TypeDef *u = DBG_UART.Instance;
    if (u == 0) return -1;
    if (u->SR & (1UL << 5))            /* RXNE */
        return (int)(u->DR & 0xFFu);
    return -1;
}

bool limitHit(int idx)
{
    /* 微动开关接内部上拉，按下 = 低电平 */
    uint16_t pin = (idx == 0) ? LIMIT_L_PIN : LIMIT_R_PIN;
    return HAL_GPIO_ReadPin(LIMIT_L_PORT, pin) == GPIO_PIN_RESET;
}

/* 电池电压：ADC 单次转换 -> 电压 -> 乘分压比 3.0（20k + 10k） */
float vbat(void)
{
    HAL_ADC_Start(&VBAT_ADC);
    if (HAL_ADC_PollForConversion(&VBAT_ADC, 5) != HAL_OK) return 0.0f;
    uint32_t raw = HAL_ADC_GetValue(&VBAT_ADC);
    if (raw > 4095) raw = 4095;
    return (float)raw * (3.3f / 4095.0f) * 3.0f;
}

int print(const char *fmt, ...)
{
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) HAL_UART_Transmit(&DBG_UART, (uint8_t *)buf, (uint16_t)n, 100);
    return n;
}

} /* namespace hal */
} /* namespace crtc */
