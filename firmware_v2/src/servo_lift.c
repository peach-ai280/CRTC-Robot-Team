#include "servo_lift.h"
#include "board_config.h"
#include "stm32f103_min.h"

#if BOARD_SERVO_ENABLE

/* 当前脉宽（微秒）。600~2400us 对应舵机约 0°~180°，
 * 具体机械行程靠装车时调 BOARD_SERVO_MIN_US / MAX_US 收窄，别让机构撞死。 */
static uint16_t g_us = BOARD_SERVO_INIT_US;
static uint32_t g_last_ms;

static void gpio_mode_local(GpioRegs *port, unsigned pin, uint32_t mode)
{
    volatile uint32_t *control = pin < 8u ? &port->CRL : &port->CRH;
    unsigned shift = (pin & 7u) * 4u;
    *control = (*control & ~(15u << shift)) | (mode << shift);
}

void servo_lift_init(uint32_t now_ms)
{
    RCC_APB2ENR |= 1u << 2;   /* IOPA 时钟 */
    RCC_APB1ENR |= 1u << 0;   /* TIM2 时钟 */
    /* PA2 = TIM2_CH3：复用推挽输出 50MHz（CNF=10, MODE=11 → 0xB）。 */
    gpio_mode_local(GPIOA, BOARD_SERVO_PORT_PIN, 0xbu);

    /* 64MHz / (63+1) = 1MHz，一个计数就是 1us。 */
    TIM2->PSC = 63u;
    TIM2->ARR = 19999u;             /* 20000us = 20ms = 50Hz */
    TIM2->CCMR2 = 6u << 4;          /* OC3M = 110  PWM1，OC3PE = 0 无预装载 */
    TIM2->CCER = 1u << 8;           /* CC3E = 1，高电平有效 */
    TIM2->EGR = 1u;                 /* 立刻把 PSC 装入 */
    TIM2->SR = 0u;
    TIM2->CCR3 = g_us;
    TIM2->CR1 = 1u;                 /* 启动 */
    g_last_ms = now_ms;
}

void servo_lift_set_us(uint16_t pulse_us)
{
    if (pulse_us < BOARD_SERVO_MIN_US) pulse_us = BOARD_SERVO_MIN_US;
    if (pulse_us > BOARD_SERVO_MAX_US) pulse_us = BOARD_SERVO_MAX_US;
    g_us = pulse_us;
    TIM2->CCR3 = g_us;
}

uint16_t servo_lift_get_us(void)
{
    return g_us;
}

void servo_lift_step(bool up, bool down, uint32_t now_ms)
{
    uint32_t dt = (uint32_t)(now_ms - g_last_ms);
    int32_t target = (int32_t)g_us;
    g_last_ms = now_ms;
    if (dt > 200u) dt = 200u;   /* 主循环偶尔卡一下时不至于一下跳很多 */
    if (up == down) return;      /* 都没按 / 同时按：保持不动 */
    if (up) target += (int32_t)(BOARD_SERVO_RATE_US_S * dt / 1000u);
    else    target -= (int32_t)(BOARD_SERVO_RATE_US_S * dt / 1000u);
    if (target < (int32_t)BOARD_SERVO_MIN_US) target = (int32_t)BOARD_SERVO_MIN_US;
    if (target > (int32_t)BOARD_SERVO_MAX_US) target = (int32_t)BOARD_SERVO_MAX_US;
    if ((uint16_t)target != g_us) {
        g_us = (uint16_t)target;
        TIM2->CCR3 = g_us;
    }
}

#else  /* BOARD_SERVO_ENABLE == 0：车一用这一支，全部空实现 */

void servo_lift_init(uint32_t now_ms) { (void)now_ms; }
void servo_lift_set_us(uint16_t pulse_us) { (void)pulse_us; }
uint16_t servo_lift_get_us(void) { return 0u; }
void servo_lift_step(bool up, bool down, uint32_t now_ms)
{
    (void)up; (void)down; (void)now_ms;
}

#endif
