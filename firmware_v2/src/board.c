#include "board.h"
#include "board_config.h"
#include "driver_encode.h"
#include "stm32f103_min.h"

static volatile uint32_t tick_ms;
static volatile uint32_t last_motor_service_ms;
static volatile bool motor_ready;
static const int8_t motor_sign[4] = {
    MOTOR_LF_SIGN, MOTOR_RF_SIGN, MOTOR_LB_SIGN, MOTOR_RB_SIGN
};

static void gpio_mode(GpioRegs *port, unsigned pin, uint32_t mode)
{
    volatile uint32_t *control = pin < 8u ? &port->CRL : &port->CRH;
    unsigned shift = (pin & 7u) * 4u;
    *control = (*control & ~(15u << shift)) | (mode << shift);
}

static bool clock_init(void)
{
    uint32_t timeout;
    /* 先回到内部 HSI，再设置 PLL，所有等待都有超时。 */
    RCC_CR |= 1u;
    timeout = 1000000u;
    while ((RCC_CR & (1u << 1)) == 0u && --timeout != 0u) { }
    if (timeout == 0u) return false;
    RCC_CFGR &= ~3u;
    timeout = 1000000u;
    while ((RCC_CFGR & 12u) != 0u && --timeout != 0u) { }
    if (timeout == 0u) return false;
    RCC_CR &= ~(1u << 24);
    timeout = 1000000u;
    while ((RCC_CR & (1u << 25)) != 0u && --timeout != 0u) { }
    if (timeout == 0u) return false;
    /* HSI/2=4MHz，PLL*16=64MHz；APB1=32MHz，APB1定时器=64MHz。 */
    FLASH_ACR = (1u << 4) | 2u; /* 预取 + 2 个等待周期。 */
    RCC_CFGR = (4u << 8) | (14u << 18) | (2u << 14);
    RCC_CR |= 1u << 24;
    timeout = 1000000u;
    while ((RCC_CR & (1u << 25)) == 0u && --timeout != 0u) { }
    if (timeout == 0u) return false;
    RCC_CFGR = (RCC_CFGR & ~3u) | 2u;
    timeout = 1000000u;
    while ((RCC_CFGR & 12u) != 8u && --timeout != 0u) { }
    return timeout != 0u;
}

void board_motors_stop(void)
{
    /* 先把占空比归零，再将 IN2 拉低。比较寄存器不使用预装载，立即生效。
     * DRV8833 最终为 IN1=0、IN2=0，即滑行停车，不是机械抱死。
     */
    TIM4->CCR1 = 0u; TIM4->CCR2 = 0u;
    TIM4->CCR3 = 0u; TIM4->CCR4 = 0u;
    GPIOB->BRR = 15u << 12;
}

bool board_init(void)
{
    unsigned pin;
    if (!clock_init()) return false;
    RCC_APB2ENR |= (1u << 0) | (1u << 2) | (1u << 3) | (1u << 4);
    RCC_APB1ENR |= 1u << 2; /* TIM4 */
    (void)RCC_APB2ENR;
    (void)RCC_APB1ENR;
    /* 默认映射：保留 SWD，不重映射 TIM4。 */
    AFIO_MAPR &= ~(1u << 12);
    GPIOB->BRR = (15u << 6) | (15u << 12);
    for (pin = 12u; pin <= 15u; ++pin) gpio_mode(GPIOB,pin,2u);
    for (pin = 6u; pin <= 9u; ++pin) gpio_mode(GPIOB,pin,2u);
    GPIOC->BSRR = 1u << 13;
    gpio_mode(GPIOC,13u,2u);
#if BOARD_LOCAL_STOP_ENABLE
    GPIOB->BSRR = 1u << 5;
    gpio_mode(GPIOB,5u,8u); /* 内部上拉输入，PB5接按钮另一端接GND。 */
#endif
    /* PS2 空闲电平：CLK/CMD/CS 高，DAT 上拉。先写ODR，再切换输出。 */
    GPIOA->BSRR = (1u << PS2_CLK_PIN) | (1u << PS2_CMD_PIN) | (1u << PS2_DAT_PIN);
    GPIOB->BSRR = 1u << PS2_CS_PIN;
    gpio_mode(GPIOA,PS2_CLK_PIN,3u);
    gpio_mode(GPIOA,PS2_CMD_PIN,3u);
    gpio_mode(GPIOA,PS2_DAT_PIN,8u);
    gpio_mode(GPIOB,PS2_CS_PIN,3u);
    RCC_APB1RSTR |= 1u << 2;
    RCC_APB1RSTR &= ~(1u << 2);
    TIM4->PSC = 0u;
    TIM4->ARR = BOARD_MOTOR_PERIOD - 1u;
    TIM4->CCMR1 = (6u << 4) | (6u << 12); /* PWM1，无CCR预装载。 */
    TIM4->CCMR2 = (6u << 4) | (6u << 12);
    board_motors_stop();
    TIM4->EGR = 1u;
    TIM4->SR = 0u;
    TIM4->CCER = 0x1111u;
    TIM4->CR1 = 1u;
    for (pin = 6u; pin <= 9u; ++pin) gpio_mode(GPIOB,pin,0xbu);
    SCB_VTOR = 0x08000000u;
    SCB_DEMCR |= 1u << 24;
    DWT_CYCCNT = 0u;
    DWT_CTRL |= 1u;
    tick_ms = 0u;
    last_motor_service_ms = 0u;
    motor_ready = true;
    SYST_RVR = BOARD_CORE_HZ / 1000u - 1u;
    SYST_CVR = 0u;
    SYST_CSR = 7u;
#if BOARD_WATCHDOG_ENABLE
    /* LSI标称40kHz /64，312+1个计数约501ms；实际受LSI误差影响。 */
    /* 先启动LSI，再等PR/RLR同步；否则停着的LSI可能使SR一直不清零。 */
    RCC_CSR |= 1u;
    {
        uint32_t timeout = 1000000u;
        while ((RCC_CSR & 2u) == 0u && --timeout != 0u) { }
        if (timeout == 0u) { board_motors_stop(); return false; }
    }
    IWDG_KR = 0x5555u;
    IWDG_PR = 4u;
    IWDG_RLR = 312u;
    {
        uint32_t timeout = 1000000u;
        while (IWDG_SR != 0u && --timeout != 0u) { }
        if (timeout == 0u) { board_motors_stop(); return false; }
    }
    IWDG_KR = 0xaaaau;
    IWDG_KR = 0xccccu;
#endif
    return true;
}

uint32_t board_millis(void) { return tick_ms; }
void board_delay_us(uint32_t microseconds)
{
    uint32_t begin = DWT_CYCCNT;
    uint32_t cycles = microseconds * (BOARD_CORE_HZ / 1000000u);
    while ((uint32_t)(DWT_CYCCNT - begin) < cycles) { }
}
void board_watchdog_feed(void)
{
#if BOARD_WATCHDOG_ENABLE
    IWDG_KR = 0xaaaau;
#endif
}
bool board_local_stop(void)
{
#if BOARD_LOCAL_STOP_ENABLE
    return (GPIOB->IDR & (1u << 5)) == 0u;
#else
    return false;
#endif
}
void board_led(bool on)
{
    if (on) gpio_low(GPIOC,13u); else gpio_high(GPIOC,13u);
}
void board_motors_apply(const int16_t wheel[4])
{
    DriverOutput output[4];
    uint32_t high = 0u, key;
    unsigned i;
    for (i = 0; i < 4u; ++i) {
        output[i] = driver_encode(wheel[i],motor_sign[i],BOARD_MOTOR_PERIOD);
        if (output[i].in2_high) high |= 1u << (12u + i);
    }
    /* 将比较值和方向GPIO更新放在短临界区，防止写到一半被中断拉长。
     * robot_control 保证正常换向至少先输出一个控制周期的零。
     */
    key = irq_lock();
    TIM4->CCR1 = output[0].compare; TIM4->CCR2 = output[1].compare;
    TIM4->CCR3 = output[2].compare; TIM4->CCR4 = output[3].compare;
    GPIOB->BSRR = high | (((15u << 12) & ~high) << 16);
    last_motor_service_ms = tick_ms;
    irq_restore(key);
}
void SysTick_Handler(void)
{
    ++tick_ms;
    /* 主循环若卡住且中断仍运行，100ms后清零输出；IWDG处理全局停滞。 */
    if (motor_ready && (uint32_t)(tick_ms-last_motor_service_ms) > BOARD_MAIN_STALL_MS)
        board_motors_stop();
}
