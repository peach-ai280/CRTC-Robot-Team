#ifndef STM32F103_MIN_H
#define STM32F103_MIN_H
#include <stdint.h>
#include <stddef.h>

/* STM32F103 寄存器最小定义；不依赖 HAL、标准外设库或 CubeMX。 */
typedef struct {
    volatile uint32_t CRL, CRH, IDR, ODR, BSRR, BRR, LCKR;
} GpioRegs;
typedef struct {
    volatile uint32_t CR1, CR2, SMCR, DIER, SR, EGR, CCMR1, CCMR2,
                      CCER, CNT, PSC, ARR, RESERVED0, CCR1, CCR2, CCR3, CCR4;
} TimerRegs;
#define REG32(address) (*(volatile uint32_t *)(uintptr_t)(address))
#define GPIOA ((GpioRegs *)(uintptr_t)0x40010800u)
#define GPIOB ((GpioRegs *)(uintptr_t)0x40010c00u)
#define GPIOC ((GpioRegs *)(uintptr_t)0x40011000u)
#define TIM4  ((TimerRegs *)(uintptr_t)0x40000800u)
#define RCC_CR      REG32(0x40021000u)
#define RCC_CFGR    REG32(0x40021004u)
#define RCC_APB2ENR REG32(0x40021018u)
#define RCC_APB1ENR REG32(0x4002101cu)
#define RCC_APB1RSTR REG32(0x40021010u)
#define RCC_CSR     REG32(0x40021024u)
#define FLASH_ACR   REG32(0x40022000u)
#define AFIO_MAPR   REG32(0x40010004u)
#define SYST_CSR    REG32(0xe000e010u)
#define SYST_RVR    REG32(0xe000e014u)
#define SYST_CVR    REG32(0xe000e018u)
#define SCB_VTOR    REG32(0xe000ed08u)
#define SCB_DEMCR   REG32(0xe000edfcu)
#define DWT_CTRL    REG32(0xe0001000u)
#define DWT_CYCCNT  REG32(0xe0001004u)
#define IWDG_KR     REG32(0x40003000u)
#define IWDG_PR     REG32(0x40003004u)
#define IWDG_RLR    REG32(0x40003008u)
#define IWDG_SR     REG32(0x4000300cu)

_Static_assert(offsetof(GpioRegs, BSRR) == 0x10u, "GPIO offset");
_Static_assert(offsetof(TimerRegs, CCR1) == 0x34u, "TIM CCR1 offset");
_Static_assert(offsetof(TimerRegs, CCR4) == 0x40u, "TIM CCR4 offset");
static inline uint32_t irq_lock(void)
{
    uint32_t value;
    __asm volatile ("mrs %0, primask\n cpsid i" : "=r"(value) : : "memory");
    return value;
}
static inline void irq_restore(uint32_t value)
{
    __asm volatile ("msr primask, %0" : : "r"(value) : "memory");
}
static inline void cpu_wait(void) { __asm volatile ("wfi" : : : "memory"); }
static inline void gpio_high(GpioRegs *port, unsigned pin) { port->BSRR = 1u << pin; }
static inline void gpio_low(GpioRegs *port, unsigned pin) { port->BRR = 1u << pin; }
#endif
