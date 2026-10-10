#ifndef STM32F103_REGS_H
#define STM32F103_REGS_H

/* ============================================================================
 * stm32f103_regs.h —— STM32F103 寄存器定义（自己写的极简 CMSIS）
 *
 * 为什么自己写而不用官方 CMSIS：
 *   官方 HAL/CMSIS 要联网下载（~150MB），本机网络下载不到。而这块芯片
 *   的寄存器地址是公开且固定的，用到的外设只有 GPIO / TIM2 / TIM4 / SPI1 /
 *   USART1 / ADC1 / RCC / SysTick / NVIC 九个，自己定义也就三百行。
 *   好处是：不依赖任何 SDK，装上编译器就能编译，出固件。
 *
 * 数据来源：RM0008 Reference Manual（STM32F103xx）第 2 章存储器映像。
 * 如果你手上有官方 CMSIS，把 firmware/Core/Inc/main.h 里的
 *   #include "hal_shim.h"  换成  #include "stm32f1xx_hal.h"
 * 即可切回官方 HAL，业务代码（motor.c / servo.c 等）一个字都不用改。
 * ==========================================================================*/

#include <stdint.h>

#define __IO  volatile
#define __O   volatile
#define __I   volatile const

/* ---------------------------------------------------------------------------
 * 1. 外设寄存器结构体
 * -------------------------------------------------------------------------*/
typedef struct {                    /* GPIO，RM0008 9.2 */
    __IO uint32_t CRL;              /* 0x00 端口配置低（pin0~7） */
    __IO uint32_t CRH;              /* 0x04 端口配置高（pin8~15） */
    __IO uint32_t IDR;              /* 0x08 输入数据 */
    __IO uint32_t ODR;              /* 0x0C 输出数据 */
    __IO uint32_t BSRR;             /* 0x10 置位/复位，低16位置位 高16位复位 */
    __IO uint32_t BRR;              /* 0x14 复位 */
    __IO uint32_t LCKR;             /* 0x18 锁定 */
} GPIO_TypeDef;

typedef struct {                    /* AFIO，RM0008 9.4 */
    __IO uint32_t EVCR, MAPR, EXTICR[4], MAPR2;
} AFIO_TypeDef;

typedef struct {                    /* RCC，RM0008 7.3 */
    __IO uint32_t CR, CFGR, CIR, APB2RSTR, APB1RSTR,
                  AHBENR, APB2ENR, APB1ENR, BDCR, CSR, AHBRSTR, CFGR2;
} RCC_TypeDef;

typedef struct {                    /* 通用定时器 TIM2~TIM5，RM0008 15.4 */
    __IO uint32_t CR1, CR2, SMCR, DIER, SR, EGR, CCMR1, CCMR2, CCER,
                  CNT, PSC, ARR, RCR, CCR1, CCR2, CCR3, CCR4, BDTR, DCR, DMAR;
} TIM_TypeDef;

typedef struct {                    /* SPI，RM0008 23.5 */
    __IO uint32_t CR1, CR2, SR, DR, CRCPR, RXCRCR, TXCRCR, I2SCFGR, I2SPR;
} SPI_TypeDef;

typedef struct {                    /* USART，RM0008 27.6 */
    __IO uint32_t SR, DR, BRR, CR1, CR2, CR3, GTPR;
} USART_TypeDef;

typedef struct {                    /* ADC，RM0008 11.12 */
    __IO uint32_t SR, CR1, CR2, SMPR1, SMPR2, JOFR1, JOFR2, JOFR3, JOFR4,
                  HTR, LTR, SQR1, SQR2, SQR3, JSQR, JDR1, JDR2, JDR3, JDR4, DR;
} ADC_TypeDef;

typedef struct {                    /* Flash 接口，RM0008 3.4 */
    __IO uint32_t ACR, KEYR, OPTKEYR, SR, CR, AR, RESERVED, OBR, WRPR;
} FLASH_TypeDef;

typedef struct {                    /* SysTick，Cortex-M3 TRM */
    __IO uint32_t CTRL, LOAD, VAL, CALIB;
} SysTick_TypeDef;

typedef struct {                    /* NVIC，Cortex-M3 TRM */
    __IO uint32_t ISER[3];  uint32_t RES0[29];
    __IO uint32_t ICER[3];  uint32_t RES1[29];
    __IO uint32_t ISPR[3];  uint32_t RES2[29];
    __IO uint32_t ICPR[3];  uint32_t RES3[29];
    __IO uint32_t IABR[3];  uint32_t RES4[61];
    __IO uint8_t  IP[84];   uint32_t RES5[683];
    __IO uint32_t STIR;
} NVIC_TypeDef;

typedef struct {                    /* SCB */
    __I  uint32_t CPUID;
    __IO uint32_t ICSR, VTOR, AIRCR, SCR, CCR, SHPR[3], SHCSR, CFSR, HFSR,
                  DFSR, MMFAR, BFAR, AFSR;
} SCB_TypeDef;

/* ---------------------------------------------------------------------------
 * 2. 基地址（RM0008 Table 3 "Register boundary addresses"）
 * -------------------------------------------------------------------------*/
#define PERIPH_BASE       0x40000000UL
#define APB1PERIPH_BASE   PERIPH_BASE
#define APB2PERIPH_BASE  (PERIPH_BASE + 0x00010000UL)
#define AHBPERIPH_BASE   (PERIPH_BASE + 0x00020000UL)

#define TIM2_BASE        (APB1PERIPH_BASE + 0x0000UL)
#define TIM3_BASE        (APB1PERIPH_BASE + 0x0400UL)
#define TIM4_BASE        (APB1PERIPH_BASE + 0x0800UL)
/* ★ TIM1 在 APB2 上（高级定时器，寄存器布局和 TIM2~TIM5 一样，
    只是多一个 BDTR 要求「主输出使能 MOE=1」才肯出波形）。 */
#define TIM1_BASE        (APB2PERIPH_BASE + 0x2C00UL)

#define RCC_BASE         (AHBPERIPH_BASE + 0x1000UL)
#define FLASH_R_BASE     (AHBPERIPH_BASE + 0x2000UL)

#define AFIO_BASE        (APB2PERIPH_BASE + 0x0000UL)
#define GPIOA_BASE       (APB2PERIPH_BASE + 0x0800UL)
#define GPIOB_BASE       (APB2PERIPH_BASE + 0x0C00UL)
#define GPIOC_BASE       (APB2PERIPH_BASE + 0x1000UL)
#define GPIOD_BASE       (APB2PERIPH_BASE + 0x1400UL)
#define ADC1_BASE        (APB2PERIPH_BASE + 0x2400UL)
#define SPI1_BASE        (APB2PERIPH_BASE + 0x3000UL)
#define USART1_BASE      (APB2PERIPH_BASE + 0x3800UL)

#define SCS_BASE         0xE000E000UL
#define SYSTICK_BASE     (SCS_BASE + 0x0010UL)
#define NVIC_BASE        (SCS_BASE + 0x0100UL)
#define SCB_BASE         (SCS_BASE + 0x0D00UL)

#define GPIOA            ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB            ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC            ((GPIO_TypeDef *)GPIOC_BASE)
#define GPIOD            ((GPIO_TypeDef *)GPIOD_BASE)
#define AFIO             ((AFIO_TypeDef *)AFIO_BASE)
#define RCC              ((RCC_TypeDef *)RCC_BASE)
#define FLASH            ((FLASH_TypeDef *)FLASH_R_BASE)
#define TIM1             ((TIM_TypeDef *)TIM1_BASE)
#define TIM2             ((TIM_TypeDef *)TIM2_BASE)
#define TIM3             ((TIM_TypeDef *)TIM3_BASE)
#define TIM4             ((TIM_TypeDef *)TIM4_BASE)
#define SPI1             ((SPI_TypeDef *)SPI1_BASE)
#define USART1           ((USART_TypeDef *)USART1_BASE)
#define ADC1             ((ADC_TypeDef *)ADC1_BASE)
#define SysTick          ((SysTick_TypeDef *)SYSTICK_BASE)
#define NVIC             ((NVIC_TypeDef *)NVIC_BASE)
#define SCB              ((SCB_TypeDef *)SCB_BASE)

/* ---------------------------------------------------------------------------
 * 3. 中断号（RM0008 Table 63，F103 中密度）
 *    ⚠ 注意 USART1 是 35 号，不是 37（37 是 USART3）。写错的表现是
 *       串口收不到数据，但发送完全正常。
 * -------------------------------------------------------------------------*/
typedef enum {
    WWDG_IRQn        = 0,   PVD_IRQn      = 1,  TAMPER_IRQn  = 2,
    RTC_IRQn         = 3,   FLASH_IRQn    = 4,  RCC_IRQn     = 5,
    EXTI0_IRQn       = 6,   EXTI1_IRQn    = 7,  EXTI2_IRQn   = 8,
    EXTI3_IRQn       = 9,   EXTI4_IRQn    = 10,
    DMA1_Channel1_IRQn = 11, DMA1_Channel2_IRQn = 12,
    DMA1_Channel3_IRQn = 13, DMA1_Channel4_IRQn = 14,
    DMA1_Channel5_IRQn = 15, DMA1_Channel6_IRQn = 16,
    DMA1_Channel7_IRQn = 17, ADC1_2_IRQn  = 18,
    USB_HP_CAN1_TX_IRQn = 19, USB_LP_CAN1_RX0_IRQn = 20,
    CAN1_RX1_IRQn    = 21,  CAN1_SCE_IRQn = 22, EXTI9_5_IRQn = 23,
    TIM1_BRK_IRQn    = 24,  TIM1_UP_IRQn  = 25, TIM1_TRG_COM_IRQn = 26,
    TIM1_CC_IRQn     = 27,  TIM2_IRQn     = 28, TIM3_IRQn    = 29,
    TIM4_IRQn        = 30,  I2C1_EV_IRQn  = 31, I2C1_ER_IRQn = 32,
    SPI1_IRQn        = 33,  SPI2_IRQn     = 34, USART1_IRQn  = 35,
    USART2_IRQn      = 36,  USART3_IRQn   = 37, EXTI15_10_IRQn = 38,
    RTC_Alarm_IRQn   = 39,  USBWakeUp_IRQn = 40
} IRQn_Type;

/* ---------------------------------------------------------------------------
 * 4. 内核指令封装（替代 CMSIS 的同名函数）
 * -------------------------------------------------------------------------*/
__attribute__((always_inline)) static inline void __NOP(void)
{ __asm volatile ("nop"); }

__attribute__((always_inline)) static inline void __disable_irq(void)
{ __asm volatile ("cpsid i"); }

__attribute__((always_inline)) static inline void __enable_irq(void)
{ __asm volatile ("cpsie i"); }

__attribute__((always_inline)) static inline void __WFI(void)
{ __asm volatile ("wfi"); }

/* ---------------------------------------------------------------------------
 * 5. 时钟使能宏
 * -------------------------------------------------------------------------*/
#define RCC_APB2ENR_AFIOEN    (1UL << 0)
#define RCC_APB2ENR_IOPAEN    (1UL << 2)
#define RCC_APB2ENR_IOPBEN    (1UL << 3)
#define RCC_APB2ENR_IOPCEN    (1UL << 4)
#define RCC_APB2ENR_ADC1EN    (1UL << 9)
#define RCC_APB2ENR_TIM1EN    (1UL << 11)   /* ★ 第 5 路舵机（回转 SG90）用 TIM1_CH4=PA11 */
#define RCC_APB2ENR_SPI1EN    (1UL << 12)
#define RCC_APB2ENR_USART1EN  (1UL << 14)

#define RCC_APB1ENR_TIM2EN    (1UL << 0)
#define RCC_APB1ENR_TIM3EN    (1UL << 1)
#define RCC_APB1ENR_TIM4EN    (1UL << 2)

#define __HAL_RCC_AFIO_CLK_ENABLE()   (RCC->APB2ENR |= RCC_APB2ENR_AFIOEN)
#define __HAL_RCC_GPIOA_CLK_ENABLE()  (RCC->APB2ENR |= RCC_APB2ENR_IOPAEN)
#define __HAL_RCC_GPIOB_CLK_ENABLE()  (RCC->APB2ENR |= RCC_APB2ENR_IOPBEN)
#define __HAL_RCC_GPIOC_CLK_ENABLE()  (RCC->APB2ENR |= RCC_APB2ENR_IOPCEN)
#define __HAL_RCC_ADC1_CLK_ENABLE()   (RCC->APB2ENR |= RCC_APB2ENR_ADC1EN)
#define __HAL_RCC_TIM1_CLK_ENABLE()   (RCC->APB2ENR |= RCC_APB2ENR_TIM1EN)
#define __HAL_RCC_SPI1_CLK_ENABLE()   (RCC->APB2ENR |= RCC_APB2ENR_SPI1EN)
#define __HAL_RCC_USART1_CLK_ENABLE() (RCC->APB2ENR |= RCC_APB2ENR_USART1EN)
#define __HAL_RCC_TIM2_CLK_ENABLE()   (RCC->APB1ENR |= RCC_APB1ENR_TIM2EN)
#define __HAL_RCC_TIM3_CLK_ENABLE()   (RCC->APB1ENR |= RCC_APB1ENR_TIM3EN)
#define __HAL_RCC_TIM4_CLK_ENABLE()   (RCC->APB1ENR |= RCC_APB1ENR_TIM4EN)

/* ADC 预分频：__HAL_RCC_ADC_CONFIG(RCC_ADCPCLK2_DIV6) -> 72MHz/6=12MHz */
#define RCC_ADCPCLK2_DIV2    ((uint32_t)0x00000000)
#define RCC_ADCPCLK2_DIV4    ((uint32_t)0x00004000)
#define RCC_ADCPCLK2_DIV6    ((uint32_t)0x00008000)
#define RCC_ADCPCLK2_DIV8    ((uint32_t)0x0000C000)
#define __HAL_RCC_ADC_CONFIG(__ADCPCLK2__) \
    do { RCC->CFGR &= ~0x0000C000UL; RCC->CFGR |= (__ADCPCLK2__); } while (0)

#endif /* STM32F103_REGS_H */
