/* ============================================================================
 * startup_stm32f103xb.c —— 用 C 写的启动文件（代替官方 startup_stm32f103xb.s）
 *
 * 为什么用 C 而不用官方汇编版：
 *   官方启动文件要随 CMSIS 一起下载。而它做的事其实就三件：
 *     1. 把中断向量表放到 flash 最开头（0x08000000）
 *     2. 把 .data 段从 flash 拷到 RAM，把 .bss 清零
 *     3. 跳到 main()
 *   这三件事用 C 写得清清楚楚，出了问题你能看懂，不用对着汇编猜。
 *
 * 中断号与向量表位置的对应关系：向量表第 N 项（从 0 数）对应 IRQn = N - 16。
 * 所以 USART1（IRQn=35）在第 51 项。填错的表现：串口能发不能收。
 * ==========================================================================*/

#include "hal_shim.h"

/* 链接脚本提供的符号 */
extern uint32_t _estack;
extern uint32_t _sidata, _sdata, _edata;
extern uint32_t _sbss,  _ebss;

int main(void);

/* 未使用的中断统一落到这里。死循环 + 关中断，防止反复触发 */
void Default_Handler(void)
{
    __disable_irq();
    while (1) { }
}

void Reset_Handler(void);

/* 把没用到的中断都弱绑定到 Default_Handler。
   ⚠ SysTick_Handler 和 USART1_IRQHandler 在 hal_shim.c 里有实体实现，
      所以这里不能再给它们做弱绑定，否则会重复定义。 */
#define W(fn)  void fn(void) __attribute__((weak, alias("Default_Handler")));

W(NMI_Handler)          W(HardFault_Handler)    W(MemManage_Handler)
W(BusFault_Handler)     W(UsageFault_Handler)   W(SVC_Handler)
W(DebugMon_Handler)     W(PendSV_Handler)
W(WWDG_IRQHandler)      W(PVD_IRQHandler)       W(TAMPER_IRQHandler)
W(RTC_IRQHandler)       W(FLASH_IRQHandler)     W(RCC_IRQHandler)
W(EXTI0_IRQHandler)     W(EXTI1_IRQHandler)     W(EXTI2_IRQHandler)
W(EXTI3_IRQHandler)     W(EXTI4_IRQHandler)
W(DMA1_Channel1_IRQHandler) W(DMA1_Channel2_IRQHandler) W(DMA1_Channel3_IRQHandler)
W(DMA1_Channel4_IRQHandler) W(DMA1_Channel5_IRQHandler) W(DMA1_Channel6_IRQHandler)
W(DMA1_Channel7_IRQHandler) W(ADC1_2_IRQHandler)
W(USB_HP_CAN1_TX_IRQHandler) W(USB_LP_CAN1_RX0_IRQHandler)
W(CAN1_RX1_IRQHandler)  W(CAN1_SCE_IRQHandler)  W(EXTI9_5_IRQHandler)
W(TIM1_BRK_IRQHandler)  W(TIM1_UP_IRQHandler)   W(TIM1_TRG_COM_IRQHandler)
W(TIM1_CC_IRQHandler)   W(TIM2_IRQHandler)      W(TIM3_IRQHandler)
W(TIM4_IRQHandler)      W(I2C1_EV_IRQHandler)   W(I2C1_ER_IRQHandler)
W(SPI1_IRQHandler)      W(SPI2_IRQHandler)
W(USART2_IRQHandler)    W(USART3_IRQHandler)    W(EXTI15_10_IRQHandler)
W(RTC_Alarm_IRQHandler) W(USBWakeUp_IRQHandler)

#undef W

/* SysTick 和 USART1 的实体在 hal_shim.c */
void SysTick_Handler(void);
void USART1_IRQHandler(void);

void Reset_Handler(void)
{
    uint32_t *src = &_sidata;
    uint32_t *dst = &_sdata;
    while (dst < &_edata) *dst++ = *src++;     /* .data: flash -> RAM */

    dst = &_sbss;
    while (dst < &_ebss)  *dst++ = 0;          /* .bss 清零 */

    main();
    while (1) { }
}

/* 中断向量表：必须放在 0x08000000，链接脚本里 .isr_vector 排在最前 */
__attribute__((section(".isr_vector"), used))
void (* const g_pfnVectors[57])(void) = {
    (void (*)(void))(&_estack),   /* 0  栈顶 */
    Reset_Handler,                /* 1  */
    NMI_Handler,                  /* 2  */
    HardFault_Handler,            /* 3  */
    MemManage_Handler,            /* 4  */
    BusFault_Handler,             /* 5  */
    UsageFault_Handler,           /* 6  */
    0, 0, 0, 0,                   /* 7~10 保留 */
    SVC_Handler,                  /* 11 */
    DebugMon_Handler,             /* 12 */
    0,                            /* 13 */
    PendSV_Handler,               /* 14 */
    SysTick_Handler,              /* 15 */
    WWDG_IRQHandler,              /* 16 */
    PVD_IRQHandler,               /* 17 */
    TAMPER_IRQHandler,            /* 18 */
    RTC_IRQHandler,               /* 19 */
    FLASH_IRQHandler,             /* 20 */
    RCC_IRQHandler,               /* 21 */
    EXTI0_IRQHandler,             /* 22 */
    EXTI1_IRQHandler,             /* 23 */
    EXTI2_IRQHandler,             /* 24 */
    EXTI3_IRQHandler,             /* 25 */
    EXTI4_IRQHandler,             /* 26 */
    DMA1_Channel1_IRQHandler,     /* 27 */
    DMA1_Channel2_IRQHandler,     /* 28 */
    DMA1_Channel3_IRQHandler,     /* 29 */
    DMA1_Channel4_IRQHandler,     /* 30 */
    DMA1_Channel5_IRQHandler,     /* 31 */
    DMA1_Channel6_IRQHandler,     /* 32 */
    DMA1_Channel7_IRQHandler,     /* 33 */
    ADC1_2_IRQHandler,            /* 34 */
    USB_HP_CAN1_TX_IRQHandler,    /* 35 */
    USB_LP_CAN1_RX0_IRQHandler,   /* 36 */
    CAN1_RX1_IRQHandler,          /* 37 */
    CAN1_SCE_IRQHandler,          /* 38 */
    EXTI9_5_IRQHandler,           /* 39 */
    TIM1_BRK_IRQHandler,          /* 40 */
    TIM1_UP_IRQHandler,           /* 41 */
    TIM1_TRG_COM_IRQHandler,      /* 42 */
    TIM1_CC_IRQHandler,           /* 43 */
    TIM2_IRQHandler,              /* 44 */
    TIM3_IRQHandler,              /* 45 */
    TIM4_IRQHandler,              /* 46 */
    I2C1_EV_IRQHandler,           /* 47 */
    I2C1_ER_IRQHandler,           /* 48 */
    SPI1_IRQHandler,              /* 49 */
    SPI2_IRQHandler,              /* 50 */
    USART1_IRQHandler,            /* 51  <- IRQn 35，别数错 */
    USART2_IRQHandler,            /* 52 */
    USART3_IRQHandler,            /* 53 */
    EXTI15_10_IRQHandler,         /* 54 */
    RTC_Alarm_IRQHandler,         /* 55 */
    USBWakeUp_IRQHandler          /* 56 */
};
