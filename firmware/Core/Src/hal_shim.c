/* ============================================================================
 * hal_shim.c —— HAL 兼容层的寄存器实现
 *
 * 每个函数都写了"它到底改了哪个寄存器的哪一位"，这样出问题你能直接对着
 * RM0008 查，而不是对着一个黑盒 HAL 猜。
 * ==========================================================================*/

#include "hal_shim.h"

uint32_t SystemCoreClock = 72000000UL;
static volatile uint32_t g_tick = 0;

/* ============================================================================
 * 时基：SysTick 1ms
 * ==========================================================================*/
void HAL_IncTick(void) { g_tick++; }

uint32_t HAL_GetTick(void) { return g_tick; }

void HAL_Delay(uint32_t ms)
{
    uint32_t start = g_tick;
    while ((uint32_t)(g_tick - start) < ms) { }
}

void HAL_Init(void)
{
    /* 1ms 中断：LOAD = HCLK/1000 - 1。
       注意 SysTick 用的是 HCLK（不分频），不是 PCLK。 */
    SysTick->LOAD  = (SystemCoreClock / 1000UL) - 1UL;
    SysTick->VAL   = 0;
    SysTick->CTRL  = 0x00000007UL;   /* CLKSOURCE=1(HCLK) | TICKINT=1 | ENABLE=1 */
}

void HAL_NVIC_SetPriority(IRQn_Type irq, uint32_t preempt, uint32_t sub)
{
    (void)sub;
    /* F1 只有 4 位优先级（高 4 位有效），要左移 4 位写进 8 位寄存器 */
    if ((int)irq >= 0) NVIC->IP[(uint32_t)irq] = (uint8_t)((preempt & 0x0F) << 4);
}

void HAL_NVIC_EnableIRQ(IRQn_Type irq)
{
    if ((int)irq >= 0) NVIC->ISER[(uint32_t)irq >> 5] = (1UL << ((uint32_t)irq & 0x1F));
}

void HAL_NVIC_DisableIRQ(IRQn_Type irq)
{
    if ((int)irq >= 0) NVIC->ICER[(uint32_t)irq >> 5] = (1UL << ((uint32_t)irq & 0x1F));
}

/* ============================================================================
 * GPIO
 *   F1 的端口配置是 4 位一组（MODE[1:0] + CNF[1:0]），pin0~7 在 CRL，
 *   pin8~15 在 CRH。这跟 F4 以后的 MODER/OTYPER 完全不一样，别照抄。
 * ==========================================================================*/
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init)
{
    for (uint8_t pin = 0; pin < 16; pin++)
    {
        uint16_t mask = (uint16_t)(1U << pin);
        if (!(init->Pin & mask)) continue;

        uint32_t mode_bits, cnf_bits;
        switch (init->Mode)
        {
        case GPIO_MODE_OUTPUT_PP:
            mode_bits = init->Speed & 0x3u;  cnf_bits = 0x0u; break;   /* 00 = GP推挽 */
        case GPIO_MODE_AF_PP:
            mode_bits = init->Speed & 0x3u;  cnf_bits = 0x2u; break;   /* 10 = 复用推挽 */
        case GPIO_MODE_ANALOG:
            mode_bits = 0x0u;                cnf_bits = 0x0u; break;   /* 输入+模拟 */
        case GPIO_MODE_INPUT:
        default:
            mode_bits = 0x0u;                                          /* 输入 */
            cnf_bits = (init->Pull == GPIO_NOPULL) ? 0x1u : 0x2u;      /* 01浮空 10上/下拉 */
            break;
        }

        uint32_t cfg = ((cnf_bits & 0x3u) << 2) | (mode_bits & 0x3u);
        uint32_t shift = (pin & 7u) * 4u;

        if (pin < 8)
        {
            port->CRL &= ~(0xFu << shift);
            port->CRL |=  (cfg  << shift);
        }
        else
        {
            port->CRH &= ~(0xFu << shift);
            port->CRH |=  (cfg  << shift);
        }

        /* 上/下拉是靠写 ODR 实现的（CNF=10 时 ODR=1 上拉、0 下拉） */
        if (init->Mode == GPIO_MODE_INPUT && init->Pull != GPIO_NOPULL)
        {
            if (init->Pull == GPIO_PULLUP)   port->BSRR = mask;   /* ODR=1 */
            else                             port->BRR  = mask;   /* ODR=0 */
        }
    }
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState s)
{
    if (s == GPIO_PIN_SET) port->BSRR = pin;
    else                   port->BRR  = pin;
}

void HAL_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin)
{
    port->ODR ^= pin;
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    return (port->IDR & pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

/* ============================================================================
 * RCC
 * ==========================================================================*/
static HAL_StatusTypeDef wait_flag(__IO uint32_t *reg, uint32_t mask, uint32_t want)
{
    for (uint32_t i = 0; i < 200000UL; i++)
        if (((*reg) & mask) == want) return HAL_OK;
    return HAL_TIMEOUT;
}

HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *o)
{
    uint32_t pll_src = 0;
    uint8_t  pll_en  = (o->PLL.PLLState == RCC_PLL_ON);

    if (o->OscillatorType & RCC_OSCILLATORTYPE_HSE)
    {
        RCC->CR |= (1UL << 16);                      /* HSEON */
        if (wait_flag(&RCC->CR, (1UL << 17), (1UL << 17)) != HAL_OK)
            return HAL_TIMEOUT;                      /* 晶振没起振 */
        pll_src = RCC_PLLSOURCE_HSE;
    }
    else if (o->OscillatorType & RCC_OSCILLATORTYPE_HSI)
    {
        RCC->CR |= (1UL << 0);                       /* HSION */
        if (wait_flag(&RCC->CR, (1UL << 1), (1UL << 1)) != HAL_OK)
            return HAL_TIMEOUT;
        pll_src = RCC_PLLSOURCE_HSI_DIV2;            /* HSI 只有 8MHz，进 PLL 前要 /2 */
    }

    if (pll_en)
    {
        /* 换 PLL 源之前必须先关 PLL，否则写入不生效 */
        RCC->CR &= ~(1UL << 24);                     /* PLLON = 0 */
        RCC->CFGR &= ~((0xFUL << 18) | (1UL << 17) | (1UL << 16));
        RCC->CFGR |= (o->PLL.PLLMUL | pll_src);
        RCC->CR |= (1UL << 24);                      /* PLLON = 1 */
        if (wait_flag(&RCC->CR, (1UL << 25), (1UL << 25)) != HAL_OK)
            return HAL_TIMEOUT;
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *c, uint32_t latency)
{
    /* Flash 等待周期：0~24MHz 用 0，24~48 用 1，48~72 用 2。
       忘了设这一位，72MHz 下跑代码会随机跑飞，是经典坑。 */
    FLASH->ACR = (FLASH->ACR & ~0x07UL) | (latency & 0x07UL) | (1UL << 4); /* 开预取 */

    RCC->CFGR &= ~((0xFUL << 4) | (0x7UL << 8) | (0x7UL << 11));
    RCC->CFGR |= (c->AHBCLKDivider | c->APB1CLKDivider | c->APB2CLKDivider);

    RCC->CFGR &= ~0x3UL;                 /* SW = 00，先切走 */
    RCC->CFGR |= (c->SYSCLKSource & 0x3UL);
    if (wait_flag(&RCC->CFGR, 0x3UL << 2, (c->SYSCLKSource & 0x3UL) << 2) != HAL_OK)
        return HAL_TIMEOUT;              /* SWS 没跟上，说明时钟源没稳 */

    /* 更新 SystemCoreClock：HAL_Delay 和 SysTick 都依赖它 */
    if (c->SYSCLKSource == RCC_SYSCLKSOURCE_PLLCLK)
    {
        uint32_t mul = ((RCC->CFGR >> 18) & 0xFUL) + 2UL;
        if (mul > 16UL) mul = 16UL;
        if (RCC->CFGR & RCC_PLLSOURCE_HSE) SystemCoreClock = 8000000UL * mul;
        else                               SystemCoreClock = 4000000UL * mul;  /* HSI/2 */
    }
    else if (c->SYSCLKSource == RCC_SYSCLKSOURCE_HSE) SystemCoreClock = 8000000UL;
    else SystemCoreClock = 8000000UL;

    /* 时钟变了，SysTick 重装一次，否则 1ms 不再是 1ms */
    SysTick->LOAD = (SystemCoreClock / 1000UL) - 1UL;
    SysTick->VAL  = 0;
    return HAL_OK;
}

/* ============================================================================
 * TIM —— PWM
 * ==========================================================================*/
static void tim_enable_clock(TIM_TypeDef *t)
{
    if      (t == TIM2) RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    else if (t == TIM3) RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    else if (t == TIM4) RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    else if (t == TIM1) RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;   /* ★ 高级定时器在 APB2 */
}

HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *h)
{
    tim_enable_clock(h->Instance);

    h->Instance->CR1 = 0;
    h->Instance->CR1 = h->Init.CounterMode | (h->Init.AutoReloadPreload ? 0x80UL : 0UL);
    h->Instance->PSC = h->Init.Prescaler;
    h->Instance->ARR = h->Init.Period;
    h->Instance->EGR = 0x01UL;      /* UG：让 PSC/ARR 立刻生效（否则要等溢出） */

    /* ★ TIM1 是「高级控制定时器」：不置 BDTR 的 MOE（主输出使能）位，
       所有通道都被强制关断，波形一个都不出 —— 这是新手最容易卡住的一步：
       寄存器全配对了、CCER 也打开了，示波器上就是没波形。
       TIM2/TIM3/TIM4 没有这个位，所以不用管。 */
    if (h->Instance == TIM1) h->Instance->BDTR |= 0x8000UL;   /* TIM_BDTR_MOE */
    return HAL_OK;
}

/* 把通道号换算成 CCMR 里的位偏移：CH1/CH3 在低 8 位，CH2/CH4 在高 8 位 */
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *h, TIM_OC_InitTypeDef *oc, uint32_t ch)
{
    uint8_t idx = (uint8_t)(ch >> 2);                 /* 0..3 */
    __IO uint32_t *ccmr = (idx < 2) ? &h->Instance->CCMR1 : &h->Instance->CCMR2;
    uint8_t shift = (idx & 1u) ? 8u : 0u;

    /* 先清该通道的 OCxM 和 OCxPE，再写 */
    *ccmr &= ~((0x7UL << 4) | (1UL << 3)) << shift;
    *ccmr |= ((oc->OCMode) | (1UL << 3)) << shift;    /* OCxPE=1：比较值用影子寄存器 */

    __HAL_TIM_SET_COMPARE(h, ch, oc->Pulse);
    return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t ch)
{
    h->Instance->CCER |= (1UL << (ch & 0xCu));        /* CCxE：通道输出使能 */
    h->Instance->CR1  |= 0x01UL;                      /* CEN：计数器启动 */
    return HAL_OK;
}

HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *h, uint32_t ch)
{
    h->Instance->CCER &= ~(1UL << (ch & 0xCu));
    return HAL_OK;
}

/* ============================================================================
 * SPI
 * ==========================================================================*/
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *h)
{
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    h->Instance->CR1 = 0;
    h->Instance->CR1 |= h->Init.Mode
                     |  h->Init.DataSize
                     |  h->Init.CLKPolarity
                     |  h->Init.CLKPhase
                     |  h->Init.BaudRatePrescaler
                     |  h->Init.FirstBit
                     |  h->Init.NSS;              /* SSM = 软件管理片选 */
    h->Instance->CR2 = 0;
    h->Instance->CR1 |= 0x40UL;                   /* SPE：SPI 使能 */
    return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h, uint8_t *tx, uint8_t *rx,
                                          uint16_t len, uint32_t timeout)
{
    (void)timeout;
    for (uint16_t i = 0; i < len; i++)
    {
        while (!(h->Instance->SR & 0x02UL)) { ; }  /* 等 TXE */
        h->Instance->DR = tx[i];
        while (!(h->Instance->SR & 0x01UL)) { ; }  /* 等 RXNE */
        rx[i] = (uint8_t)h->Instance->DR;
    }
    return HAL_OK;
}

/* ============================================================================
 * USART
 * ==========================================================================*/
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *h)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    /* BRR = DIV_Mantissa<<4 | DIV_Fraction
       USARTDIV = PCLK2 / (16 * baud)
       72MHz 下 115200 -> 39.0625 -> 0x271 */
    uint32_t pclk = SystemCoreClock;                 /* USART1 挂 APB2，不分频 */
    float div_f = (float)pclk / (16.0f * (float)h->Init.BaudRate);
    uint32_t mant = (uint32_t)div_f;
    uint32_t frac = (uint32_t)((div_f - (float)mant) * 16.0f + 0.5f);
    if (frac > 15UL) { frac = 0UL; mant += 1UL; }
    h->Instance->BRR = ((mant & 0xFFFUL) << 4) | (frac & 0xFUL);

    h->Instance->CR2 = h->Init.StopBits;
    h->Instance->CR1 = h->Init.WordLength | h->Init.Parity | h->Init.Mode | (1UL << 13); /* UE */
    h->Instance->CR3 = h->Init.HwFlowCtl;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *p, uint16_t len, uint32_t timeout)
{
    (void)timeout;
    for (uint16_t i = 0; i < len; i++)
    {
        while (!(h->Instance->SR & 0x80UL)) { ; }   /* 等 TXE */
        h->Instance->DR = p[i];
    }
    while (!(h->Instance->SR & 0x40UL)) { ; }       /* 等 TC，最后一字节发完 */
    return HAL_OK;
}

/* USART1 句柄指针：中断服务函数要用，但 hal_shim.c 不认识 main.c 里的 huart1，
   所以在 Receive_IT 时把指针记下来。这样启动文件不用反向依赖应用层。 */
static UART_HandleTypeDef *g_uart1_ptr = 0;

HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *h, uint8_t *p, uint16_t len)
{
    h->pRxBuf = p;
    h->RxLen  = len;
    h->RxBusy = 1;
    g_uart1_ptr = h;
    h->Instance->CR1 |= (1UL << 5);                 /* RXNEIE */
    return HAL_OK;
}

/* ---------------------------------------------------------------------------
 * 两个实体中断服务函数（启动文件的向量表直接引用它们）
 * -------------------------------------------------------------------------*/
void SysTick_Handler(void)
{
    HAL_IncTick();
}

void USART1_IRQHandler(void)
{
    if (g_uart1_ptr) HAL_UART_IRQHandler(g_uart1_ptr);
}

void HAL_UART_IRQHandler(UART_HandleTypeDef *huart)
{
    uint32_t sr = huart->Instance->SR;

    if ((sr & (1UL << 5)) && (huart->Instance->CR1 & (1UL << 5)))   /* RXNE */
    {
        uint8_t b = (uint8_t)huart->Instance->DR;                   /* 读 DR 自动清 RXNE */
        if (huart->RxBusy && huart->pRxBuf)
        {
            *huart->pRxBuf = b;
            huart->RxBusy = 0;
            HAL_UART_RxCpltCallback(huart);                         /* 用户在 main.c 里实现 */
        }
    }
    /* 溢出错误：ORE 必须先读 SR 再读 DR 才能清，不清会一直进中断 */
    if (sr & (1UL << 3))
    {
        volatile uint32_t tmp = huart->Instance->DR;
        (void)tmp;
    }
}

__attribute__((weak)) void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    (void)huart;
}

/* ============================================================================
 * ADC
 * ==========================================================================*/
HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef *h)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    h->Instance->CR1 = 0;
    h->Instance->CR1 |= (h->Init.ScanConvMode ? (1UL << 8) : 0UL);   /* SCAN */
    h->Instance->CR2 = 0;
    h->Instance->CR2 |= h->Init.DataAlign
                     |  (h->Init.ContinuousConvMode ? (1UL << 1) : 0UL)   /* CONT */
                     |  (1UL << 20)                                        /* EXTTRIG */
                     |  (0x7UL << 17);                                     /* EXTSEL = SWSTART */
    return HAL_OK;
}

HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *h)
{
    h->Instance->CR2 |= (1UL << 0);                 /* ADON：先上电 */
    for (volatile uint32_t i = 0; i < 2000; i++) { ; }   /* 等 ADC 稳定（>1us，给足） */
    h->Instance->CR2 |= (1UL << 2);                 /* CAL */
    for (uint32_t i = 0; i < 200000UL; i++)
        if (!(h->Instance->CR2 & (1UL << 2))) return HAL_OK;
    return HAL_TIMEOUT;
}

HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *h, ADC_ChannelConfTypeDef *c)
{
    uint32_t ch = c->Channel & 0x1FUL;

    /* 采样时间：通道 0~9 在 SMPR2，10~17 在 SMPR1，每通道 3 位 */
    if (ch <= 9UL)
    {
        h->Instance->SMPR2 &= ~(0x7UL << (ch * 3UL));
        h->Instance->SMPR2 |=  ((c->SamplingTime & 0x7UL) << (ch * 3UL));
    }
    else
    {
        uint32_t k = ch - 10UL;
        h->Instance->SMPR1 &= ~(0x7UL << (k * 3UL));
        h->Instance->SMPR1 |=  ((c->SamplingTime & 0x7UL) << (k * 3UL));
    }

    /* 规则序列：我们只用 1 个转换，写 SQR3 的 SQ1 即可 */
    if ((c->Rank & 0x1FUL) == 1UL)
    {
        h->Instance->SQR3 &= ~0x1FUL;
        h->Instance->SQR3 |=  ch;
    }
    h->Instance->SQR1 &= ~(0xFUL << 20);            /* L = 0 -> 共 1 次转换 */
    return HAL_OK;
}

HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *h)
{
    h->Instance->CR2 |= (1UL << 0);                 /* ADON（已在则保持） */
    h->Instance->SR  &= ~((1UL << 4) | (1UL << 1)); /* 清 STRT / EOC */
    h->Instance->CR2 |= (1UL << 22);                /* SWSTART：软件触发一次 */
    return HAL_OK;
}

HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *h, uint32_t timeout)
{
    for (uint32_t i = 0; i < (timeout * 1000UL); i++)
        if (h->Instance->SR & (1UL << 1)) return HAL_OK;    /* EOC */
    return HAL_TIMEOUT;
}

uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *h)
{
    return h->Instance->DR & 0xFFFFUL;
}
