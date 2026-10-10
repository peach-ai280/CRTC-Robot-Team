/* ============================================================================
 * bsp.c —— 时钟 + 外设初始化（完整固件和三个测试固件共用）
 *
 * 【PWM 频率与别处的耦合 —— 改之前务必看】
 *   TIM2: PSC=71,  ARR=19999 -> 1MHz 计数, 50Hz
 *         servo.c 直接往比较寄存器填"微秒数"（500~2500），靠的就是 1MHz
 *   TIM4: PSC=0,   ARR=7199  -> 72MHz 计数, 10kHz
 *         motor.c 里 MOTOR_PWM_PERIOD 必须等于 7199，否则占空比全错
 *   TIM1: PSC=71,  ARR=19999 -> 1MHz 计数, 50Hz（只开 CH4，别的通道留给 USART1）
 *         ⚠ TIM1 是高级定时器，BDTR.MOE 不置 1 就一个波形都不出（hal_shim 里已处理）
 * ==========================================================================*/

#include "bsp.h"
#include "board.h"
#include "config.h"

/* ---------------------------------------------------------------------------
 * 全局外设句柄（main.h 里 extern，别的模块通过 board.h 的宏引用它们）
 * -------------------------------------------------------------------------*/
TIM_HandleTypeDef  htim1;      /* ★ PA11，第 5 路舵机（回转 SG90） */
TIM_HandleTypeDef  htim2;
TIM_HandleTypeDef  htim4;
SPI_HandleTypeDef  hspi1;
UART_HandleTypeDef huart1;
ADC_HandleTypeDef  hadc1;

volatile uint8_t g_uart1_rx = 0;

/* ============================================================================
 * 系统时钟：72MHz（HSE 8MHz × 9），带 HSE 失败回退
 *    蓝丸板晶振虚焊是常见病。不回退的话，新手只会看到"灯不闪、串口没输出"，
 *    完全无从下手；回退到 HSI 至少能让串口把故障打印出来。
 * ==========================================================================*/
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    uint8_t use_hse = 1;

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_ON;
    osc.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLMUL     = RCC_PLL_MUL9;          /* 8MHz × 9 = 72MHz */
    if (HAL_RCC_OscConfig(&osc) != HAL_OK)
        use_hse = 0;                            /* 晶振没起振 */

    if (use_hse)
    {
        clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                             RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
        clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
        clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;   /* HCLK  72MHz */
        clk.APB1CLKDivider = RCC_HCLK_DIV2;     /* PCLK1 36MHz（F1 上限） */
        clk.APB2CLKDivider = RCC_HCLK_DIV1;     /* PCLK2 72MHz */
        if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK)
            Error_Handler();
    }
    else
    {
        osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
        osc.HSIState       = RCC_HSI_ON;
        osc.PLL.PLLState   = RCC_PLL_ON;
        osc.PLL.PLLSource  = RCC_PLLSOURCE_HSI_DIV2;   /* 8MHz / 2 */
        osc.PLL.PLLMUL     = RCC_PLL_MUL16;            /* 4MHz × 16 = 64MHz */
        if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

        clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                             RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
        clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
        clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
        clk.APB1CLKDivider = RCC_HCLK_DIV2;     /* 32MHz */
        clk.APB2CLKDivider = RCC_HCLK_DIV1;     /* 64MHz */
        if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK)
            Error_Handler();
    }

    /* 定时器时钟提醒：APB1 分频不为 1 时，TIM2/TIM4 时钟 = PCLK1 × 2 = 72MHz，
       与 motor.c / servo.c 的假设一致。回退到 HSI 64MHz 时它们变成 64MHz，
       PWM 会偏 ~11%，不影响调试，但正式跑请修好晶振。 */
}

/* ============================================================================
 * GPIO
 * ==========================================================================*/
void BSP_GPIO_Init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* --- 舵机 PWM：PA0~PA3（TIM2_CH1~CH4）复用推挽 --- */
    g.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    g.Mode  = GPIO_MODE_AF_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;      /* 50Hz 不需要快翻转，慢一点干扰小 */
    HAL_GPIO_Init(GPIOA, &g);

    /* --- ★ 第 5 路舵机：PA11（TIM1_CH4）复用推挽 ---
     *   ⚠ PA11/PA12 在蓝丸板上同时接着 micro-USB 的 D-/D+。
     *     我们不用板载 USB（供电和烧录都走 ST-LINK），所以当普通引脚用没问题；
     *     但**调车时别把 micro-USB 插上当电源**（会跟舵机脉冲打架）。
     *   为什么是 PA11：TIM2 的 4 个通道（PA0~PA3）已经全用满、TIM4 给了电机，
     *   TIM3 的重映射引脚会跟 PS2 片选/电池检测/按键撞车 ——
     *   TIM1_CH4 = PA11 是唯一「不重映射、不冲突、板上还引出来了」的选择。 */
    g.Pin   = GPIO_PIN_11;
    g.Mode  = GPIO_MODE_AF_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &g);

    /* --- 蜂鸣器 PA8（可选件，没焊也不影响） --- */
    g.Pin   = BUZZ_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BUZZ_PORT, &g);
    HAL_GPIO_WritePin(BUZZ_PORT, BUZZ_PIN, GPIO_PIN_RESET);

    /* --- USART1：PA9 TX 复用推挽 / PA10 RX 上拉输入 --- */
    g.Pin   = GPIO_PIN_9;
    g.Mode  = GPIO_MODE_AF_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);

    g.Pin   = GPIO_PIN_10;
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_PULLUP;              /* 视觉模块没接时不会收到满屏乱码 */
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &g);

    /* --- SPI1：PA5 SCK / PA7 MOSI 复用推挽，PA6 MISO 上拉输入 --- */
    g.Pin   = GPIO_PIN_5 | GPIO_PIN_7;
    g.Mode  = GPIO_MODE_AF_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);

    g.Pin   = GPIO_PIN_6;
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);

    /* --- PS2 片选 PB0：推挽输出，空闲必须为高（不选中） --- */
    g.Pin   = PS2_CS_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(PS2_CS_PORT, &g);
    HAL_GPIO_WritePin(PS2_CS_PORT, PS2_CS_PIN, GPIO_PIN_SET);

    /* --- 电池电压 PB1：模拟输入 --- */
    g.Pin   = GPIO_PIN_1;
    g.Mode  = GPIO_MODE_ANALOG;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);

    /* --- 板载按键 PB5：上拉输入，按下为低 --- */
    g.Pin   = KEY_PIN;
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(KEY_PORT, &g);

    /* --- 电机 PWM PB6~PB9（TIM4_CH1~CH4）复用推挽 --- */
    g.Pin   = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    g.Mode  = GPIO_MODE_AF_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &g);

    /* --- 限位开关 PB10 / PB11：上拉输入（另一端接地，按下 = 低） --- */
    g.Pin   = LIMIT_L_PIN | LIMIT_R_PIN;
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);

    /* --- 电机方向 PB12~PB15：推挽输出，初始低 --- */
    g.Pin   = MOTOR_LF_DIR_PIN | MOTOR_RF_DIR_PIN |
              MOTOR_LB_DIR_PIN | MOTOR_RB_DIR_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(MOTOR_DIR_PORT, &g);
    HAL_GPIO_WritePin(MOTOR_DIR_PORT,
                      MOTOR_LF_DIR_PIN | MOTOR_RF_DIR_PIN |
                      MOTOR_LB_DIR_PIN | MOTOR_RB_DIR_PIN, GPIO_PIN_RESET);

    /* --- LED PC13：蓝丸板载，低电平点亮，初始灭 --- */
    g.Pin   = LED_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &g);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
}

/* ============================================================================
 * TIM2 —— 4 路舵机，50Hz
 *   72MHz / (71+1) = 1MHz => 1 个计数 = 1 微秒，周期 20000 计数 = 20ms
 *   servo.c 因此可以直接把"微秒"写进比较寄存器
 * ==========================================================================*/
void BSP_TIM2_Init(void)
{
    TIM_OC_InitTypeDef oc = {0};

    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 71;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 19999;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) Error_Handler();

    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 1500;                 /* 1.5ms = 中位，上电不甩 */
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &oc, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim2, &oc, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim2, &oc, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&htim2, &oc, TIM_CHANNEL_4);
}

/* ============================================================================
 * ★ TIM1_CH4 —— 第 5 路舵机（回转 SG90），50Hz
 *   和 TIM2 完全同参数（PSC=71/ARR=19999 → 1 个计数 = 1µs，周期 20ms），
 *   但 TIM1 挂在 APB2 上。APB2 分频 = 1，所以 TIM1CLK = 72MHz，和 TIM2 一样。
 *   ★ 只开 CH4：CH1/CH2/CH3 分别是 PA8/PA9/PA10，那三个脚给蜂鸣器和串口了，
 *     一个定时器的不同通道可以单独用，互不影响。
 * ==========================================================================*/
void BSP_TIM1_Init(void)
{
    TIM_OC_InitTypeDef oc = {0};

    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 71;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 19999;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Handler();

    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 1500;                 /* 1.5ms = 中位，上电不甩 */
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_4);
}

/* ============================================================================
 * TIM4 —— 4 路电机，10kHz
 *   72MHz / 1 = 72MHz，周期 7200 计数 = 10kHz
 *   为什么是 10kHz 不是 20kHz：TT 马达 + DRV8833 在 10kHz 时噪声最小、
 *   驱动发热也低；再高开关损耗上升，DRV8833 会烫手。
 * ==========================================================================*/
void BSP_TIM4_Init(void)
{
    TIM_OC_InitTypeDef oc = {0};

    __HAL_RCC_TIM4_CLK_ENABLE();

    htim4.Instance               = TIM4;
    htim4.Init.Prescaler         = 0;
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4.Init.Period            = 7199;
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) Error_Handler();

    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 0;                    /* 上电必须 0，否则一通电车就窜 */
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim4, &oc, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim4, &oc, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim4, &oc, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&htim4, &oc, TIM_CHANNEL_4);
}

/* ============================================================================
 * SPI1 —— PS2 手柄，模式 3，281kHz
 *   ⚠ 绝大多数 PS2 例程都用模式 3。改成模式 0 的表现是：
 *     能读到 0xFF 但永远握不上手（rx[2] != 0x5A）。
 * ==========================================================================*/
void BSP_SPI1_Init(void)
{
    __HAL_RCC_SPI1_CLK_ENABLE();

    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_HIGH;    /* CPOL=1 */
    hspi1.Init.CLKPhase          = SPI_PHASE_2EDGE;      /* CPHA=1 */
    hspi1.Init.NSS               = SPI_NSS_SOFT;         /* CS 用 PB0 手动拉 */
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;   /* 72M/256 = 281kHz */
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial     = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}

/* ============================================================================
 * USART1 —— 115200，调试打印 + 视觉模块共用
 * ==========================================================================*/
void BSP_USART1_Init(void)
{
    __HAL_RCC_USART1_CLK_ENABLE();

    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();

    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&g_uart1_rx, 1);
}

/* ============================================================================
 * ADC1 —— 电池电压（PB1 / IN9，1/3 分压）
 *   ADC 时钟 = PCLK2 / 6 = 12MHz（F1 上限 14MHz，超了读数会飘）
 *   采样 239.5 周期：分压是 20k+10k，源阻抗高，采快了会偏低
 * ==========================================================================*/
void BSP_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef ch = {0};

    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_ADC_CONFIG(RCC_ADCPCLK2_DIV6);     /* 别漏，否则 ADC 时钟超限 */

    hadc1.Instance                   = ADC1;
    hadc1.Init.ScanConvMode          = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;  /* sensor.c 用轮询单次触发 */
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) Error_Handler();

    ch.Channel      = VBAT_ADC_CHANNEL;
    ch.Rank         = ADC_REGULAR_RANK_1;
    ch.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &ch) != HAL_OK) Error_Handler();
}

/* ========================================================================== */
void BSP_InitAll(void)
{
    BSP_GPIO_Init();
    BSP_TIM2_Init();
    BSP_TIM1_Init();
    BSP_TIM4_Init();
    BSP_SPI1_Init();
    BSP_USART1_Init();
    BSP_ADC1_Init();
}

/* ============================================================================
 * 死机入口：关中断 + LED 快闪
 *   比"什么都不做"好排查得多 —— 看到灯狂闪就知道初始化哪一步挂了。
 * ==========================================================================*/
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        for (volatile uint32_t i = 0; i < 100000UL; i++) __NOP();
    }
}
