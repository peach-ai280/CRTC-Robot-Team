#ifndef HAL_SHIM_H
#define HAL_SHIM_H

/* ============================================================================
 * hal_shim.h —— HAL 兼容层（纯寄存器实现）
 *
 * 【它干什么】
 *   用直接操作寄存器的方式，实现了 motor.c / servo.c / ps2.c / sensor.c /
 *   main.c 里用到的全部 HAL 函数，函数签名和官方 HAL 一模一样。
 *   所以业务代码一个字都不用改，就能在没有官方库的情况下编译出固件。
 *
 * 【为什么敢这么干】
 *   用到的 HAL 函数只有 20 来个，全是"配置一次性外设"的性质，不涉及
 *   DMA、中断优先级组、超时重传这些复杂状态机。逐个用寄存器实现，
 *   行为是可枚举、可核对的。比 HAL 的好处是：看得见、改得动。
 *
 * 【切回官方 HAL】
 *   在 firmware/Core/Inc/main.h 里把 #include "hal_shim.h" 换成
 *   #include "stm32f1xx_hal.h" 即可（前提是工程里有官方库）。
 * ==========================================================================*/

#include "stm32f103_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------------
 * 通用
 * -------------------------------------------------------------------------*/
typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 } HAL_StatusTypeDef;
typedef enum { RESET = 0, SET = 1 } FlagStatus;
typedef enum { DISABLE = 0, ENABLE = 1 } FunctionalState;

extern uint32_t SystemCoreClock;     /* Hz */

void HAL_Init(void);
void HAL_IncTick(void);
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);

/* ---------------------------------------------------------------------------
 * GPIO
 * -------------------------------------------------------------------------*/
#define GPIO_PIN_0   ((uint16_t)0x0001)
#define GPIO_PIN_1   ((uint16_t)0x0002)
#define GPIO_PIN_2   ((uint16_t)0x0004)
#define GPIO_PIN_3   ((uint16_t)0x0008)
#define GPIO_PIN_4   ((uint16_t)0x0010)
#define GPIO_PIN_5   ((uint16_t)0x0020)
#define GPIO_PIN_6   ((uint16_t)0x0040)
#define GPIO_PIN_7   ((uint16_t)0x0080)
#define GPIO_PIN_8   ((uint16_t)0x0100)
#define GPIO_PIN_9   ((uint16_t)0x0200)
#define GPIO_PIN_10  ((uint16_t)0x0400)
#define GPIO_PIN_11  ((uint16_t)0x0800)
#define GPIO_PIN_12  ((uint16_t)0x1000)
#define GPIO_PIN_13  ((uint16_t)0x2000)
#define GPIO_PIN_14  ((uint16_t)0x4000)
#define GPIO_PIN_15  ((uint16_t)0x8000)
#define GPIO_PIN_ALL ((uint16_t)0xFFFF)

typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;

#define GPIO_MODE_INPUT     0x00000000u   /* 浮空/上下拉输入 */
#define GPIO_MODE_OUTPUT_PP 0x00000001u   /* 通用推挽输出 */
#define GPIO_MODE_OUTPUT_OD 0x00000002u
#define GPIO_MODE_AF_PP     0x00000003u   /* 复用推挽（PWM/SPI/UART 用） */
#define GPIO_MODE_AF_OD     0x00000004u
#define GPIO_MODE_ANALOG    0x00000005u   /* ADC */

#define GPIO_NOPULL   0x00000000u
#define GPIO_PULLUP   0x00000001u
#define GPIO_PULLDOWN 0x00000002u

#define GPIO_SPEED_FREQ_LOW    0x00000001u   /* 10MHz */
#define GPIO_SPEED_FREQ_MEDIUM 0x00000002u   /* 2MHz  */
#define GPIO_SPEED_FREQ_HIGH   0x00000003u   /* 50MHz */

typedef struct {
    uint32_t Pin;
    uint32_t Mode;
    uint32_t Pull;
    uint32_t Speed;
} GPIO_InitTypeDef;

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState s);
void HAL_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);

/* ---------------------------------------------------------------------------
 * RCC / Flash
 * -------------------------------------------------------------------------*/
#define RCC_OSCILLATORTYPE_NONE 0x00000000u
#define RCC_OSCILLATORTYPE_HSE  0x00000001u
#define RCC_OSCILLATORTYPE_HSI  0x00000002u
#define RCC_HSE_OFF  0x00000000u
#define RCC_HSE_ON   0x00000001u
#define RCC_HSI_OFF  0x00000000u
#define RCC_HSI_ON   0x00000001u
#define RCC_HSE_PREDIV_DIV1 0x00000000u
#define RCC_PLL_NONE 0x00000000u
#define RCC_PLL_ON   0x00000001u
#define RCC_PLLSOURCE_HSI_DIV2 0x00000000u
#define RCC_PLLSOURCE_HSE      0x00010000u
#define RCC_PLL_MUL9  0x001C0000u   /* (9-2) << 18 */
#define RCC_PLL_MUL16 0x00380000u   /* (16-2) << 18 */

#define RCC_CLOCKTYPE_SYSCLK 0x00000001u
#define RCC_CLOCKTYPE_HCLK   0x00000002u
#define RCC_CLOCKTYPE_PCLK1  0x00000004u
#define RCC_CLOCKTYPE_PCLK2  0x00000008u
#define RCC_SYSCLKSOURCE_HSI 0x00000000u
#define RCC_SYSCLKSOURCE_HSE 0x00000001u
#define RCC_SYSCLKSOURCE_PLLCLK 0x00000002u
#define RCC_SYSCLK_DIV1 0x00000000u
#define RCC_HCLK_DIV1   0x00000000u
#define RCC_HCLK_DIV2   0x00000400u
#define FLASH_LATENCY_0 0x00000000u
#define FLASH_LATENCY_1 0x00000001u
#define FLASH_LATENCY_2 0x00000002u

typedef struct {
    uint32_t PLLState;
    uint32_t PLLSource;
    uint32_t PLLMUL;
    uint32_t PLLPrediv;      /* F1 不用，留着保持签名一致 */
} RCC_PLLInitTypeDef;

typedef struct {
    uint32_t OscillatorType;
    uint32_t HSEState;
    uint32_t HSEPredivValue;
    uint32_t LSEState;
    uint32_t HSIState;
    uint32_t HSICalibrationValue;
    uint32_t LSIState;
    RCC_PLLInitTypeDef PLL;
} RCC_OscInitTypeDef;

typedef struct {
    uint32_t ClockType;
    uint32_t SYSCLKSource;
    uint32_t AHBCLKDivider;
    uint32_t APB1CLKDivider;
    uint32_t APB2CLKDivider;
} RCC_ClkInitTypeDef;

HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *o);
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *c, uint32_t latency);

/* ---------------------------------------------------------------------------
 * TIM —— 只实现 PWM 输出（舵机/电机用）
 * -------------------------------------------------------------------------*/
#define TIM_CHANNEL_1 0x00000000u
#define TIM_CHANNEL_2 0x00000004u
#define TIM_CHANNEL_3 0x00000008u
#define TIM_CHANNEL_4 0x0000000Cu
#define TIM_COUNTERMODE_UP   0x00000000u
#define TIM_COUNTERMODE_DOWN 0x00000010u
#define TIM_CLOCKDIVISION_DIV1 0x00000000u
#define TIM_AUTORELOAD_PRELOAD_DISABLE 0x00000000u
#define TIM_AUTORELOAD_PRELOAD_ENABLE  0x00000080u
#define TIM_OCMODE_PWM1 0x00000060u      /* OCxM = 110 */
#define TIM_OCMODE_PWM2 0x00000070u
#define TIM_OCPOLARITY_HIGH 0x00000000u
#define TIM_OCPOLARITY_LOW  0x00000002u
#define TIM_OCFAST_DISABLE  0x00000000u

typedef struct {
    uint32_t Prescaler;
    uint32_t CounterMode;
    uint32_t Period;
    uint32_t ClockDivision;
    uint32_t AutoReloadPreload;
} TIM_Base_InitTypeDef;

typedef struct {
    TIM_TypeDef *Instance;
    TIM_Base_InitTypeDef Init;
} TIM_HandleTypeDef;

typedef struct {
    uint32_t OCMode;
    uint32_t Pulse;
    uint32_t OCPolarity;
    uint32_t OCFastMode;
} TIM_OC_InitTypeDef;

HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *h);
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *h, TIM_OC_InitTypeDef *oc, uint32_t ch);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t ch);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *h, uint32_t ch);

/* 直接写比较寄存器。servo.c 靠它把"微秒"写进去，motor.c 靠它写占空比。
   用 switch 而不是指针算术：四个 CCR 虽然地址连续，但写成 switch 更好读，
   也不会因为通道常量改动而产生越界写。 */
#define __HAL_TIM_SET_COMPARE(__H__, __CH__, __V__)                        \
    do {                                                                   \
        TIM_TypeDef *_t = (__H__)->Instance;                               \
        switch ((__CH__) >> 2) {                                           \
            case 0: _t->CCR1 = (__V__); break;                              \
            case 1: _t->CCR2 = (__V__); break;                              \
            case 2: _t->CCR3 = (__V__); break;                              \
            default: _t->CCR4 = (__V__); break;                             \
        }                                                                   \
    } while (0)

/* ---------------------------------------------------------------------------
 * SPI —— 只实现主机全双工轮询（PS2 手柄用）
 * -------------------------------------------------------------------------*/
#define SPI_MODE_MASTER               0x00000104u   /* MSTR | SSI */
#define SPI_MODE_SLAVE                0x00000000u
#define SPI_DIRECTION_2LINES          0x00000000u
#define SPI_DIRECTION_1LINE           0x00008000u
#define SPI_DATASIZE_8BIT             0x00000000u
#define SPI_DATASIZE_16BIT            0x00000800u
#define SPI_POLARITY_LOW              0x00000000u
#define SPI_POLARITY_HIGH             0x00000002u
#define SPI_PHASE_1EDGE               0x00000000u
#define SPI_PHASE_2EDGE               0x00000001u
#define SPI_NSS_SOFT                  0x00000200u
#define SPI_NSS_HARD_OUTPUT           0x00000004u
#define SPI_BAUDRATEPRESCALER_2       0x00000000u
#define SPI_BAUDRATEPRESCALER_256     0x00000038u
#define SPI_FIRSTBIT_MSB              0x00000000u
#define SPI_FIRSTBIT_LSB              0x00000080u
#define SPI_TIMODE_DISABLE            0x00000000u
#define SPI_CRCCALCULATION_DISABLE    0x00000000u

typedef struct {
    uint32_t Mode, Direction, DataSize, CLKPolarity, CLKPhase, NSS,
             BaudRatePrescaler, FirstBit, TIMode, CRCCalculation, CRCPolynomial;
} SPI_InitTypeDef;

typedef struct {
    SPI_TypeDef *Instance;
    SPI_InitTypeDef Init;
} SPI_HandleTypeDef;

HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *h);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h, uint8_t *tx, uint8_t *rx,
                                          uint16_t len, uint32_t timeout);

/* ---------------------------------------------------------------------------
 * USART —— 轮询发送 + 中断接收
 * -------------------------------------------------------------------------*/
#define UART_WORDLENGTH_8B  0x00000000u
#define UART_STOPBITS_1     0x00000000u
#define UART_STOPBITS_2     0x00002000u
#define UART_PARITY_NONE    0x00000000u
#define UART_HWCONTROL_NONE 0x00000000u
#define UART_MODE_RX        0x00000004u
#define UART_MODE_TX        0x00000008u
#define UART_MODE_TX_RX     0x0000000Cu
#define UART_OVERSAMPLING_16 0x00000000u

typedef struct {
    uint32_t BaudRate, WordLength, StopBits, Parity, Mode, HwFlowCtl, OverSampling;
} UART_InitTypeDef;

typedef struct {
    USART_TypeDef *Instance;
    UART_InitTypeDef Init;
    uint8_t  *pRxBuf;        /* shim 内部：中断接收目标 */
    uint16_t  RxLen;
    uint8_t   RxBusy;
} UART_HandleTypeDef;

HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *h);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *p, uint16_t len, uint32_t timeout);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *h, uint8_t *p, uint16_t len);
void HAL_UART_IRQHandler(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);   /* 弱符号，用户在 main.c 里实现 */

/* ---------------------------------------------------------------------------
 * ADC —— 单通道轮询（电池电压）
 * -------------------------------------------------------------------------*/
#define ADC_SOFTWARE_START      0x00000000u
#define ADC_DATAALIGN_RIGHT     0x00000000u
#define ADC_DATAALIGN_LEFT      0x00000800u
#define ADC_CHANNEL_9           0x00000009u
#define ADC_REGULAR_RANK_1      0x00000001u
#define ADC_SAMPLETIME_239CYCLES_5 0x00000007u

typedef struct {
    uint32_t ScanConvMode;
    uint32_t ContinuousConvMode;
    uint32_t DiscontinuousConvMode;
    uint32_t ExternalTrigConv;
    uint32_t DataAlign;
    uint32_t NbrOfConversion;
} ADC_InitTypeDef;

typedef struct {
    ADC_TypeDef *Instance;
    ADC_InitTypeDef Init;
} ADC_HandleTypeDef;

typedef struct {
    uint32_t Channel;
    uint32_t Rank;
    uint32_t SamplingTime;
} ADC_ChannelConfTypeDef;

HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef *h);
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *h);
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *h, ADC_ChannelConfTypeDef *c);
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *h);
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *h, uint32_t timeout);
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *h);

/* ---------------------------------------------------------------------------
 * NVIC
 * -------------------------------------------------------------------------*/
void HAL_NVIC_SetPriority(IRQn_Type irq, uint32_t preempt, uint32_t sub);
void HAL_NVIC_EnableIRQ(IRQn_Type irq);
void HAL_NVIC_DisableIRQ(IRQn_Type irq);

#ifdef __cplusplus
}
#endif

#endif /* HAL_SHIM_H */
