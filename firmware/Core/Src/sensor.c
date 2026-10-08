/* ============================================================================
 * sensor.c —— 限位开关去抖 + 电池电压采样
 * ==========================================================================*/

#include "sensor.h"
#include "board.h"
#include "config.h"

static uint32_t g_l_last_change = 0, g_r_last_change = 0;
static uint8_t  g_l_stable = 0, g_r_stable = 0;
static uint32_t g_l_hit_ms = 0, g_r_hit_ms = 0;
static float    g_vbat = 0.0f;
static uint32_t g_adc_mark = 0;

void Sensor_Init(void)
{
    g_l_stable = g_r_stable = 0;
    g_vbat = VBAT_NOMINAL;

    /* 先采一次，避免第一帧是 0 */
    HAL_ADC_Start(&VBAT_ADC);
    if (HAL_ADC_PollForConversion(&VBAT_ADC, 10) == HAL_OK)
    {
        uint32_t raw = HAL_ADC_GetValue(&VBAT_ADC);
        g_vbat = (float)raw / 4095.0f * 3.3f * VBAT_ADC_DIV;
    }
}

/* 开关是低电平有效（按下接地，内部上拉） */
static uint8_t read_raw(GPIO_TypeDef *port, uint16_t pin)
{
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) ? 1 : 0;
}

void Sensor_Update(uint32_t now_ms)
{
    uint8_t l = read_raw(LIMIT_L_PORT, LIMIT_L_PIN);
    uint8_t r = read_raw(LIMIT_R_PORT, LIMIT_R_PIN);

    if (l != g_l_stable)
    {
        if ((uint32_t)(now_ms - g_l_last_change) >= LIMIT_DEBOUNCE_MS)
        {
            g_l_stable = l;
            if (l) g_l_hit_ms = now_ms;      /* 记录“被压下”的时刻 */
        }
    }
    else g_l_last_change = now_ms;

    if (r != g_r_stable)
    {
        if ((uint32_t)(now_ms - g_r_last_change) >= LIMIT_DEBOUNCE_MS)
        {
            g_r_stable = r;
            if (r) g_r_hit_ms = now_ms;
        }
    }
    else g_r_last_change = now_ms;

    /* 电压采样：200ms 一次够了，ADC 转换别抢 CPU */
    if ((uint32_t)(now_ms - g_adc_mark) >= 200)
    {
        g_adc_mark = now_ms;
        HAL_ADC_Start(&VBAT_ADC);
        if (HAL_ADC_PollForConversion(&VBAT_ADC, 5) == HAL_OK)
        {
            uint32_t raw = HAL_ADC_GetValue(&VBAT_ADC);
            float v = (float)raw / 4095.0f * 3.3f * VBAT_ADC_DIV;
            /* 一阶滤波，避免电机启停时电压毛刺导致占空比抖动 */
            g_vbat = g_vbat * 0.8f + v * 0.2f;
        }
    }
}

uint8_t Limit_Left(void)  { return g_l_stable; }
uint8_t Limit_Right(void) { return g_r_stable; }

/* “对中”判定：左右两个开关都在 LIMIT_BOTH_WINDOW_MS 内被压过 */
uint8_t Limit_Centered(uint32_t now_ms)
{
    if (!g_l_stable || !g_r_stable) return 0;
    uint32_t dl = now_ms - g_l_hit_ms;
    uint32_t dr = now_ms - g_r_hit_ms;
    if (dl > LIMIT_BOTH_WINDOW_MS) return 0;
    if (dr > LIMIT_BOTH_WINDOW_MS) return 0;
    int32_t diff = (int32_t)g_l_hit_ms - (int32_t)g_r_hit_ms;
    if (diff < 0) diff = -diff;
    return ((uint32_t)diff <= LIMIT_BOTH_WINDOW_MS) ? 1 : 0;
}

float Sensor_GetVbat(void) { return g_vbat; }

uint8_t Sensor_KeyPressed(void)
{
    return (HAL_GPIO_ReadPin(KEY_PORT, KEY_PIN) == GPIO_PIN_RESET) ? 1 : 0;
}
