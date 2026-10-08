/* ============================================================================
 * vision.c —— 定长帧接收状态机
 * 用法：在 stm32f1xx_it.c 的 HAL_UART_RxCpltCallback 里调用 Vision_RxByte()。
 *      最简单可靠的接法：CubeMX 打开 USART1 全局中断，main 里先
 *      HAL_UART_Receive_IT(&huart1, &g_rx1, 1)，回调里再重新开启。
 * ==========================================================================*/

#include "vision.h"
#include "config.h"

#if USE_VISION

Vision_t vision;

#define FRAME_LEN   14
#define H1          0xAA
#define H2          0x55

static uint8_t  g_buf[FRAME_LEN];
static uint8_t  g_len = 0;
static uint8_t  g_state = 0;   /* 0=等AA 1=等55 2=收数据 */

void Vision_Init(void)
{
    g_len = 0; g_state = 0;
    vision.cls = 0; vision.x = 0; vision.y = 0;
    vision.w = 0; vision.h = 0; vision.area = 0;
    vision.fresh = 0; vision.last_ms = 0;
}

void Vision_RxByte(uint8_t b)
{
    switch (g_state)
    {
    case 0:
        if (b == H1) { g_buf[0] = b; g_len = 1; g_state = 1; }
        break;
    case 1:
        if (b == H2) { g_buf[1] = b; g_len = 2; g_state = 2; }
        else { g_state = 0; g_len = 0; }
        break;
    case 2:
        g_buf[g_len++] = b;
        if (g_len >= FRAME_LEN)
        {
            /* 校验 */
            uint8_t chk = 0;
            for (uint8_t i = 2; i < FRAME_LEN - 1; i++) chk ^= g_buf[i];
            if (chk == g_buf[FRAME_LEN - 1])
            {
                vision.cls  = g_buf[2];
                vision.x    = (int16_t)(g_buf[3] | (g_buf[4] << 8));
                vision.y    = (int16_t)(g_buf[5] | (g_buf[6] << 8));
                vision.w    = (uint16_t)(g_buf[7]  | (g_buf[8]  << 8));
                vision.h    = (uint16_t)(g_buf[9]  | (g_buf[10] << 8));
                vision.area = (uint16_t)(g_buf[11] | (g_buf[12] << 8));
                vision.fresh = 1;
                vision.last_ms = HAL_GetTick();
            }
            g_state = 0; g_len = 0;
        }
        break;
    default:
        g_state = 0; g_len = 0;
        break;
    }
}

void Vision_Update(uint32_t now_ms)
{
    if (vision.fresh) vision.fresh = 0;
    if ((uint32_t)(now_ms - vision.last_ms) > VISION_TIMEOUT_MS)
        vision.cls = 0;
}

uint8_t Vision_Valid(void)
{
    if (vision.cls == 0) return 0;
    if ((uint32_t)(HAL_GetTick() - vision.last_ms) > VISION_TIMEOUT_MS) return 0;
    return 1;
}

#else

/* 没开视觉时给一套空实现，保证 main.c 不用写 #if */
Vision_t vision;
void    Vision_Init(void)                  { vision.cls = 0; vision.fresh = 0; }
void    Vision_RxByte(uint8_t b)           { (void)b; }
void    Vision_Update(uint32_t now_ms)     { (void)now_ms; }
uint8_t Vision_Valid(void)                 { return 0; }

#endif /* USE_VISION */
