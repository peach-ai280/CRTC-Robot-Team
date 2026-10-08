/* ============================================================================
 * test_vision.c —— 只测视觉串口协议（不需要 K210 也能测）
 *
 * 【为什么能不接摄像头就测】
 *   按 t 键，单片机会自己造一帧合法数据喂给接收状态机。这样你可以在没有
 *   摄像头的时候先验证"帧解析 + 校验 + P 控制"这一整条链路是对的，
 *   等 K210 到了只需要对协议，不用从头 debug。
 *
 * 【串口指令（115200，不用回车）】
 *   t   发一帧自造测试数据（cls=1, x=40, y=30, area=6000）
 *   y   发一帧黄色能量核心（cls=2, 面积大到触发"已到位"）
 *   n   发一帧"没找到目标"（cls=0）
 *   p   持续打印当前视觉状态 3 秒
 *
 * 【打印内容说明】
 *   cls   1=白色能量单元  2=黄色能量核心  0=没找到
 *   x/y   目标中心像素坐标（画面中心一般是 160,120）
 *   area  像素面积（越大说明越近）
 *   turn  P 控制器算出来的转向量 = KP * (x - 画面中心)
 *         正数 = 目标在右边 -> 车要往右转
 *          这个值只在 |偏差| > VISION_X_DEADZONE 时非零，死区内不纠（防抖）
 * ==========================================================================*/

#include "main.h"
#include "bsp.h"
#include "board.h"
#include "config.h"
#include "vision.h"
#include <stdio.h>

static volatile char g_cmd = 0;
static uint32_t g_print_until = 0;

/* 画面中心：K210 用 QVGA 320x240，中心就是 (160,120) */
#define IMG_CENTER_X  160

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        /* 不是本固件的指令字符时，就当作视觉帧数据喂给状态机 */
        char c = (char)g_uart1_rx;
        if (c == 't' || c == 'y' || c == 'n' || c == 'p')
            g_cmd = c;
        else
            Vision_RxByte((uint8_t)c);

        HAL_UART_Receive_IT(&huart1, (uint8_t *)&g_uart1_rx, 1);
    }
}

/* 造一帧 14 字节数据塞进状态机（模拟 K210 发来的数据） */
static void feed_frame(uint8_t cls, int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t area)
{
    uint8_t f[14];
    f[0]  = 0xAA;
    f[1]  = 0x55;
    f[2]  = cls;
    f[3]  = (uint8_t)(x & 0xFF);
    f[4]  = (uint8_t)((x >> 8) & 0xFF);
    f[5]  = (uint8_t)(y & 0xFF);
    f[6]  = (uint8_t)((y >> 8) & 0xFF);
    f[7]  = (uint8_t)(w & 0xFF);
    f[8]  = (uint8_t)((w >> 8) & 0xFF);
    f[9]  = (uint8_t)(h & 0xFF);
    f[10] = (uint8_t)((h >> 8) & 0xFF);
    f[11] = (uint8_t)(area & 0xFF);
    f[12] = (uint8_t)((area >> 8) & 0xFF);

    uint8_t chk = 0;
    for (uint8_t i = 2; i < 13; i++) chk ^= f[i];
    f[13] = chk;

    for (uint8_t i = 0; i < 14; i++) Vision_RxByte(f[i]);
}

/* P 控制：把像素偏差换算成转向量 */
static float vision_turn(void)
{
    int16_t err = vision.x - IMG_CENTER_X;
    if (err >  VISION_X_DEADZONE) { ; }
    else if (err < -VISION_X_DEADZONE) { ; }
    else return 0.0f;

    float turn = VISION_TURN_KP * (float)err;
    if (turn >  VISION_TURN_MAX) turn =  VISION_TURN_MAX;
    if (turn < -VISION_TURN_MAX) turn = -VISION_TURN_MAX;
    return turn;
}

int main(void)
{
    uint32_t t_log = 0;

    HAL_Init();
    SystemClock_Config();
    BSP_GPIO_Init();
    BSP_USART1_Init();

    Vision_Init();

    printf("\r\n==== VISION TEST (USE_VISION=%d) ====\r\n", USE_VISION);
    printf("SYSCLK = %lu Hz\r\n", SystemCoreClock);
    printf("keys: t=white frame  y=yellow frame  n=no target  p=print 3s\r\n");
    printf("or send raw 14-byte frames from K210 at 115200\r\n");

    t_log = HAL_GetTick();

    while (1)
    {
        uint32_t now = HAL_GetTick();

        if (g_cmd)
        {
            char c = g_cmd;
            g_cmd = 0;
            if      (c == 't') feed_frame(1,  40, 130, 60, 60, 6000);
            else if (c == 'y') feed_frame(2, 250, 120, 90, 90, 12000);
            else if (c == 'n') feed_frame(0,   0,   0,  0,  0,     0);
            else if (c == 'p') g_print_until = now + 3000;
        }

        Vision_Update(now);

        if ((uint32_t)(now - t_log) >= 300)
        {
            t_log = now;
            if (now < g_print_until || Vision_Valid())
            {
                printf("vis cls=%d x=%d y=%d w=%u h=%u area=%u | valid=%d turn=%+.3f | near=%d\r\n",
                       vision.cls, vision.x, vision.y, vision.w, vision.h, vision.area,
                       Vision_Valid(), vision_turn(),
                       (vision.area > VISION_NEAR_AREA) ? 1 : 0);
            }
        }
    }
}
