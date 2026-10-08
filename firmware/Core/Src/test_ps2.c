/* ============================================================================
 * test_ps2.c —— 只测 PS2 手柄接线对不对（**轮子、舵机一个都不动**）
 *
 * 【为什么要单独一个固件】
 *   PS2 是整车唯一的人机入口，但它也是最容易接错的东西：6 根线里
 *   DAT / CMD 两根一接反就读不到数据，VCC 接到 5V 会直接烧接收头。
 *   这个固件把底盘和机构全部拿掉，只留 SPI1 + 串口，接上线就能看结果：
 *   **车不会动、不会窜**，桌面上就能把线接对。
 *
 * 【要做的三件事】
 *   1. 按下面 board.h / ps2.c 顶部的接线表接 6 根线（VCC 必须 3.3V！）
 *   2. 烧这个固件 —— 有 USB 转串口就顺便看串口 115200，没有也不影响，看板载 LED
 *   3. 摇杆会跟着变 = 成功（0x41 是数字模式，重插一次电）
 *
 * 【结果怎么看】
 *   有串口就看串口（模式那一行），没有串口就看板载 LED：
 *
 *     LED 常亮          = 通了！模拟模式，可以装车
 *     LED 每秒闪 2 下   = 线在校准范围外——**PA6 / PA7 那两根反了**，对调
 *     LED 每秒闪 1 下   = 对面压根没回应：查 3.3V、PB0(CS)、PA5(CLK)、共地、对频
 *     LED 每秒双闪（亮-灭-亮-灭，占 0.2 秒）= 连上了但还在数字模式，断电重插一次
 *     LED 完全不亮       = 程序没跑起来（没烧进去 / 没供电）
 *   mode=0x73  → 模拟、绿灯（手柄上 MODE 红灯没亮）  ✅ 能用
 *   mode=0x79  → 模拟、红灯（按过 MODE 键）          ✅ 能用，但我们只读摇杆，无影响
 *   connected=0 → 一直连不上，见下面【连不上怎么办】
 *   LX/LY 不动 → DAT / CMD 接反了（这是最常见的原因）
 *   摇杆松手不是 0 → 正常，128 是中心，固件里已经减过 128，松手应该在 ±3 以内
 *
 * 【连不上怎么办（按顺序排查）】
 *   ① 手柄电池有电吗？手柄上红/绿灯一直闪 = 没连上接收头
 *   ② VCC 是 3.3V 吗？接 5V 大概率把接收头烧了（换一个试试）
 *   ③ ATT→PB0、CLK→PA5、CMD→PA7、DAT→PA6，CMD/DAT 对调试一下试试
 *   ④ GND 和 STM32 共地了吗（不共地通信全乱）
 *   ⑤ 两个手柄红蓝方别拿混，接收头和手柄是配对锁定的
 * ==========================================================================*/

#include "main.h"
#include "bsp.h"
#include "board.h"
#include "config.h"
#include "ps2.h"
#include <stdio.h>

static const struct { uint16_t bit; const char *name; } KEY_NAME[] = {
    {PSB_SELECT,   "SEL"},
    {PSB_START,    "START"},
    {PSB_L1,       "L1"},
    {PSB_R1,       "R1"},
    {PSB_L2,       "L2"},
    {PSB_R2,       "R2"},
    {PSB_TRIANGLE, "△"},
    {PSB_CIRCLE,   "○"},
    {PSB_CROSS,    "×"},
    {PSB_SQUARE,   "□"},
    {PSB_PAD_UP,   "↑"},
    {PSB_PAD_DOWN, "↓"},
    {PSB_PAD_LEFT, "←"},
    {PSB_PAD_RIGHT,"→"},
};

/* ---------------------------------------------------------------------------
 * 板载 LED 状态灯（PC13，低电平点亮）
 *   手边没有 USB 转串口模块时，靠这个也能判断，见下面对照表
 * -------------------------------------------------------------------------*/
static void led_update(void)
{
    uint32_t t   = HAL_GetTick();
    uint32_t p   = t % 1000;
    uint8_t  on  = 0;

    if (ps2.connected && (ps2.mode == 0x73 || ps2.mode == 0x79))
    {
        on = 1;                                     /* 常亮   = 通了，模拟模式 ✅ */
    }
    else if (ps2.connected)
    {
        on = (p < 100) || (p >= 200 && p < 300);    /* 每秒双闪 = 数字模式(0x41)，断电重插 */
    }
    else
    {
        int allff = 1;
        for (int i = 0; i < 9; i++)
            if (ps2.raw[i] != 0xFF) { allff = 0; break; }

        if (allff) on = (p < 100);                  /* 每秒闪 1 下 = 对面没回应 */
        else       on = ((t % 500) < 100);          /* 每秒闪 2 下 = 数据线接反了 */
    }

    HAL_GPIO_WritePin(LED_PORT, LED_PIN, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    BSP_GPIO_Init();
    BSP_SPI1_Init();        /* PS2：PA5/PA6/PA7 + PB0 软片选 */
    BSP_USART1_Init();      /* 打印：PA9 */

    printf("\r\n==== PS2 TEST ====\r\n");
    printf("SYSCLK = %lu Hz\r\n", (unsigned long)SystemCoreClock);
    printf("wiring: VCC->3.3V  GND->GND  ATT->PB0  CLK->PA5  CMD->PA7  DAT->PA6\r\n");
    printf("*** motors and servos are DISABLED in this firmware ***\r\n");

    PS2_Init();

    uint32_t t_log = HAL_GetTick();

    while (1)
    {
        PS2_Scan();                       /* 10ms 一次轮询 + 掉线保护 */
        led_update();                     /* 板载 LED 同步显示状态 */
        uint32_t now = HAL_GetTick();

        if (now - t_log >= 250)           /* 每 250ms 打一行，别刷屏 */
        {
            t_log = now;
            printf("%s mode=0x%02X lost=%u | LX=%4d LY=%4d RX=%4d RY=%4d | ",
                   ps2.connected ? "[ ON ]" : "[ OFF]",
                   ps2.mode, (unsigned)ps2.lost_cnt,
                   ps2.lx, ps2.ly, ps2.rx, ps2.ry);

            if (ps2.btn)
            {
                for (unsigned i = 0; i < sizeof(KEY_NAME) / sizeof(KEY_NAME[0]); i++)
                    if (ps2.btn & KEY_NAME[i].bit)
                        printf("%s ", KEY_NAME[i].name);
            }
            else
            {
                printf("-");
            }
            printf("\r\n");

            /* 连不上时把原始 9 字节打出来 —— 一眼看出是"没回应"还是"接反了" */
            if (!ps2.connected)
            {
                printf("   raw = %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                       ps2.raw[0], ps2.raw[1], ps2.raw[2],
                       ps2.raw[3], ps2.raw[4], ps2.raw[5],
                       ps2.raw[6], ps2.raw[7], ps2.raw[8]);
                printf("   全 FF = 对面压根没回应(查VCC/CS/CLK/共地)"
                       "；不是全 FF = 线接反了(对调 PA6/PA7)\r\n");
            }
        }

        HAL_Delay(10);
    }
}
