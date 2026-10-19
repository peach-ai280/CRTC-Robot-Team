/* ============================================================================
 * test_ps2scan.c —— PS2 接线「24 种接法全试一遍」诊断固件
 *                  （一次性诊断用，比赛固件不是这个）
 *
 * 【为什么要有这个东西】
 *   2026-10-17 实测电气状态：
 *     PA5 = 悬空       PA7 = 悬空
 *     PA6 = 一直在高    PB0 = 一直在高
 *   这说明接收器是活的（有东西在驱动），但六种「CLK/CMD/CS 互换」全试过
 *   都不回话 —— 说明问题不在那三个脚之间，而是**哪根线才是 DAT** 搞错了。
 *
 *   所以这一版把 A5 / A7 / A6 / B0 **四个脚在四个角色之间做全排列**
 *   （4! = 24 种），每种都用软件打时钟硬试一帧 9 字节。
 *   哪个排列能收到 0x73 / 0x41 / 0x79，真实接线就是它。
 *
 * 【为什么不用 SPI 外设】
 *   SPI1 的引脚是焊死映射的（PA5=SCK、PA6=MISO、PA7=MOSI），换不了。
 *   要做全排列只能用软件一位一位地打时钟。
 *
 * 【安全性】
 *   - 每种排列只试 1 帧（约 1ms），一轮 24 种 ≈ 25ms，总共只跑 3 轮。
 *   - 如果某一轮把接收器的输出脚当成了输出脚去拉低，会有极短的对灌，
 *     所以刻意把时间压到最短，跑完立刻把所有脚改回「输入+上拉」释放掉。
 *
 * 【结果怎么读】
 *   由 tools/ps2_scan.py 通过 ST-LINK 读 RAM：
 *     g_scan[0]                    = 0x50534341 表示扫完了
 *     g_scan[1]                    = 成功的排列编号 0~23，0xFFFFFFFF = 全失败
 *     g_scan[2]                    = 成功时的状态字节
 *     g_scan[3]                    = 电气签名（见下）
 *     g_scan[8 + p*10 + i]         = 排列 p 收到的第 i 字节
 *     g_scan[8 + p*10 + 9]         = 排列 p 的得分（非 FF 字节数），0xFF = 判定成功
 * ==========================================================================*/
#include "main.h"
#include "bsp.h"
#include "board.h"

volatile uint32_t g_scan[512];

#define P_A5   0
#define P_A7   1
#define P_A6   2
#define P_B0   3

/* 角色顺序：[0]=CLK  [1]=CMD  [2]=CS  [3]=DAT */
static uint8_t PERM[24][4];

static void build_perms(void)
{
    uint8_t k = 0;
    for (uint8_t a = 0; a < 4; a++)
        for (uint8_t b = 0; b < 4; b++) { if (b == a) continue;
            for (uint8_t c = 0; c < 4; c++) { if (c == a || c == b) continue;
                for (uint8_t d = 0; d < 4; d++) { if (d == a || d == b || d == c) continue;
                    PERM[k][0] = a; PERM[k][1] = b;
                    PERM[k][2] = c; PERM[k][3] = d;
                    k++;
                } } }
}

/* ------------------------------------------------------------------ 引脚 */
static uint8_t pa_of(uint8_t id)
{
    if (id == P_A5) return 5;
    if (id == P_A7) return 7;
    return 6;
}

static void pin_write(uint8_t id, uint8_t v)
{
    if (id == P_B0)
    {
        if (v) GPIOB->BSRR = 0x0001u; else GPIOB->BRR = 0x0001u;
    }
    else
    {
        uint8_t p = pa_of(id);
        if (v) GPIOA->BSRR = (1u << p); else GPIOA->BRR = (1u << p);
    }
}

static uint8_t pin_read(uint8_t id)
{
    if (id == P_B0) return (uint8_t)(GPIOB->IDR & 1u);
    return (uint8_t)((GPIOA->IDR >> pa_of(id)) & 1u);
}

/* nib: 0x8 = 输入(上/下拉由 ODR 定)  0x3 = 推挽输出 50MHz */
static void set_mode(uint8_t id, uint8_t nib)
{
    if (id == P_B0)
    {
        uint32_t v = GPIOB->CRL;
        v = (v & ~0xFu) | nib;
        GPIOB->CRL = v;
    }
    else
    {
        uint8_t p = pa_of(id);
        uint32_t v = GPIOA->CRL;
        v = (v & ~(0xFu << (p * 4))) | ((uint32_t)nib << (p * 4));
        GPIOA->CRL = v;
    }
}

/* 半周期延时：约 4us → 位周期 8us → 约 125kHz */
static void bb_delay(void)
{
    for (volatile uint32_t i = 0; i < 55; i++) { __NOP(); }
}

static void gap(uint16_t n)
{
    for (volatile uint16_t i = 0; i < n; i++) { __NOP(); }
}

/* ------------------------------------------------------------------ 协议 */
/* 模式 3：时钟空闲高；下降沿摆数据，上升沿双方采样 */
static uint8_t xfer_byte(uint8_t clk, uint8_t cmd, uint8_t dat, uint8_t tx)
{
    uint8_t in = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
        pin_write(clk, 0);
        pin_write(cmd, (uint8_t)((tx >> (7 - i)) & 1));
        bb_delay();
        pin_write(clk, 1);
        bb_delay();
        in = (uint8_t)((in << 1) | pin_read(dat));
        bb_delay();
    }
    return in;
}

static uint8_t ps2_poll(uint8_t clk, uint8_t cmd, uint8_t cs, uint8_t dat, uint8_t *rx)
{
    static const uint8_t tx[9] = {0x01, 0x42, 0, 0, 0, 0, 0, 0, 0};

    pin_write(cs, 1);
    gap(150);
    pin_write(cs, 0);
    gap(250);
    for (uint8_t i = 0; i < 9; i++)
    {
        rx[i] = xfer_byte(clk, cmd, dat, tx[i]);
        gap(150);
    }
    pin_write(cs, 1);
    gap(2500);

    if (rx[1] == 0x41 || rx[1] == 0x73 || rx[1] == 0x79) return 2;
    if (rx[2] == 0x5A) return 1;
    return 0;
}

/* ------------------------------------------------------------------ 初始化 */
static void scan_gpio_init(void)
{
    RCC->APB2ENR |= (1u << 2) | (1u << 3) | (1u << 4);

    /* 四个候选脚全部先设成「输入 + 上拉」（释放，谁都不驱动） */
    for (uint8_t id = 0; id < 4; id++)
    {
        set_mode(id, 0x8);
        pin_write(id, 1);
    }

    /* PC13 指示灯 */
    uint32_t crh = GPIOC->CRH;
    crh = (crh & ~(0xFu << 20)) | (0x3u << 20);
    GPIOC->CRH = crh;
    GPIOC->BSRR = (1u << 13);
}

/* 电气签名：outs[0..3] = A5,A7,A6,B0 的下拉读数；outs[4..7] = 上拉读数 */
static void elec_signature(uint32_t *out)
{
    for (uint8_t id = 0; id < 4; id++)
    {
        set_mode(id, 0x8);
        pin_write(id, 0);            /* 下拉 */
        gap(60);
        out[id] = pin_read(id);
    }
    for (uint8_t id = 0; id < 4; id++)
    {
        set_mode(id, 0x8);
        pin_write(id, 1);            /* 上拉 */
        gap(60);
        out[4 + id] = pin_read(id);
    }
    /* 收工，全部释放 */
    for (uint8_t id = 0; id < 4; id++) { set_mode(id, 0x8); pin_write(id, 1); }
}

/* ------------------------------------------------------------------ main */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    BSP_GPIO_Init();
    scan_gpio_init();
    build_perms();

    for (uint16_t i = 0; i < 512; i++) g_scan[i] = 0;
    g_scan[1] = 0xFFFFFFFF;
    g_scan[2] = 0xFFFFFFFF;

    /* 先量一遍电气状态（此时谁都没驱动） */
    uint32_t sig[8];
    elec_signature(sig);
    g_scan[3] = (sig[0] & 1u) | ((sig[1] & 1u) << 1) | ((sig[2] & 1u) << 2) | ((sig[3] & 1u) << 3)
              | ((sig[4] & 1u) << 8) | ((sig[5] & 1u) << 9) | ((sig[6] & 1u) << 10) | ((sig[7] & 1u) << 11);

    for (uint8_t round = 0; round < 3; round++)
    {
        for (uint8_t p = 0; p < 24; p++)
        {
            uint8_t clk = PERM[p][0], cmd = PERM[p][1];
            uint8_t cs  = PERM[p][2], dat = PERM[p][3];

            /* 四个脚先全部释放成上拉输入 */
            for (uint8_t id = 0; id < 4; id++) { set_mode(id, 0x8); pin_write(id, 1); }
            /* 再把三个输出角色设成推挽输出并拉高（CS 不选中） */
            set_mode(clk, 0x3); pin_write(clk, 1);
            set_mode(cmd, 0x3); pin_write(cmd, 1);
            set_mode(cs,  0x3); pin_write(cs,  1);
            gap(80);

            uint8_t rx[9];
            for (uint8_t i = 0; i < 9; i++) rx[i] = 0xFF;
            uint8_t res = ps2_poll(clk, cmd, cs, dat, rx);

            /* 释放，别一直顶着 */
            for (uint8_t id = 0; id < 4; id++) { set_mode(id, 0x8); pin_write(id, 1); }

            uint8_t score = 0;
            for (uint8_t i = 0; i < 9; i++) if (rx[i] != 0xFF) score++;
            if (res == 2) score = 0xFF;          /* 判定通过 */

            uint32_t base = 8u + (uint32_t)p * 10u;
            uint8_t old = (uint8_t)(g_scan[base + 9] & 0xFF);
            if (score > old || old == 0)
            {
                for (uint8_t i = 0; i < 9; i++) g_scan[base + i] = rx[i];
                g_scan[base + 9] = score;
            }
            if (res == 2 && g_scan[1] == 0xFFFFFFFF)
            {
                g_scan[1] = p;
                g_scan[2] = rx[1];
            }
        }
        g_scan[0] = 0x50534341;

        GPIOC->BRR = (1u << 13); HAL_Delay(30);
        GPIOC->BSRR = (1u << 13); HAL_Delay(30);
    }

    /* 收工：所有脚释放，灯常亮 */
    for (uint8_t id = 0; id < 4; id++) { set_mode(id, 0x8); pin_write(id, 1); }
    GPIOC->BRR = (1u << 13);
    while (1) { }
}
