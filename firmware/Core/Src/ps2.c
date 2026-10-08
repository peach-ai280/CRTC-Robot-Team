/* ============================================================================
 * ps2.c —— PS2 无线手柄 SPI 驱动
 *
 * 【硬件前提】
 *   STM32 SPI1 配置：全双工主机、软件 NSS、MSB First、8 位、
 *   CPOL = High（空闲时钟为高）、CPHA = 2 Edge（第二个边沿采样）、
 *   波特率预分频 = 256（72MHz/256 ≈ 281kHz，PS2 上限约 500kHz，慢一点更稳）
 *
 * 【为什么必须用模拟量模式】
 *   手柄上电默认是数字模式，摇杆只回 00/FF 两档，没法做速度控制。
 *   PS2_Init() 里会进配置模式打开模拟量 + 关掉震动，红灯常亮才说明成功。
 *
 * 【调试现象对照表】
 *   串口一直打印 "PS2: no handshake"  -> 检查 DI/DI-DAT 是否接反（MISO/MOSI 反）
 *   mode 一直是 0x41                  -> 没进配置模式，检查 CS 脚和延时
 *   摇杆只有 0 和 255 两个值           -> 还是数字模式，重新上电一次再试
 *   偶尔掉线                          -> 手柄没对频 / 电池没电 / SPI 太快
 * ==========================================================================*/

#include "ps2.h"
#include "board.h"
#include "config.h"
#include <string.h>

PS2_t ps2;

/* ---------------------------------------------------------------------------
 * 微秒级延时：72MHz 下大致校准，不精确但够 PS2 用。
 * 如果你的工程主频改成了别的，这里的循环次数要按比例改。
 * -------------------------------------------------------------------------*/
static void ps2_delay_us(uint16_t us)
{
    uint32_t n = (uint32_t)us * 9;   /* 72MHz 实测约 9 个循环 = 1us */
    while (n--) __NOP();
}

/* SPI 收发一个字节（PS2 是半双工式问答，主机发什么不重要，重要的是同时收回来） */
static uint8_t ps2_xfer(uint8_t tx)
{
    uint8_t rx = 0xFF;
    HAL_SPI_TransmitReceive(&PS2_SPI, &tx, &rx, 1, 100);
    ps2_delay_us(15);                /* 字节间延时，便宜手柄必须要 */
    return rx;
}

#define PS2_CS_LOW()   HAL_GPIO_WritePin(PS2_CS_PORT, PS2_CS_PIN, GPIO_PIN_RESET)
#define PS2_CS_HIGH()  HAL_GPIO_WritePin(PS2_CS_PORT, PS2_CS_PIN, GPIO_PIN_SET)

/* ---------------------------------------------------------------------------
 * 底层：发一串命令，CS 全程拉低
 * -------------------------------------------------------------------------*/
static void ps2_cmd(const uint8_t *tx, uint8_t *rx, uint8_t len)
{
    PS2_CS_LOW();
    ps2_delay_us(20);
    for (uint8_t i = 0; i < len; i++)
        rx[i] = ps2_xfer(tx[i]);
    PS2_CS_HIGH();
    ps2_delay_us(50);
}

/* ---------------------------------------------------------------------------
 * 读一次手柄数据（9 字节标准帧）
 * -------------------------------------------------------------------------*/
uint8_t PS2_Poll(void)
{
    static const uint8_t poll[9] = {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t rx[9];

    memset(rx, 0xFF, sizeof(rx));
    ps2_cmd(poll, rx, 9);
    memcpy(ps2.raw, rx, 9);           /* 留着给调试固件打印，排查接线用 */

    /* rx[0] 必须是 0xFF（空闲），rx[2] 必须是 0x5A（数据就绪握手位） */
    if (rx[1] == 0x00 || rx[2] != 0x5A)
        return 0;

    ps2.btn_prev = ps2.btn;
    ps2.btn      = (uint16_t)(rx[4] << 8) | rx[3];       /* 低电平有效 */
    ps2.btn      = (uint16_t)(~ps2.btn);                 /* 反转成“1=按下”，逻辑更好读 */
    ps2.btn_click = (uint16_t)(ps2.btn & (ps2.btn ^ ps2.btn_prev));

    ps2.lx = (int16_t)rx[7] - 128;
    ps2.ly = (int16_t)rx[8] - 128;
    ps2.rx = (int16_t)rx[5] - 128;
    ps2.ry = (int16_t)rx[6] - 128;

    ps2.mode      = rx[1];
    ps2.lost_cnt  = 0;
    ps2.connected = 1;
    return 1;
}

/* ---------------------------------------------------------------------------
 * 初始化：进配置模式 -> 开模拟量 -> 关震动 -> 退出
 * 注意：配置命令之间必须留足够延时，否则手柄不认
 * -------------------------------------------------------------------------*/
void PS2_Init(void)
{
    uint8_t rx[9];

    PS2_CS_HIGH();
    HAL_Delay(50);

    /* 短轮询 3 次，手柄要先“醒”过来 */
    for (uint8_t i = 0; i < 3; i++) { PS2_Poll(); HAL_Delay(20); }

    /* 进入配置模式 */
    const uint8_t enter_cfg[]  = {0x01, 0x43, 0x00, 0x01, 0x00};
    ps2_cmd(enter_cfg, rx, 5);
    HAL_Delay(10);

    /* 打开模拟量模式（0x01=开，0x00=关） */
    const uint8_t set_mode[]   = {0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
    ps2_cmd(set_mode, rx, 9);
    HAL_Delay(10);

    /* 关掉大震动电机（省电，也避免震动把接插件震松） */
    const uint8_t set_rumble[] = {0x01, 0x4D, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF};
    ps2_cmd(set_rumble, rx, 9);
    HAL_Delay(10);

    /* 退出配置模式 */
    const uint8_t exit_cfg[]   = {0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};
    ps2_cmd(exit_cfg, rx, 9);
    HAL_Delay(20);

    ps2.connected = 0;
    ps2.lost_cnt  = 0;
    memset(ps2.raw, 0xFF, sizeof(ps2.raw));   /* 初始=全 FF，便于分辨"从没回应过" */
    ps2.btn = ps2.btn_prev = ps2.btn_click = 0;
    ps2.lx = ps2.ly = ps2.rx = ps2.ry = 0;
}

uint8_t PS2_Key(uint16_t key)     { return (ps2.btn & key) ? 1 : 0; }
uint8_t PS2_Clicked(uint16_t key) { return (ps2.btn_click & key) ? 1 : 0; }

/* ---------------------------------------------------------------------------
 * 放在 5ms 周期任务里。掉线超过阈值就上报，由 app 层决定停机。
 * -------------------------------------------------------------------------*/
void PS2_Scan(void)
{
    if (!PS2_Poll())
    {
        if (ps2.lost_cnt < 0xFFFF) ps2.lost_cnt++;
        if (ps2.lost_cnt > PS2_LOST_THRESHOLD)
        {
            ps2.connected = 0;
            ps2.btn = ps2.btn_click = 0;
            ps2.lx = ps2.ly = ps2.rx = ps2.ry = 0;   /* 掉线时不要残留摇杆量 */
        }
    }
}
