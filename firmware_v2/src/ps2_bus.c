#include "ps2_bus.h"
#include "board.h"
#include "board_config.h"
#include "stm32f103_min.h"

volatile uint8_t g_ps2_last_bytes[21];
volatile uint8_t g_ps2_last_length;
static bool due(uint32_t now, uint32_t deadline)
{
    return (int32_t)(now - deadline) >= 0;
}
static uint8_t transfer(uint8_t tx)
{
    uint8_t rx = 0u;
    unsigned bit;
    /* PS2是低位先发，不能照搬旧工程的 SPI_FIRSTBIT_MSB。
     * CLK空闲高，下降沿换数据，上升沿后读取DAT。
     */
    for (bit = 0u; bit < 8u; ++bit) {
        if ((tx & (1u << bit)) != 0u) gpio_high(GPIOA,PS2_CMD_PIN);
        else gpio_low(GPIOA,PS2_CMD_PIN);
        gpio_low(GPIOA,PS2_CLK_PIN);
        board_delay_us(PS2_HALF_BIT_US);
        gpio_high(GPIOA,PS2_CLK_PIN);
        /* ⚠ 2026-10-19 实测修正：原来在上升沿后再等一个半位(4us)才采，
         *   读到的每一位都是上一位的值（收到 FF 39 2D…，标准应为 FF 73 5A…，
         *   每个字节正好 >>1），robot_ps2_decode 因 bytes[2]!=0x5A 判无效。
         *   改为上升沿后立即采样，采样点前移半个周期。周期不变(仍 125kHz)。 */
        if ((GPIOA->IDR & (1u << PS2_DAT_PIN)) != 0u)
            rx |= (uint8_t)(1u << bit);
        board_delay_us(PS2_HALF_BIT_US);
    }
    board_delay_us(16u); /* 字节间空隙，不依赖接收器ACK线。 */
    return rx;
}
static void begin_packet(void)
{
    gpio_high(GPIOA,PS2_CLK_PIN);
    gpio_high(GPIOA,PS2_CMD_PIN);
    gpio_low(GPIOB,PS2_CS_PIN);
    board_delay_us(20u);
}
static void end_packet(void)
{
    gpio_high(GPIOB,PS2_CS_PIN);
    gpio_high(GPIOA,PS2_CMD_PIN);
    board_delay_us(20u);
}
static void command(const uint8_t *data, unsigned length)
{
    unsigned i;
    begin_packet();
    for (i = 0u; i < length; ++i) (void)transfer(data[i]);
    end_packet();
}
static bool poll_packet(RobotInput *out)
{
    uint8_t rx[21];
    unsigned i, length;
    begin_packet();
    rx[0] = transfer(0x01u);
    rx[1] = transfer(0x42u);
    rx[2] = transfer(0x00u);
    /* 由模式低4位决定包长：数字5字节、模拟9字节、压感21字节。 */
    length = 3u + 2u * (unsigned)(rx[1] & 0x0fu);
    if (length < 5u || length > 21u) length = 9u;
    for (i = 3u; i < length; ++i) rx[i] = transfer(0x00u);
    end_packet();
    for (i = 0u; i < 21u; ++i) g_ps2_last_bytes[i] = i < length ? rx[i] : 0u;
    g_ps2_last_length = (uint8_t)length;
    return robot_ps2_decode(rx,length,out);
}
void ps2_service_init(Ps2Service *service, uint32_t now_ms)
{
    *service = (Ps2Service){0u,now_ms,now_ms + 300u};
}
bool ps2_service_poll(Ps2Service *service, bool allow_config,
                      uint32_t now_ms, RobotInput *out)
{
    static const uint8_t enter[] = {0x01,0x43,0x00,0x01,0x00};
    static const uint8_t mode[] = {0x01,0x44,0x00,0x01,0x03,0x00,0x00,0x00,0x00};
    static const uint8_t rumble_off[] = {0x01,0x4d,0x00,0xff,0xff,0xff,0xff,0xff,0xff};
    static const uint8_t leave[] = {0x01,0x43,0x00,0x00,0x5a,0x5a,0x5a,0x5a,0x5a};
    bool valid;
    *out = (RobotInput){0};
    /* 包括退出配置后的等待，不能只在 stage 非零时检查间隔。 */
    if (!due(now_ms,service->due_ms)) return false;
    if (service->stage != 0u) {
        switch (service->stage) {
        case 1u: command(enter,sizeof enter); break;
        case 2u: command(mode,sizeof mode); break;
        case 3u: command(rumble_off,sizeof rumble_off); break;
        default: command(leave,sizeof leave); break;
        }
        ++service->stage;
        if (service->stage > 4u) service->stage = 0u;
        service->due_ms = now_ms + 20u;
        return false;
    }
    valid = poll_packet(out);
    service->due_ms = now_ms + BOARD_CONTROL_MS;
    /* 仅在未启用行驶时协商模拟模式，不能在车行驶时做配置事务。 */
    if (allow_config && (!valid || out->mode == 0x41u) &&
        due(now_ms,service->next_config_ms)) {
        service->stage = 1u;
        service->due_ms = now_ms + 10u;
        service->next_config_ms = now_ms + 2000u;
        out->valid = false;
        return false;
    }
    return valid;
}
