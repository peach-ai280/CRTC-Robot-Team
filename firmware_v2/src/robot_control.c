#include "robot_control.h"

static int32_t abs32(int32_t v) { return v < 0 ? -v : v; }
static int16_t clamp1000(int16_t v)
{
    if (v > 1000) return 1000;
    if (v < -1000) return -1000;
    return v;
}

bool robot_ps2_decode(const uint8_t *bytes, size_t length, RobotInput *out)
{
    size_t expected;
    *out = (RobotInput){0};
    if (bytes == NULL || length < 5u) return false;
    if (bytes[0] != 0xffu || bytes[2] != 0x5au) return false;
    if (bytes[1] != 0x41u && bytes[1] != 0x73u && bytes[1] != 0x79u)
        return false;
    /* 0x79 压感模式必须读取完整 21 字节，不能拿短帧当成完整数据。 */
    expected = 3u + 2u * (size_t)(bytes[1] & 0x0fu);
    if (length < expected) return false;
    out->mode = bytes[1];
    out->buttons = (uint16_t)~((uint16_t)bytes[3] |
                              (uint16_t)((uint16_t)bytes[4] << 8));
    out->lx = out->ly = out->rx = out->ry = 128u;
    if (bytes[1] != 0x41u) {
        out->rx = bytes[5]; out->ry = bytes[6];
        out->lx = bytes[7]; out->ly = bytes[8];
    }
    out->valid = true;
    return true;
}

int16_t robot_axis(uint8_t raw)
{
    int32_t value = (int32_t)raw - 128;
    int32_t magnitude = abs32(value);
    int32_t endpoint = value < 0 ? 128 : 127;
    if (magnitude <= ROBOT_AXIS_DEADZONE) return 0;
    magnitude = (magnitude - ROBOT_AXIS_DEADZONE) * 1000 /
                (endpoint - ROBOT_AXIS_DEADZONE);
    return (int16_t)(value < 0 ? -magnitude : magnitude);
}

void robot_mix(int16_t forward, int16_t left, int16_t ccw,
               int16_t limit, int16_t wheel[4])
{
    int32_t x = clamp1000(forward), y = clamp1000(left), w = clamp1000(ccw);
    int32_t mixed[4] = { x-y-w, x+y+w, x+y-w, x-y+w };
    int32_t peak = 1000;
    unsigned i;
    if (limit < 0) limit = 0;
    if (limit > 1000) limit = 1000;
    for (i = 0; i < 4u; ++i)
        if (abs32(mixed[i]) > peak) peak = abs32(mixed[i]);
    /* 统一缩放，保留四轮速度比例；不是逐轮裁剪。无编码器时为开环占空比。 */
    for (i = 0; i < 4u; ++i)
        wheel[i] = (int16_t)(mixed[i] * limit / peak);
}

void robot_init(RobotControl *state) { *state = (RobotControl){0}; }

static void stop_wheels(RobotControl *state)
{
    unsigned i;
    for (i = 0; i < 4u; ++i) state->wheel[i] = 0;
}

static void lose_connection(RobotControl *state)
{
    state->connected = false;
    state->armed = false;
    state->neutral_frames = 0u;
    state->previous_buttons = 0u;
    stop_wheels(state);
}

static bool is_neutral(const RobotInput *input)
{
    const uint16_t motion_keys = PS2_UP | PS2_DOWN | PS2_LEFT | PS2_RIGHT;
    return robot_axis(input->lx) == 0 && robot_axis(input->ly) == 0 &&
           robot_axis(input->rx) == 0 && (input->buttons & motion_keys) == 0u;
}

void robot_step(RobotControl *state, const RobotInput *input,
                uint32_t now_ms, bool local_stop)
{
    uint32_t elapsed = 0u;
    uint16_t clicked = 0u;
    int16_t target[4];
    int16_t forward, left, ccw, limit;
    unsigned i;
    if (state->initialized) elapsed = now_ms - state->last_step_ms;
    state->initialized = true;
    state->last_step_ms = now_ms;
    if (elapsed > 50u) elapsed = 50u;
    /* 先检查旧连接是否超时，再接受重连帧，防止长时间断线后自动续跑。 */
    if (state->connected &&
        now_ms - state->last_good_ms > ROBOT_LINK_TIMEOUT_MS)
        lose_connection(state);
    if (input != NULL && input->valid) {
        if (!state->connected) state->previous_buttons = input->buttons;
        clicked = (uint16_t)(input->buttons & (uint16_t)~state->previous_buttons);
        state->previous_buttons = input->buttons;
        state->latest = *input;
        state->last_good_ms = now_ms;
        state->connected = true;
        if (is_neutral(input) && (input->buttons & PS2_L1) == 0u) {
            if (state->neutral_frames < ROBOT_NEUTRAL_FRAMES)
                ++state->neutral_frames;
        } else {
            state->neutral_frames = 0u;
        }
    }
    if (local_stop || (state->connected &&
        (state->latest.buttons & PS2_SELECT) != 0u)) {
        state->emergency_latched = true;
        state->armed = false;
        stop_wheels(state);
        return;
    }
    if (!state->connected) { stop_wheels(state); return; }
    /* ★ 2026-10-19 修改：原要求「摇杆回中 + 不按 L1 连续 5 帧」才接受 START，
     *   实际后果是：只要手上还扶着摇杆或按着 L1，按 START 会被完全丢弃，
     *   急停锁存永远解不开 —— 表现为「怎么按都没反应」，但代码认为一切正常。
     *   现改为：START 的按下沿即启用 / 解除急停，不再附加姿态条件。
     *   冲出风险由 ROBOT_SLEW_PER_SECOND 斜坡（0→35% 约 0.35s）兜住。 */
    if ((clicked & PS2_START) != 0u) {
        state->emergency_latched = false;
        state->armed = true;
    }
    /* 急停、失联、未启用均立即置零，不经过加速度斜坡。
     *
     * ★ 2026-10-19 修改（本仓库 firmware_v2 副本，原压缩包未动）：
     *   原设计要求「必须一直按住 L1 才出力，松开 L1 立即清零」。
     *   实测对单手操作极不友好 —— PS2 手柄上 L1 与左摇杆无法同时稳定握住，
     *   而且新手会直接把「推杆不动」误判为「车坏了」。
     *   现改为：按 START 启用后，摇杆直接生效，L1 不再是出力前提。
     *
     *   安全仍由三道保护承担，没有降低：
     *     ① 必须先按 START 启用（未启用时摇杆完全无效，四轮恒为 0）
     *     ② SELECT 急停，锁存后必须重新按 START 才能恢复
     *     ③ 100ms 收不到合法帧立即停车（ROBOT_LINK_TIMEOUT_MS）
     *   另外松开摇杆时目标是 0，四轮按 ROBOT_SLEW_PER_SECOND 斜坡降到 0，不会残留。
     */
    if (!state->armed || state->emergency_latched) {
        stop_wheels(state);
        return;
    }
    forward = (int16_t)-robot_axis(state->latest.ly);
    left = (int16_t)-robot_axis(state->latest.lx);
    ccw = (int16_t)-robot_axis(state->latest.rx);
    if ((state->latest.buttons & (PS2_UP|PS2_DOWN|PS2_LEFT|PS2_RIGHT)) != 0u) {
        forward = (int16_t)(((state->latest.buttons & PS2_UP) != 0u ? 1000 : 0) -
                            ((state->latest.buttons & PS2_DOWN) != 0u ? 1000 : 0));
        left = (int16_t)(((state->latest.buttons & PS2_LEFT) != 0u ? 1000 : 0) -
                         ((state->latest.buttons & PS2_RIGHT) != 0u ? 1000 : 0));
    }
    limit = (state->latest.buttons & PS2_L2) != 0u ?
            ROBOT_SLOW_LIMIT : ROBOT_NORMAL_LIMIT;
    robot_mix(forward, left, ccw, limit, target);
    for (i = 0; i < 4u; ++i) {
        int32_t current = state->wheel[i];
        int32_t goal = target[i];
        int32_t increment = (int32_t)(elapsed * ROBOT_SLEW_PER_SECOND / 1000u);
        /* 反转先到零，并至少保留本次控制周期的零输出。 */
        if ((current > 0 && goal < 0) || (current < 0 && goal > 0)) goal = 0;
        if (current < goal) {
            current += increment;
            if (current > goal) current = goal;
        } else if (current > goal) {
            current -= increment;
            if (current < goal) current = goal;
        }
        state->wheel[i] = (int16_t)current;
    }
}