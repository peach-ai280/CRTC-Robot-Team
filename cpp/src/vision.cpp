/* ============================================================================
 * vision.cpp —— 视觉串口帧解析（C++ 版）
 *
 * 帧格式（14 字节定长，和 vision/k210_find_cube.py 一一对应）：
 *   0    0xAA
 *   1    0x55
 *   2    cls     0=没目标 1=白块(能量单元) 2=黄块(能量核心)
 *   3~4  x       int16，画面中心为 0，右为正（像素）
 *   5~6  y       int16，画面中心为 0，下为正（像素）
 *   7~8  w       像素宽
 *   9~10 h       像素高
 *   11~12 area   像素面积，用来判断远近
 *   13   chk     第 2~12 字节的异或
 *
 * 【为什么用定长帧 + 异或校验，而不是直接 printf 文本】
 *   文本帧解析要处理数字位数、负数、换行，单片机上又慢又容易错。
 *   定长二进制帧解析就是"数到第 14 个字节就收一帧"，出错自动重新同步，
 *   串口偶尔丢一个字节也不会永久错位。异或校验能挡掉绝大多数错帧。
 *
 * 【状态机怎么保证能自恢复】
 *   收到非 AA 就一直在 state 0；收到 AA 后下一个不是 55 就退回 state 0。
 *   所以哪怕中途插入垃圾数据，最多浪费两帧就能重新对齐。
 * ==========================================================================*/

#include "crtc.hpp"

namespace crtc {

static const uint8_t kH1 = 0xAAu;
static const uint8_t kH2 = 0x55u;

void VisionLink::init(void)
{
    f_.cls = 0; f_.x = 0; f_.y = 0;
    f_.w = 0; f_.h = 0; f_.area = 0;
    f_.lastMs = 0; f_.fresh = 0;
    len_ = 0; state_ = 0;
    for (int i = 0; i < kFrameLen; i++) buf_[i] = 0;
}

void VisionLink::feedByte(uint8_t b, uint32_t nowMs)
{
    switch (state_)
    {
    case 0:
        if (b == kH1) { buf_[0] = b; len_ = 1; state_ = 1; }
        break;

    case 1:
        if (b == kH2) { buf_[1] = b; len_ = 2; state_ = 2; }
        else          { state_ = 0; len_ = 0; }
        break;

    case 2:
        buf_[len_++] = b;
        if (len_ >= kFrameLen)
        {
            uint8_t chk = 0;
            for (int i = 2; i < kFrameLen - 1; i++) chk ^= buf_[i];

            if (chk == buf_[kFrameLen - 1])
            {
                f_.cls    = buf_[2];
                f_.x      = (int16_t)(uint16_t)(buf_[3] | (buf_[4] << 8));
                f_.y      = (int16_t)(uint16_t)(buf_[5] | (buf_[6] << 8));
                f_.w      = (uint16_t)(buf_[7]  | (buf_[8]  << 8));
                f_.h      = (uint16_t)(buf_[9]  | (buf_[10] << 8));
                f_.area   = (uint16_t)(buf_[11] | (buf_[12] << 8));
                f_.fresh  = 1;
                f_.lastMs = nowMs;
            }
            state_ = 0; len_ = 0;
        }
        break;

    default:
        state_ = 0; len_ = 0;
        break;
    }
}

void VisionLink::update(uint32_t nowMs)
{
    if (f_.fresh) f_.fresh = 0;
}

bool VisionLink::valid(uint32_t nowMs, uint16_t timeoutMs) const
{
    if (f_.cls == 0) return false;
    if ((uint32_t)(nowMs - f_.lastMs) > (uint32_t)timeoutMs) return false;
    return true;
}

/* 自动对位：目标在画面右边（x>0）就往右转，偏差越大转越快，限制在 maxOut */
float VisionLink::turnCommand(uint16_t deadzonePx, float kp, float maxOut) const
{
    if (f_.cls == 0) return 0.0f;

    float ex = (float)f_.x;
    if (ex >  (float)deadzonePx) ex -= (float)deadzonePx;
    else if (ex < -(float)deadzonePx) ex += (float)deadzonePx;
    else return 0.0f;

    float out = kp * ex;
    if (out >  maxOut) out =  maxOut;
    if (out < -maxOut) out = -maxOut;
    return out;
}

} /* namespace crtc */
