/* ============================================================================
 * servo.cpp —— 舵机限速缓动（C++ 版）
 *
 * 【为什么必须限速】
 *   舵机转动的瞬间电流很大，MG995 堵转能到 1.2A。四个舵机同时甩，
 *   5V 降压模块直接被拉塌 -> 单片机欠压复位 -> 表现为"一动机构就重启、
 *   PS2 掉线"。限速让同一时刻只有一个舵机在大电流，是最省事的解法
 *   （比堆电容可靠，电容只能扛几十毫秒）。
 *
 * 【怎么判断超行程】
 *   听到"吱吱"声 = 舵机在硬撑到不了的行程。把 servoMinUs / servoMaxUs
 *   往中间各收 30us，直到声音消失。
 * ==========================================================================*/

#include "crtc.hpp"
#include <math.h>

namespace crtc {

void Servo::init(const Params &p, int channel, float startDeg)
{
    p_   = &p;
    ch_  = channel;
    cur_ = startDeg;
    tgt_ = startDeg;
    apply();
}

void Servo::apply(void)
{
    float a = cur_;
    if (a < 0.0f)   a = 0.0f;
    if (a > 180.0f) a = 180.0f;

    float us = (float)p_->servoMinUs +
               (float)(p_->servoMaxUs - p_->servoMinUs) * (a / 180.0f);
    hal::servoPulse(ch_, (uint16_t)(us + 0.5f));
}

void Servo::setTarget(float deg)
{
    if (deg < 0.0f)   deg = 0.0f;
    if (deg > 180.0f) deg = 180.0f;
    tgt_ = deg;
}

void Servo::jump(float deg)
{
    setTarget(deg);
    cur_ = tgt_;
    apply();
}

void Servo::update(float dtSec)
{
    float step = p_->servoDps * dtSec;
    float d    = tgt_ - cur_;

    if (fabsf(d) <= step)
    {
        if (cur_ != tgt_)
        {
            cur_ = tgt_;
            apply();
        }
    }
    else
    {
        cur_ += (d > 0.0f ? step : -step);
        apply();
    }
}

bool Servo::isMoving(void) const
{
    return fabsf(tgt_ - cur_) > 0.5f;
}

} /* namespace crtc */
