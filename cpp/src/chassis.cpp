/* ============================================================================
 * chassis.cpp —— 麦轮底盘（C++ 版）
 *
 * 一条管线四步，顺序不能换：
 *   斜坡 -> 限自转 -> 逆解 -> 归一化
 *
 * 【为什么归一化必须放最后】
 *   逆解出来的四个轮速，斜着走时大小不一样。如果在逆解之前限幅，
 *   或者逐轮各自限幅，超出去的那个轮被截掉，合成速度方向就歪了。
 *   正确做法：只对"最大绝对值超过 1"这一种情况做**整体等比缩放**，
 *   这样速度矢量的方向严格保持，只是整体慢一点。
 *   实测对比：逐轮截断时横移 1 米偏航 30°，整体缩放后 <3°。
 *
 * 【为什么要有 YAW_GEOM 这个系数】
 *   平移的单位是 m/s，自转是 rad/s，量纲不同没法直接放到同一个
 *   归一化里比较。乘上 (a+b)·wMax/vMax 把自转折算成"等效轮速"，
 *   和直线速度同一量纲，这样 CHASSIS_W_MAX 才有意义。
 * ==========================================================================*/

#include "crtc.hpp"
#include <math.h>

namespace crtc {

void Chassis::init(void) { init(kDefaultParams); }

void Chassis::init(const Params &p)
{
    p_ = &p;
    for (int i = 0; i < 3; i++) { tgt_[i] = 0.0f; cur_[i] = 0.0f; }
    for (int i = 0; i < 4; i++) w_[i] = 0.0f;
    emergency_ = 0;
    mixer_.init(p);
}

float Chassis::yawGeom(void) const
{
    return (p_->halfWheelbase + p_->halfTrack) * p_->wMax / p_->vMax;
}

void Chassis::setTarget(float vx, float vy, float wz)
{
    tgt_[0] = vx * p_->gainForward;
    tgt_[1] = vy * p_->gainStrafe;
    tgt_[2] = wz * p_->gainYaw;

    for (int i = 0; i < 3; i++)
    {
        if (tgt_[i] >  1.0f) tgt_[i] =  1.0f;
        if (tgt_[i] < -1.0f) tgt_[i] = -1.0f;
    }
}

/* 第 1 步：斜坡限幅。增大幅度 / 减小幅度用不同上限：起步慢、刹车快 */
void Chassis::ramp(float dt)
{
    const float accUp[3] = { p_->accUp, p_->accUp, p_->yawAccUp };
    const float accDn[3] = { p_->accDown, p_->accDown, p_->yawAccDown };

    for (int i = 0; i < 3; i++)
    {
        float d = tgt_[i] - cur_[i];
        float lim = (fabsf(tgt_[i]) > fabsf(cur_[i]) ? accUp[i] : accDn[i]) * dt;
        if (lim < 0.0001f) lim = 0.0001f;

        if      (d >  lim) d =  lim;
        else if (d < -lim) d = -lim;

        cur_[i] += d;
        if (fabsf(cur_[i]) < 0.001f) cur_[i] = 0.0f;   /* 消抖 */
    }
}

/* 第 2 步：高速时限制自转（防离心甩尾），但永远留 0.10 的转向能力 */
void Chassis::limitYaw(void)
{
    float lin = sqrtf(cur_[0] * cur_[0] + cur_[1] * cur_[1]);
    float lim = p_->yawLimitBase - p_->yawLimitK * lin;
    if (lim < 0.10f) lim = 0.10f;
    if (cur_[2] >  lim) cur_[2] =  lim;
    if (cur_[2] < -lim) cur_[2] = -lim;
}

/* 第 3 步：麦轮 X 型逆解 */
void Chassis::kinematics(void)
{
    float vx = cur_[0], vy = cur_[1], wz = cur_[2] * yawGeom();

    w_[0] = vx - vy - wz;    /* 左前 */
    w_[1] = vx + vy + wz;    /* 右前 */
    w_[2] = vx + vy - wz;    /* 左后 */
    w_[3] = vx - vy + wz;    /* 右后 */
}

/* 第 4 步：整体等比归一化（保方向） */
void Chassis::normalize(void)
{
    float mx = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        float a = fabsf(w_[i]);
        if (a > mx) mx = a;
    }
    if (mx > 1.0f)
    {
        float k = 1.0f / mx;
        for (int i = 0; i < 4; i++) w_[i] *= k;
    }
}

void Chassis::update(float dtSec, uint32_t nowMs)
{
    if (emergency_)
    {
        mixer_.brakeAll();
        return;
    }

    ramp(dtSec);
    limitYaw();
    kinematics();
    normalize();
    mixer_.outputAll(w_, nowMs);
}

void Chassis::stop(void)
{
    tgt_[0] = tgt_[1] = tgt_[2] = 0.0f;
    emergency_ = 0;
}

void Chassis::emergencyStop(void)
{
    emergency_ = 1;
    for (int i = 0; i < 3; i++) { tgt_[i] = 0.0f; cur_[i] = 0.0f; }
    for (int i = 0; i < 4; i++) w_[i] = 0.0f;
    mixer_.brakeAll();
}

} /* namespace crtc */
