/* ============================================================================
 * motor.cpp —— 电机输出层（C++ 版）
 *
 * 管的三件事：
 *   1) 死区补偿：TT 马达在占空比低于约 0.18 时根本不转。不补偿的话，
 *      表现是"推小油门车不动，推到一半突然蹿出去"。
 *   2) 起步助力（kick）：静摩擦比动摩擦大，起步瞬间多叠一点占空比，
 *      120ms 内线性衰减掉。缺了它会"咔"一下才动。
 *   3) 电压补偿：电池从 8.4V 掉到 6.8V，同样占空比速度差很多。
 *
 * 输出拆分（DRV8833 的 PHASE/ENABLE 接法）：
 *   IN1 = PWM，IN2 = 方向电平
 *   正转：duty = |v|，DIR = 0
 *   反转：duty = 1 - |v|，DIR = 1
 *   这个"反转让占空比反着算"是新手最容易搞错的地方：DRV8833 在
 *   PHASE/ENABLE 模式下，IN1 拉高才通电，所以 PWM 占空比越大转越快，
 *   而方向由 IN2 决定；反转时必须把有效占空比取反。
 * ==========================================================================*/

#include "crtc.hpp"
#include <math.h>

namespace crtc {

void MotorMixer::init(const Params &p)
{
    p_ = &p;
    for (int i = 0; i < 4; i++)
    {
        duty_[i]      = 0.0f;
        kickStart_[i] = 0;
        wasIdle_[i]   = 1;
    }
    vcomp_ = 1.0f;
}

void MotorMixer::setVbat(float vbat)
{
    if (!p_->vbatComp) { vcomp_ = 1.0f; return; }
    /* 没接电池（USB 5V 供电）时不做补偿，否则会按 7.4/5.0 放大占空比 */
    if (vbat < 3.0f) { vcomp_ = 1.0f; return; }

    float k = p_->vbatNominal / vbat;
    if (k > p_->vbatCompMax) k = p_->vbatCompMax;
    if (k < 0.8f)            k = 0.8f;
    vcomp_ = k;
}

void MotorMixer::set(int idx, float speed, uint32_t nowMs)
{
    if (idx < 0 || idx > 3) return;

    /* 1) 限幅 */
    if (speed >  1.0f) speed =  1.0f;
    if (speed < -1.0f) speed = -1.0f;

    /* 2) 转向修正 + 左右微调 */
    float s = speed * (float)p_->dirSign[idx] * p_->trim[idx];
    if (s >  1.0f) s =  1.0f;
    if (s < -1.0f) s = -1.0f;

    float mag = fabsf(s);

    /* 3) 起步助力：从静止跨过阈值时叠 boost，然后线性衰减 */
    float kick = 0.0f;
    if (mag > 0.03f)
    {
        if (wasIdle_[idx])
        {
            kickStart_[idx] = nowMs;
            wasIdle_[idx]   = 0;
        }
        uint32_t dt = nowMs - kickStart_[idx];
        if (dt < (uint32_t)p_->kickMs)
            kick = p_->kickDuty * (1.0f - (float)dt / (float)p_->kickMs);
    }
    else
    {
        wasIdle_[idx] = 1;
    }

    /* 4) 死区补偿：0..1 的速度映射到 DEADZONE..1 的占空比 */
    float duty;
    if (mag < 0.001f) duty = 0.0f;
    else              duty = p_->deadzone + mag * (1.0f - p_->deadzone);

    duty += kick;

    /* 5) 电压补偿 */
    duty *= vcomp_;
    if (duty > 1.0f) duty = 1.0f;
    if (duty < 0.0f) duty = 0.0f;

    /* 6) 拆成 PWM + DIR */
    uint16_t cmp;
    if (s > 0.001f)
    {
        hal::motorDir(idx, false);
        cmp = (uint16_t)(duty * (float)kPwmPeriod);
    }
    else if (s < -0.001f)
    {
        hal::motorDir(idx, true);
        cmp = (uint16_t)((1.0f - duty) * (float)kPwmPeriod);
    }
    else
    {
        if (p_->idleBrake)
        {
            hal::motorDir(idx, true);
            cmp = kPwmPeriod;          /* DIR=1 + 满占空比 = 双高刹车 */
        }
        else
        {
            hal::motorDir(idx, false);
            cmp = 0;                   /* coast */
        }
        duty = 0.0f;
    }

    if (cmp > kPwmPeriod) cmp = kPwmPeriod;
    hal::motorCompare(idx, cmp);
    duty_[idx] = duty;
}

void MotorMixer::outputAll(const float s[4], uint32_t nowMs)
{
    for (int i = 0; i < 4; i++) set(i, s[i], nowMs);
}

void MotorMixer::brakeAll(void)
{
    for (int i = 0; i < 4; i++)
    {
        hal::motorDir(i, true);
        hal::motorCompare(i, kPwmPeriod);
        duty_[i]    = 1.0f;
        wasIdle_[i] = 1;
    }
}

void MotorMixer::coastAll(void)
{
    for (int i = 0; i < 4; i++)
    {
        hal::motorDir(i, false);
        hal::motorCompare(i, 0);
        duty_[i]    = 0.0f;
        wasIdle_[i] = 1;
    }
}

} /* namespace crtc */
