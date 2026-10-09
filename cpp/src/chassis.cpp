/* ============================================================================
 * chassis.cpp —— 底盘（C++ 版，麦轮 / 差速两套运动学）
 *
 * 一条管线四步，顺序不能换：
 *   斜坡 -> 限自转 -> 运动学 -> 归一化
 *
 * 【两台车怎么共用这一份】
 *   外圈三步（斜坡/限自转/归一化）**两台车完全一样**，唯一不同的是第三步：
 *     kMecanum -> 麦轮 X 型逆解（vy 有效，能横移）
 *     kDiff    -> 差速/滑移转向（vy 物理上不存在，丢弃）
 *   所以只用编译期常量 kChassisKind 分派那一个函数，其余零改动。
 *   ⚠ 橡胶轮千万别照抄麦轮公式：麦轮公式里有 vy 项，橡胶轮没有辊子，
 *     照抄的后果是"手柄推横移 → 四轮两两反向对打 → 车原地抽搐、电流翻倍"。
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
 *   差速转向的 TURN_K 本身就是无单位的（半轮距/轮半径），不需要再折算。
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

/* 差速转弯系数：轮心到车体中心线的横向距离 / 轮半径。
 * 纯几何、无单位，量纲上是"rad/s 的 wz 乘以它变成轮速"。
 * ⚠ 换轮子必须改 p_->wheelRadius，否则转向量算错。 */
float Chassis::turnK(void) const
{
    return p_->halfTrack / p_->wheelRadius;
}

void Chassis::setTarget(float vx, float vy, float wz)
{
    tgt_[0] = vx * p_->gainForward;
    tgt_[1] = vy * p_->gainStrafe;
    tgt_[2] = wz * (CRTC_IS_DIFF() ? p_->gainYawDiff : p_->gainYaw);

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

/* 第 3 步：按车型分派运动学内核（宏数值直接比，见 crtc.hpp 0.1 节的说明） */
void Chassis::kinematics(void)
{
#if CRTC_IS_DIFF()
    kinematicsDiff();
#else
    kinematicsMecanum();
#endif
}

/* 第 3 步-a：麦轮 X 型逆解（A 车）
 *   vy 项在这里真正起作用，所以麦轮能横移、能斜走。 */
void Chassis::kinematicsMecanum(void)
{
    float vx = cur_[0], vy = cur_[1], wz = cur_[2] * yawGeom();

    w_[0] = vx - vy - wz;    /* 左前 */
    w_[1] = vx + vy + wz;    /* 右前 */
    w_[2] = vx + vy - wz;    /* 左后 */
    w_[3] = vx - vy + wz;    /* 右后 */
}

/* 第 3 步-b：差速 / 滑移转向（B 车，橡胶轮）
 *
 *   左侧两轮 = vx + K·wz
 *   右侧两轮 = vx - K·wz
 *
 *   【为什么逐字对应那个"轮距/轮半径"】
 *   车要原地转 wz（rad/s），左右轮心画的是半径等于"半轮距"的圆，
 *   线速度 = wz × 半轮距（m/s）；电机看的是**轮子转多快**（rad/s），
 *   所以要再除以轮半径 —— 合起来就是 K = 半轮距 / 轮半径。
 *
 *   【vy 去哪了】
 *   橡胶轮是刚性圆柱，滚动方向只有一个。横向平移在物理上不存在，
 *   这里直接丢掉（不是 bug）。想保住横移只能回到麦轮。
 *   前两个数还有个副作用：斜走（vx=vy≠0）会退化成纯前进。 */
void Chassis::kinematicsDiff(void)
{
    float vx   = cur_[0];
    float turn = cur_[2] * turnK();

    w_[0] = vx + turn;       /* 左前 */
    w_[1] = vx - turn;       /* 右前 */
    w_[2] = vx + turn;       /* 左后 */
    w_[3] = vx - turn;       /* 右后 */
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
