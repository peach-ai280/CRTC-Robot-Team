/* ============================================================================
 * hal_sim.cpp —— 硬件抽象层的「模拟器版」
 *
 * 同一份 crtc::Chassis / ArmSequencer / VisionLink 代码，
 * 真机链接 hal_mcu.cpp，模拟器链接这个文件，逻辑一行不用改。
 *
 * 它干的事：
 *   - 把 PWM / 舵机脉宽记到数组里，供 Python 读出来检查
 *   - 把 printf 重定向到一块内存缓冲（g_log），Python 读完打印出来
 *   - 时间由测试程序自己推进（simAdvance），不等真实时钟
 * ==========================================================================*/

#include "crtc.hpp"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stddef.h>
#include <errno.h>

/* ---- 观测区：Python 侧通过符号表读这些地址 ------------------------------ */
extern "C" {
    uint32_t g_simMillis = 0;
    uint16_t g_simPwm[4]   = { 0, 0, 0, 0 };
    uint8_t  g_simDir[4]   = { 0, 0, 0, 0 };
    uint16_t g_simServo[4] = { 0, 0, 0, 0 };
    char     g_log[16384]  = { 0 };
    uint32_t g_logLen      = 0;
    uint32_t g_done        = 0;
}

/* printf 落到这里，模拟器里没有 stdout，只能写内存 */
extern "C" int _write(int file, const char *ptr, int len)
{
    (void)file;
    if (len <= 0) return 0;

    uint32_t room = sizeof(g_log) - g_logLen - 1;
    uint32_t n = ((uint32_t)len < room) ? (uint32_t)len : room;
    if (n == 0) return len;

    memcpy(g_log + g_logLen, ptr, n);
    g_logLen += n;
    g_log[g_logLen] = '\0';
    return (int)n;
}

/* newlib 的 stdio 会顺带引用一堆系统调用桩。
 * 真机上有 syscalls.c 提供，模拟器里只用到 _write，其余给空壳就行，
 * 少了它们 ld 会报 "Unknown destination type (ARM/Thumb)"。 */
extern "C" {
    int   _close(int f)                     { (void)f; return -1; }
    int   _fstat(int f, void *st)           { (void)f; (void)st; return -1; }
    int   _isatty(int f)                    { (void)f; return 1; }
    int   _lseek(int f, int off, int wh)    { (void)f; (void)off; (void)wh; return -1; }
    int   _read(int f, char *p, int n)      { (void)f; (void)p; (void)n; return 0; }
    int   _kill(int pid, int sig)           { (void)pid; (void)sig; return -1; }
    int   _getpid(void)                     { return 1; }
    void  _exit(int s)                      { (void)s; for (;;) {} }
}

/* newlib 偶尔会要一点堆（dtoa 之类），给个静态池就够了 */
extern "C" void *_sbrk(ptrdiff_t incr)
{
    static uint8_t  heap[8192];
    static uint32_t used = 0;

    if (incr < 0) return (void *)-1;
    if (used + (uint32_t)incr > sizeof(heap)) { errno = ENOMEM; return (void *)-1; }

    void *p = heap + used;
    used += (uint32_t)incr;
    return p;
}

namespace crtc {
namespace hal {

/* 模拟器专用：手动推进时间。真机版本没有这个函数。 */
void simAdvance(uint32_t ms)
{
    g_simMillis += ms;
}

void motorCompare(int idx, uint16_t cmp)
{
    if (idx >= 0 && idx < 4) g_simPwm[idx] = cmp;
}

void motorDir(int idx, bool high)
{
    if (idx >= 0 && idx < 4) g_simDir[idx] = high ? 1 : 0;
}

void servoPulse(int ch, uint16_t us)
{
    if (ch >= 0 && ch < 4) g_simServo[ch] = us;
}

uint32_t millis(void) { return g_simMillis; }

int uartRead(void) { return -1; }        /* 模拟器里没有串口输入 */

bool limitHit(int idx) { (void)idx; return false; }

float vbat(void) { return 7.40f; }      /* 模拟器假设满电 */

int print(const char *fmt, ...)
{
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    _write(1, buf, n);
    return n;
}

} /* namespace hal */
} /* namespace crtc */
