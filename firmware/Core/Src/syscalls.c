/* ============================================================================
 * syscalls.c —— newlib 需要的系统接口
 *
 * 不写这些，链接会报一堆 "undefined reference to _sbrk / _write / _close"。
 * 大部分给个空实现就行，只有 _write 要真的把 printf 的内容发到串口。
 * ==========================================================================*/

#include "hal_shim.h"
#include <sys/stat.h>
#include <errno.h>
#include <stddef.h>

/* huart1 定义在应用层（main.c / test_*.c） */
extern UART_HandleTypeDef huart1;

/* printf 重定向：让 printf 直接打到 USART1（115200） */
int _write(int file, char *ptr, int len)
{
    if (file != 1 && file != 2) { errno = EBADF; return -1; }   /* 只管 stdout/stderr */
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, (uint16_t)len, 500);
    return len;
}

int _read(int file, char *ptr, int len)
{
    (void)file; (void)ptr; (void)len;
    errno = EBADF;
    return -1;
}

/* 堆：给 printf / newlib 内部用。STM32F103C8 只有 20KB RAM，
   堆给多了会挤掉栈，栈溢出是最难查的死机原因，所以这里封顶 1KB。
   用整数而非指针做比较，否则 gcc 会对 &_end 的数组边界发告警。 */
extern uint32_t _estack;
extern uint32_t _end;                        /* 链接脚本里 .bss 之后 */
static uint32_t g_heap_cur = 0;

void *_sbrk(ptrdiff_t incr)
{
    uint32_t base = (uint32_t)&_end;
    uint32_t limit = (uint32_t)&_estack - 2048UL;   /* 给栈留 2KB */

    if (g_heap_cur == 0) g_heap_cur = base;
    if (g_heap_cur + (uint32_t)incr > limit) { errno = ENOMEM; return (void *)-1; }

    uint32_t prev = g_heap_cur;
    g_heap_cur += (uint32_t)incr;
    return (void *)prev;
}

int _close(int file) { (void)file; return -1; }
int _fstat(int file, struct stat *st) { (void)file; st->st_mode = S_IFCHR; return 0; }
int _isatty(int file) { (void)file; return 1; }
int _lseek(int file, int ptr, int dir) { (void)file; (void)ptr; (void)dir; return 0; }
void _kill(int pid) { (void)pid; }
int _getpid(void) { return 1; }
void _exit(int status) { (void)status; __disable_irq(); while (1) { } }
