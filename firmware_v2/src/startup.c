#include <stdint.h>

int main(void);
void SysTick_Handler(void);
static void Default_Handler(void);
void Reset_Handler(void);

/* 64KB Flash / 20KB RAM，向量表由硬件在复位时加载栈顶和入口。
 * 一共59项：16个内核向量 + STM32F103中容量43个外设向量。
 * 本版只启用SysTick，所有未使用中断进入默认入口。
 */
typedef void (*Vector)(void);
__attribute__((used,section("RESET"),aligned(256)))
const Vector g_vectors[59] = {
    (Vector)(uintptr_t)0x20005000u, Reset_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    Default_Handler, 0, 0, 0, 0, Default_Handler,
    Default_Handler, 0, Default_Handler, SysTick_Handler,
    Default_Handler,Default_Handler,Default_Handler,Default_Handler,
    Default_Handler,Default_Handler,Default_Handler,Default_Handler,
    Default_Handler,Default_Handler,Default_Handler,Default_Handler,
    Default_Handler,Default_Handler,Default_Handler,Default_Handler,
    Default_Handler,Default_Handler,Default_Handler,Default_Handler,
    Default_Handler,Default_Handler,Default_Handler,Default_Handler,
    Default_Handler,Default_Handler,Default_Handler,Default_Handler,
    Default_Handler,Default_Handler,Default_Handler,Default_Handler,
    Default_Handler,Default_Handler,Default_Handler,Default_Handler,
    Default_Handler,Default_Handler,Default_Handler,Default_Handler,
    Default_Handler,Default_Handler,Default_Handler
};

#ifdef __ARMCC_VERSION
/* Keil ARM Compiler 6：由C运行库完成散装加载和清零，再进入main。 */
extern void __main(void);
void Reset_Handler(void)
{
    __main();
    for (;;) { }
}
#else
/* GNU Arm编译器路径：使用随工程附带的 .ld 脚本，无需厂家库。 */
extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss;
void Reset_Handler(void)
{
    uint32_t *source = &_sidata;
    volatile uint32_t *dest;
    for (dest = &_sdata; dest < &_edata; ++dest) *dest = *source++;
    for (dest = &_sbss; dest < &_ebss; ++dest) *dest = 0u;
    (void)main();
    for (;;) { }
}
#endif
static void Default_Handler(void)
{
    /* 异常时由独立看门狗复位；立即撤掉电机输出，不能依赖SysTick。 */
    volatile uint32_t *tim4 = (volatile uint32_t *)(uintptr_t)0x40000800u;
    volatile uint32_t *br = (volatile uint32_t *)(uintptr_t)0x40010c14u;
    tim4[13] = 0u; tim4[14] = 0u; tim4[15] = 0u; tim4[16] = 0u;
    *br = 15u << 12;
    for (;;) { }
}
