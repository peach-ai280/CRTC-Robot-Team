/*
 * sim.ld —— 离线验证用的扁平链接脚本
 * 代码 / 只读数据 / 已初始化数据 / bss 全部排在一块连续地址里，
 * 这样 objcopy -O binary 出来的就是一个整块镜像，模拟器直接一把装进去。
 * bss 排在最后（它在模拟器里本来就是全 0 的内存，镜像里可以不带）。
 */
ENTRY(main)

SECTIONS
{
  . = 0x08000000;
  .text   : { *(.text*)   }
  .rodata : { *(.rodata*) }
  .data   : { *(.data*)   }
  .bss    : { *(.bss*) *(COMMON) }
}
