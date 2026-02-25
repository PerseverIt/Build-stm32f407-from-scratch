/**
 * STM32F407VET6 最小链接脚本
 *
 * 链接脚本告诉链接器：
 * 1. 芯片有哪些内存区域
 * 2. 各个段（.text, .data, .bss等）放在哪里
 * 3. 导出符号供启动文件使用
 */

/* 程序入口点 */
ENTRY(Reset_Handler)

/* 内存布局 - 查阅STM32F407数据手册 */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 512K   /* 512KB Flash */
    RAM (rwx)   : ORIGIN = 0x20000000, LENGTH = 128K   /* 128KB SRAM */
    /* CCM RAM暂时不用 */
}

/* 栈顶地址 = RAM结束地址 */
_estack = ORIGIN(RAM) + LENGTH(RAM);

/* 段定义 */
SECTIONS
{
    /* 中断向量表 - 必须放在Flash最开始 */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))   /* KEEP防止被优化掉 */
        . = ALIGN(4);
    } >FLASH

    /* 代码段 */
    .text :
    {
        . = ALIGN(4);
        *(.text)               /* 所有.text段 */
        *(.text*)              /* 所有.text开头的段 */
        . = ALIGN(4);
        _etext = .;            /* 代码段结束地址 */
    } >FLASH

    /* 只读数据段（常量、字符串等）*/
    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } >FLASH

    /*
     * .data段 - 有初始值的全局变量
     *
     * 关键概念：LMA vs VMA
     * - VMA (Virtual Memory Address): 运行时地址，在RAM中
     * - LMA (Load Memory Address): 加载地址，在Flash中
     *
     * .data段运行时在RAM，但初始值存储在Flash
     * 启动时需要从Flash复制到RAM
     */
    _sidata = LOADADDR(.data);   /* .data在Flash中的地址 */

    .data :
    {
        . = ALIGN(4);
        _sdata = .;              /* RAM中.data起始地址 */
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;              /* RAM中.data结束地址 */
    } >RAM AT>FLASH              /* VMA在RAM，LMA在FLASH */

    /*
     * .bss段 - 未初始化的全局变量
     * 只需要在RAM中分配空间，启动时清零
     */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;               /* .bss起始地址 */
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;               /* .bss结束地址 */
    } >RAM
}
