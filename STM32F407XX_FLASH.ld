/*
******************************************************************************
**
**  File        : LinkerScript.ld
**
**  Author      : STM32CubeMX
**
**  Abstract    : Linker script for STM32F407VETx series
**                512Kbytes FLASH and 192Kbytes RAM
**
**                Set heap size, stack size and stack location according
**                to application requirements.
**
**                Set memory bank area and size if external memory is used.
**
**  Target      : STMicroelectronics STM32
**
**  Distribution: The file is distributed "as is," without any warranty
**                of any kind.
**
*****************************************************************************
** @attention
**
** <h2><center>&copy; COPYRIGHT(c) 2025 STMicroelectronics</center></h2>
**
** Redistribution and use in source and binary forms, with or without modification,
** are permitted provided that the following conditions are met:
**   1. Redistributions of source code must retain the above copyright notice,
**      this list of conditions and the following disclaimer.
**   2. Redistributions in binary form must reproduce the above copyright notice,
**      this list of conditions and the following disclaimer in the documentation
**      and/or other materials provided with the distribution.
**   3. Neither the name of STMicroelectronics nor the names of its contributors
**      may be used to endorse or promote products derived from this software
**      without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
** AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
** IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
** FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
** DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
** SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
** CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
** OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
*****************************************************************************
*/


/*-----------------------------------------------------------------------------
 * ENTRY - 程序入口点
 *-----------------------------------------------------------------------------
 * 语法：ENTRY(符号名)
 * 作用：
 *   1. 告诉 GDB 调试器从哪里开始设置断点和反汇编
 *   2. 作为链接器垃圾回收的根节点：链接器会从这个入口开始，递归标记所有被引用的
 *      函数和数据，未被标记的会被回收。如果没有 ENTRY，链接器可能认为所有代码都
 *      没用，全部回收掉
 *   3. 生成 ELF 文件头中的 e_entry 字段
 *
 * 注意：这个入口点不影响 CPU 实际从哪里开始执行！
 *       Cortex-M 的启动地址由中断向量表的第二个字（Reset_Handler 地址）决定
 *-----------------------------------------------------------------------------
 */
ENTRY(Reset_Handler)

/*-----------------------------------------------------------------------------
 * 全局符号定义
 *-----------------------------------------------------------------------------
 * 语法：符号 = 表达式;
 * 这些符号会被导出，启动文件可以通过 .word 或 ldr 指令引用它们
 *
 * _estack：栈顶地址，Cortex-M 使用满递减栈（Full Descending Stack），
 *          栈从高地址向低地址生长，所以栈顶设在 RAM 的最高地址
 *
 * ORIGIN(RAM) 获取 RAM 区域起始地址：0x20000000
 * LENGTH(RAM) 获取 RAM 区域长度：128K = 0x20000
 * 所以 _estack = 0x20000000 + 0x20000 = 0x20020000
 *-----------------------------------------------------------------------------
 */
_estack = ORIGIN(RAM) + LENGTH(RAM);    /* end of RAM */

/*-----------------------------------------------------------------------------
 * 堆栈大小检查阈值
 *-----------------------------------------------------------------------------
 * 这两个符号定义了堆和栈的最小保留空间
 * 链接器会在 ._user_heap_stack 段中检查 RAM 是否足够容纳这些空间
 * 如果 .bss 段太大导致剩余空间不足，链接器会报错：
 *   "region `RAM' overflowed by xxx bytes"
 *
 * _Min_Heap_Size：堆的最小大小，供 malloc() 等动态内存分配使用
 * _Min_Stack_Size：栈的最小大小，供函数调用、局部变量使用
 *-----------------------------------------------------------------------------
 */
_Min_Heap_Size = 0x200;      /* required amount of heap  */
_Min_Stack_Size = 0x400;     /* required amount of stack */

/*-----------------------------------------------------------------------------
 * MEMORY 命令 - 定义芯片的物理内存布局
 *-----------------------------------------------------------------------------
 * 语法：MEMORY { 区域名 (属性) : ORIGIN = 起始地址, LENGTH = 长度 }
 *
 * 属性说明：
 *   r = 可读 (readable)
 *   w = 可写 (writable)
 *   x = 可执行 (executable)
 *   a = 可分配 (allocatable)，表示运行时需要占用空间
 *   ! = 取反，如 !w 表示不可写
 *
 * 常见组合：
 *   (rx)  = 只读可执行，用于 Flash（存放代码和常量）
 *   (xrw) = 可读可写可执行，用于 RAM（存放变量，也可执行 RAM 中的代码）
 *
 * STM32F407 内存映射（参考 Reference Manual）：
 *   - Flash：0x08000000 - 0x0807FFFF (512KB)
 *   - SRAM1：0x20000000 - 0x2001BFFF (112KB)  ┐
 *   - SRAM2：0x2001C000 - 0x2001FFFF (16KB)   ├─ 共128KB，连续
 *   - CCM：  0x10000000 - 0x1000FFFF (64KB)   ← 仅CPU可访问，DMA无法访问
 *-----------------------------------------------------------------------------
 */
MEMORY
{
  RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 128K  /* 主 SRAM，代码和DMA都能访问 */
  CCMRAM (xrw)    : ORIGIN = 0x10000000, LENGTH = 64K   /* Core Coupled Memory，仅CPU可访问，适合放栈或频繁访问的数据 */
  FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 512K  /* 片内 Flash，只读可执行 */
}

/*-----------------------------------------------------------------------------
 * SECTIONS 命令 - 定义输出段的布局
 *-----------------------------------------------------------------------------
 * 这是链接脚本的核心，决定了：
 *   1. 哪些输入段合并成哪个输出段
 *   2. 每个输出段放在哪个内存区域
 *   3. 段的加载地址（LMA）和运行地址（VMA）
 *
 * 输出段定义语法：
 *   .段名 : { 段内容 } >VMA区域 AT>LMA区域
 *
 * 注意花括号内的点号 . 是局部的位置计数器，表示当前段内的偏移
 *-----------------------------------------------------------------------------
 */
SECTIONS
{
  /*---------------------------------------------------------------------------
   * .isr_vector 段 - 中断向量表
   *---------------------------------------------------------------------------
   * 【为什么必须放在最前面？】
   * Cortex-M 芯片复位后，硬件会自动从地址 0x00000000 读取：
   *   - 前4字节：初始栈顶指针（MSP）
   *   - 后4字节：复位处理函数地址（Reset_Handler）
   *
   * STM32 通过 BOOT 引脚配置地址映射：
   *   - BOOT0=0：0x08000000 映射到 0x00000000（从 Flash 启动）
   *   - BOOT0=1, BOOT1=0：从系统存储器启动（内置 Bootloader）
   *   - BOOT0=1, BOOT1=1：从 SRAM 启动
   *
   * 所以中断向量表必须位于 Flash 的起始位置（0x08000000）
   *
   * 【KEEP 的作用】
   * 链接器有垃圾回收功能（-gc-sections），会删除未被引用的段
   * 但中断向量表不是被代码"调用"的，而是被硬件读取的
   * KEEP 告诉链接器：这个段很重要，不要删除！
   *
   * 【ALIGN(4) 的作用】
   * ARM 要求向量表 4 字节对齐，否则会触发 HardFault
   * . = ALIGN(4) 将位置计数器对齐到 4 字节边界
   *---------------------------------------------------------------------------
   */
  .isr_vector :
  {
    . = ALIGN(4);                /* 确保4字节对齐 */
    KEEP(*(.isr_vector))         /* 保留所有输入文件中的 .isr_vector 段，这是启动文件中定义的向量表 */
    . = ALIGN(4);
  } >FLASH                       /* 放入 FLASH 区域，从 0x08000000 开始 */

  /*---------------------------------------------------------------------------
   * .text 段 - 程序代码
   *---------------------------------------------------------------------------
   * 收集所有目标文件中的代码段，按顺序排列
   *
   * 【通配符语法】
   * *(.text)   - 匹配所有文件的 .text 段
   * *(.text*)  - 匹配所有以 .text 开头的段，如 .text.foo、.text.bar
   *              GCC 使用 -ffunction-sections 时，每个函数会生成独立的 .text.xxx 段
   *
   * 【.glue_7 和 .glue_7t 段】
   * ARM 和 Thumb 指令集之间切换时需要的桥接代码（interworking veneer）
   * - .glue_7：ARM → Thumb 的桥接
   * - .glue_7t：Thumb → ARM 的桥接
   * Cortex-M 只支持 Thumb，但编译器可能仍会生成这些段
   *
   * 【.eh_frame 段】
   * Exception Handling Frame，C++ 异常处理和调试器栈回溯用的元数据
   * 裸机嵌入式一般不用 C++ 异常，但 GDB 调试时可能需要
   *
   * 【.init 和 .fini 段】
   * .init：程序初始化代码，在 main() 之前执行
   * .fini：程序结束代码，在 main() 返回后执行
   * 通常放置 C++ 全局构造/析构函数或 __attribute__((constructor/destructor))
   *
   * 【_etext 符号】
   * 定义一个全局符号，值为 .text 段结束地址
   * 可用于计算代码段大小或作为后续段的起始参考
   *---------------------------------------------------------------------------
   */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* 所有 .text 段（主代码） */
    *(.text*)          /* 所有 .text.xxx 段（使用 -ffunction-sections 时生成） */
    *(.glue_7)         /* ARM→Thumb 桥接代码 */
    *(.glue_7t)        /* Thumb→ARM 桥接代码 */
    *(.eh_frame)       /* 异常处理帧信息，调试器栈回溯用 */

    KEEP (*(.init))    /* 初始化代码段，不要被垃圾回收 */
    KEEP (*(.fini))    /* 结束代码段，不要被垃圾回收 */

    . = ALIGN(4);
    _etext = .;        /* 导出符号：代码段结束地址 */
  } >FLASH

  /*---------------------------------------------------------------------------
   * .rodata 段 - 只读数据
   *---------------------------------------------------------------------------
   * 存放 const 常量、字符串字面量、查找表等只读数据
   * 放在 Flash 中，节省 RAM 空间
   *
   * 示例：
   *   const int lookup_table[] = {1, 2, 3};  → .rodata
   *   char *str = "Hello";                   → 字符串 "Hello" 在 .rodata
   *---------------------------------------------------------------------------
   */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* 所有 .rodata 段 */
    *(.rodata*)        /* 所有 .rodata.xxx 段 */
    . = ALIGN(4);
  } >FLASH

  /*---------------------------------------------------------------------------
   * .ARM.extab 段 - ARM 异常展开表
   *---------------------------------------------------------------------------
   * 语法：.段名 (READONLY) : { ... }
   * READONLY 是 GCC11+ 的新特性，明确标记该段只读，帮助链接器优化
   *
   * 这个段包含 C++ 异常处理的展开信息（unwinding info）
   * 当抛出异常时，运行时库用它来找到应该执行的 catch 块
   * 裸机嵌入式通常不用 C++ 异常，但保留此段以兼容某些库
   *
   * .gnu.linkonce.armextab.* 是老式的段合并语法，现已被 COMDAT 替代
   *---------------------------------------------------------------------------
   */
  .ARM.extab (READONLY) : /* The "READONLY" keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(4);
  } >FLASH

  /*---------------------------------------------------------------------------
   * .ARM 段 - ARM 异常索引表
   *---------------------------------------------------------------------------
   * 也叫 .ARM.exidx 段，是异常展开的索引表
   * 每个函数对应一个条目，指向该函数的展开信息
   *
   * __exidx_start 和 __exidx_end：
   * 这两个符号标记异常索引表的边界，C++ 运行时库（如 libstdc++）会引用它们
   * 即使不用 C++ 异常，某些库（如 newlib）也可能引用这些符号
   *---------------------------------------------------------------------------
   */
  .ARM (READONLY) : /* The "READONLY" keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    __exidx_start = .;      /* 异常索引表起始地址 */
    *(.ARM.exidx*)          /* 收集所有函数的异常索引 */
    __exidx_end = .;        /* 异常索引表结束地址 */
    . = ALIGN(4);
  } >FLASH

  /*---------------------------------------------------------------------------
   * 初始化数组段 - C++ 全局构造函数和 __attribute__((constructor))
   *---------------------------------------------------------------------------
   * 【执行顺序】
   * 启动代码调用 __libc_init_array() 时，会按以下顺序执行：
   *   1. .preinit_array - 预初始化函数（极少使用）
   *   2. .init 段中的代码
   *   3. .init_array - 构造函数数组（主要用这个）
   *
   * 【数组结构】
   * 这些段是函数指针数组，每个元素是一个 void (*)(void) 函数指针
   * 编译器会自动把全局对象的构造函数地址放入这个数组
   *
   * 【PROVIDE_HIDDEN 的作用】
   * 语法：PROVIDE_HIDDEN(符号 = 值)
   * - PROVIDE：定义弱符号，如果用户代码已经定义了同名符号，使用用户的
   * - HIDDEN：符号不导出到动态符号表（对静态链接的裸机程序影响不大）
   * 这里用它来标记数组的起止地址，供 __libc_init_array() 遍历
   *
   * 【SORT() 的作用】
   * 对段名进行字典序排序，确保 .init_array.00001 在 .init_array.00002 之前
   * 这样可以控制构造函数的执行顺序
   *---------------------------------------------------------------------------
   */
  .preinit_array (READONLY) : /* The "READONLY" keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__preinit_array_start = .);   /* 数组起始地址 */
    KEEP (*(.preinit_array*))                     /* 保留所有 .preinit_array 段 */
    PROVIDE_HIDDEN (__preinit_array_end = .);     /* 数组结束地址 */
    . = ALIGN(4);
  } >FLASH

  .init_array (READONLY) : /* The "READONLY" keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))                 /* 先放排序后的 .init_array.xxx */
    KEEP (*(.init_array*))                        /* 再放普通的 .init_array */
    PROVIDE_HIDDEN (__init_array_end = .);
    . = ALIGN(4);
  } >FLASH

  /*---------------------------------------------------------------------------
   * .fini_array 段 - C++ 全局析构函数和 __attribute__((destructor))
   *---------------------------------------------------------------------------
   * 与 .init_array 对应，存放析构函数指针数组
   * 在 main() 返回后由 __libc_fini_array() 调用
   * 裸机程序一般不会从 main() 返回，所以这个段通常不会被执行
   *---------------------------------------------------------------------------
   */
  .fini_array (READONLY) : /* The "READONLY" keyword is only supported in GCC11 and later, remove it if using GCC10 or earlier. */
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
    . = ALIGN(4);
  } >FLASH

  /*---------------------------------------------------------------------------
   * _sidata 符号 - .data 段在 Flash 中的加载地址
   *---------------------------------------------------------------------------
   * 【LOADADDR() 函数】
   * 语法：LOADADDR(.段名)
   * 返回指定段的 LMA（Load Memory Address，加载地址）
   *
   * 【为什么需要这个符号？】
   * .data 段包含已初始化的全局变量，运行时需要在 RAM 中
   * 但程序烧录时，初始值必须存在 Flash 中
   * 所以 .data 段有两个地址：
   *   - VMA (运行地址)：RAM 中的地址，由 _sdata 标记
   *   - LMA (加载地址)：Flash 中的地址，由 _sidata 标记
   *
   * 启动代码用这三个符号完成搬运：
   *   for (src = _sidata, dst = _sdata; dst < _edata; )
   *       *dst++ = *src++;
   *---------------------------------------------------------------------------
   */
  _sidata = LOADADDR(.data);

  /*---------------------------------------------------------------------------
   * .data 段 - 已初始化的可读写数据
   *---------------------------------------------------------------------------
   * 存放有非零初始值的全局变量和静态变量
   * 例如：int count = 100; 或 static char buffer[] = "hello";
   *
   * 【>RAM AT> FLASH 语法】
   * >RAM     : VMA 在 RAM，即程序运行时访问的是 RAM 地址
   * AT>FLASH : LMA 在 FLASH，即初始值存储在 Flash 中
   *
   * 这就是"影子拷贝"机制：
   *   Flash: [代码][常量][.data初始值]  ← 烧录时的样子
   *   RAM:          [.data运行时]       ← 启动后搬运过来
   *
   * 【.RamFunc 段】
   * 某些函数需要在 RAM 中执行（如 Flash 编程函数、高性能代码）
   * 用 __attribute__((section(".RamFunc"))) 标记的函数会放在这里
   * 启动时和 .data 一起被搬运到 RAM
   *---------------------------------------------------------------------------
   */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;        /* 导出符号：.data 段起始地址（RAM 中） */
    *(.data)           /* 所有 .data 段 */
    *(.data*)          /* 所有 .data.xxx 段 */
    *(.RamFunc)        /* 需要在 RAM 中执行的函数 */
    *(.RamFunc*)

    . = ALIGN(4);
    _edata = .;        /* 导出符号：.data 段结束地址（RAM 中） */
  } >RAM AT> FLASH     /* VMA 在 RAM，LMA 在 FLASH */

  /*---------------------------------------------------------------------------
   * .ccmram 段 - CCM RAM 数据
   *---------------------------------------------------------------------------
   * CCM (Core Coupled Memory) 是 STM32F4 特有的 64KB RAM
   * 特点：
   *   - 仅 CPU 可以访问，DMA 无法访问
   *   - 通过 D-Bus 直连 CPU，访问速度极快
   *   - 适合存放：栈、频繁访问的数据、临时缓冲区
   *
   * 使用方法：
   *   __attribute__((section(".ccmram"))) int fast_buffer[1024];
   *
   * 【重要提示】
   * 如果在 CCM 中放置已初始化变量，启动代码需要额外处理搬运
   * 当前启动文件可能没有搬运 .ccmram 的代码，需要自行添加
   *---------------------------------------------------------------------------
   */
  _siccmram = LOADADDR(.ccmram);   /* CCM 数据在 Flash 中的加载地址 */

  .ccmram :
  {
    . = ALIGN(4);
    _sccmram = .;       /* CCM 段起始地址 */
    *(.ccmram)
    *(.ccmram*)

    . = ALIGN(4);
    _eccmram = .;       /* CCM 段结束地址 */
  } >CCMRAM AT> FLASH   /* VMA 在 CCMRAM，LMA 在 FLASH */

  /*---------------------------------------------------------------------------
   * .bss 段 - 未初始化的可读写数据
   *---------------------------------------------------------------------------
   * 存放未初始化或初始化为零的全局变量和静态变量
   * 例如：int counter; 或 static char buffer[1024];
   *
   * 【为什么 .bss 不需要 AT>FLASH？】
   * .bss 段的变量初始值都是 0，没必要在 Flash 中存储一堆 0
   * 链接器只记录 .bss 的大小，不分配 Flash 空间
   * 启动代码负责将 .bss 区域清零
   *
   * 【COMMON 段】
   * 存放"未初始化的全局变量"（C 语言的 tentative definition）
   * 例如：int x; （没有 extern，也没有初始化）
   * 链接器会合并多个文件中的同名 COMMON 符号
   *
   * 【双符号定义】
   * _sbss 和 __bss_start__ 指向同一位置，为兼容不同的库（newlib、glibc 等）
   *---------------------------------------------------------------------------
   */
  . = ALIGN(4);
  .bss :
  {
    _sbss = .;         /* .bss 起始地址 */
    __bss_start__ = _sbss;
    *(.bss)            /* 所有 .bss 段 */
    *(.bss*)           /* 所有 .bss.xxx 段 */
    *(COMMON)          /* C 语言的 tentative definition */

    . = ALIGN(4);
    _ebss = .;         /* .bss 结束地址 */
    __bss_end__ = _ebss;
  } >RAM               /* 只有 VMA，不占用 Flash 空间 */

  /*---------------------------------------------------------------------------
   * ._user_heap_stack 段 - 堆栈空间检查
   *---------------------------------------------------------------------------
   * 这个段不存放实际数据，而是作为"占位符"检查 RAM 是否足够
   *
   * 【内存布局示意】（RAM 从低地址到高地址）
   *   +------------------+ ← 0x20000000 (RAM 起始)
   *   |      .data       |
   *   +------------------+
   *   |      .bss        |
   *   +------------------+ ← _ebss / end / _end
   *   |       堆 ↓       | ← 向高地址增长 (malloc)
   *   |                  |
   *   |       ...        |
   *   |                  |
   *   |       栈 ↑       | ← 向低地址增长 (函数调用)
   *   +------------------+ ← _estack = 0x20020000 (RAM 结束)
   *
   * 【PROVIDE 的作用】
   * PROVIDE(end = .)：如果代码中没有定义 end 符号，就用当前地址
   * end 符号通常被 malloc() 用作堆的起始地址
   *
   * 【ALIGN(8) 的原因】
   * ARM AAPCS 要求栈指针 8 字节对齐，堆也应该 8 字节对齐
   *---------------------------------------------------------------------------
   */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );      /* 堆起始地址（被 _sbrk() 使用） */
    PROVIDE ( _end = . );     /* 同上，兼容不同命名习惯 */
    . = . + _Min_Heap_Size;   /* 预留堆空间 */
    . = . + _Min_Stack_Size;  /* 预留栈空间 */
    . = ALIGN(8);
  } >RAM

  /*---------------------------------------------------------------------------
   * /DISCARD/ 段 - 丢弃不需要的段
   *---------------------------------------------------------------------------
   * 语法：/DISCARD/ : { 要丢弃的内容 }
   * 作用：显式丢弃指定的段，不输出到最终文件
   *
   * 这里丢弃标准库的调试信息和其他不需要的数据
   * libc.a、libm.a、libgcc.a 是 GCC 的运行时库
   * 裸机环境通常用 newlib-nano 替代完整的标准库，这些可以丢弃
   *
   * 注意：如果使用了这些库的功能（如 printf），可能需要保留
   *---------------------------------------------------------------------------
   */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }
}
