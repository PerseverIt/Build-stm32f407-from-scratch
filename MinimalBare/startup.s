/**
 * STM32F407 最小启动文件
 *
 * 启动流程：
 * 1. 复位后，CPU从0x00000000读取初始SP，从0x00000004读取Reset_Handler地址
 * 2. 由于BOOT引脚配置，0x00000000被映射到0x08000000（Flash）
 * 3. Reset_Handler执行初始化，然后跳转到main
 */

.syntax unified          /* 使用统一的ARM/Thumb语法 */
.cpu cortex-m4           /* 目标CPU */
.fpu softvfp             /* 软件浮点（简化，不用FPU） */
.thumb                   /* 使用Thumb指令集 */

/* 外部符号由链接脚本定义 */
.global _estack          /* 栈顶地址 */
.global Reset_Handler    /* 复位处理函数 */

/*==========================================================================
 * 中断向量表
 * 放在Flash最开始，地址0x08000000
 * Cortex-M4复位时从这里读取SP和PC
 *==========================================================================*/
.section .isr_vector, "a", %progbits
.type vector_table, %object

vector_table:
    .word _estack              /* 0x00: 初始栈指针 */
    .word Reset_Handler        /* 0x04: 复位处理函数 */
    .word NMI_Handler          /* 0x08: NMI */
    .word HardFault_Handler    /* 0x0C: 硬件错误 */
    .word MemManage_Handler    /* 0x10: 内存管理错误 */
    .word BusFault_Handler     /* 0x14: 总线错误 */
    .word UsageFault_Handler   /* 0x18: 用法错误 */
    .word 0                    /* 0x1C: 保留 */
    .word 0                    /* 0x20: 保留 */
    .word 0                    /* 0x24: 保留 */
    .word 0                    /* 0x28: 保留 */
    .word SVC_Handler          /* 0x2C: SVCall */
    .word 0                    /* 0x30: 保留 */
    .word 0                    /* 0x34: 保留 */
    .word PendSV_Handler       /* 0x38: PendSV */
    .word SysTick_Handler      /* 0x3C: SysTick */
    /* 后面还有很多外设中断，暂时省略 */

.size vector_table, .-vector_table

/*==========================================================================
 * Reset_Handler - 复位后的入口点
 *==========================================================================*/
.section .text.Reset_Handler
.type Reset_Handler, %function
.weak Reset_Handler

Reset_Handler:
    /* 1. 设置栈指针（其实复位时已经自动设置了，这里确保一下） */
    ldr sp, =_estack

    /* 2. 复制 .data 段从Flash到RAM
     *    .data段包含有初始值的全局变量
     *    存储在Flash中，需要复制到RAM才能运行时修改
     */
    ldr r0, =_sdata      /* RAM中.data段起始地址（目标） */
    ldr r1, =_edata      /* RAM中.data段结束地址 */
    ldr r2, =_sidata     /* Flash中.data段起始地址（源） */

copy_data_loop:
    cmp r0, r1           /* 比较当前地址和结束地址 */
    bge copy_data_done   /* 如果 r0 >= r1，复制完成 */
    ldr r3, [r2], #4     /* 从Flash读取4字节，r2自增 */
    str r3, [r0], #4     /* 写入RAM，r0自增 */
    b copy_data_loop     /* 继续循环 */

copy_data_done:

    /* 3. 清零 .bss 段
     *    .bss段包含未初始化的全局变量
     *    C标准要求它们初始化为0
     */
    ldr r0, =_sbss       /* .bss段起始地址 */
    ldr r1, =_ebss       /* .bss段结束地址 */
    mov r2, #0           /* 用0填充 */

zero_bss_loop:
    cmp r0, r1
    bge zero_bss_done
    str r2, [r0], #4     /* 写入0，地址自增 */
    b zero_bss_loop

zero_bss_done:

    /* 4. 跳转到main函数 */
    bl main

    /* 5. 如果main返回（不应该），进入死循环 */
hang:
    b hang

.size Reset_Handler, .-Reset_Handler

/*==========================================================================
 * 默认的异常处理函数（弱定义，可以被用户覆盖）
 *==========================================================================*/
.section .text.Default_Handler, "ax", %progbits
.type Default_Handler, %function
.weak Default_Handler

Default_Handler:
    b .    /* 死循环 */

.size Default_Handler, .-Default_Handler

/* 将所有异常处理函数弱定义为Default_Handler */
.weak NMI_Handler
.thumb_set NMI_Handler, Default_Handler

.weak HardFault_Handler
.thumb_set HardFault_Handler, Default_Handler

.weak MemManage_Handler
.thumb_set MemManage_Handler, Default_Handler

.weak BusFault_Handler
.thumb_set BusFault_Handler, Default_Handler

.weak UsageFault_Handler
.thumb_set UsageFault_Handler, Default_Handler

.weak SVC_Handler
.thumb_set SVC_Handler, Default_Handler

.weak PendSV_Handler
.thumb_set PendSV_Handler, Default_Handler

.weak SysTick_Handler
.thumb_set SysTick_Handler, Default_Handler
