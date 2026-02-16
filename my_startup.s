.synatx unified //unified或divided,unified是统一汇编语法
.cpu cortex-m4            //.dn和.qn是NEON技术用于Cortex-A或者Cortex-R
.fpu fpv4-sp-d16 //开启fpu,也可以softvfp软件模拟，软件模拟有更好的兼容性，可以移植到没有FPU的微控制器，fpv4是cortex-m4,-sp是指定单精度指令，不加默认双精度，d16指有16个64位寄存器
.thumb

.equ CPACR,0xE000ED88 //.equ是编译器提供的伪指令，功能类似#define,CPACR这个常量的数值表示SCB系统控制块中的CPACR协处理器访问控制寄存器所在地址，该地址是书中固定的

.section .Isr_Vector,"a",%progbits //定义一个叫做Isr_Vector的段
.type g_pfnVectors,%object

g_pfnVectors:
    .word _estack //初始堆栈
    .word Reset_Handler //复位入口
    .word NMI_Handler //不可屏蔽中断
    .word HardFault_Handler //硬件错误
    .word MemManage_Handler //内存管理错误
    .word BusFault_Handler //总线错误
    .word UsageFault_Handler //使用错误
    .word 0
    .word 0
    .word 0
    .word 0 //未实现
    .word SVC_Handler //请求管理调用异常（OS）
    .word DebugMon_Handler //调试监控
    .word 0
    .word PendSV_Handler //可挂起的系统调用
    .word SysTick_Handler //系统节拍定时器

.section .text.Reset_Handler
.weak Reset_Handler
.type Reset_Handler,%function

.fnstart

Reset_Handler:
    .cantunwind

    ldr r0,=CPACR
    ldr r1,[r0]
    orr r1,r1,#(0Xf << 20)

.fnend
