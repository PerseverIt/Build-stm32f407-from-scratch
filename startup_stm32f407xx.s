/**
  ******************************************************************************
  * @file      startup_stm32f407xx.s
  * @author    MCD Application Team
  * @brief     STM32F407xx Devices vector table for GCC based toolchains.
  *            This module performs:
  *                - Set the initial SP
  *                - Set the initial PC == Reset_Handler,
  *                - Set the vector table entries with the exceptions ISR address
  *                - Branches to main in the C library (which eventually
  *                  calls main()).
  *            After Reset the Cortex-M4 processor is in Thread mode,
  *            priority is Privileged, and the Stack is set to Main.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

  .syntax unified //默认的分法devided，ARM和thumb有各自独立语法，通过unified指令，进行统一
  .cpu cortex-m4  //cpu选择cortex-m4
  .fpu fpv4-sp-d16  //softvfp启用软件模拟，而不是启用硬件，默认软件模拟
  .thumb //翻译成thumb机器码，而不是arm机器码

.global  g_pfnVectors //global pointer to function 中断向量表 //.bss段起始地址
.global  Default_Handler //默认处理

/* start address for the initialization values of the .data section.
defined in linker script */
.word  _sidata //链接器重定向现场
/* start address for the .data section. defined in linker script */
.word  _sdata //.data段起始地址
/* end address for the .data section. defined in linker script */
.word  _edata //.data段尾地址
/* start address for the .bss section. defined in linker script */
.word  _sbss //.bss段起始地址
/* end address for the .bss section. defined in linker script */
.word  _ebss //.bss段尾地址
/* stack used for SystemInit_ExtMemCtl; always internal RAM used */
//栈必须在内部RAM使用，若使用外部ram,需要调用SystemInit_ExtMemCtl函数配置GPIO、时钟、FSMC寄存器（灵活静态存储控制器可在芯片外部焊接一块外部RAM），但是运行C函数需要栈，但是栈被设置在还未初始化的外部ram中
/**
 * @brief  This is the code that gets called when the processor first
 *          starts execution following a reset event. Only the absolutely
 *          necessary set is performed, after which the application
 *          supplied main() routine is called.
 * @param  None
 * @retval : None
*/
//复位事件后，处理器执行的第一段代码,复位或者刚插电，CPU会在一个固定地址，拿两个字大小的内容，一个用于栈顶地址SP,一个用于程序计数器PC,此时PC会精准的指向以下指令
    .section  .text.Reset_Handler  //定义一个段，类型是代码段，段名为Reset_Handler
  .weak  Reset_Handler	//弱符号定义，若未有强符号，则采取若符号，若有强符号，则采取强符号
  .type  Reset_Handler, %function //.type定义一个名为Reset_Handler的变量，类型是函数，联接器看到%function,且知道是cortex-M,则会记住这是thumb指令集的函数，会在中断向量表最低位置一，进入thumb模式执行，如果不加%function,联接器会认为是普通数据，向量表不会在低位置一，会引起HardFault异常导致死机
Reset_Handler://复位函数标号
  ldr   sp, =_estack     /* set stack pointer */
  //Risc奉行（加载/存储）架构，所有算术计算只能在寄存器之间进行，不能直接碰内存。带了=不是原生arm指令，是汇编器as提供的特殊语法糖，汇编器会把常数放到代码段末尾的文字池中，然后把这条语句翻译成基于PC的相对寻址

/* Call the clock system initialization function.*/
  bl  SystemInit //branch link,跳转链接，PC指向SystemInit函数的地址，并把bl下条指令地址读到LR寄存器（链接寄存器）中

/* Copy the data segment initializers from flash to SRAM */
  ldr r0, =_sdata //把.data段起始地址加载到r0
  ldr r1, =_edata //把.data段尾地址加载到r1
  ldr r2, =_sidata //把重定向后的flash源地址加载到r2
  movs r3, #0 //mov传送指令处理寄存器之间，或者传送常数
  b LoopCopyDataInit

CopyDataInit:
  ldr r4, [r2, r3] //以flash源地址开始进行偏移复制，链接器在将.o文件链接时，会严格、紧密的物理布局排列，中断向量表、.text、.rodata、分界线也就是_sidata,然后是所有带初始值的全局变量
  str r4, [r0, r3] //把加载到r4的指令存储到内存中，实现复制
  adds r3, r3, #4 //因为一次搬运了32位，所以指针挪4步

LoopCopyDataInit:
  adds r4, r0, r3 //r4用于记录当前的搬运地址
  cmp r4, r1 //搬运的当前地址是否和尾地址相同，搬运是否结束了呢
  bcc CopyDataInit //如果r1大于r4则说明未全部搬运，跳到CopyDataInit并执行搬运，小于等于则说明搬运完成，继续向下执行

/* Zero fill the bss segment. */
  ldr r2, =_sbss //用r2记录bss段首地址，进行偏移
  ldr r4, =_ebss
  movs r3, #0
  b LoopFillZerobss

FillZerobss:
  str  r3, [r2] //把0存储到内存中
  adds r2, r2, #4

LoopFillZerobss:
  cmp r2, r4 //检测是否有未初始化的变量,没有的话bss段首和段尾指针相同
  bcc FillZerobss

/* Call static constructors */
    bl __libc_init_array //libc是C标准库，这个函数意思是在main函数执行前，把所有需要执行“初始化函数”执行一遍,为什么需要这个函数呢，需要什么函数直接转移不行吗，这涉及到高内聚，低耦合的设计思想。比如项目中没用到malloc函数，会导致找不到这个函数，如果强行加入malloc函数，会导致不必要的内存开销，这是绝对不允许的。需要初始化的函数内部有一个标记，会把初始化函数指针送到这个段里，联接器在链接多个.o文件时会不断收集初始化函数指针，整齐排放到flash的一块连续空间，形成一个物理上的数组。最后调用这个函数，他不关心这个数组中有什么，碰到什么函数指针就执行什么函数
/* Call the application's entry point.*/
  bl  main //跳转到主函数并执行，更新链接寄存器
  bx  lr //branch and eXchange,跳转并交换状态，会检查地址的最后一位，如果是0就会切换到arm,如果是1就会切换到thumb，但是Cortex-M4砍掉了arm,所以为0时会触发HardFault异常
.size  Reset_Handler, .-Reset_Handler //Reset_Handler是需要度量的函数，符号.是当前位置计数器，理解成汇编器翻译到内存的哪个物理地址，当前物理地址减去函数开头物理地址，计算出这个函数占用的字节数

/**
 * @brief  This is the code that gets called when the processor receives an
 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 *         the system state for examination by a debugger.
 * @param  None
 * @retval None
*/

//处理为在预期的中断，比如没用强符号的中断，会进入死循环
    .section  .text.Default_Handler,"ax",%progbits//"ax"是属性标签，a表示这段代码是要真实占用Flash空间，x告诉CPU，这段空间装的是机器码，而且是可执行的，别当成普通数据了，%progbits是程序数据的意思，意思是这里是机器码，不是像bss那样是个空壳子
Default_Handler:
Infinite_Loop://这两个标号指向同一个内存地址
  b  Infinite_Loop //死循环
  .size  Default_Handler, .-Default_Handler//计算Defaulet_Handler函数所占字节大小
/******************************************************************************
*
* The minimal vector table for a Cortex M3. Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
*

//ARM公司卖给ST的仅仅是一个Cortex-M4内核，只要通电复位，CPU就会去总线的0x00000000读取4个字节当栈顶指针,在0x00000004读取4个字节当复位函数入口（PC），但是ST想要搭建外设、加Flash,如果把Flash焊死在0x00000000，再想从RAM启动，或者从外部ROM启动，就不行了。ST通过地址重映射把Flash的真实地址映射到0x08000000，但是ST在芯片内部做了一个类似硬件路由网关的开关，也就是BOOT0和BOOT1引脚，当BOOT0接地正常从Flash启动上电时，ST的硬件电路会把0x08000000整个空间映射到0x00000000，CPU以为自己读的是0地址，实际上总线会把请求转发到0x08000000的Flash中
*******************************************************************************/
   .section  .isr_vector,"a",%progbits //这个数组很特殊，会在链接脚本中进行keep,防止被链接器进行垃圾回收，或者随意乱扔，如果位置不对，读取不到sp和pc，会直接死机
  .type  g_pfnVectors, %object


g_pfnVectors:
  .word  _estack
  .word  Reset_Handler
  .word  NMI_Handler
  .word  HardFault_Handler
  .word  MemManage_Handler
  .word  BusFault_Handler
  .word  UsageFault_Handler
  .word  0
  .word  0
  .word  0
  .word  0
  .word  SVC_Handler
  .word  DebugMon_Handler
  .word  0
  .word  PendSV_Handler
  .word  SysTick_Handler

  /* External Interrupts */
  .word     WWDG_IRQHandler                   /* Window WatchDog              */
  .word     PVD_IRQHandler                    /* PVD through EXTI Line detection */
  .word     TAMP_STAMP_IRQHandler             /* Tamper and TimeStamps through the EXTI line */
  .word     RTC_WKUP_IRQHandler               /* RTC Wakeup through the EXTI line */
  .word     FLASH_IRQHandler                  /* FLASH                        */
  .word     RCC_IRQHandler                    /* RCC                          */
  .word     EXTI0_IRQHandler                  /* EXTI Line0                   */
  .word     EXTI1_IRQHandler                  /* EXTI Line1                   */
  .word     EXTI2_IRQHandler                  /* EXTI Line2                   */
  .word     EXTI3_IRQHandler                  /* EXTI Line3                   */
  .word     EXTI4_IRQHandler                  /* EXTI Line4                   */
  .word     DMA1_Stream0_IRQHandler           /* DMA1 Stream 0                */
  .word     DMA1_Stream1_IRQHandler           /* DMA1 Stream 1                */
  .word     DMA1_Stream2_IRQHandler           /* DMA1 Stream 2                */
  .word     DMA1_Stream3_IRQHandler           /* DMA1 Stream 3                */
  .word     DMA1_Stream4_IRQHandler           /* DMA1 Stream 4                */
  .word     DMA1_Stream5_IRQHandler           /* DMA1 Stream 5                */
  .word     DMA1_Stream6_IRQHandler           /* DMA1 Stream 6                */
  .word     ADC_IRQHandler                    /* ADC1, ADC2 and ADC3s         */
  .word     CAN1_TX_IRQHandler                /* CAN1 TX                      */
  .word     CAN1_RX0_IRQHandler               /* CAN1 RX0                     */
  .word     CAN1_RX1_IRQHandler               /* CAN1 RX1                     */
  .word     CAN1_SCE_IRQHandler               /* CAN1 SCE                     */
  .word     EXTI9_5_IRQHandler                /* External Line[9:5]s          */
  .word     TIM1_BRK_TIM9_IRQHandler          /* TIM1 Break and TIM9          */
  .word     TIM1_UP_TIM10_IRQHandler          /* TIM1 Update and TIM10        */
  .word     TIM1_TRG_COM_TIM11_IRQHandler     /* TIM1 Trigger and Commutation and TIM11 */
  .word     TIM1_CC_IRQHandler                /* TIM1 Capture Compare         */
  .word     TIM2_IRQHandler                   /* TIM2                         */
  .word     TIM3_IRQHandler                   /* TIM3                         */
  .word     TIM4_IRQHandler                   /* TIM4                         */
  .word     I2C1_EV_IRQHandler                /* I2C1 Event                   */
  .word     I2C1_ER_IRQHandler                /* I2C1 Error                   */
  .word     I2C2_EV_IRQHandler                /* I2C2 Event                   */
  .word     I2C2_ER_IRQHandler                /* I2C2 Error                   */
  .word     SPI1_IRQHandler                   /* SPI1                         */
  .word     SPI2_IRQHandler                   /* SPI2                         */
  .word     USART1_IRQHandler                 /* USART1                       */
  .word     USART2_IRQHandler                 /* USART2                       */
  .word     USART3_IRQHandler                 /* USART3                       */
  .word     EXTI15_10_IRQHandler              /* External Line[15:10]s        */
  .word     RTC_Alarm_IRQHandler              /* RTC Alarm (A and B) through EXTI Line */
  .word     OTG_FS_WKUP_IRQHandler            /* USB OTG FS Wakeup through EXTI line */
  .word     TIM8_BRK_TIM12_IRQHandler         /* TIM8 Break and TIM12         */
  .word     TIM8_UP_TIM13_IRQHandler          /* TIM8 Update and TIM13        */
  .word     TIM8_TRG_COM_TIM14_IRQHandler     /* TIM8 Trigger and Commutation and TIM14 */
  .word     TIM8_CC_IRQHandler                /* TIM8 Capture Compare         */
  .word     DMA1_Stream7_IRQHandler           /* DMA1 Stream7                 */
  .word     FSMC_IRQHandler                   /* FSMC                         */
  .word     SDIO_IRQHandler                   /* SDIO                         */
  .word     TIM5_IRQHandler                   /* TIM5                         */
  .word     SPI3_IRQHandler                   /* SPI3                         */
  .word     UART4_IRQHandler                  /* UART4                        */
  .word     UART5_IRQHandler                  /* UART5                        */
  .word     TIM6_DAC_IRQHandler               /* TIM6 and DAC1&2 underrun errors */
  .word     TIM7_IRQHandler                   /* TIM7                         */
  .word     DMA2_Stream0_IRQHandler           /* DMA2 Stream 0                */
  .word     DMA2_Stream1_IRQHandler           /* DMA2 Stream 1                */
  .word     DMA2_Stream2_IRQHandler           /* DMA2 Stream 2                */
  .word     DMA2_Stream3_IRQHandler           /* DMA2 Stream 3                */
  .word     DMA2_Stream4_IRQHandler           /* DMA2 Stream 4                */
  .word     ETH_IRQHandler                    /* Ethernet                     */
  .word     ETH_WKUP_IRQHandler               /* Ethernet Wakeup through EXTI line */
  .word     CAN2_TX_IRQHandler                /* CAN2 TX                      */
  .word     CAN2_RX0_IRQHandler               /* CAN2 RX0                     */
  .word     CAN2_RX1_IRQHandler               /* CAN2 RX1                     */
  .word     CAN2_SCE_IRQHandler               /* CAN2 SCE                     */
  .word     OTG_FS_IRQHandler                 /* USB OTG FS                   */
  .word     DMA2_Stream5_IRQHandler           /* DMA2 Stream 5                */
  .word     DMA2_Stream6_IRQHandler           /* DMA2 Stream 6                */
  .word     DMA2_Stream7_IRQHandler           /* DMA2 Stream 7                */
  .word     USART6_IRQHandler                 /* USART6                       */
  .word     I2C3_EV_IRQHandler                /* I2C3 event                   */
  .word     I2C3_ER_IRQHandler                /* I2C3 error                   */
  .word     OTG_HS_EP1_OUT_IRQHandler         /* USB OTG HS End Point 1 Out   */
  .word     OTG_HS_EP1_IN_IRQHandler          /* USB OTG HS End Point 1 In    */
  .word     OTG_HS_WKUP_IRQHandler            /* USB OTG HS Wakeup through EXTI */
  .word     OTG_HS_IRQHandler                 /* USB OTG HS                   */
  .word     DCMI_IRQHandler                   /* DCMI                         */
  .word     0                                 /* CRYP crypto                  */
  .word     HASH_RNG_IRQHandler               /* Hash and Rng                 */
  .word     FPU_IRQHandler                    /* FPU                          */



  .size  g_pfnVectors, .-g_pfnVectors

/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*
*******************************************************************************/
   .weak      NMI_Handler
   .thumb_set NMI_Handler,Default_Handler

   .weak      HardFault_Handler
   .thumb_set HardFault_Handler,Default_Handler

   .weak      MemManage_Handler
   .thumb_set MemManage_Handler,Default_Handler

   .weak      BusFault_Handler
   .thumb_set BusFault_Handler,Default_Handler

   .weak      UsageFault_Handler
   .thumb_set UsageFault_Handler,Default_Handler

   .weak      SVC_Handler
   .thumb_set SVC_Handler,Default_Handler

   .weak      DebugMon_Handler
   .thumb_set DebugMon_Handler,Default_Handler

   .weak      PendSV_Handler
   .thumb_set PendSV_Handler,Default_Handler

   .weak      SysTick_Handler
   .thumb_set SysTick_Handler,Default_Handler

   .weak      WWDG_IRQHandler
   .thumb_set WWDG_IRQHandler,Default_Handler

   .weak      PVD_IRQHandler
   .thumb_set PVD_IRQHandler,Default_Handler

   .weak      TAMP_STAMP_IRQHandler
   .thumb_set TAMP_STAMP_IRQHandler,Default_Handler

   .weak      RTC_WKUP_IRQHandler
   .thumb_set RTC_WKUP_IRQHandler,Default_Handler

   .weak      FLASH_IRQHandler
   .thumb_set FLASH_IRQHandler,Default_Handler

   .weak      RCC_IRQHandler
   .thumb_set RCC_IRQHandler,Default_Handler

   .weak      EXTI0_IRQHandler
   .thumb_set EXTI0_IRQHandler,Default_Handler

   .weak      EXTI1_IRQHandler
   .thumb_set EXTI1_IRQHandler,Default_Handler

   .weak      EXTI2_IRQHandler
   .thumb_set EXTI2_IRQHandler,Default_Handler

   .weak      EXTI3_IRQHandler
   .thumb_set EXTI3_IRQHandler,Default_Handler

   .weak      EXTI4_IRQHandler
   .thumb_set EXTI4_IRQHandler,Default_Handler

   .weak      DMA1_Stream0_IRQHandler
   .thumb_set DMA1_Stream0_IRQHandler,Default_Handler

   .weak      DMA1_Stream1_IRQHandler
   .thumb_set DMA1_Stream1_IRQHandler,Default_Handler

   .weak      DMA1_Stream2_IRQHandler
   .thumb_set DMA1_Stream2_IRQHandler,Default_Handler

   .weak      DMA1_Stream3_IRQHandler
   .thumb_set DMA1_Stream3_IRQHandler,Default_Handler

   .weak      DMA1_Stream4_IRQHandler
   .thumb_set DMA1_Stream4_IRQHandler,Default_Handler

   .weak      DMA1_Stream5_IRQHandler
   .thumb_set DMA1_Stream5_IRQHandler,Default_Handler

   .weak      DMA1_Stream6_IRQHandler
   .thumb_set DMA1_Stream6_IRQHandler,Default_Handler

   .weak      ADC_IRQHandler
   .thumb_set ADC_IRQHandler,Default_Handler

   .weak      CAN1_TX_IRQHandler
   .thumb_set CAN1_TX_IRQHandler,Default_Handler

   .weak      CAN1_RX0_IRQHandler
   .thumb_set CAN1_RX0_IRQHandler,Default_Handler

   .weak      CAN1_RX1_IRQHandler
   .thumb_set CAN1_RX1_IRQHandler,Default_Handler

   .weak      CAN1_SCE_IRQHandler
   .thumb_set CAN1_SCE_IRQHandler,Default_Handler

   .weak      EXTI9_5_IRQHandler
   .thumb_set EXTI9_5_IRQHandler,Default_Handler

   .weak      TIM1_BRK_TIM9_IRQHandler
   .thumb_set TIM1_BRK_TIM9_IRQHandler,Default_Handler

   .weak      TIM1_UP_TIM10_IRQHandler
   .thumb_set TIM1_UP_TIM10_IRQHandler,Default_Handler

   .weak      TIM1_TRG_COM_TIM11_IRQHandler
   .thumb_set TIM1_TRG_COM_TIM11_IRQHandler,Default_Handler

   .weak      TIM1_CC_IRQHandler
   .thumb_set TIM1_CC_IRQHandler,Default_Handler

   .weak      TIM2_IRQHandler
   .thumb_set TIM2_IRQHandler,Default_Handler

   .weak      TIM3_IRQHandler
   .thumb_set TIM3_IRQHandler,Default_Handler

   .weak      TIM4_IRQHandler
   .thumb_set TIM4_IRQHandler,Default_Handler

   .weak      I2C1_EV_IRQHandler
   .thumb_set I2C1_EV_IRQHandler,Default_Handler

   .weak      I2C1_ER_IRQHandler
   .thumb_set I2C1_ER_IRQHandler,Default_Handler

   .weak      I2C2_EV_IRQHandler
   .thumb_set I2C2_EV_IRQHandler,Default_Handler

   .weak      I2C2_ER_IRQHandler
   .thumb_set I2C2_ER_IRQHandler,Default_Handler

   .weak      SPI1_IRQHandler
   .thumb_set SPI1_IRQHandler,Default_Handler

   .weak      SPI2_IRQHandler
   .thumb_set SPI2_IRQHandler,Default_Handler

   .weak      USART1_IRQHandler
   .thumb_set USART1_IRQHandler,Default_Handler

   .weak      USART2_IRQHandler
   .thumb_set USART2_IRQHandler,Default_Handler

   .weak      USART3_IRQHandler
   .thumb_set USART3_IRQHandler,Default_Handler

   .weak      EXTI15_10_IRQHandler
   .thumb_set EXTI15_10_IRQHandler,Default_Handler

   .weak      RTC_Alarm_IRQHandler
   .thumb_set RTC_Alarm_IRQHandler,Default_Handler

   .weak      OTG_FS_WKUP_IRQHandler
   .thumb_set OTG_FS_WKUP_IRQHandler,Default_Handler

   .weak      TIM8_BRK_TIM12_IRQHandler
   .thumb_set TIM8_BRK_TIM12_IRQHandler,Default_Handler

   .weak      TIM8_UP_TIM13_IRQHandler
   .thumb_set TIM8_UP_TIM13_IRQHandler,Default_Handler

   .weak      TIM8_TRG_COM_TIM14_IRQHandler
   .thumb_set TIM8_TRG_COM_TIM14_IRQHandler,Default_Handler

   .weak      TIM8_CC_IRQHandler
   .thumb_set TIM8_CC_IRQHandler,Default_Handler

   .weak      DMA1_Stream7_IRQHandler
   .thumb_set DMA1_Stream7_IRQHandler,Default_Handler

   .weak      FSMC_IRQHandler
   .thumb_set FSMC_IRQHandler,Default_Handler

   .weak      SDIO_IRQHandler
   .thumb_set SDIO_IRQHandler,Default_Handler

   .weak      TIM5_IRQHandler
   .thumb_set TIM5_IRQHandler,Default_Handler

   .weak      SPI3_IRQHandler
   .thumb_set SPI3_IRQHandler,Default_Handler

   .weak      UART4_IRQHandler
   .thumb_set UART4_IRQHandler,Default_Handler

   .weak      UART5_IRQHandler
   .thumb_set UART5_IRQHandler,Default_Handler

   .weak      TIM6_DAC_IRQHandler
   .thumb_set TIM6_DAC_IRQHandler,Default_Handler

   .weak      TIM7_IRQHandler
   .thumb_set TIM7_IRQHandler,Default_Handler

   .weak      DMA2_Stream0_IRQHandler
   .thumb_set DMA2_Stream0_IRQHandler,Default_Handler

   .weak      DMA2_Stream1_IRQHandler
   .thumb_set DMA2_Stream1_IRQHandler,Default_Handler

   .weak      DMA2_Stream2_IRQHandler
   .thumb_set DMA2_Stream2_IRQHandler,Default_Handler

   .weak      DMA2_Stream3_IRQHandler
   .thumb_set DMA2_Stream3_IRQHandler,Default_Handler

   .weak      DMA2_Stream4_IRQHandler
   .thumb_set DMA2_Stream4_IRQHandler,Default_Handler

   .weak      ETH_IRQHandler
   .thumb_set ETH_IRQHandler,Default_Handler

   .weak      ETH_WKUP_IRQHandler
   .thumb_set ETH_WKUP_IRQHandler,Default_Handler

   .weak      CAN2_TX_IRQHandler
   .thumb_set CAN2_TX_IRQHandler,Default_Handler

   .weak      CAN2_RX0_IRQHandler
   .thumb_set CAN2_RX0_IRQHandler,Default_Handler

   .weak      CAN2_RX1_IRQHandler
   .thumb_set CAN2_RX1_IRQHandler,Default_Handler

   .weak      CAN2_SCE_IRQHandler
   .thumb_set CAN2_SCE_IRQHandler,Default_Handler

   .weak      OTG_FS_IRQHandler
   .thumb_set OTG_FS_IRQHandler,Default_Handler

   .weak      DMA2_Stream5_IRQHandler
   .thumb_set DMA2_Stream5_IRQHandler,Default_Handler

   .weak      DMA2_Stream6_IRQHandler
   .thumb_set DMA2_Stream6_IRQHandler,Default_Handler

   .weak      DMA2_Stream7_IRQHandler
   .thumb_set DMA2_Stream7_IRQHandler,Default_Handler

   .weak      USART6_IRQHandler
   .thumb_set USART6_IRQHandler,Default_Handler

   .weak      I2C3_EV_IRQHandler
   .thumb_set I2C3_EV_IRQHandler,Default_Handler

   .weak      I2C3_ER_IRQHandler
   .thumb_set I2C3_ER_IRQHandler,Default_Handler

   .weak      OTG_HS_EP1_OUT_IRQHandler
   .thumb_set OTG_HS_EP1_OUT_IRQHandler,Default_Handler

   .weak      OTG_HS_EP1_IN_IRQHandler
   .thumb_set OTG_HS_EP1_IN_IRQHandler,Default_Handler

   .weak      OTG_HS_WKUP_IRQHandler
   .thumb_set OTG_HS_WKUP_IRQHandler,Default_Handler

   .weak      OTG_HS_IRQHandler
   .thumb_set OTG_HS_IRQHandler,Default_Handler

   .weak      DCMI_IRQHandler
   .thumb_set DCMI_IRQHandler,Default_Handler

   .weak      HASH_RNG_IRQHandler
   .thumb_set HASH_RNG_IRQHandler,Default_Handler

   .weak      FPU_IRQHandler
   .thumb_set FPU_IRQHandler,Default_Handler
