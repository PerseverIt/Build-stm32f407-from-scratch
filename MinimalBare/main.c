/**
 * STM32F407 最小裸机程序 - 寄存器直接操作
 *
 * 目标：点亮LED，不使用任何库
 *
 * 你需要根据自己的开发板修改LED引脚
 * 常见配置：
 * - 正点原子: PF9, PF10
 * - 野火:     PH10, PH11, PH12
 * - ST官方Discovery: PD12, PD13, PD14, PD15
 */

#include <stdint.h>

/*===========================================================================
 * 寄存器地址定义
 *
 * 这些地址来自STM32F407参考手册(RM0090)
 * 你可以打开手册搜索这些寄存器名称找到对应章节
 *===========================================================================*/

/* 基地址 - 来自参考手册 "Memory map" 章节 */
#define PERIPH_BASE 0x40000000UL
#define AHB1PERIPH_BASE (PERIPH_BASE + 0x00020000UL)

/* RCC寄存器 - 控制时钟 */
#define RCC_BASE (AHB1PERIPH_BASE + 0x3800UL)
#define RCC_AHB1ENR                                                            \
  (*(volatile uint32_t *)(RCC_BASE + 0x30)) /* GPIO时钟使能                \
                                             */

/* GPIO寄存器 - 每个端口偏移0x400 */
#define GPIOA_BASE (AHB1PERIPH_BASE + 0x0000UL)
#define GPIOB_BASE (AHB1PERIPH_BASE + 0x0400UL)
#define GPIOC_BASE (AHB1PERIPH_BASE + 0x0800UL)
#define GPIOD_BASE (AHB1PERIPH_BASE + 0x0C00UL)
#define GPIOE_BASE (AHB1PERIPH_BASE + 0x1000UL)
#define GPIOF_BASE (AHB1PERIPH_BASE + 0x1400UL)

/* GPIO寄存器偏移 - 来自参考手册 "GPIO registers" 章节 */
#define GPIO_MODER(base)                                                       \
  (*(volatile uint32_t *)((base) + 0x00)) /* 模式寄存器 */
#define GPIO_OTYPER(base)                                                      \
  (*(volatile uint32_t *)((base) + 0x04)) /* 输出类型                      \
                                           */
#define GPIO_OSPEEDR(base)                                                     \
  (*(volatile uint32_t *)((base) + 0x08))                        /* 输出速度 */
#define GPIO_PUPDR(base) (*(volatile uint32_t *)((base) + 0x0C)) /* 上下拉 */
#define GPIO_IDR(base) (*(volatile uint32_t *)((base) + 0x10))   /* 输入数据 */
#define GPIO_ODR(base) (*(volatile uint32_t *)((base) + 0x14))   /* 输出数据 */
#define GPIO_BSRR(base)                                                        \
  (*(volatile uint32_t *)((base) + 0x18)) /* 置位/复位                     \
                                           */

/*===========================================================================
 * LED配置 - 根据你的开发板修改这里
 *===========================================================================*/
#define LED_GPIO_BASE GPIOD_BASE /* LED所在的GPIO端口 */
#define LED_PIN 0                /* LED引脚号 (PD0) */
#define LED_GPIO_CLK_BIT 3       /* GPIOD在RCC_AHB1ENR中是bit3 */

/*
 * RCC_AHB1ENR 位定义：
 * bit0: GPIOA, bit1: GPIOB, bit2: GPIOC, bit3: GPIOD,
 * bit4: GPIOE, bit5: GPIOF, bit6: GPIOG, bit7: GPIOH, bit8: GPIOI
 */

/*===========================================================================
 * 延时函数 - 简单的软件延时
 *===========================================================================*/
void delay(volatile uint32_t count) {
  while (count--) {
    __asm__("nop"); /* 空操作，防止被优化掉 */
  }
}

/* 你可以在main中用这些变量测试启动文件是否正确工作 */
/*===========================================================================
 * 主函数
 *===========================================================================*/
int main(void) {

  /*
   * 步骤1: 使能GPIO时钟
   *
   * STM32为了省电，外设时钟默认是关闭的
   * 使用任何外设之前必须先使能它的时钟
   */
  RCC_AHB1ENR |= (1 << LED_GPIO_CLK_BIT);

  /*
   * 步骤2: 配置GPIO为输出模式
   *
   * MODER寄存器：每个引脚占2位
   * 00: 输入模式
   * 01: 通用输出模式
   * 10: 复用功能模式
   * 11: 模拟模式
   */
  GPIO_MODER(LED_GPIO_BASE) &= ~(0x3 << (LED_PIN * 2)); /* 清除原有配置 */
  GPIO_MODER(LED_GPIO_BASE) |= (0x1 << (LED_PIN * 2));  /* 设置为输出 */

  /*
   * 步骤3: 配置输出类型（可选）
   *
   * OTYPER寄存器：每个引脚占1位
   * 0: 推挽输出
   * 1: 开漏输出
   */
  GPIO_OTYPER(LED_GPIO_BASE) &= ~(1 << LED_PIN); /* 推挽输出 */

  /*
   * 步骤4: LED闪烁
   */
  while (1) {
    /* 方法1: 直接操作ODR寄存器 */
    GPIO_ODR(LED_GPIO_BASE) |= (1 << LED_PIN); /* 点亮 (输出高电平) */
    delay(500000);

    GPIO_ODR(LED_GPIO_BASE) &= ~(1 << LED_PIN); /* 熄灭 (输出低电平) */
    delay(500000);

    /*
     * 方法2: 使用BSRR寄存器（更推荐）
     * BSRR是"原子操作"，不需要读-改-写
     * 低16位写1置位，高16位写1复位
     *
     * GPIO_BSRR(LED_GPIO_BASE) = (1 << LED_PIN);        // 置位（点亮）
     * GPIO_BSRR(LED_GPIO_BASE) = (1 << (LED_PIN + 16)); // 复位（熄灭）
     */
  }

  return 0; /* 永远不会执行到这里 */
}
