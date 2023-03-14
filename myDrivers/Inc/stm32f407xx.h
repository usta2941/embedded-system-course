#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include <string.h>  // bir yerde memset kullanmam gerekiyordu kullanmayı unuttum muhtemelen interrupt kısmında
#include <stddef.h>
/*
 *
 * Microprocessor Defines
 *
 */

#define NVIC_ISER0					( (uint32_t*)(0xE000E100) )

#define __IO    volatile

#define SET_BIT(REG, BIT)			((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)			((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)			((REG) & (BIT) )
#define UNUSED(x)					(void)x


/*
 *
 *  IRQ Numbers of MCU == vVector Table
 *
 */


typedef enum
{
	EXTI0_IRQNumber = 6,
	EXTI1_IRQNumber = 7,
	EXTI2_IRQNumber = 8,
	EXTI3_IRQNumber = 9,
	SPI1_IRQNumber = 35
}IRQNumber_TypeDef_t;

typedef enum
{
	DISABLE = 0x0U,
	ENABLE  = !DISABLE
}FunctionalState_t;


/*
 * Memory Base Address
 */
#define FLASH_BASE_ADDR				(0x08000000UL)                      /* Flash Base Address (up to 1MB) */
#define SRAM1_BASE_ADDR             (0x20000000UL)                      /* SRAM1 Base Address 128KB       */
#define SRAM2_BASE_ADDR             (0x2001C000UL)                      /* SRAM2 Base Address 16KB        */

/*
 * Peripheral Base Addresses
 *
 */

#define PERIPH_BASE_ADDR			(0x40000000UL)  					/* Base Address for All peripherals */

#define APB1_BASE_ADDR				PERIPH_BASE_ADDR  					/* APB1 Bus Domain Base Address     */
#define APB2_BASE_ADDR				(PERIPH_BASE_ADDR + 0x00010000UL)   /* APB2 Bus Domain Base Address     */
#define AHB1_BASE_ADDR				(PERIPH_BASE_ADDR + 0x00020000UL)   /* AHB1 Bus Domain Base Address     */
#define AHB2_BASE_ADDR				(PERIPH_BASE_ADDR + 0x10000000UL)   /* AHB2 Bus Domain Base Address     */


/*
 * APB1 Peripherals base Addresses
 *
 */

#define TIM2_BASE_ADDR			    (APB1_BASE_ADDR + 0x0000UL)         /* Timer 2 Base Address             */
#define TIM3_BASE_ADDR			    (APB1_BASE_ADDR + 0x0400UL)         /* Timer 3 Base Address             */
#define TIM4_BASE_ADDR			    (APB1_BASE_ADDR + 0x0800UL)         /* Timer 4 Base Address             */

#define SPI2_BASE_ADDR				(APB1_BASE_ADDR + 0x3800UL)			/* SPI 2 Base Address               */
#define SPI3_BASE_ADDR				(APB1_BASE_ADDR + 0x3C00UL)			/* SPI 3 Base Address               */



#define USART2_BASE_ADDR			(APB1_BASE_ADDR + 0x4400UL)			/* USART 2 Base Address             */
#define USART3_BASE_ADDR			(APB1_BASE_ADDR + 0x4800UL)			/* USART 3 Base Address             */
#define UART4_BASE_ADDR			    (APB1_BASE_ADDR + 0x4C00UL)			/* UART  4 Base Address             */
#define UART5_BASE_ADDR			    (APB1_BASE_ADDR + 0x5000UL)			/* UART  5 Base Address             */


#define I2C1_BASE_ADDR				(APB1_BASE_ADDR + 0x5400UL) 		/* 12C 1 Base Address               */
#define I2C2_BASE_ADDR				(APB1_BASE_ADDR + 0x5800UL) 		/* 12C 2 Base Address               */
#define I2C3_BASE_ADDR				(APB1_BASE_ADDR + 0x5C00UL) 		/* 12C 3 Base Address               */

/*
 * APB2 Peripherals base Addresses
 *
 */

#define TIM1_BASE_ADDR				(APB2_BASE_ADDR + 0x0000UL)         /* Timer 1 Base Address             */
#define TIM8_BASE_ADDR				(APB2_BASE_ADDR + 0x0400UL)         /* Timer 8 Base Address             */


#define USART1_BASE_ADDR			(APB2_BASE_ADDR + 0x1000UL)         /* USART 1 Base Address             */
#define USART6_BASE_ADDR			(APB2_BASE_ADDR + 0x1400UL)         /* USART 6 Base Address             */

#define SPI1_BASE_ADDR 				(APB2_BASE_ADDR + 0x3000UL)         /* SPI 1 Base Address               */
#define SPI4_BASE_ADDR 				(APB2_BASE_ADDR + 0x3400UL)         /* SPI 4 Base Address               */


#define SYSCFG_BASE_ADDR			(APB2_BASE_ADDR + 0x3800UL)         /* SYSCFG Base Address              */
#define EXTI_BASE_ADDR				(APB2_BASE_ADDR + 0x3C00UL)         /* EXTI Base Address                */



/*
 * AHB1 Peripherals base Addresses
 *
 */

#define GPIOA_BASE_ADDR				(AHB1_BASE_ADDR + 0x0000UL)         /* GPIOA Base Address               */
#define GPIOB_BASE_ADDR				(AHB1_BASE_ADDR + 0x0400UL)         /* GPIOB Base Address               */
#define GPIOC_BASE_ADDR				(AHB1_BASE_ADDR + 0x0800UL)         /* GPIOC Base Address               */
#define GPIOD_BASE_ADDR				(AHB1_BASE_ADDR + 0x0C00UL)         /* GPIOD Base Address               */
#define GPIOE_BASE_ADDR				(AHB1_BASE_ADDR + 0x1000UL)         /* GPIOE Base Address               */

#define RCC_BASE_ADDR				(AHB1_BASE_ADDR + 0x3800UL)         /* RCC Base Address                 */


/*
 * Peripheral Structure Definitions
 *
 */

typedef struct
{
	__IO uint32_t MODER;
	__IO uint32_t OTYPER;
	__IO uint32_t OSPEEDR;
	__IO uint32_t PUPDR;
	__IO uint32_t IDR;
	__IO uint32_t ODR;
	__IO uint32_t BSRR;
	__IO uint32_t LCKR;
	__IO uint32_t AFR[2];
}GPIO_TypeDef_t;


typedef struct
{
	__IO uint32_t CR;
	__IO uint32_t PLLCFGR;
	__IO uint32_t CFGR;
	__IO uint32_t CIR;
	__IO uint32_t AHB1RSTR;
	__IO uint32_t AHB2RSTR;
	__IO uint32_t AHB3RSTR;
	__IO uint32_t RESERVED0;
	__IO uint32_t APB1RSTR;
	__IO uint32_t APB2RSTR;
	__IO uint32_t RESERVED1[2];
	__IO uint32_t AHB1ENR;
	__IO uint32_t AHB2ENR;
	__IO uint32_t AHB3ENR;
	__IO uint32_t RESERVED2;
	__IO uint32_t APB1ENR;
	__IO uint32_t APB2ENR;
	__IO uint32_t RESERVED3[2];
	__IO uint32_t AHB1LPENR;
	__IO uint32_t AHB2LPENR;
	__IO uint32_t AHB3LPENR;
	__IO uint32_t RESERVED4;
	__IO uint32_t APB1LPENR;
	__IO uint32_t APB2LPENR;
	__IO uint32_t RESERVED5[2];
	__IO uint32_t BDCR;
	__IO uint32_t CSR;
	__IO uint32_t RESERVED6[2];
	__IO uint32_t SSCGR;
	__IO uint32_t PLLI2SCFGR;
}RCC_TypeDef_t;


typedef struct
{
	__IO uint32_t MEMRMP;
	__IO uint32_t PMC;
	__IO uint32_t EXTI_CR[4];
	__IO uint32_t CMPCR;
}SYSCFG_TypeDef_t;


typedef struct
{
	__IO uint32_t CR1;
	__IO uint32_t CR2;
	__IO uint32_t SR;
	__IO uint32_t DR;
	__IO uint32_t CRCPR;
	__IO uint32_t RXCRCR;
	__IO uint32_t TXCRCR;
	__IO uint32_t I2SCFGR;
	__IO uint32_t I2SPR;

}SPI_TypeDef_t;

typedef struct
{
	__IO uint32_t SR;
	__IO uint32_t DR;
	__IO uint32_t BRR;
	__IO uint32_t CR1;
	__IO uint32_t CR2;
	__IO uint32_t CR3;
	__IO uint32_t GTPR;
}USART_TypeDef_t;

typedef struct
{
	__IO uint32_t CR1;
	__IO uint32_t CR2;
	__IO uint32_t OAR1;
	__IO uint32_t OAR2;
	__IO uint32_t DR;
	__IO uint32_t SR1;
	__IO uint32_t SR2;
	__IO uint32_t CCR;
	__IO uint32_t TRISE;
	__IO uint32_t FLTR;
}I2C_TypeDef_t;

typedef struct
{
	__IO uint32_t IMR;
	__IO uint32_t EMR;
	__IO uint32_t RTSR;
	__IO uint32_t FTSR;
	__IO uint32_t SWIR;
	__IO uint32_t PR;
}EXTI_TypeDef_t;

#define GPIOA						((GPIO_TypeDef_t *)(GPIOA_BASE_ADDR))
#define GPIOB						((GPIO_TypeDef_t *)(GPIOB_BASE_ADDR))
#define GPIOC						((GPIO_TypeDef_t *)(GPIOC_BASE_ADDR))
#define GPIOD						((GPIO_TypeDef_t *)(GPIOD_BASE_ADDR))
#define GPIOE						((GPIO_TypeDef_t *)(GPIOE_BASE_ADDR))

#define RCC 						((RCC_TypeDef_t *)(RCC_BASE_ADDR)    )

#define SYSCFG						( (SYSCFG_TypeDef_t*)(SYSCFG_BASE_ADDR))

#define EXTI						( (EXTI_TypeDef_t*)(EXTI_BASE_ADDR)    )

#define SPI1							( (SPI_TypeDef_t*)(SPI1_BASE_ADDR) )
#define SPI2							( (SPI_TypeDef_t*)(SPI2_BASE_ADDR) )
#define SPI3							( (SPI_TypeDef_t*)(SPI3_BASE_ADDR) )
#define SPI4							( (SPI_TypeDef_t*)(SPI4_BASE_ADDR) )

#define USART2  						( (USART_TypeDef_t*)(USART2_BASE_ADDR))
#define USART3  						( (USART_TypeDef_t*)(USART3_BASE_ADDR))
#define UART4  							( (USART_TypeDef_t*)(UART4_BASE_ADDR))
#define UART5  							( (USART_TypeDef_t*)(UART5_BASE_ADDR))

#define USART1  							( (USART_TypeDef_t*)(USART1_BASE_ADDR))
#define USART6  							( (USART_TypeDef_t*)(USART6_BASE_ADDR))

#define I2C1									( (I2C_TypeDef_t*)(I2C1_BASE_ADDR))
#define I2C2									( (I2C_TypeDef_t*)(I2C2_BASE_ADDR))
#define I2C3									( (I2C_TypeDef_t*)(I2C3_BASE_ADDR))


/*
 * Bit definitions
 *
 */

#define RCC_AHB1ENR_GPIOAEN_Pos		(0U)
#define RCC_AHB1ENR_GPIOAEN_Msk		(0x1 << RCC_AHB1ENR_GPIOAEN_Pos)
#define RCC_AHB1ENR_GPIOAEN			 RCC_AHB1ENR_GPIOAEN_Msk

#define RCC_AHB1ENR_GPIOBEN_Pos		(1U)
#define RCC_AHB1ENR_GPIOBEN_Msk		(0x1 << RCC_AHB1ENR_GPIOBEN_Pos)
#define RCC_AHB1ENR_GPIOBEN			 RCC_AHB1ENR_GPIOBEN_Msk

#define RCC_AHB1ENR_GPIOCEN_Pos		(2U)
#define RCC_AHB1ENR_GPIOCEN_Msk		(0x1 << RCC_AHB1ENR_GPIOCEN_Pos)
#define RCC_AHB1ENR_GPIOCEN			 RCC_AHB1ENR_GPIOCEN_Msk

#define RCC_AHB1ENR_GPIODEN_Pos		(3U)
#define RCC_AHB1ENR_GPIODEN_Msk		(0x1 << RCC_AHB1ENR_GPIODEN_Pos)
#define RCC_AHB1ENR_GPIODEN			 RCC_AHB1ENR_GPIODEN_Msk


#define RCC_APB2ENR_SYSCFGEN_Pos					(14U)
#define RCC_APB2ENR_SYSCFGEN_Msk					(0x1 << RCC_APB2ENR_SYSCFGEN_Pos )
#define RCC_APB2ENR_SYSCFGEN						RCC_APB2ENR_SYSCFGEN_Msk


#define RCC_APB2ENR_SPI1EN_Pos					(12U)
#define RCC_APB2ENR_SPI1EN_Msk					(0x1 << RCC_APB2ENR_SPI1EN_Pos )
#define RCC_APB2ENR_SPI1EN						RCC_APB2ENR_SPI1EN_Msk


#define RCC_APB1ENR_SPI2EN_Pos					(14U)
#define RCC_APB1ENR_SPI2EN_Msk					(0x1 << RCC_APB2ENR_SPI2EN_Pos )
#define RCC_APB1ENR_SPI2EN						RCC_APB2ENR_SPI1EN_Msk


#define RCC_APB1ENR_USART2EN_Pos				(17U)
#define RCC_APB1ENR_USART2EN_Msk				(0x1 << RCC_APB1ENR_USART2EN_Pos )
#define RCC_APB1ENR_USART2EN					RCC_APB1ENR_USART2EN_Msk


#define RCC_APB1ENR_12C1EN_Pos					(21U)
#define RCC_APB1ENR_I2C1EN_Msk					(0x1 << RCC_APB1ENR_12C1EN_Pos )
#define RCC_APB1ENR_I2C1EN						RCC_APB1ENR_I2C1EN_Msk

#define RCC_APB1ENR_12C2EN_Pos					(22U)
#define RCC_APB1ENR_I2C2EN_Msk					(0x1 << RCC_APB1ENR_12C2EN_Pos )
#define RCC_APB1ENR_I2C2EN						RCC_APB1ENR_I2C2EN_Msk


#define RCC_APB1ENR_12C3EN_Pos					(23U)
#define RCC_APB1ENR_I2C3EN_Msk					(0x1 << RCC_APB1ENR_12C3EN_Pos )
#define RCC_APB1ENR_I2C3EN						RCC_APB1ENR_I2C13EN_Msk


#define SPI_SR_Busy								(7U)
#define SPI_SR_TxE							    (1U)
#define SPI_SR_RxNE								(0U)

#define SPI_CR1_SPE								(6U)
#define SPI_CR1_DFF								(11U)

#define SPI_CR2_TXEIE							(7U)
#define SPI_CR2_RXNEIE							(6U)

#define USART_CR1_UE							(13U)
#define USART_CR1_TxEIE							(7U)
#define USART_CR1_RxNEIE						(5U)

#define UART_CR2_STOP							(12U)

#define USART_SR_TxE							(7U)
#define USART_SR_TC								(6U)
#define USART_SR_RxNE							(5U)

#define I2C_CR1_PE								(0U)

/*
 *
 * Flag definitions
 *
 *
 */

#define SPI_TxE_FLAG							(0x1U << SPI_SR_TxE)
#define SPI_Busy_FLAG							(0x1U << SPI_SR_Busy)
#define SPI_RxNE_FLAG							(0x1U << SPI_SR_RxNE)

#define USART_TxE_FLAG							(0x1U << USART_SR_TxE)
#define USART_TC_FLAG							(0x1U << USART_SR_TC)
#define USART_RxNE_FLAG							(0x1U << USART_SR_RxNE)




#include "RCC.h"
#include "GPIO.h"
#include "EXTI.h"
#include "SPI.h"
#include "USART.h"
#include "I2C.h"

#endif /* INC_STM32F407XX_H_ */
