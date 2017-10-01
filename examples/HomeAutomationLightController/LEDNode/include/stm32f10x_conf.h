/**
 * @file    stm32f10x_conf.h
 * @author  SirVolta
 * @date    Mar 1, 2014
 * @brief   STM32 configuration file
 * @note    COPYRIGHT 2011 STMicroelectronics
 */

#ifndef __STM32F10x_CONF_H
#define __STM32F10x_CONF_H

//#include "stm32f10x_adc.h"
//#include "stm32f10x_bkp.h"
#include "stm32f10x_can.h"
//#include "stm32f10x_cec.h"
//#include "stm32f10x_crc.h"
//#include "stm32f10x_dac.h"
#include "stm32f10x_dbgmcu.h"
//#include "stm32f10x_dma.h"
#include "stm32f10x_exti.h"
//#include "stm32f10x_flash.h"
//#include "stm32f10x_fsmc.h"
#include "stm32f10x_gpio.h"
//#include "stm32f10x_i2c.h"
//#include "stm32f10x_iwdg.h"
//#include "stm32f10x_pwr.h"
#include "stm32f10x_rcc.h"
//#include "stm32f10x_rtc.h"
//#include "stm32f10x_sdio.h"
//#include "stm32f10x_spi.h"
#include "stm32f10x_tim.h"
//#include "stm32f10x_usart.h"
//#include "stm32f10x_wwdg.h"
#include "misc.h" //misc NVIC and clock
#include "diag/Trace.h" // trace functions (disabled automatically in release builds)

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Uncomment the line below to expanse the "assert_param" macro in the 
 Standard Peripheral Library drivers code */
#define USE_FULL_ASSERT    1

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT

/**
 * @brief  The assert_param macro is used for function's parameters check.
 * @param  expr: If expr is false, it calls assert_failed function which reports
 *         the name of the source file and the source line number of the call
 *         that failed. If expr is true, it returns no value.
 * @retval None
 */
#define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
void
assert_failed (uint8_t* file, uint32_t line);
#else
#define assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT */

//#define  VECT_TAB_SRAM

#endif /* __STM32F10x_CONF_H */
