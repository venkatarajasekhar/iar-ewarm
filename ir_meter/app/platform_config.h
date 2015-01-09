/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : platform_config.h
* Author             : MCD Application Team
* Version            : V2.2.1
* Date               : 09/22/2008
* Description        : Evaluation board specific configuration file.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PLATFORM_CONFIG_H
#define __PLATFORM_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_type.h"
#include "hw_config.h"
#include "stm32f10x_conf.h"
#include "stm32f10x_lib.h"
#include "intrinsics.h"

#include "usb_lib.h"
#include "ds3231.h"
#include "spi_flash.h"
#include "dtu_uart.h"
#include "dtu_timer.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Uncomment the line corresponding to the STMicroelectronics evaluation board
   used to run the example */
#if !defined (USE_STM3210B_EVAL) &&  !defined (USE_STM3210E_EVAL)
 //#define USE_STM3210B_EVAL
 #define USE_STM3210E_EVAL
#endif

#define   WWDG_EN()           IWDG_Enable()
#define   WWDG_WEI()          IWDG_ReloadCounter()   //ι��

#define   RXD_SUM  100
#define   TXD_SUM  100
//------------------------------------------------------------------------------
#define WKUP                                GPIOA
#define WKUP_PIN                            GPIO_Pin_0
#define RCC_APB2Periph_GPIO_WKUP            RCC_APB2Periph_GPIOA

#define HWRXD_EN                            GPIOB
#define HWRXD_EN_PIN                        GPIO_Pin_13
#define RCC_APB2Periph_GPIO_HWRXD_EN        RCC_APB2Periph_GPIOA

#define USBDA_EN                            GPIOA
#define USBDA_EN_PIN                        GPIO_Pin_11
#define RCC_APB2Periph_GPIO_USBDA_EN        RCC_APB2Periph_GPIOA

#define USBDB_EN                            GPIOA
#define USBDB_EN_PIN                        GPIO_Pin_12
#define RCC_APB2Periph_GPIO_USBDB_EN        RCC_APB2Periph_GPIOA

//------------------------------------------------------------------------------
//#define LED_LAMP                            GPIOB
//#define LED_LAMP_PIN                        GPIO_Pin_8
//#define RCC_APB2Periph_GPIO_LED             RCC_APB2Periph_GPIOB

#define TIM1_OUTPUT_GPIO_TYPE				GPIOA
#define TIM1_OUTPUT_GPIO_PIN				GPIO_Pin_1



#define W25_EN                              GPIOC
#define W25_EN_PIN                          GPIO_Pin_4
#define RCC_APB2Periph_GPIO_W25_EN          RCC_APB2Periph_GPIOC

#define TEST_KEY                            GPIOD
#define TEST_KEY_PIN                        GPIO_Pin_2
#define RCC_APB2Periph_GPIO_KEY             RCC_APB2Periph_GPIOD

#define USB_DISCONNECT                      GPIOC
#define USB_DISCONNECT_PIN                  GPIO_Pin_11
#define RCC_APB2Periph_GPIO_DISCONNECT      RCC_APB2Periph_GPIOC

#define USB_TEST                            GPIOA
#define USB_TEST_PIN                        GPIO_Pin_8
#define RCC_APB2Periph_GPIO_USB_TEST        RCC_APB2Periph_GPIOA

#define HWRXD_EN_H()                        GPIO_SetBits(HWRXD_EN, HWRXD_EN_PIN)
#define HWRXD_EN_L()                        GPIO_ResetBits(HWRXD_EN, HWRXD_EN_PIN)

#define W25_EN_H()                          GPIO_SetBits(W25_EN, W25_EN_PIN)
#define W25_EN_L()                          GPIO_ResetBits(W25_EN, W25_EN_PIN)

#define TEST_KEY_READ()                     GPIO_ReadInputDataBit(TEST_KEY, TEST_KEY_PIN)
#define USB_TEST_READ()                     GPIO_ReadInputDataBit(USB_TEST, USB_TEST_PIN)

#define UART2_TX_H()                        GPIO_SetBits(GPIOA, GPIO_Pin_2)
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __PLATFORM_CONFIG_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/

