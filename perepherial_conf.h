#ifndef PEREPHERIAL_CONF_H
#define PEREPHERIAL_CONF_H

#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_tim.h"
//#include "stm32f10x_usart.h"
//#include "stm32f10x_spi.h"
//#include "stm32f10x_i2c.h"
//#include "stm32f10x_adc.h"
#include <misc.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define I2C1_SLAVE_ADDRESS7  0x3A
#define I2C2_SLAVE_ADDRESS7	 0x3B
////////////////////////////////////////////////////////////////////////////
/// Functions prototypes
////////////////////////////////////////////////////////////////////////////

xSemaphoreHandle xButtonSemaphore;

void prvSetupHardware( void );
//static void LEDGPIOInit(void);
//static void LED2PWMTimerInit(void);
/*

static void UART1Init(void);
void UART1SendChar(uint8_t ch);
uint8_t UART1GetChar(void);
void UARTSendBuffer(const uint16_t *pucBuffer, unsigned long ulCount);

static void SPI1Init(void);
static void SPI2Init(void);
static void I2C1Init(void);
static void I2C2Init(void);

static void ADC1In1Init(void);
void ADC1Start(void);
void ADC1_IRQHandler(void);
*/
#endif /* PEREPHERIAL_CONF_H */
