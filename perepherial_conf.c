#include "perepherial_conf.h"


////////////////////////////////////////////////////////////////////////////
/// HW init
////////////////////////////////////////////////////////////////////////////


 void LEDGPIOInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC , ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9
    							| GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOC, &GPIO_InitStructure);
}
/*
static void LED2PWMTimerInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;

	RCC_APB2PeriphClockCmd(
	            RCC_APB2Periph_GPIOC |
	            RCC_APB2Periph_AFIO, ENABLE );

	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM3, ENABLE );

	GPIO_StructInit(&GPIO_InitStructure); // Reset init structure

	// Setup Blue & Green LED on STM32-Discovery Board to use PWM.
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;            // Alt Function - Push Pull
	GPIO_Init( GPIOC, &GPIO_InitStructure );
	GPIO_PinRemapConfig( GPIO_FullRemap_TIM3, ENABLE );        // Map TIM3_CH3 to GPIOC.Pin8, TIM3_CH4 to GPIOC.Pin9

	// Let PWM frequency equal 100Hz.
	// Let period equal 1000. Therefore, timer runs from zero to 1000. Gives 0.1Hz resolution.
	// Solving for prescaler gives 240.
	TIM_TimeBaseStructInit( &TIM_TimeBaseInitStruct );
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV4;
	TIM_TimeBaseInitStruct.TIM_Period = 1000 - 1;   // 0..999
	TIM_TimeBaseInitStruct.TIM_Prescaler = 240 - 1; // Div 240
	TIM_TimeBaseInit( TIM3, &TIM_TimeBaseInitStruct );

	TIM_OCStructInit( &TIM_OCInitStruct );
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	// Initial duty cycle equals 0%. Value can range from zero to 1000.
	TIM_OCInitStruct.TIM_Pulse = 0; // 0 .. 1000 (0=Always Off, 1000=Always On)

	TIM_OC3Init( TIM3, &TIM_OCInitStruct ); // Channel 3 Blue LED
	TIM_OC4Init( TIM3, &TIM_OCInitStruct ); // Channel 4 Green LED

	TIM_Cmd( TIM3, ENABLE );
}


// PA9 - Rx, PA10 - Tx
static void UART1Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	USART_InitTypeDef USART_InitStructure;

	USART_ClockInitTypeDef USART_ClockInitStructure;

	//enable bus clocks

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	//Set USART1 Tx (PA.09) as AF push-pull

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//Set USART1 Rx (PA.10) as input floating

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;

	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_ClockStructInit(&USART_ClockInitStructure);

	USART_ClockInit(USART1, &USART_ClockInitStructure);

	USART_InitStructure.USART_BaudRate = 9600;

	USART_InitStructure.USART_WordLength = USART_WordLength_8b;

	USART_InitStructure.USART_StopBits = USART_StopBits_1;

	USART_InitStructure.USART_Parity = USART_Parity_No ;

	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

	//Write USART1 parameters

	USART_Init(USART1, &USART_InitStructure);

	//Enable USART1

	USART_Cmd(USART1, ENABLE);
}

void UART1SendChar(uint8_t ch)
{
      USART_SendData(USART1, (uint8_t) ch);
      //Loop until the end of transmission
      while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
      {
      }
}

void UARTSendBuffer(const uint16_t *pucBuffer, unsigned long ulCount)
{
    while(ulCount--)
    {
        USART_SendData(USART1, (uint16_t) *pucBuffer++);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
        {
        }
    }
}

uint8_t UART1GetChar(void)
{
     while ( USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
        return (uint8_t)USART_ReceiveData(USART1);
}


//SPI2 MOSI - PB13 , MISO - PB14 , CLK - PB15  , SS - PB12
static void SPI1Init(void)
{
	 	 	// Initialize SPI
	        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	        GPIO_InitTypeDef GPIO_InitStruct;

	        // MOSI & CLK
	        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_5;
	        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

	        GPIO_Init(GPIOA, &GPIO_InitStruct);

	        // MISO
	        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

	        GPIO_Init(GPIOA, &GPIO_InitStruct);

	        // SS
	        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
	        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

	        GPIO_Init(GPIOA, &GPIO_InitStruct);

	        // RESET
	        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

	        GPIO_Init(GPIOA, &GPIO_InitStruct);

	        SPI_InitTypeDef SPI_InitStruct;

	        SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	        SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	        SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	        SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	        SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
	        SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
	        SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	        SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	        SPI_InitStruct.SPI_CRCPolynomial = 7;
	        SPI_Init(SPI1, &SPI_InitStruct);

	        SPI_Cmd(SPI1, ENABLE);
}



//SPI2 MOSI - PB13 , MISO - PB14 , CLK - PB15  , SS - PB12
static void SPI2Init(void)
{
	 	 	// Initialize SPI
	        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	        RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	        GPIO_InitTypeDef GPIO_InitStruct;

	        // MOSI & CLK
	        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
	        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

	        GPIO_Init(GPIOB, &GPIO_InitStruct);

	        // MISO
	        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14;
	        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

	        GPIO_Init(GPIOB, &GPIO_InitStruct);

	        // SS
	        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
	        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

	        GPIO_Init(GPIOB, &GPIO_InitStruct);

	        // RESET
	        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
	        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

	        GPIO_Init(GPIOA, &GPIO_InitStruct);

	        SPI_InitTypeDef SPI_InitStruct;

	        SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	        SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	        SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	        SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	        SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
	        SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
	        SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	        SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	        SPI_InitStruct.SPI_CRCPolynomial = 7;
	        SPI_Init(SPI2, &SPI_InitStruct);

	        SPI_Cmd(SPI2, ENABLE);
}


// SDA - PB7, SCL - PB6,
static void I2C1Init(void)
{
	  GPIO_InitTypeDef GPIO_InitStructure;
	  I2C_InitTypeDef  I2C_InitStructure;

	  // I2C1 and I2C2 Periph clock enable
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	  // Configure I2C1 pins: SCL and SDA
	  // GPIOB Periph clock enable
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD; // Open Drain, I2C bus pulled high externally
	  GPIO_Init(GPIOB, &GPIO_InitStructure);

	  // I2C1 configuration
	  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	  I2C_InitStructure.I2C_OwnAddress1 = I2C1_SLAVE_ADDRESS7;
	  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	  I2C_InitStructure.I2C_ClockSpeed = 100000;
	  I2C_Init(I2C1, &I2C_InitStructure);

	  I2C_Cmd(I2C1, ENABLE);
}

// SDA - PB11, SCL - PB10
static void I2C2Init(void)
{
	  GPIO_InitTypeDef GPIO_InitStructure;
	  I2C_InitTypeDef  I2C_InitStructure;

	  // I2C1 and I2C2 Periph clock enable
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

	  // Configure I2C2 pins: SCL and SDA
	  // GPIOB Periph clock enable
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD; // Open Drain, I2C bus pulled high externally
	  GPIO_Init(GPIOB, &GPIO_InitStructure);

	  // I2C2 configuration
	  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	  I2C_InitStructure.I2C_OwnAddress1 = I2C2_SLAVE_ADDRESS7;
	  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	  I2C_InitStructure.I2C_ClockSpeed =  100000;
	  I2C_Init(I2C2, &I2C_InitStructure);

	  I2C_Cmd(I2C2, ENABLE);
}


 uint32_t volatile  adc_val;

static void ADC1In1Init(void)
{
	// Подал тактовые импульсы на АЦП.
	// Названия констант нашел в stm32f10x_rcc.h
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	// Сбросил настройку
	ADC_DeInit(ADC1);

	// В stm32f10x_adc.h нашел название структуры ADC_InitTypeDef,
	// объявил переменную с этим типом
	ADC_InitTypeDef  ADC_InitStructure;

	// Заполнил значения. Имена констант нашел в том же stm32f10x_adc.h,
	// еще полистал Reference Manual для ознакомления с режимами
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;

	// Инициализировал АЦП заполненной структурой
	ADC_Init(ADC1, &ADC_InitStructure);

	// Включил
	ADC_Cmd(ADC1, ENABLE);

	// Разрешил прерывания по окончанию преобразования
	// Список возможных констант получен поиском по ADC_ITConfig
	// (найдено в stm32f10x_adc.c), из них выбрал нужную.
	// EOC это End of conversion
	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

	// Гугление показало, что для АЦП надо производить калибровку
	// Тупо скопировал найденный код. Специфичная для АЦП вещь,
	// поэтому в алгоритме ее нет.
	ADC_ResetCalibration(ADC1);
	while (ADC_GetResetCalibrationStatus(ADC1)) { };
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1)) { }

	// Настроил группы приоритета прерываний
	// (эта строка пишется, один раз за всю программу, где-то в начале)
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

	// Создал структуру NVIC и заполнил ее значениями
	// Название константы ADC1_IRQn взял из stm32f10x.h
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

// Написал обработчик прерывания по рекомендуемому шаблону,
// не забыв проверить источник прерывания и сбросить бит
void ADC1_IRQHandler(void)
{
    if (ADC_GetITStatus(ADC1, ADC_IT_EOC)) {
        ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
        // Чтение значения АЦП
        // Функцию нашел в stm32f10x_adc.h
        adc_val = ADC_GetConversionValue(ADC1);
    };
}

void ADC1Start(void)
{
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_1Cycles5);
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}
*/

 ////////////////////////////////////////////////////////////////////////////
 /// IRQ Handler
 ////////////////////////////////////////////////////////////////////////////

 //Button EXTI IRQ Handler
 void EXTI0_IRQHandler(void)
 {
 	{
 		EXTI_ClearFlag(EXTI_Line0);

 		portBASE_TYPE xTaskWoken = pdFALSE;
 		xSemaphoreGiveFromISR( xButtonSemaphore, &xTaskWoken );
 		if( xTaskWoken == pdTRUE)
 		{
 		        taskYIELD();
 		}

 	};
 };

 void ButtonInit(void)
{
	// Configure frequency on ports A and C and AFIO block
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	// Configure port A for input button on PA0 pin
	GPIO_InitTypeDef GPIO_Config_Port_A;
	GPIO_Config_Port_A.GPIO_Pin = GPIO_Pin_0;
	GPIO_Config_Port_A.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Config_Port_A.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_Config_Port_A);

}

 void InitInterruptController(void)
{
    // Configure NVIC
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&NVIC_InitStructure);

    return;
}

void InitExternalInterruptUnit(void)
{
	//Configure EXTI Line0 to generate an interrupt on rising or falling edge
	EXTI_InitTypeDef EXTI_Config;
	EXTI_Config.EXTI_Line = EXTI_Line0;
	EXTI_Config.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_Config.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_Config.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_Config);

	return;
}

void prvSetupHardware()
{


	InitInterruptController();
	InitExternalInterruptUnit();
	ButtonInit();
	LEDGPIOInit();
    //LED2PWMTimerInit();
}
