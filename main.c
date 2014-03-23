
#include "perepherial_conf.h"





static void prvLed1BlinkTask( void *pvParameters );
static void prvLed2BlinkTask( void *pvParameters );
static void prvButtonHandlerTask( void *pvParameters );
//static void prvTimerPWMLed2(void *pvParameters);

int main(void)
{
	//NVIC_SetVectorTable(NVIC_VectTab_FLASH,0x2000);

	prvSetupHardware();

	NVIC_SetPriority(EXTI0_IRQn, 1);
	NVIC_EnableIRQ(EXTI0_IRQn);
	//Create binary semaphore
	vSemaphoreCreateBinary( xButtonSemaphore );

    if( xButtonSemaphore != NULL )
	{

	 xTaskCreate( &prvButtonHandlerTask, (signed char *)"GreenBlink", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL );


	//Create task that will blink by blue diode on PC8
	//xTaskCreate(prvLed1BlinkTask,(signed char*)"LED1",configMINIMAL_STACK_SIZE,
	//           NULL, tskIDLE_PRIORITY + 1, NULL);

	//Create task that will blink by blue diode on PC8
	    xTaskCreate(prvLed2BlinkTask,(signed char*)"LED2",configMINIMAL_STACK_SIZE,
		        NULL, tskIDLE_PRIORITY + 1, NULL);
	  }
//	xTaskCreate(prvTimerPWMLed2,(signed char*)"LED_PWM",configMINIMAL_STACK_SIZE,
//		            NULL, tskIDLE_PRIORITY + 1, NULL);
    NVIC_EnableIRQ(EXTI0_IRQn);
	// Start the scheduler.
	vTaskStartScheduler();

	while(1);

	return 0;
}

////////////////////////////////////////////////////////////////////////////
/// Tasks
////////////////////////////////////////////////////////////////////////////
/*
void prvLed1BlinkTask( void *pvParameters )
{
    for (;;) {
                 GPIO_SetBits(GPIOC, GPIO_Pin_9);
                 vTaskDelay(800);					//Go to Idle for 800 ticks
                 GPIO_ResetBits(GPIOC, GPIO_Pin_9);
                 vTaskDelay(400);					//Go to Idle for 400 ticks
         	 }
    vTaskDelete( NULL );
}
*/
void prvLed2BlinkTask( void *pvParameters )
{
    for (;;) {
                 GPIO_SetBits(GPIOC, GPIO_Pin_8);
                 vTaskDelay(400);                   //Go to Idle for 400 ticks
                 GPIO_ResetBits(GPIOC, GPIO_Pin_8);
                 vTaskDelay(800);					//Go to Idle for 800 ticks
         	 }
    vTaskDelete( NULL );
}

void prvButtonHandlerTask( void *pvParameters )
{
    for (;;) {
    			xSemaphoreTake( xButtonSemaphore, portMAX_DELAY );

    			//Blink by green diode on PC9
    			GPIO_SetBits(GPIOC, GPIO_Pin_9);
    			vTaskDelay(800);					//Go to Idle for 800 ticks
    			GPIO_ResetBits(GPIOC, GPIO_Pin_9);
    			vTaskDelay(400);					//Go to Idle for 400 ticks
         	 }
    vTaskDelete( NULL );
}



/*
void prvTimerPWMLed2(void *pvParameters)
{
	int n = 1;
	int brightness = 0;
	for (;;){
		      if (((brightness + n) >= 1000) || ((brightness + n) <= 0))
		        n = -n; // if  brightness maximum/maximum change direction

		      brightness += n;

		      TIM3->CCR3 = brightness; // set brightness for blue LED PC8
		      TIM3->CCR4 = 1000 - brightness; // set brightness for green LED PC8

		      vTaskDelay(1);  // Go to Idle for 1 tick
		    }
	vTaskDelete( NULL );
}
*/



