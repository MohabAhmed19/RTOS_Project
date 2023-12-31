/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"
#include "semphr.h"

/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )

xSemaphoreHandle LED_semaphore ;  /** define handler for the semaphore **/

TaskHandle_t LEDtask_Handler    = NULL ;  /** definition of task handler for led task    **/
TaskHandle_t Buttontask_Handler = NULL ;  /** definition of task handler for button task **/

/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/

QueueHandle_t xQueue1=NULL;

/**
 ** @brief     : task to check the button status 
 ** @arguments : void pointer(to be casted latter when we need to pass arguments)
 ** @return    : void 
 **/

void Button_1_Monitor(void * pvParameters)
{
	TickType_t start;
	start= xTaskGetTickCount();
	
	for( ; ; )
	{
		signed char* RISING_0;
		signed char* FALLING_0;
		static uint8_t flag1=0,flag2=0;
		GPIO_write(PORT_0,PIN3,PIN_IS_HIGH);
		RISING_0 = "BTN_1_RISING\n";
		FALLING_0 = "BTN_1_FALLING\n";
		flag2=GPIO_read(PORT_0, PIN0);
		if(flag2==0 && flag1==1)
		{
			xQueueSend( xQueue1, ( void * ) &FALLING_0, ( TickType_t ) 0 );
			//vSerialPutString(FALLING_0, 15);
		}
		else if(flag2==1 && flag1==0)
		{
			xQueueSend( xQueue1, ( void * ) &RISING_0, ( TickType_t ) 0 );
			//vSerialPutString(RISING_0, 14);
		}
		flag1=flag2;
		GPIO_write(PORT_0,PIN3,PIN_IS_LOW);
		vTaskDelayUntil(&start, 50);
	}
}

void Button_2_Monitor(void * pvParameters)
{
	TickType_t start;
	start= xTaskGetTickCount();
	
	for( ; ; )
	{
		signed char* RISING_1;
		signed char* FALLING_1;
		static uint8_t flag1=0,flag2=0;
		GPIO_write(PORT_0,PIN4,PIN_IS_HIGH);
		RISING_1 = "BTN_2_RISING\n";
		FALLING_1 = "BTN_2_FALLING\n";
		flag2=GPIO_read(PORT_0, PIN1);
		if(flag2==0 && flag1==1)
		{
			xQueueSend( xQueue1, ( void * ) &FALLING_1, ( TickType_t ) 0 );
			//vSerialPutString(FALLING_1, 15);
		}
		else if(flag2==1 && flag1==0)
		{
			xQueueSend( xQueue1, ( void * ) &RISING_1, ( TickType_t ) 0 );
			//vSerialPutString(RISING_1, 14);
		}
		flag1=flag2;
		GPIO_write(PORT_0,PIN4,PIN_IS_LOW);
		vTaskDelayUntil(&start,50);
	}
}

void Periodic_Transmitter(void * pvParameters)
{
	TickType_t start;
	start= xTaskGetTickCount();
	GPIO_write(PORT_0,PIN5,PIN_IS_HIGH);
	for( ; ; )
	{
		
		signed char* Hello;
		GPIO_write(PORT_0,PIN5,PIN_IS_HIGH);
		start= xTaskGetTickCount();
		Hello = "Hello\n";
		xQueueSend( xQueue1, ( void * ) &Hello, ( TickType_t ) 0 );
		//vSerialPutString(Hello, 7);
		GPIO_write(PORT_0,PIN5,PIN_IS_LOW);
		vTaskDelayUntil(&start,100);
	}
}

void Uart_Receiver(void * pvParameters)
{
	TickType_t start;
	start= xTaskGetTickCount();
	
	for( ; ; )
	{
		uint8_t n=0,i=0;
		char*  PTR;
		GPIO_write(PORT_0,PIN6,PIN_IS_HIGH);
		if(pdTRUE == xQueueReceive(xQueue1, &( PTR ), ( TickType_t ) 10))
		{
			n=0;
			i=0;
			while(PTR[i] != 0)
			{
				n++;
				i++;
			}
			vSerialPutString(PTR, n);
		}
		GPIO_write(PORT_0,PIN6,PIN_IS_LOW);
		vTaskDelayUntil(&start, 20);
	}
}

void Load_1_Simulation(void * pvParameters)
{
	TickType_t start;
	start= xTaskGetTickCount();
	for( ; ; )
	{
		int i;
		GPIO_write(PORT_0,PIN7,PIN_IS_HIGH);
		for(i=0;i<37430;i++);
		GPIO_write(PORT_0,PIN7,PIN_IS_LOW);
		vTaskDelayUntil(&start, 10);
	}
}

void Load_2_Simulation(void * pvParameters)
{
	TickType_t start;
	start= xTaskGetTickCount();
	for( ; ; )
	{
		int i;
		GPIO_write(PORT_0,PIN8,PIN_IS_HIGH);
		for(i=0;i<74861;i++);
		GPIO_write(PORT_0,PIN8,PIN_IS_LOW);
		vTaskDelayUntil(&start, 100);
	}
}

/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();
  
	/** create binary semaphore with initial value 0 (not available) **/
	xQueue1 = xQueueCreate( 10, sizeof( unsigned char )*15 );

	/** Create Button Task here **/
  xTaskCreate(
			Button_1_Monitor,
			"PTN_0_Task",
			configMINIMAL_STACK_SIZE,
			(void * ) NULL,
			1,
			NULL);
			
		xTaskCreate(
			Button_2_Monitor,
			"BTN_1_Task",
			configMINIMAL_STACK_SIZE,
			(void * ) NULL,
			1,
			NULL);

			xTaskCreate(
			Periodic_Transmitter,
			"Send_STR_Task",
			configMINIMAL_STACK_SIZE,
			(void * ) NULL,
			1,
			NULL);
			
			xTaskCreate(
			Uart_Receiver,
			"Uart_Receiver",
			configMINIMAL_STACK_SIZE,
			(void * ) NULL,
			1,
			NULL);
			
			xTaskCreate(
			Load_1_Simulation,
			"Load_1_Simulation",
			configMINIMAL_STACK_SIZE,
			(void * ) NULL,
			1,
			NULL);
			
			xTaskCreate(
			Load_2_Simulation,
			"Load_2_Simulation",
			configMINIMAL_STACK_SIZE,
			(void * ) NULL,
			1,
			NULL);

	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

/* Function to reset timer 1 */
void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/* Function to initialize and start timer 1 */
static void configTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();
	
	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/


