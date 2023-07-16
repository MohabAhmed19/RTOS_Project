/*
 * FreeRTOS Kernel V10.3.0
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
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


#include <stdlib.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"
#include "serial.h"
#include "GPIO.h"
#include <limits.h>
#include <semphr.h>
#include <string.h>


#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )


static void prvSetupHardware( void );




int	 				 Timer_Task1_In , Timer_Task1_Out , Timer_Task1_Total 
						,Timer_Task2_In , Timer_Task2_Out , Timer_Task2_Total 
						,Timer_Task3_In , Timer_Task3_Out , Timer_Task3_Total 
						,Timer_Task4_In , Timer_Task4_Out , Timer_Task4_Total 		
						,Timer_Task5_In , Timer_Task5_Out , Timer_Task5_Total
						,Timer_Task6_In , Timer_Task6_Out , Timer_Task6_Total;

TickType_t CurrentSystemTime = NULL;
float Cpu_Load = NULL; 

/*******************Task Handlers************************************/

TaskHandle_t Button1TMonitorHandler 		= NULL;
TaskHandle_t Button2TMonitorHandler 		= NULL;
TaskHandle_t PeriodicTransmitterHandler = NULL;
TaskHandle_t UartReceiverHandler 				= NULL;
TaskHandle_t Load1SimulationHandler 		= NULL;
TaskHandle_t Load2SimulationHandler			= NULL;

/*********************Queue Handler********************/

QueueHandle_t EventQueue;

/********************Tasks Implementation********************/

void vApplicationIdleHook( void )
{
	static int CNT = 0;
	
	if( CNT == 0 )
	{
		vTaskSetApplicationTaskTag( NULL, ( void * ) PIN8 );
		CNT++;
	}
}

void vApplicationTickHook( void )
{
	GPIO_write( PORT_0, PIN9, PIN_IS_HIGH );
	GPIO_write( PORT_0, PIN9, PIN_IS_LOW );
	
	CurrentSystemTime = T1TC;
	
	Cpu_Load = ( ( float )( Timer_Task1_Total + Timer_Task2_Total + Timer_Task3_Total + Timer_Task4_Total + Timer_Task5_Total + Timer_Task6_Total ) / ( float ) CurrentSystemTime ) * 100;
}

void Button_1_Monitor( void *pvParameters ) 
{
	uint8_t u8_PressFlag = pdFALSE;
	uint8_t ButtonState;
	
	const char *eventRisingString = "\n Button 1 Rising Edge\n";
	const char *eventFallingString = "\n Button 1 Falling Edge\n";
	
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	for(;;)
	{
		ButtonState = GPIO_read( PORT_0, PIN0 );
		if ( ( ButtonState == pdTRUE ) && ( u8_PressFlag == pdFALSE ) )
		{
			xQueueSend( EventQueue, &eventRisingString, portMAX_DELAY );
			u8_PressFlag = pdTRUE;
		} 
		else if ( ( ButtonState == pdFALSE ) && ( u8_PressFlag == pdTRUE ) )
		{
			xQueueSend( EventQueue, &eventFallingString, portMAX_DELAY );			
			u8_PressFlag = pdFALSE;
		}
		else
		{
			/* Do Nothing */
		}	
		vTaskDelayUntil( &xLastWakeTime, 50 );
	}
}
void Button_2_Monitor( void *pvParameters )
{
	uint8_t u8_PressFlag = pdFALSE;
	uint8_t ButtonState;
	
	const char *eventRisingString = "\n Button 2 Rising Edge\n";
	const char *eventFallingString = "\n Button 2 Falling Edge\n";
	
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

  for(;;)
	{
		ButtonState = GPIO_read( PORT_0, PIN1 );
		
		if ( ( ButtonState == pdTRUE ) && ( u8_PressFlag == pdFALSE ) )
		{			
			xQueueSend(EventQueue, &eventRisingString, portMAX_DELAY);
			u8_PressFlag = pdTRUE;
		}
		else if ( ( ButtonState == pdFALSE ) && ( u8_PressFlag == pdTRUE ) )
		{			
			xQueueSend( EventQueue, &eventFallingString, portMAX_DELAY );			
			u8_PressFlag = pdFALSE;
		}
		else
		{
			/* Do Nothing */
		}
			
		vTaskDelayUntil( &xLastWakeTime, 50 );
	}
}

void Periodic_Transmitter( void *pvParameters )
{
	const char *eventString = "UART Still Working Periodicity\n";
	
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	for(;;) 
	{		
		xQueueSend( EventQueue, &eventString, portMAX_DELAY );
		
		vTaskDelayUntil( &xLastWakeTime, 100 );
	}
}

void Uart_Receiver( void *pvParameters )
{
	const char *eventString;
	
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	for(;;) 
	{
		if ( xQueueReceive( EventQueue, &eventString, NULL ) ) 
		{
			vSerialPutString( ( const signed char* ) eventString, strlen( eventString ) );
		}
		
		vTaskDelayUntil( &xLastWakeTime, 20 );
	}
}

void Load_1_Simulation( void *pvParameters ) 
{
	uint32_t u32_Counter = NULL;

	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	for(;;) 
	{				
		for( u32_Counter = NULL; u32_Counter < 37400; u32_Counter++ )
		{
			/*Heavy Load Simulation*/
		}
		
		vTaskDelayUntil( &xLastWakeTime,10 );
		
	}
}

void Load_2_Simulation( void *pvParameters )
{
	
	uint32_t u32_Counter = NULL;

	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	for(;;) 
	{
		for( u32_Counter = NULL; u32_Counter < 89760; u32_Counter++ )
		{
			/*Heavy Load Simulation*/
		}
		
		vTaskDelayUntil( &xLastWakeTime, 100 );
	}
}

int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();

	EventQueue = xQueueCreate( 10, sizeof( const char * ) );

    /* Create Tasks here */

		xTaskPeriodicCreate( Button_1_Monitor,
												 "Button1Monitor",
												 configMINIMAL_STACK_SIZE,
												 ( void * ) NULL,
												 1,
												 &Button1TMonitorHandler,
												 50 );
								 
								 
		xTaskPeriodicCreate( Button_2_Monitor,
												 "Button2Monitor",
												 configMINIMAL_STACK_SIZE,
												 ( void * ) NULL,
												 1,
												 &Button2TMonitorHandler,
												 50 );	
								 
								 
		xTaskPeriodicCreate( Periodic_Transmitter,
												 "PeriodicTransmitter",
												 configMINIMAL_STACK_SIZE,
												 ( void * ) NULL,
												 1,
												 &PeriodicTransmitterHandler,
												 100 );	
								 
								 
		xTaskPeriodicCreate( Uart_Receiver,
												 "UARTreceiver",
												 configMINIMAL_STACK_SIZE,
												 ( void * ) NULL,
												 1,
												 &UartReceiverHandler,
												 20 );

								 
		xTaskPeriodicCreate( Load_1_Simulation,
												 "Load1Simulation",
												 configMINIMAL_STACK_SIZE,
												 ( void * ) NULL,
												 1,
												 &Load1SimulationHandler,
												 10 );
								 
								 
		xTaskPeriodicCreate( Load_2_Simulation,
												 "Load2Simulation",
												 configMINIMAL_STACK_SIZE,
												 ( void * ) NULL,
												 1,
												 &Load2SimulationHandler,
												 100 );


	vTaskSetApplicationTaskTag( Button1TMonitorHandler, ( void * ) PIN2 );
	vTaskSetApplicationTaskTag( Button2TMonitorHandler, ( void * ) PIN3 );
	vTaskSetApplicationTaskTag( PeriodicTransmitterHandler, ( void * ) PIN4 );
	vTaskSetApplicationTaskTag( UartReceiverHandler, ( void * ) PIN5 );
	vTaskSetApplicationTaskTag( Load1SimulationHandler, ( void * ) PIN6 );
	vTaskSetApplicationTaskTag( Load2SimulationHandler, ( void * ) PIN7 );

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
static void configTimer1( void )
{
	T1PR = 1000;
	T1TCR |= 0x1;
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(115200);

	/* Configure GPIO */
	GPIO_init();
	
	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/


