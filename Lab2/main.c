/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

/* Kernel includes. */
#include "stm32f4xx_conf.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"


/*----------------------------- DEFINITIONS ------------------------------*/
/* The rate at which data is sent to the queue, specified in milliseconds, and
converted to ticks using the portTICK_RATE_MS constant. */
#define mainQUEUE_SEND_PERIOD_MS			(200 / portTICK_RATE_MS)

/* The period of the example software timer, specified in milliseconds, and
converted to ticks using the portTICK_RATE_MS constant. */
#define mainTRAFFIC_MOVE_PERIOD_MS		(500 / portTICK_RATE_MS)

/* The number of items the queue can hold. */
#define QUEUE_LENGTH					(1)

/*------------------------------ GLOBAL VARIABLES -----------------------------*/

/* Queues used to communicate between tasks */
static xQueueHandle xTrafficLoadForCreatorQueue = NULL;
static xQueueHandle xTrafficLoadForLightQueue = NULL;
static xQueueHandle xTrafficQueue = NULL;
static xQueueHandle xTrafficLightStatusQueue = NULL;

/* The semaphore (in this case binary) that is used by the FreeRTOS tick hook
 * function and the event semaphore task.
 */
static xSemaphoreHandle xEventSemaphore = NULL;

// GPIOE
uint16_t new_car_pin = GPIO_Pin_0;
uint16_t on_always = GPIO_Pin_1;
uint16_t intersection_car_pin = GPIO_Pin_2;
uint16_t red = GPIO_Pin_3;
uint16_t yellow = GPIO_Pin_4;
uint16_t green = GPIO_Pin_5;

// GPIOD
uint16_t clock_pin = GPIO_Pin_0;
uint16_t clear = GPIO_Pin_2;

// GPIOA
uint16_t adc_pin = GPIO_Pin_1;

uint16_t clock_on = 0;

/*----------------------------- INITIALIZATION FUNCTIONS ------------------------------*/

/* Initializes the GPIO ports for the ADC, traffic lights, and shift registers.
 * Sets clear pin and B pins to 1 (as these should be on by default).
 */
void initGPIO() {
	// Init GPIOA (for ADC input)
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructA;
    GPIO_InitStructA.GPIO_Pin = adc_pin;
    GPIO_InitStructA.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructA.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructA);

    // Init GPIOD (for clocks and clear)
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructD;
	GPIO_InitStructD.GPIO_Pin = clock_pin | clear;
	GPIO_InitStructD.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructD.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructD.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructD.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructD);

	// Set clear bit to 1
	GPIO_SetBits(GPIOD, clear);

    // Init GPIOE (for A and B signals)
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructE;
	GPIO_InitStructE.GPIO_Pin = new_car_pin | on_always | intersection_car_pin | red | yellow | green;
	GPIO_InitStructE.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructE.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructE.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructE.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructE);

	// Set B signal to 1
	GPIO_SetBits(GPIOE, on_always);
}

/* Initializes the ADC to 12-bit continuous conversion mode */
void initADC() {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	ADC_DeInit();
	ADC_InitTypeDef ADC_InitStruct;
	ADC_StructInit(&ADC_InitStruct);
	/* Initialize the ADC_ContinuousConvMode member */
	ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;
	ADC_Init(ADC1, &ADC_InitStruct);
    ADC_Cmd(ADC1, ENABLE);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_144Cycles);
}

/* Creates queues used for inter-task communication:
 *	 1) TrafficLoadForCreatorQueue
 *   2) TrafficLoadForLightQueue
 *   3) TrafficQueue
 *   4) TrafficLightStatusQueue
 */
void createQueues() {
	/* TrafficLoadForCreatorQueue: sender is TrafficFlowAdjustmentTask, receiver is TrafficCreatorTask */
	xTrafficLoadForCreatorQueue = xQueueCreate(QUEUE_LENGTH, /* The number of items the queue can hold. */
		sizeof(float));	/* The size of each item the queue holds. */

	/* Add to the registry, for the benefit of kernel aware debugging. */
	vQueueAddToRegistry(xTrafficLoadForCreatorQueue, "TrafficLoadForCreatorQueue");

	/* TrafficLoadForLightQueue: sender is TrafficFlowAdjustmentTask, receiver is TrafficLightTask */
	xTrafficLoadForLightQueue = xQueueCreate(QUEUE_LENGTH, sizeof(float));
	vQueueAddToRegistry(xTrafficLoadForLightQueue, "TrafficLoadForLightQueue");

	/* TrafficQueue: sender is TrafficCreatorTask, receiver is TrafficDisplayTask */
	xTrafficQueue = xQueueCreate(QUEUE_LENGTH, sizeof(uint16_t));
	vQueueAddToRegistry(xTrafficQueue, "TrafficQueue");

	/* TrafficLightStatusQueue: sender is trafficLightTask, receiver is trafficDisplayTask */
	xTrafficLightStatusQueue = xQueueCreate(QUEUE_LENGTH, sizeof(uint16_t));
	vQueueAddToRegistry(xTrafficLightStatusQueue, "TrafficLightStatusQueue");
}

/*------------------- MIDDLEWARE FUNCTIONS --------------------------------*/

/* Reads and returns an integer value from the ADC. */
uint16_t usReadADC() {
	ADC_SoftwareStartConv(ADC1);
	uint16_t conversionValue = 0;
    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)) {
        conversionValue = ADC_GetConversionValue(ADC1);
    }
    return conversionValue;
}

/* Pads bits in an integer to account for the bits lost due
*  to the chaining of shift registers.
*  Returns padded integer.
*/
uint32_t uPadBits(uint32_t desiredTrafficPattern) {
	// Bitmasks
	uint32_t maskForLastRegister = 0b0000000000000111111;
	uint32_t maskForMiddleRegister = 0b00000001111111000000;
	uint32_t maskForFirstRegister = 0b1111110000000000000;
	uint32_t mask6thDigit = 0b100000;
	uint32_t mask7thDigit = 0b1000000;
	uint32_t mask13thDigit = 0b1000000000000;
	uint32_t mask15thDigit = 0b100000000000000;

	// Isolate each register from desired traffic pattern
	uint32_t lastRegister = desiredTrafficPattern & maskForLastRegister;
	uint32_t digit6 = desiredTrafficPattern & mask6thDigit;

	uint32_t middleRegister = desiredTrafficPattern & maskForMiddleRegister;
	uint32_t digit13 = desiredTrafficPattern & mask13thDigit;

	uint32_t firstRegister = desiredTrafficPattern & maskForFirstRegister;

	// Pad the last register with whichever digit is in the 6th place
	lastRegister = lastRegister | ((digit6 == 0) ? 0 : mask7thDigit);

	// Pad the middle register with whichever digit is in the 13th place,
	// and shift it to account for the last register's padding
	middleRegister = (middleRegister << 1) | ((digit13 == 0) ? 0 : mask15thDigit);

	// Shift the first register to account for the previous two registers' padding
	firstRegister = firstRegister << 2;

	// Consolidate each padded register, and return the result
	uint32_t paddedTraffic = (firstRegister | middleRegister | lastRegister);
	return paddedTraffic;
}

/* Pushes the desired bit pattern to the shift registers */
void vPushToShiftRegisters(uint32_t valueToDisplay) {
	// Set up registers to receive new information: clear registers,
	// turn off new car pin (pin A) and the clock pin
	GPIO_ResetBits(GPIOD, clear);
	GPIO_SetBits(GPIOD, clear);
	GPIO_ResetBits(GPIOE, new_car_pin);
	GPIO_ResetBits(GPIOD, clock_pin);

	// Pad bits to account for register chaining
	valueToDisplay = uPadBits(valueToDisplay);

	// Push the value to the registers
	int i=0;
	for(i=0; i < 21; i++) {
		// If there is a 1 in the front of the value, set new car pin,
		// turn on clock pin to push 1 to register, and then reset new car pin
		if ((valueToDisplay & 1)) {
			GPIO_SetBits(GPIOE, new_car_pin);
			GPIO_SetBits(GPIOD, clock_pin);
			GPIO_ResetBits(GPIOE, new_car_pin);
		}
		// If there is a 0 in front of the value, just set clock pin to push 0 to register
		else {
			GPIO_SetBits(GPIOD, clock_pin);
		}

		// Reset clock pin to be ready for next iteration
		GPIO_ResetBits(GPIOD, clock_pin);
		// Shift value so next car is in the front
		valueToDisplay = valueToDisplay >> 1;
	}
}

/* Calculates traffic pattern for next cycle, based on the current status of the
*  traffic light and whether a new car should be generated.
*  Returns new traffic pattern.
*/
uint32_t uMoveCars(int newCar, int yellowOrRedLight, uint32_t uTotalTraffic) {
	// Make a copy of the pre-intersection traffic
	uint8_t preIntersectionTraffic = (uTotalTraffic & 0b1111111100000000000) >> 11;
	// Shift all the traffic up one space
	uTotalTraffic = uTotalTraffic >> 1;


	if (yellowOrRedLight) {
		// Shift cars pre-intersection to fill space in front of them if any, stay in place if not
		int i = 0;
		int carInFront = 0b1;
		int carBehind = 0b10;
		for (i = 0; i < 7; i++) {
			if (!(preIntersectionTraffic & carInFront) && (preIntersectionTraffic & carBehind)) {
				preIntersectionTraffic = preIntersectionTraffic | carInFront;
				preIntersectionTraffic = preIntersectionTraffic ^ carBehind;
			}
			carInFront = carInFront << 1;
			carBehind = carBehind << 1;
		}

		// Overwrite pre-intersection part of total traffic with newly shifted pre-intersection traffic
		uTotalTraffic = (uTotalTraffic & 0b0000000011111111111) | (preIntersectionTraffic << 11);
		// Ensure that the first intersection space is empty
		uTotalTraffic = uTotalTraffic & 0b1111111101111111111;
	}

	// If there should be a new car generated, add the new car to the first traffic space
	if (newCar) {
		uTotalTraffic = uTotalTraffic | 0b1000000000000000000;
	}

	return uTotalTraffic;
}

/* Sets the yellow light, and puts the new traffic light status on the queue. */
void vSetYellowLight() {
	GPIO_ResetBits(GPIOE, red);
	GPIO_ResetBits(GPIOE, green);
	GPIO_SetBits(GPIOE, yellow);

	uint16_t xTrafficLightStatus = 1; // 0 = green, 1 = yellow or red
	xQueueSend(xTrafficLightStatusQueue, &xTrafficLightStatus, 0);
}

/* Sets the red light. Since the traffic light status was set by the yellow light,
*  and won't be reset until the green light, no new addition to the queue is necessary.
*/
void vSetRedLight() {
	GPIO_ResetBits(GPIOE, green);
	GPIO_ResetBits(GPIOE, yellow);
	GPIO_SetBits(GPIOE, red);
}

/*-------------------------------------- TASKS ----------------------------------*/

/* Task responsible for reading the ADC, and calculating a traffic load from that value.
*  Puts the traffic load on the trafficLoadForCreatorQueue and the trafficLoadForLightQueue.
*  Repeats on a period determined by the mainQUEUE_SEND_PERIOD_MS global variable.
*/
static void trafficFlowAdjustmentTask (void *pvParameters) {
	portTickType xNextWakeTime;

		/* Initialize xNextWakeTime - this only needs to be done once. */
		xNextWakeTime = xTaskGetTickCount();

		for(;;) {
			/* Place this task in the blocked state until it is time to run again.
			The block time is specified in ticks, the constant used converts ticks
			to ms.  While in the Blocked state this task will not consume any CPU
			time.  http://www.freertos.org/vtaskdelayuntil.html */
			vTaskDelayUntil(&xNextWakeTime, mainQUEUE_SEND_PERIOD_MS);

			uint16_t conversionValue = usReadADC();
			/* Normalize conversionValue to trafficLoadValue so that trafficLoadValue is [0,1]
			 * Divided by 4096 because ADC register is 12 bits.*/
			float trafficLoadValue = (float) conversionValue / 4096.0;

			/* Overwrite any existing load on the queue, or send to queue if empty */
			xQueueOverwrite(xTrafficLoadForCreatorQueue, &trafficLoadValue);
			xQueueOverwrite(xTrafficLoadForLightQueue, &trafficLoadValue);
		}
}

/* Task responsible for creating new traffic.
*  Reads from the trafficLoadForCreatorQueue, determines if a new car should be generated,
*  and puts that on the trafficQueue.
*/
static void trafficCreatorTask () {
	uint16_t newCar = 1;
	float xTrafficLoadValue;
	time_t t;

	// initialize random number generator
	srand((unsigned) time(&t));
		for(;;)
		{
			/* Wait until something arrives in the queue - this task will block
			indefinitely provided INCLUDE_vTaskSuspend is set to 1 in
			FreeRTOSConfig.h.  http://www.freertos.org/a00118.html */
			xQueueReceive(xTrafficLoadForCreatorQueue, &xTrafficLoadValue, portMAX_DELAY);

			// generate random number between 0 and 1
			float random_num = (float)rand() / (float)RAND_MAX;

			// shrink scale of traffic load value such that traffic is more likely to be
			// generated at low load or not generated at high load
			xTrafficLoadValue = xTrafficLoadValue < 0.1 ? 0.1 : xTrafficLoadValue;
			xTrafficLoadValue = xTrafficLoadValue > 0.9 ? 0.9 : xTrafficLoadValue;

			// If the traffic load value is higher than the random number, a new car should be generated
			newCar = (random_num <= xTrafficLoadValue);

			// Put whether to generate a new car or not on the queue
			xQueueSend(xTrafficQueue, &newCar, 0);
		}
}

/* Task responsible for managing traffic lights.
*  Reads from trafficLoadForLightQueue, and writes to trafficLightStatusQueue.
*  Calculates duration of each traffic light based on traffic load, and sets
*  software timers to change the lights when required.
*/
static void trafficLightTask() {
	float xTrafficLoadValue = 0.0;
	/* Initialize xNextWakeTime - this only needs to be done once. */
	portTickType xNextWakeTime = xTaskGetTickCount();
	uint16_t totalCycleTime = mainTRAFFIC_MOVE_PERIOD_MS;

	for (;;) {
		/* Check for value in the queue  */
		xQueueReceive(xTrafficLoadForLightQueue, &xTrafficLoadValue, 0);

		// Calculate length of each light in traffic cycles
		uint16_t usGreenLightCycles = 15 - xTrafficLoadValue * 10;
		uint16_t usRedLightCycles = 15 - xTrafficLoadValue * 14;

		// Calculate total duration of traffic light cycle, and set task to run again at beginning of next cycle
		totalCycleTime = (usRedLightCycles + usGreenLightCycles + 4) * mainTRAFFIC_MOVE_PERIOD_MS;
		vTaskDelayUntil(&xNextWakeTime, totalCycleTime);

		// Set green light
		GPIO_SetBits(GPIOE, green);
		GPIO_ResetBits(GPIOE, red);
		GPIO_ResetBits(GPIOE, yellow);

		// Put green light status on the queue
		uint16_t xTrafficLightStatus = 0; // 0 = green, 1 = yellow or red
		xQueueSend(xTrafficLightStatusQueue, &xTrafficLightStatus, 0);

		/* Create the yellow light software timer as described in the comments at the top of
		this file.  http://www.freertos.org/FreeRTOS-timers-xTimerCreate.html. */
		TimerHandle_t xYellowLightTimer = xTimerCreate("YellowLightTimer", /* A text name, purely to help debugging. */
									usGreenLightCycles * mainTRAFFIC_MOVE_PERIOD_MS,		/* The timer period. */
									pdFALSE,								/* This is a one-time timer, so xAutoReload is set to pdFALSE. */
									(void *) 0,								/* The ID is not used, so can be set to anything. */
									vSetYellowLight							/* The callback function that switches to the yellow LED, and puts
																			   the new traffic light status on the queue. */
		);

		/* Start the created timer.  A block time of zero is used as the timer
		command queue cannot possibly be full here (this is the first timer to
		be created, and it is not yet running).
		http://www.freertos.org/FreeRTOS-timers-xTimerStart.html */
		xTimerStart(xYellowLightTimer, 0);

		/* Create the red light software timer. */
		TimerHandle_t xRedLightTimer = xTimerCreate("RedLightTimer",
									(usGreenLightCycles + 4) * mainTRAFFIC_MOVE_PERIOD_MS,
									pdFALSE,
									(void *) 0,
									vSetRedLight
		);

		/* Start the created timer.*/
		xTimerStart(xRedLightTimer, 0);
	}
}

/* Task responsible for displaying traffic.
*  Reads from trafficQueue and trafficLightStatusQueue.
*  Calculates new traffic pattern based on light and whether a new car should be
*  generated, and pushes this pattern to the shift registers.
*/
static void trafficDisplayTask() {
	uint16_t usTrafficLightStatus = 0;
	uint16_t usTrafficQueueValue = 0;
	portTickType xNextWakeTime;
	uint32_t uTotalTraffic = 0;

	/* Initialize xNextWakeTime - this only needs to be done once. */
	xNextWakeTime = xTaskGetTickCount();

	for (;;) {
		vTaskDelayUntil(&xNextWakeTime, mainTRAFFIC_MOVE_PERIOD_MS);

		xQueueReceive(xTrafficQueue, &usTrafficQueueValue, 0);
		xQueueReceive(xTrafficLightStatusQueue, &usTrafficLightStatus, 0);

		// Calculate new traffic pattern
		uTotalTraffic = uMoveCars(usTrafficQueueValue, usTrafficLightStatus, uTotalTraffic);

		// Push new traffic pattern to shift registers
		vPushToShiftRegisters(uTotalTraffic);
	}
}

/* ------------------------------------------- MAIN ------------------------------------------------------ */

int main() {
	// Initialize everything
	initADC();
	initGPIO();
	createQueues();

	// Create tasks
	xTaskCreate(trafficFlowAdjustmentTask, "Traffic_Flow_Adjustment", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(trafficCreatorTask, "Traffic_Creator", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(trafficLightTask, "Traffic_Light", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(trafficDisplayTask, "Traffic_Display", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

	/* Start the tasks running. */
	vTaskStartScheduler();

    return 0;
}


/*---------------------------- Functions included in demo file ------------------------------*/

void vApplicationTickHook(void)
{
portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
static uint32_t ulCount = 0;

	/* The RTOS tick hook function is enabled by setting configUSE_TICK_HOOK to
	1 in FreeRTOSConfig.h.

	"Give" the semaphore on every 500th tick interrupt. */
	ulCount++;
	if(ulCount >= 500UL)
	{
		/* This function is called from an interrupt context (the RTOS tick
		interrupt),	so only ISR safe API functions can be used (those that end
		in "FromISR()".

		xHigherPriorityTaskWoken was initialised to pdFALSE, and will be set to
		pdTRUE by xSemaphoreGiveFromISR() if giving the semaphore unblocked a
		task that has equal or higher priority than the interrupted task.
		http://www.freertos.org/a00124.html */
		xSemaphoreGiveFromISR(xEventSemaphore, &xHigherPriorityTaskWoken);
		ulCount = 0UL;
	}

	/* If xHigherPriorityTaskWoken is pdTRUE then a context switch should
	normally be performed desiredTrafficPattern leaving the interrupt (because during the
	execution of the interrupt a task of equal or higher priority than the
	running task was unblocked).  The syntax required to context switch from
	an interrupt is port dependent, so check the documentation of the port you
	are using.  http://www.freertos.org/a00090.html

	In this case, the function is running in the context of the tick interrupt,
	which will automatically check for the higher priority task to run anyway,
	so no further action is required. */
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook(void)
{
	/* The malloc failed hook is enabled by setting
	configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

	Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	for(;;);
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName)
{
	(void) pcTaskName;
	(void) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected.  pxCurrentTCB can be
	inspected in the debugger if the task name passed into this function is
	corrupt. */
	for(;;);
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook(void)
{
volatile size_t xFreeStackSpace;

	/* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
	FreeRTOSConfig.h.

	This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amount of FreeRTOS heap that
	remains unallocated. */
	xFreeStackSpace = xPortGetFreeHeapSize();

	if(xFreeStackSpace > 100)
	{
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}
}
/*-----------------------------------------------------------*/


