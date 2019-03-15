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
// TODO figure out how long these need to be
#define CREATE_TASK_QUEUE_LENGTH	4
#define DELETE_TASK_QUEUE_LENGTH	4
#define TASK1_QUEUE_SEND_PERIOD_MS	(100 / portTICK_RATE_MS)
#define TASK2_QUEUE_SEND_PERIOD_MS	(50 / portTICK_RATE_MS)
#define TASK3_QUEUE_SEND_PERIOD_MS	(10 / portTICK_RATE_MS)
#define SCHEDULER_TASK_PERIOD_MS	(1 / portTICK_RATE_MS)

typedef struct TaskListItem {
	TaskHandle_t tHandle;
	uint32_t deadline;
	uint32_t taskType;
	uint32_t creationTime;
	struct ActiveTaskListItem *nextCell;
	struct ActiveTaskListItem *previousCell;
} TaskListItem;

/*------------------------------ GLOBAL VARIABLES -----------------------------*/

/* Queues used to communicate between create/delete tasks and scheduler tasks */
static xQueueHandle xCreateTaskQueue = NULL;
static xQueueHandle xDeleteTaskQueue = NULL;
static xQueueHandle xActiveListRequestQueue = NULL;
static xQueueHandle xOutdatedListRequestQueue = NULL;

/* The semaphore (in this case binary) that is used by the FreeRTOS tick hook
 * function and the event semaphore task.
 */
static xSemaphoreHandle xEventSemaphore = NULL;


/*----------------------------- INITIALIZATION FUNCTIONS ------------------------------*/

/* Initializes the GPIO ports */
void initGPIO() { /* TODO */ }

/* Creates queues used for inter-task communication:
 *	 1) CreateTaskQueue
 *   2) DeleteTaskQueue
 */
void createQueues() {
	/* CreateTaskQueue: sender is dd_tcreate, receiver is DDSchedulerTask */
	xCreateTaskQueue = xQueueCreate(CREATE_TASK_QUEUE_LENGTH, /* The number of items the queue can hold. */
		sizeof(float));	/* The size of each item the queue holds. */

	/* Add to the registry, for the benefit of kernel aware debugging. */
	vQueueAddToRegistry(xCreateTaskQueue, "CreateTaskQueue");

	/* DeleteTaskQueue: sender is dd_tdelete, receiver is DDSchedulerTask */
	xDeleteTaskQueue = xQueueCreate(DELETE_TASK_QUEUE_LENGTH, sizeof(float));
	vQueueAddToRegistry(xDeleteTaskQueue, "DeleteTaskQueue");
}

/*------------------- FUNCTIONS --------------------------------*/
// TODO comments

/*
 * Takes taskHandle as parameter (sent by task creation callback function)
 * and adds the task to the queue of active tasks to be scheduled
 */
static void ddTCreate(xTaskHandle taskToSchedule) {
	static xQueueHandle xSchedulerResponseQueue = NULL;
	// open a SchedulerResponseQueue to scheduler task
	xSchedulerResponseQueue = xQueueCreate(1, sizeof(float));
	// add the queue to the registry
	vQueueAddToRegistry(xSchedulerResponseQueue, "SchedulerResponseQueue");
	// put task on CreateTaskQueue (w task handle)
	// TODO? Should we be putting the task handle on the CreateTaskQueue, or the SchedulerResponseQueue on the
	// CreateTaskQueue? I don't see how the xSchedulerResponseQueue would ever be modified if we're passing
	// the task handle to the CreateTaskQueue
	// (THIS APPLIES TO ALL THE dd FUNCTIONS)
	xQueueSend(xCreateTaskQueue, taskToSchedule, 0);
	// wait for response from scheduler task
	xQueueReceive(xSchedulerResponseQueue, NULL, 0);
	// destroy SchedulerResponseQueue
	vQueueDelete(&xSchedulerResponseQueue);
	return;
}

/*
 * Requests that the scheduler deletes the task with the specified handle
 * from the list of active tasks.
 */
static void ddTDelete(TaskHandle_t taskToDelete) {
	static xQueueHandle xSchedulerResponseQueue = NULL;
	// open a SchedulerResponseQueue to scheduler task
	xSchedulerResponseQueue = xQueueCreate(1, sizeof(float));
	// add the queue to the registry
	vQueueAddToRegistry(xSchedulerResponseQueue, "SchedulerResponseQueue");
	// put task on DeleteTaskQueue (w task handle) which is read by scheduler
	xQueueSend(xDeleteTaskQueue, taskToDelete, 0);
	// wait for response from scheduler
	xQueueReceive(xSchedulerResponseQueue, NULL, 0);
	// destroy SchedulerResponseQueue
	vQueueDelete(&xSchedulerResponseQueue);
	return;
}

/*
 * Gets a copy of the current list of active tasks.
 */
static TaskListItem* ddReturnActiveList() {
	TaskListItem* activeTasks = NULL;
	static xQueueHandle xSchedulerResponseQueue = NULL;
	// open a SchedulerResponseQueue to scheduler task
	xSchedulerResponseQueue = xQueueCreate(1, sizeof(TaskListItem*));
	// add the queue to the registry
	vQueueAddToRegistry(xSchedulerResponseQueue, "SchedulerResponseQueue");
	// Ask scheduler for list of active tasks (will have been copied from scheduler internal list at time of request)
	xQueueSend(xActiveListRequestQueue, &activeTasks, 0);
	// wait for response from scheduler
	xQueueReceive(xSchedulerResponseQueue, &activeTasks, 0);
	// destroy SchedulerResponseQueue
	vQueueDelete(&xSchedulerResponseQueue);
	return activeTasks;
}

/*
 * Gets a copy of the current list of overdue tasks.
 */
static TaskListItem* ddReturnOverdueList() {
	TaskListItem* overdueTasks = NULL;
	static xQueueHandle xSchedulerResponseQueue = NULL;
	// open a SchedulerResponseQueue to scheduler task
	xSchedulerResponseQueue = xQueueCreate(1, sizeof(TaskListItem*));
	// add the queue to the registry
	vQueueAddToRegistry(xSchedulerResponseQueue, "SchedulerResponseQueue");
	// Ask scheduler for list of overdue tasks
	xQueueSend(xOutdatedListRequestQueue, &overdueTasks, 0);
	// wait for response from scheduler
	xQueueReceive(xSchedulerResponseQueue, &overdueTasks, 0);
	// destroy SchedulerResponseQueue
	vQueueDelete(&xSchedulerResponseQueue);
	return overdueTasks;
}

/*-------------------------------------- TASKS ----------------------------------*/

static void ddSchedulerTask(void *pvParameters) {
	portTickType xNextWakeTime = xTaskGetTickCount();
	TaskListItem* createdTasks = NULL;
	TaskListItem* deletedTasks = NULL;

	//loop
	for(;;) {
		// delay on interval
		vTaskDelayUntil(&xNextWakeTime, SCHEDULER_TASK_PERIOD_MS);
		// check CreateTaskQueue
		xQueuePeek(xCreateTaskQueue, &createdTasks, 0);
		// check DeleteTaskQueue
		xQueuePeek(xDeleteTaskQueue, &deletedTasks, 0);
		// if task created, schedule new task
		if (createdTasks != NULL) {
			// TODO schedule the task(s)
		}
		// if task deleted, remove from schedule
		if (deletedTasks != NULL) {
			// TODO delete the task(s)
		}
		// check for overdue tasks; remove from active list and add to overdue list
	}
}


// TODO this is an abstraction of tasks that would be generated from createtask
static void userTask1(void *pvParameters) {
	// do nothing for specified time period
}
static void userTask2(void *pvParameters) { }
static void userTask3(void *pvParameters) { }

/* Task generators generate user tasks at specified intervals */
static void ddTask1GeneratorTask(void *pvParameters) {
	TaskHandle_t xTask1Handle = NULL;

	portTickType xNextWakeTime = xTaskGetTickCount();

	for(;;) {
		// on interval: (vTaskDelayUntil...)
		vTaskDelayUntil(&xNextWakeTime, TASK1_QUEUE_SEND_PERIOD_MS);
		// create new user task
		xTaskCreate(userTask1, "Periodic_Task_1", configMINIMAL_STACK_SIZE, &pvParameters, 3, &xTask1Handle);
		// call ddTCreate function
		ddTCreate(&xTask1Handle);
	}
}
static void ddTask2GeneratorTask(void *pvParameters) {
	TaskHandle_t xTask2Handle = NULL;

	portTickType xNextWakeTime = xTaskGetTickCount();

	for(;;) {
		// delay for interval
		vTaskDelayUntil(&xNextWakeTime, TASK2_QUEUE_SEND_PERIOD_MS);
		// create new user task 2
		xTaskCreate(userTask2, "Periodic_Task_2", configMINIMAL_STACK_SIZE, &pvParameters, 2, &xTask2Handle);
		// call ddTCreate
		ddTCreate(&xTask2Handle);
	}
}
static void ddTask3GeneratorTask(void *pvParameters) {
	TaskHandle_t xTask3Handle = NULL;

	portTickType xNextWakeTime = xTaskGetTickCount();

	for(;;) {
		// delay for interval
		vTaskDelayUntil(&xNextWakeTime, TASK3_QUEUE_SEND_PERIOD_MS);
		// create new user task 3
		xTaskCreate(userTask3, "Periodic_Task_3", configMINIMAL_STACK_SIZE, &pvParameters, 1, &xTask3Handle);
		// call ddTCreate
		ddTCreate(&xTask3Handle);
	}
}

static void monitorTask(void *pvParameters) {
	// lowest priority!
	// records own running time
	// records utilization (total running time - own running time / total running time)
}

/* ------------------------------------------- MAIN ------------------------------------------------------ */

int main() {
	// Initialize queues
	createQueues();

	// TODO init total running time counter

	// Create tasks
	xTaskCreate(ddSchedulerTask, "DD_SCHEDULER_TASK", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(ddTask1GeneratorTask, "TASK_GENERATOR_TASK", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(ddTask2GeneratorTask, "TASK_GENERATOR_TASK", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(ddTask3GeneratorTask, "TASK_GENERATOR_TASK", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(monitorTask, "MONITOR_TASK", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

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