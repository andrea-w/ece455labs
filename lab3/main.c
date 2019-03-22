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
#define TASK_QUEUE_LENGTH	8
#define TASK1_QUEUE_SEND_PERIOD_MS	(100 / portTICK_RATE_MS)
#define TASK2_QUEUE_SEND_PERIOD_MS	(50 / portTICK_RATE_MS)
#define TASK3_QUEUE_SEND_PERIOD_MS	(10 / portTICK_RATE_MS)
#define SCHEDULER_TASK_PERIOD_MS	(1 / portTICK_RATE_MS)
#define CREATION_MESSAGE				1
#define DELETION_MESSAGE				2
#define ACTIVE_REQUEST					3
#define OVERDUE_REQUEST					4

typedef struct TaskListItem {
	TaskHandle_t tHandle;
	uint32_t deadline;
	uint32_t taskType;
	uint32_t creationTime;
	struct TaskListItem *nextCell;
	struct TaskListItem *previousCell;
} TaskListItem;

typedef struct TaskParams {
	uint32_t relativeDeadline;
	TaskFunction_t taskCode;
	const char* taskName;
} TaskParams;

typedef struct SchedulerMessage{
	xQueueHandle responseQueueHandle;
	TaskListItem taskListItem;
} SchedulerMessage;

/*------------------------------ GLOBAL VARIABLES -----------------------------*/

/* Queue used to communicate between create/delete tasks and scheduler tasks */
static xQueueHandle xSchedulerMessageQueue = NULL;

/* The semaphore (in this case binary) that is used by the FreeRTOS tick hook
 * function and the event semaphore task.
 */
static xSemaphoreHandle xEventSemaphore = NULL;

/*
 * MonitorTask uses this
 */
uint32_t runningTime;

/*
 * Processor utilization rate as calculated by MonitorTas
 */
float utilization = 0.0;

/*----------------------------- INITIALIZATION FUNCTIONS ------------------------------*/

/* Initializes the GPIO ports */
void initGPIO() { /* TODO */ }

/* Creates queues used for inter-task communication:
 *	 1) CreateTaskQueue
 *   2) DeleteTaskQueue
 */
void createQueues() {
	/* CreateTaskQueue: sender is dd_tcreate, receiver is DDSchedulerTask */
	xSchedulerMessageQueue = xQueueCreate(TASK_QUEUE_LENGTH, /* The number of items the queue can hold. */
		sizeof(float));	/* The size of each item the queue holds. */

	/* Add to the registry, for the benefit of kernel aware debugging. */
	vQueueAddToRegistry(xSchedulerMessageQueue, "SchedulerMessageQueue");
}

/*------------------- FUNCTIONS --------------------------------*/
// TODO comments

/*
 * Takes taskHandle as parameter (sent by task creation callback function)
 * and adds the task to the queue of active tasks to be scheduled
 */
static void ddTCreate(TaskParams taskParams) {
	static xQueueHandle xSchedulerResponseQueue = NULL;
	// open a SchedulerResponseQueue to scheduler task
	xSchedulerResponseQueue = xQueueCreate(1, sizeof(float));
	// add the queue to the registry
	vQueueAddToRegistry(xSchedulerResponseQueue, "SchedulerResponseQueue");

	TaskHandle_t thandle = NULL;

	// initialize new TaskListItem
	TaskListItem newTask;
	newTask.creationTime = clock()/CLOCKS_PER_SEC*1000; // now in units of ms
	newTask.taskType = CREATION_MESSAGE;
	newTask.deadline = newTask.creationTime + taskParams.relativeDeadline;
	newTask.nextCell = NULL;
	newTask.previousCell = NULL;
	newTask.tHandle = thandle;

	// create the task
	xTaskCreate(taskParams.taskCode, taskParams.taskName, configMINIMAL_STACK_SIZE, NULL, 1, thandle);

	// create CreateTaskMessage
	SchedulerMessage createMessage = {xSchedulerMessageQueue, newTask};

	// put task on Queue (w createMessage)
	xQueueSend(xSchedulerMessageQueue, createMessage, 0);
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

	// initialize new TaskListItem
	TaskListItem newTask;
	newTask.taskType = DELETION_MESSAGE;
	newTask.nextCell = NULL;
	newTask.previousCell = NULL;
	newTask.tHandle = taskToDelete;


	// create SchedulerMessage
	SchedulerMessage deleteMessage = {xSchedulerResponseQueue, newTask};
	// put task on DeleteTaskQueue (w task handle) which is read by scheduler
	xQueueSend(xSchedulerMessageQueue, deleteMessage, 0);
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
	TaskListItem* activeTasks;
	static xQueueHandle xSchedulerResponseQueue = NULL;
	// open a SchedulerResponseQueue to scheduler task
	xSchedulerResponseQueue = xQueueCreate(1, sizeof(TaskListItem*));
	// add the queue to the registry
	vQueueAddToRegistry(xSchedulerResponseQueue, "SchedulerResponseQueue");
	// initialize TaskListItem
	TaskListItem activeItem = {.taskType = ACTIVE_REQUEST};
	// create SchedulerMessage
	SchedulerMessage activeMessage = {xSchedulerResponseQueue, activeItem};
	// Ask scheduler for list of active tasks (will have been copied from scheduler internal list at time of request)
	xQueueSend(xSchedulerMessageQueue, activeMessage, 0);
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
	TaskListItem* overdueTasks;
	static xQueueHandle xSchedulerResponseQueue = NULL;
	// open a SchedulerResponseQueue to scheduler task
	xSchedulerResponseQueue = xQueueCreate(1, sizeof(TaskListItem*));
	// add the queue to the registry
	vQueueAddToRegistry(xSchedulerResponseQueue, "SchedulerResponseQueue");
	// initialize TaskListItem
	TaskListItem overdueItem = {.taskType = OVERDUE_REQUEST};
	// create SchedulerMessage
	SchedulerMessage overdueMessage = {xSchedulerResponseQueue, overdueItem};
	// Ask scheduler for list of overdue tasks
	xQueueSend(xSchedulerMessageQueue, overdueMessage, 0);
	// wait for response from scheduler
	xQueueReceive(xSchedulerResponseQueue, &overdueTasks, 0);
	// destroy SchedulerResponseQueue
	vQueueDelete(&xSchedulerResponseQueue);
	return overdueTasks;
}

/*
 * Returns a pointer to the first item in the list
 */
static TaskListItem* getBeginningOfList(TaskListItem* currentTask) {
	while(currentTask->previousCell != NULL) {
		currentTask = currentTask->previousCell;
	}
	return currentTask;
}

/*
 * Adds the task list item to the end of the list
 */
static void addTaskToEndOfList(TaskListItem* listPointer, TaskListItem* taskToAdd) {
	// Make sure new task isn't pointing to anything
	taskToAdd->nextCell = NULL;

	// find the end of the list
	while(listPointer->nextCell != NULL) {
		listPointer->nextCell = listPointer;
	}
	listPointer->nextCell = taskToAdd;
	return;
}

/*
 * Deletes the specified task from the list of active tasks by iterating
 * through the list of active tasks and comparing task handles
 */
// TODO maaaaaaaaybe there should be error handling here if the taskHandle isn't found?????
static TaskListItem* deleteTaskByHandle(TaskListItem* activeTasks, TaskHandle_t taskHandle) {
	while (activeTasks != NULL) {
		if (activeTasks->tHandle == taskHandle) {
			TaskListItem* prev = activeTasks->previousCell;
			TaskListItem* next = activeTasks->nextCell;
			if (next != NULL) {
				next->previousCell = prev;
			}

			if (prev != NULL) {
				prev->nextCell = next;
				// Return pointer to front of list
				return getBeginningOfList(activeTasks);
			} else {
				// This is the front of the list
				return activeTasks;
			}
		}
		else {
			activeTasks = activeTasks->nextCell;
		}
	}
}

static TaskListItem* sortTasksEDF(TaskListItem* activeTasks) {
	// Task list will always be very short for this application so efficiency doesn't much matter, using bubble sort
	int swapFlag = 0;
	uint32_t deadline1;
	uint32_t deadline2;

	// Check list for needed swaps, iterate through list repeatedly until no swaps are made
	do {
		swapFlag = 0;
		while(activeTasks->nextCell != NULL) {
			deadline1 = activeTasks->deadline;
			deadline2 = activeTasks->nextCell->deadline;
			// if deadline of next task is closer than current tasks, swap tasks
			if(deadline2 < deadline1) {
				swapFlag = 1;
				TaskListItem* previous = activeTasks->previousCell;
				TaskListItem* next = activeTasks->nextCell;

				// Previous task points to next task
				previous->nextCell = activeTasks->nextCell;
				next->previousCell = previous;
				next->nextCell = activeTasks;

				// active task between next task and task after that
				activeTasks->previousCell = next;
				activeTasks->nextCell = next->nextCell;
				next->nextCell->previousCell = activeTasks;
			}

			// Move on to next pair of tasks in list
			activeTasks = activeTasks->nextCell;
		}
	} while(swapFlag);
}

/*-------------------------------------- TASKS ----------------------------------*/

static void ddSchedulerTask(void *pvParameters) {
	portTickType xNextWakeTime = xTaskGetTickCount();
	// TODO  should this be global???? Danger of passing pointer to invalid data if not global
	TaskListItem* activeTasks = NULL;
	TaskListItem* overdueTasks = NULL;
	SchedulerMessage receivedMessage;

	//loop
	for(;;) {
		// block on xSchedulerMessageQueue
		xQueueReceive(xSchedulerMessageQueue, receivedMessage, portMAX_DELAY);
		// parse type of message
		TaskListItem taskListItem = receivedMessage.taskListItem;
		switch(taskListItem.taskType) {
		case CREATION_MESSAGE:
			addTaskToEndOfList(activeTasks, &taskListItem);
			break;
		case DELETION_MESSAGE:
			activeTasks = deleteTaskByHandle(activeTasks, taskListItem.tHandle);
			break;
		case ACTIVE_REQUEST:
			// TODO make copy
			xQueueSend(receivedMessage.responseQueueHandle, activeTasks, 0);
			break;
		case OVERDUE_REQUEST:
			// TODO make copy
			xQueueSend(receivedMessage.responseQueueHandle, overdueTasks, 0);
			break;
		}

		// Sort task list by deadline
		activeTasks = sortTasksEDF(activeTasks);

		// Check for overdue tasks
		uint32_t currentTimeMS = clock() / CLOCKS_PER_SEC * 1000;
		while(activeTasks->deadline < currentTimeMS) {
			//remove task from active list and add to overdue list
			addTaskToEndOfList(overdueTasks, activeTasks);
			activeTasks = activeTasks->nextCell;
			activeTasks->previousCell = NULL;
		}

		// Run task at the front of the list
		// TODO - implement task notification for this, have each user task block on receiving the notification

	}
}


// TODO this is an abstraction of tasks that would be generated from createtask
static void userTask1(void *pvParameters) {
	// do nothing for specified time period
	printf("userTask1 completed.\n");
}
static void userTask2(void *pvParameters) {
	printf("userTask2 completed.\n");
}
static void userTask3(void *pvParameters) {
	printf("userTask3 completed.\n");
}

/* Task generators generate user tasks at specified intervals */
static void ddTask1GeneratorTask(void *pvParameters) {
	portTickType xNextWakeTime = xTaskGetTickCount();

	for(;;) {
		// on interval: (vTaskDelayUntil...)
		vTaskDelayUntil(&xNextWakeTime, TASK1_QUEUE_SEND_PERIOD_MS);
		// create TaskParams
		TaskParams task1Params;
		task1Params.taskName = "Task_1";
		task1Params.taskCode = userTask1;
		task1Params.relativeDeadline = 100;
		// call ddTCreate function
		ddTCreate(task1Params);
	}
}
static void ddTask2GeneratorTask(void *pvParameters) {
	portTickType xNextWakeTime = xTaskGetTickCount();

	for(;;) {
		// delay for interval
		vTaskDelayUntil(&xNextWakeTime, TASK2_QUEUE_SEND_PERIOD_MS);
		// create TaskParams
		TaskParams task2Params;
		task2Params.taskName = "Task_2";
		task2Params.taskCode = userTask2;
		task2Params.relativeDeadline = 200;
		// call ddTCreate function
		ddTCreate(task2Params);
	}
}
static void ddTask3GeneratorTask(void *pvParameters) {
	portTickType xNextWakeTime = xTaskGetTickCount();

	for(;;) {
		// delay for interval
		vTaskDelayUntil(&xNextWakeTime, TASK3_QUEUE_SEND_PERIOD_MS);
		// create TaskParams
		TaskParams task3Params;
		task3Params.taskName = "Task_3";
		task3Params.taskCode = userTask3;
		task3Params.relativeDeadline = 400;
		// call ddTCreate function
		ddTCreate(task3Params);
	}
}

static void monitorTask(void *pvParameters) {
	// lowest priority!
	// records own running time
	// records utilization (total running time - own running time / total running time)
	portTickType xNextWakeTime;
	xNextWakeTime = xTaskGetTickCount();
	uint32_t totalTime;
	for(;;) {
		// delay for 1 ms
		vTaskDelayUntil(&xNextWakeTime, 1);
		runningTime++;
		totalTime = clock()/CLOCKS_PER_SEC*1000;
		utilization =  totalTime - runningTime / totalTime;
	}


}

/* ------------------------------------------- MAIN ------------------------------------------------------ */

int main() {
	// Initialize queues
	createQueues();

	// init total running time counter
	runningTime = 0;

	// Create tasks
	xTaskCreate(ddSchedulerTask, "DD_SCHEDULER_TASK", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES-1, NULL);
	xTaskCreate(ddTask1GeneratorTask, "TASK_GENERATOR_TASK", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate(ddTask2GeneratorTask, "TASK_GENERATOR_TASK", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate(ddTask3GeneratorTask, "TASK_GENERATOR_TASK", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
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
