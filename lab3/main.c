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
#include <string.h>

/* Kernel includes. */
#include "stm32f4xx_conf.h"
#include "stm32f4_discovery.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"
#include "../FreeRTOS_Source/portable/MemMang/heap_4.c"

/*----------------------------- DEFINITIONS ------------------------------*/
#define TASK_QUEUE_LENGTH				8

#define TASK1_PERIOD_MS					500
#define TASK2_PERIOD_MS					500
#define	TASK3_PERIOD_MS					750

#define TASK1_EXECUTION_MS				95
#define TASK2_EXECUTION_MS				150
#define TASK3_EXECUTION_MS				250

#define TASK1_PERIOD_TICKS				pdMS_TO_TICKS(TASK1_PERIOD_MS)
#define TASK2_PERIOD_TICKS				pdMS_TO_TICKS(TASK2_PERIOD_MS)
#define TASK3_PERIOD_TICKS				pdMS_TO_TICKS(TASK3_PERIOD_MS)

#define TASK1_QUEUE_SEND_PERIOD_MS		TASK1_PERIOD_MS / portTICK_RATE_MS
#define TASK2_QUEUE_SEND_PERIOD_MS		TASK2_PERIOD_MS / portTICK_RATE_MS
#define TASK3_QUEUE_SEND_PERIOD_MS		TASK3_PERIOD_MS / portTICK_RATE_MS

#define CREATION_MESSAGE				1
#define DELETION_MESSAGE				2
#define ACTIVE_REQUEST					3
#define OVERDUE_REQUEST					4

typedef struct TaskListItem {
	TaskHandle_t tHandle;
	TickType_t deadline;
	uint32_t taskType;
	TickType_t creationTime;
	struct TaskListItem *nextCell;
	struct TaskListItem *previousCell;
} TaskListItem;

typedef struct TaskParams {
	TickType_t relativeDeadline;
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
 * Processor utilization rate as calculated by MonitorTask
 */
float utilization = 0.0;

/* Aperiodic task generator task handle so interrupt task can call it */
TaskHandle_t aperiodicGeneratorHandle;

/* Variables used by monitor task to store runtime statistics, and printed by aperiodic task */
UBaseType_t uxArraySize;
TaskStatus_t * pxTaskStatusArray;
uint32_t ulTotalRunTime;

/*----------------------------- INITIALIZATION FUNCTIONS ------------------------------*/

/* Initialize board LEDs and push button */
static void initBoard() {
	// Initialize LEDs
	STM_EVAL_LEDInit(0); //amber
	STM_EVAL_LEDInit(1); //green
	STM_EVAL_LEDInit(2); //red
	STM_EVAL_LEDInit(3); //blue

	// Initialize user push button
	STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);

	/* Set priority bits */
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

	// Fix priority of ISR
	NVIC_SetPriority(USER_BUTTON_EXTI_IRQn, 5);
}

/* Creates queues used for inter-task communication:
 *	 1) CreateTaskQueue
 *   2) DeleteTaskQueue
 */
static void createQueues() {
	/* CreateTaskQueue: sender is dd_tcreate, receiver is DDSchedulerTask */
	xSchedulerMessageQueue = xQueueCreate(TASK_QUEUE_LENGTH, /* The number of items the queue can hold. */
		sizeof(struct SchedulerMessage));	/* The size of each item the queue holds. */

	/* Add to the registry, for the benefit of kernel aware debugging. */
	vQueueAddToRegistry(xSchedulerMessageQueue, "SchedulerMessageQueue");
}
/*------------------- FUNCTIONS --------------------------------*/


// TODO comments

/* Turn off all board LEDs */
static void turnOffLEDs() {
	STM_EVAL_LEDOff(0);
	STM_EVAL_LEDOff(1);
	STM_EVAL_LEDOff(2);
	STM_EVAL_LEDOff(3);
}


/*
 * Takes taskHandle as parameter (sent by task creation callback function)
 * and adds the task to the queue of active tasks to be scheduled
 */
static void ddTCreate(struct TaskParams* taskParams) {
	TaskHandle_t thandle = NULL;

	// initialize new TaskListItem
	struct TaskListItem newTask;
	newTask.creationTime = xTaskGetTickCount();
	newTask.taskType = CREATION_MESSAGE;
	newTask.deadline = newTask.creationTime + taskParams->relativeDeadline;
	if (newTask.deadline < newTask.creationTime) {
		printf("int overflow on new tasklistitem\n");
	}
	newTask.nextCell = NULL;
	newTask.previousCell = NULL;
	newTask.tHandle = thandle;

	// create the task
	xTaskCreate(taskParams->taskCode, taskParams->taskName, configMINIMAL_STACK_SIZE, NULL, 1, &newTask.tHandle);

	// open a SchedulerResponseQueue to scheduler task
	xQueueHandle xSchedulerResponseQueue = xQueueCreate(1, sizeof(struct TaskListItem));
	// add the queue to the registry
	vQueueAddToRegistry(xSchedulerResponseQueue, "SchedulerCreateResponseQueue");

	// create CreateTaskMessage
	struct SchedulerMessage createMessage = {xSchedulerResponseQueue, newTask};

	// put task on Queue (w createMessage)
	xQueueSend(xSchedulerMessageQueue, &createMessage, 0);

	// wait for response from scheduler task
	struct TaskListItem response;
	xQueueReceive(xSchedulerResponseQueue, &response, portMAX_DELAY);

	// destroy SchedulerResponseQueue
	vQueueUnregisterQueue(xSchedulerResponseQueue);
	vQueueDelete(xSchedulerResponseQueue);
	return;
}

/*
 * Requests that the scheduler deletes the task with the specified handle
 * from the list of active tasks.
 */
static void ddTDelete(TaskHandle_t taskToDelete) {
	// open a SchedulerResponseQueue to scheduler task
	xQueueHandle xSchedulerResponseQueue = xQueueCreate(1, sizeof(struct TaskListItem));
	// add the queue to the registry
	vQueueAddToRegistry(xSchedulerResponseQueue, "SchedulerDeleteResponseQueue");

	// initialize new TaskListItem
	struct TaskListItem newTask;
	newTask.taskType = DELETION_MESSAGE;
	newTask.nextCell = NULL;
	newTask.previousCell = NULL;
	newTask.tHandle = taskToDelete;


	// create SchedulerMessage
	struct SchedulerMessage deleteMessage = {xSchedulerResponseQueue, newTask};
	// put task on DeleteTaskQueue (w task handle) which is read by scheduler
	xQueueSend(xSchedulerMessageQueue, &deleteMessage, 0);
	// wait for response from scheduler
	struct TaskListItem response;
	xQueueReceive(xSchedulerResponseQueue, &response, portMAX_DELAY);
	// destroy SchedulerResponseQueue
	vQueueUnregisterQueue(xSchedulerResponseQueue);
	vQueueDelete(xSchedulerResponseQueue);

	return;
}

/*
 * Gets a copy of the current list of active tasks.
 */
static struct TaskListItem* ddReturnActiveList() {
	struct TaskListItem* activeTasks;
	static xQueueHandle xSchedulerResponseQueue = NULL;
	// open a SchedulerResponseQueue to scheduler task
	xSchedulerResponseQueue = xQueueCreate(1, sizeof(TaskListItem*));
	// add the queue to the registry
	vQueueAddToRegistry(xSchedulerResponseQueue, "SchedulerResponseQueue");
	// initialize TaskListItem
	struct TaskListItem activeItem = {.taskType = ACTIVE_REQUEST};
	// create SchedulerMessage
	struct SchedulerMessage activeMessage = {xSchedulerResponseQueue, activeItem};
	// Ask scheduler for list of active tasks (will have been copied from scheduler internal list at time of request)
	xQueueSend(xSchedulerMessageQueue, &activeMessage, 0);
	// wait for response from scheduler
	xQueueReceive(xSchedulerResponseQueue, &activeTasks, 0);
	// destroy SchedulerResponseQueue
	vQueueUnregisterQueue(xSchedulerResponseQueue);
	vQueueDelete(xSchedulerResponseQueue);
	return activeTasks;
}

/*
 * Gets a copy of the current list of overdue tasks.
 */
static struct TaskListItem* ddReturnOverdueList() {
	struct TaskListItem* overdueTasks;
	static xQueueHandle xSchedulerResponseQueue = NULL;
	// open a SchedulerResponseQueue to scheduler task
	xSchedulerResponseQueue = xQueueCreate(1, sizeof(TaskListItem*));
	// add the queue to the registry
	vQueueAddToRegistry(xSchedulerResponseQueue, "SchedulerResponseQueue");
	// initialize TaskListItem
	struct TaskListItem overdueItem = {.taskType = OVERDUE_REQUEST};
	// create SchedulerMessage
	struct SchedulerMessage overdueMessage = {xSchedulerResponseQueue, overdueItem};
	// Ask scheduler for list of overdue tasks
	xQueueSend(xSchedulerMessageQueue, &overdueMessage, 0);
	// wait for response from scheduler
	xQueueReceive(xSchedulerResponseQueue, &overdueTasks, 0);
	// destroy SchedulerResponseQueue
	vQueueUnregisterQueue(xSchedulerResponseQueue);
	vQueueDelete(xSchedulerResponseQueue);
	return overdueTasks;
}

/*
 * Returns a pointer to the first item in the list, or NULL if list is empty
 */
static struct TaskListItem* getBeginningOfList(struct TaskListItem* currentTask) {
	while(currentTask != NULL && currentTask->previousCell != NULL) {
		currentTask = currentTask->previousCell;
	}
	return currentTask;
}

/*
 * Adds the task list item to the end of the list; returns a pointer to the beginning of the modified list
 */
static struct TaskListItem* addTaskToEndOfList(struct TaskListItem* beginningOfList, struct TaskListItem* taskToAdd) {
	struct TaskListItem* currentTask = beginningOfList;

	// If list is empty, new task is head of list
	if(currentTask == NULL) {
		currentTask = pvPortMalloc(sizeof(struct TaskListItem));
		if(currentTask == NULL) {
			printf("Houston we have a problem\n");
			return NULL;
		}
		currentTask->creationTime = taskToAdd->creationTime;
		currentTask->deadline = taskToAdd->deadline;
		currentTask->tHandle = taskToAdd->tHandle;
		currentTask->nextCell = NULL;
		currentTask->previousCell = NULL;
	} else {
		// else find the end of the list
		while(currentTask->nextCell != NULL) {
			currentTask = currentTask->nextCell;
		}

		// create the new item at the end of the list
		currentTask->nextCell = pvPortMalloc(sizeof(struct TaskListItem));
		currentTask->nextCell->creationTime = taskToAdd->creationTime;
		currentTask->nextCell->deadline = taskToAdd->deadline;
		currentTask->nextCell->tHandle = taskToAdd->tHandle;
		currentTask->nextCell->nextCell = NULL;
		currentTask->nextCell->previousCell = currentTask;
	}

	return getBeginningOfList(currentTask);
}

/*
 * Deletes the specified task from the list of active tasks by iterating
 * through the list of active tasks and comparing task handles
 */
static struct TaskListItem* deleteTaskByHandle(struct TaskListItem* activeTasks, TaskHandle_t taskHandle) {
	struct TaskListItem* currentTask = activeTasks;
	struct TaskListItem* beginningOfList = NULL;

	while (currentTask != NULL) {
		if (currentTask->tHandle == taskHandle) {
			TaskListItem* prev = currentTask->previousCell;
			TaskListItem* next = currentTask->nextCell;
			if (next != NULL) {
				next->previousCell = prev;
				beginningOfList = getBeginningOfList(next);
			}

			// This will be infrequent, we will almost always be deleting from head of list
			if (prev != NULL) {
				prev->nextCell = next;
				beginningOfList = getBeginningOfList(prev);
			}

			// Free memory associated with item
			vPortFree(currentTask);

			// Return pointer to front of list
			return beginningOfList;
		}
		else {
			currentTask = currentTask->nextCell;
		}
	}

	// If list is empty, or if task handle isn't in list, return beginning of list (or NULL)
	return getBeginningOfList(currentTask);
}

static struct TaskListItem* sortTasksEDF(struct TaskListItem* activeTasks) {
	// Task list will always be very short for this application so efficiency doesn't much matter, using bubble sort
	int swapFlag = 0;
	uint32_t deadline1;
	uint32_t deadline2;
	struct TaskListItem* currentTask = activeTasks;

	// Check list for needed swaps, iterate through list repeatedly until no swaps are made
	do {
		swapFlag = 0;
		while(currentTask != NULL && currentTask->nextCell != NULL) {
			deadline1 = currentTask->deadline;
			deadline2 = currentTask->nextCell->deadline;
			// if deadline of next task is closer than current tasks, swap tasks
			if(deadline2 < deadline1) {
				swapFlag = 1;
				TaskListItem* previous = currentTask->previousCell;
				TaskListItem* next = currentTask->nextCell;

				// Previous task points to next task
				if(previous != NULL) {
					previous->nextCell = next;
				}
				next->previousCell = previous;

				currentTask->nextCell = next->nextCell;
				if(next->nextCell != NULL) {
					next->nextCell->previousCell = currentTask;
				}

				next->nextCell = currentTask;
				currentTask->previousCell = next;
			}

			// Move on to next pair of tasks in list
			currentTask = currentTask->nextCell;
		}
	} while(swapFlag);

	return currentTask;
}

/*-------------------------------------- TASKS ----------------------------------*/

static void ddSchedulerTask(void *pvParameters) {
	struct TaskListItem* activeTasks = NULL;
	struct TaskListItem* overdueTasks = NULL;
	struct SchedulerMessage receivedMessage;

	//loop
	for(;;) {
		// block on xSchedulerMessageQueue
		xQueueReceive(xSchedulerMessageQueue, &receivedMessage, portMAX_DELAY);

		// parse type of message
		struct TaskListItem taskListItem = receivedMessage.taskListItem;
		struct TaskListItem responseMessage = {};
		switch(taskListItem.taskType) {
			case CREATION_MESSAGE:
				// Add task to active task list, then respond to caller
				activeTasks = addTaskToEndOfList(activeTasks, &taskListItem);
				xQueueSend(receivedMessage.responseQueueHandle, &responseMessage, 0);
				break;
			case DELETION_MESSAGE:
				// Delete task from active task list, then respond to caller
				activeTasks = deleteTaskByHandle(activeTasks, taskListItem.tHandle);
				xQueueSend(receivedMessage.responseQueueHandle, &responseMessage, 0);
				break;
			case ACTIVE_REQUEST:
				// TODO make copy?
				// Send a copy of the active tasks list to the caller
				xQueueSend(receivedMessage.responseQueueHandle, &activeTasks, 0);
				break;
			case OVERDUE_REQUEST:
				// Send a copy of the overdue tasks list to the caller
				xQueueSend(receivedMessage.responseQueueHandle, &overdueTasks, 0);
				break;
			default:
				printf("in default\n");
				break;
		}

		// Sort task list by deadline
		activeTasks = sortTasksEDF(activeTasks);

		// Check for overdue tasks
		TickType_t currentTimeTicks = xTaskGetTickCount();
		while(activeTasks != NULL && activeTasks->deadline < currentTimeTicks) {
			printf("overdue task detected\n");
			// TODO fix
			//remove task from active list and add to overdue list
//			overdueTasks = addTaskToEndOfList(overdueTasks, activeTasks);
			activeTasks = activeTasks->nextCell;
			activeTasks->previousCell = NULL;
		}

		// Run task at the front of the list, if list isn't empty
		if(activeTasks != NULL) {
			xTaskNotifyGive(activeTasks->tHandle);
		}
	}

	printf("this shouldn't happen\n");
}

static void userTask1(void *pvParameters) {
	// Block indefinitely until notified by scheduler
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	// Turn off other LEDS
	turnOffLEDs();
	// Turn on amber LED
	STM_EVAL_LEDOn(0);

	// busywork for defined execution time
	TickType_t xDelayTicks = TASK1_EXECUTION_MS / portTICK_PERIOD_MS;
	TickType_t targetTimeTicks = xTaskGetTickCount() + xDelayTicks;
	while(xTaskGetTickCount() < targetTimeTicks) { };

	// Request scheduler to delete this task from active task list
	const char* taskName = "Task_1";
	TaskHandle_t tHandle = xTaskGetHandle(taskName);
	ddTDelete(tHandle);
	vTaskDelete(NULL);
}

static void userTask2(void *pvParameters) {
	// Block indefinitely until notified by scheduler
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	// Turn off other LEDS
	turnOffLEDs();
	// Turn on green LED
	STM_EVAL_LEDOn(1);

	// busywork for defined execution time
	TickType_t xDelayTicks = TASK2_EXECUTION_MS / portTICK_PERIOD_MS;
	TickType_t targetTimeTicks = xTaskGetTickCount() + xDelayTicks;
	while(xTaskGetTickCount() < targetTimeTicks) { };

	// Request scheduler to delete this task from active task list
	TaskHandle_t tHandle = xTaskGetHandle("Task_2");
	ddTDelete(tHandle);
	vTaskDelete(NULL);

}

static void userTask3(void *pvParameters) {
	// Block indefinitely until notified by scheduler
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	// Turn off other LEDS
	turnOffLEDs();
	// Turn on blue LED
	STM_EVAL_LEDOn(3);

	// busywork for defined execution time
	TickType_t xDelayTicks = TASK3_EXECUTION_MS / portTICK_PERIOD_MS;
	TickType_t targetTimeTicks = xTaskGetTickCount() + xDelayTicks;
	while(xTaskGetTickCount() < targetTimeTicks) { };

	// Request scheduler to delete this task from active task list
	TaskHandle_t tHandle = xTaskGetHandle("Task_3");
	ddTDelete(tHandle);
	vTaskDelete(NULL);

}

static void printStats() {
	unsigned long ulMonitorRunTime = 0;
	TaskStatus_t * taskStatus = pxTaskStatusArray;

	UBaseType_t i = 0;
	printf("MONITOR TASK REPORT\n\nNAME\t\tRUNTIME\n");
	for(i = 0; i < uxArraySize; i++) {
		printf("%s\t\t%lu\n", taskStatus->pcTaskName, taskStatus->ulRunTimeCounter);
		if (strcmp("MONITOR_TASK", taskStatus->pcTaskName) == 0) {
			// TODO remove
			printf("found monitor task\n");
			ulMonitorRunTime = taskStatus->ulRunTimeCounter;

		}
		taskStatus += sizeof(TaskStatus_t);
	}

	unsigned long utilization = (unsigned long) (ulTotalRunTime - ulMonitorRunTime) / ulTotalRunTime;
	printf("\nTOTAL RUNTIME:\t%u\nUTILIZATION:\t%ul\n", ulTotalRunTime, utilization);
}

// TODO fix this. Right now it gets stuck in an infinite loop
/* The aperiodic task triggered by user button on board */
static void aperiodicTask(void *pvParameters) {
	// TODO delete
	printf("in aperiodic task\n");

	// Block indefinitely until notified by scheduler
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	// Turn off other LEDS
	turnOffLEDs();
	// Turn on red LED
	STM_EVAL_LEDOn(2);

	// Print out the stats collected by the monitor task
	// printStats();

	int i = 0;
	for (i = 0; i < 1000; i++) { }
	//TODO do something with this
	// struct TaskListItem* activeTasks = ddReturnActiveList();
	// struct TaskListItem* overdueTasks = ddReturnOverdueList();

	printf("aperiodic task completed.\n");

	// Request scheduler to delete this task from active task list
	TaskHandle_t tHandle = xTaskGetHandle("ap_task");
	ddTDelete(tHandle);
	vTaskDelete(NULL);
}

static void ddAperiodicTaskGeneratorTask(void *pvParameters) {
	// create TaskParams
	struct TaskParams aperiodicTaskParams;
	aperiodicTaskParams.taskName = "ap_task";
	aperiodicTaskParams.taskCode = aperiodicTask;
	aperiodicTaskParams.relativeDeadline = pdMS_TO_TICKS(100);

	for(;;) {
		// Block indefinitely until notified by scheduler
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		printf("generating aperiodic task\n");

		// call ddTCreate function
		ddTCreate(&aperiodicTaskParams);
	}
}


/* Task generators generate user tasks at specified intervals */
static void ddTask1GeneratorTask(void *pvParameters) {
	portTickType xNextWakeTime = xTaskGetTickCount();

	for(;;) {

		// on interval: (vTaskDelayUntil...)
		vTaskDelayUntil(&xNextWakeTime, TASK1_QUEUE_SEND_PERIOD_MS);
		// create TaskParams
		struct TaskParams task1Params;
		task1Params.taskName = "Task_1";
		task1Params.taskCode = userTask1;
		task1Params.relativeDeadline = TASK1_PERIOD_TICKS;
		ddTCreate(&task1Params);
	}
}

static void ddTask2GeneratorTask(void *pvParameters) {
	portTickType xNextWakeTime = xTaskGetTickCount();

	for(;;) {

		// delay for interval
		vTaskDelayUntil(&xNextWakeTime, TASK2_QUEUE_SEND_PERIOD_MS);
		// create TaskParams
		struct TaskParams task2Params;
		task2Params.taskName = "Task_2";
		task2Params.taskCode = userTask2;
		task2Params.relativeDeadline = TASK2_PERIOD_TICKS;
		ddTCreate(&task2Params);
	}
}

static void ddTask3GeneratorTask(void *pvParameters) {
	portTickType xNextWakeTime = xTaskGetTickCount();

	for(;;) {
		// delay for interval
		vTaskDelayUntil(&xNextWakeTime, TASK3_QUEUE_SEND_PERIOD_MS);
		// create TaskParams
		struct TaskParams task3Params;
		task3Params.taskName = "Task_3";
		task3Params.taskCode = userTask3;
		task3Params.relativeDeadline = TASK3_PERIOD_TICKS;
		ddTCreate(&task3Params);
	}
}

static void monitorTask(void *pvParameters) {
	/* TODO this task calculates processor utilization and system overhead */
	printf("monitor task\n");
	for(;;) {}
//	printf("num tasks = %u\nallocating status array now\n", uxArraySize);
//	pxTaskStatusArray = pvPortMalloc(uxArraySize * sizeof(TaskStatus_t));
//	BaseType_t test = uxArraySize;
//
//	printf("entering infinite loop\n");
//	for(;;) {
//		uxArraySize = uxTaskGetNumberOfTasks();
//		uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalRunTime);
//	}
}

/* Interrupt handler */

/* Awakens aperiodic task generator from button interrupt */
void EXTI0_IRQHandler(void) {
	// Ensure interrupt bit is set
	if(EXTI_GetITStatus(EXTI_Line0) != RESET) {
		// clear interrupt flag
		EXTI_ClearITPendingBit(EXTI_Line0);
		// Send notification to generator task
		vTaskNotifyGiveFromISR(aperiodicGeneratorHandle, NULL);
	}
}

/* ------------------------------------------- MAIN ------------------------------------------------------ */

int main() {
	// Initialize queues
	createQueues();
	// Setup lights, user input on board
	initBoard();

	// init total running time counter
	runningTime = 0;

	// Create tasks
	xTaskCreate(ddSchedulerTask, "DD_SCHEDULER_TASK", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate(ddTask1GeneratorTask, "TASK1_GEN_TASK", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
	xTaskCreate(ddTask2GeneratorTask, "TASK2_GEN_TASK", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
	xTaskCreate(ddTask3GeneratorTask, "TASK3_GEN_TASK", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
	xTaskCreate(ddAperiodicTaskGeneratorTask, "AP_G_TASK", configMINIMAL_STACK_SIZE, NULL, 3, &aperiodicGeneratorHandle);
	xTaskCreate(monitorTask, "MON_TASK", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

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
