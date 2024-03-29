/* ECE 455 Lab 3 main.c file
*  Authored by Meagan Russell and Andrea Williams
*  Spring 2019
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
#include "../FreeRTOS_Source/include/event_groups.h"

/*----------------------------- DEFINITIONS ------------------------------*/
#define TASK_QUEUE_LENGTH				8

#define TASK1_EXECUTION_MS				95
#define TASK2_EXECUTION_MS				150
#define TASK3_EXECUTION_MS				250

#define TASK1_PERIOD_MS					500
#define TASK2_PERIOD_MS					500
#define	TASK3_PERIOD_MS					750

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

#define clockFreqGHz					SystemCoreClock / 1000000

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

typedef struct SchedulerMessage {
	xQueueHandle responseQueueHandle;
	TaskListItem taskListItem;
} SchedulerMessage;

typedef struct RunTimeStats {
	uint32_t timeExecutingTasks;
	uint32_t timeInScheduler;
	uint32_t totalRunTime;
} RunTimeStats;

/*------------------------------ GLOBAL VARIABLES -----------------------------*/

/* Queue used to communicate between create/delete tasks and scheduler tasks */
static xQueueHandle xSchedulerMessageQueue = NULL;

/* The semaphore (in this case binary) that is used by the FreeRTOS tick hook
 * function and the event semaphore task.
 */
static xSemaphoreHandle xEventSemaphore = NULL;

/* Aperiodic task generator task handle so interrupt task can call it */
TaskHandle_t aperiodicGeneratorHandle;

/* Task run time storage, used by monitor task to calculate utilization and overhead */
struct RunTimeStats runTimeStats;

/* Event group (flag) to signal that a task has been run */
EventGroupHandle_t xTaskExecutedFlag;

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

	// Set priority bits for interrupt
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	// Fix priority of ISR
	NVIC_SetPriority(USER_BUTTON_EXTI_IRQn, 5);

	// Initialize the timer
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseInitTypeDef timerInitStructure;
	// set timer such that it only counts per microsecond
	timerInitStructure.TIM_Prescaler = clockFreqGHz;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = 0xFFFFFFFF;
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	timerInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &timerInitStructure);
	TIM_Cmd(TIM2, ENABLE);

	// create the flag for task execution
	xTaskExecutedFlag = xEventGroupCreate();
	// sanity check that flag was created ok
	if (xTaskExecutedFlag == NULL) {
		printf("Failed to create TaskExecutedFlag");
	}
}

/* Creates CreateTaskQueue used for inter-task communication: sender is ddTCreate, receiver is ddSchedulerTask */
static void createQueues() {
	xSchedulerMessageQueue = xQueueCreate(TASK_QUEUE_LENGTH, sizeof(struct SchedulerMessage));

	/* Add to the registry, for the benefit of kernel aware debugging. */
	vQueueAddToRegistry(xSchedulerMessageQueue, "SchedulerMessageQueue");
}
/*----------------------------- FUNCTIONS --------------------------------*/

/* Turn off all board LEDs */
void turnOffLEDs() {
	STM_EVAL_LEDOff(0);
	STM_EVAL_LEDOff(1);
	STM_EVAL_LEDOff(2);
	STM_EVAL_LEDOff(3);
}

/* Utility function that prints handle and deadline of each task in the provided active tasks list */
void printActiveTasks(struct TaskListItem* activeTasks) {
	if (activeTasks == NULL) {
		printf("No active tasks.\n");
	}
	else {
		printf("Active Tasks:\n");

		while (activeTasks != NULL) {
			printf("%u, %u\n",activeTasks->tHandle, activeTasks->deadline);
			activeTasks = activeTasks->nextCell;
		}
	}
}

/* Utility function that prints handle and deadline of each task in the provided overdue tasks list */
void printOverdueTasks(struct TaskListItem* overdueTasks) {
	if (overdueTasks == NULL) {
		printf("No overdue tasks.\n");
	}
	else {
		printf("Overdue Tasks:\n");

		while (overdueTasks != NULL) {
			printf("%u, %u\n", overdueTasks->tHandle, overdueTasks->deadline);
			overdueTasks = overdueTasks->nextCell;
		}
	}
}


/*
 * Takes taskHandle as parameter (sent by task creation callback function)
 * and adds the task to the queue of active tasks to be scheduled
 */
void ddTCreate(struct TaskParams* taskParams) {
	TaskHandle_t thandle = NULL;

	// initialize new TaskListItem
	struct TaskListItem newTask;
	newTask.creationTime = xTaskGetTickCount();
	newTask.taskType = CREATION_MESSAGE;

	// if the task is aperiodic, it has relative deadline of 0
	if (taskParams->relativeDeadline == 0) {
		newTask.deadline = 0;
	} else {
		newTask.deadline = newTask.creationTime + taskParams->relativeDeadline;
		if (newTask.deadline < newTask.creationTime) {
			printf("Deadline less than creation time in createTask - possible int overflow on new tasklistitem\n");
		}
	}

	newTask.nextCell = NULL;
	newTask.previousCell = NULL;
	newTask.tHandle = thandle;

	xTaskCreate(taskParams->taskCode, taskParams->taskName, configMINIMAL_STACK_SIZE, NULL, 2, &newTask.tHandle);

	// open a SchedulerResponseQueue to scheduler task and add to registry
	xQueueHandle xSchedulerResponseQueue = xQueueCreate(1, sizeof(struct TaskListItem));
	vQueueAddToRegistry(xSchedulerResponseQueue, "SchedulerCreateResponseQueue");

	struct SchedulerMessage createMessage = {xSchedulerResponseQueue, newTask};
	xQueueSend(xSchedulerMessageQueue, &createMessage, 0);

	// wait for response from scheduler task, then destroy response queue
	struct TaskListItem response;
	xQueueReceive(xSchedulerResponseQueue, &response, portMAX_DELAY);

	vQueueUnregisterQueue(xSchedulerResponseQueue);
	vQueueDelete(xSchedulerResponseQueue);
	return;
}

/*
 * Requests that the scheduler deletes the task with the specified handle
 * from the list of active tasks.
 */
void ddTDelete(TaskHandle_t taskToDelete, const char* taskName) {
	// open a SchedulerResponseQueue to scheduler task
	xQueueHandle xSchedulerResponseQueue = xQueueCreate(1, sizeof(struct TaskListItem));
	vQueueAddToRegistry(xSchedulerResponseQueue, "SchedulerDeleteResponseQueue");

	// initialize new TaskListItem
	struct TaskListItem newTask;
	newTask.taskType = DELETION_MESSAGE;
	newTask.nextCell = NULL;
	newTask.previousCell = NULL;
	newTask.tHandle = taskToDelete;

	// If aperiodic, set deadline = 0 to signal to scheduler
	newTask.deadline = strcmp(taskName, "ap_task");

	struct SchedulerMessage deleteMessage = {xSchedulerResponseQueue, newTask};
	xQueueSend(xSchedulerMessageQueue, &deleteMessage, 0);

	// wait for response from scheduler, then destroy response queue
	struct TaskListItem response;
	xQueueReceive(xSchedulerResponseQueue, &response, portMAX_DELAY);

	vQueueUnregisterQueue(xSchedulerResponseQueue);
	vQueueDelete(xSchedulerResponseQueue);
	return;
}

/*
 * Gets a copy of the list of active tasks.
 */
struct TaskListItem* ddReturnActiveList() {
	struct TaskListItem* activeTasks;
	static xQueueHandle xSchedulerResponseQueue = NULL;

	// open a SchedulerResponseQueue to scheduler task
	xSchedulerResponseQueue = xQueueCreate(1, sizeof(TaskListItem*));
	vQueueAddToRegistry(xSchedulerResponseQueue, "SchedulerResponseQueue");

	struct TaskListItem activeItem = {.taskType = ACTIVE_REQUEST};
	struct SchedulerMessage activeMessage = {xSchedulerResponseQueue, activeItem};

	// Ask scheduler for list of active tasks
	xQueueSend(xSchedulerMessageQueue, &activeMessage, 0);

	// wait for response from scheduler
	xQueueReceive(xSchedulerResponseQueue, &activeTasks, 0);

	// destroy response queue
	vQueueUnregisterQueue(xSchedulerResponseQueue);
	vQueueDelete(xSchedulerResponseQueue);
	return activeTasks;
}

/*
 * Gets a copy of the list of overdue tasks.
 */
struct TaskListItem* ddReturnOverdueList() {
	struct TaskListItem* overdueTasks;
	static xQueueHandle xSchedulerResponseQueue = NULL;

	// open a SchedulerResponseQueue to scheduler task
	xSchedulerResponseQueue = xQueueCreate(1, sizeof(TaskListItem*));
	vQueueAddToRegistry(xSchedulerResponseQueue, "SchedulerResponseQueue");

	struct TaskListItem overdueItem = {.taskType = OVERDUE_REQUEST};
	struct SchedulerMessage overdueMessage = {xSchedulerResponseQueue, overdueItem};

	// Ask scheduler for list of overdue tasks
	xQueueSend(xSchedulerMessageQueue, &overdueMessage, 0);

	// wait for response from scheduler
	xQueueReceive(xSchedulerResponseQueue, &overdueTasks, 0);
	vQueueUnregisterQueue(xSchedulerResponseQueue);

	// destroy response queue
	vQueueDelete(xSchedulerResponseQueue);
	return overdueTasks;
}

/*
 * Returns a pointer to the first item in the list, or NULL if list is empty
 */
struct TaskListItem* getBeginningOfList(struct TaskListItem* currentTask) {
	while(currentTask != NULL && currentTask->previousCell != NULL) {
		currentTask = currentTask->previousCell;
	}
	return currentTask;
}

/*
 * Adds the task list item to the end of the list; returns a pointer to the beginning of the modified list.
 */
struct TaskListItem* addTaskToEndOfList(struct TaskListItem* beginningOfList, struct TaskListItem* taskToAdd) {
	struct TaskListItem* currentTask = beginningOfList;

	// If list is empty, new task is head of list
	if(currentTask == NULL) {
		currentTask = pvPortMalloc(sizeof(struct TaskListItem));
		if(currentTask == NULL) {
			printf("Failed to allocate memory for list item!\n");
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
		if(currentTask->nextCell == NULL) {
			printf("Failed to allocate memory for list item!\n");
			return NULL;
		}
		currentTask->nextCell->creationTime = taskToAdd->creationTime;
		currentTask->nextCell->deadline = taskToAdd->deadline;
		currentTask->nextCell->tHandle = taskToAdd->tHandle;
		currentTask->nextCell->nextCell = NULL;
		currentTask->nextCell->previousCell = currentTask;
	}

	return getBeginningOfList(currentTask);
}

/*
 * Deletes the specified task from the list of active tasks.
 * Should be passed the pointer to the beginning of the list.
 */
struct TaskListItem* deleteTaskByHandle(struct TaskListItem* activeTasks, TaskHandle_t taskHandle) {
	struct TaskListItem* currentTask = activeTasks;
	struct TaskListItem* beginningOfList = NULL;

	// Find the task in the list by handle
	while (currentTask != NULL) {
		if (currentTask->tHandle == taskHandle) {
			TaskListItem* prev = currentTask->previousCell;
			TaskListItem* next = currentTask->nextCell;

			// Make sure next item's previousCell pointer is no longer pointing to item to delete
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

/* Sorts the linked list of tasks, earliest deadline first.
*  Should be passed a pointer to the head of the active tasks list.
*  Returns a pointer to the beginning of the list. */
struct TaskListItem* sortTasksEDF(struct TaskListItem* activeTasks) {
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

	return getBeginningOfList(currentTask);
}

/*-------------------------------------- TASKS ----------------------------------*/

/* This task is responsible for scheduling the user tasks.
*  The scheduler uses an EDF scheduling algorithm to maintain a list of active
*  periodic tasks and their priorities. It also maintains a list of overdue tasks,
*  and a list of aperiodic tasks.
*
*  The scheduler is normally suspended (it blocks on its input queue for scheduling
*  request messages to arrive). On receipt of a request, it fulfils the request by
*  adding a task to the active tasks list, deleting the task from the list, or
*  returning the list of active or overdue tasks. It then sorts the list of active
*  tasks in increasing deadline order. It checks for any overdue tasks, adds these
*  to the overdue tasks list, and deletes them from the active tasks list. It then
*  runs the active task at the front of the list. If no periodic tasks are ready,
*  the scheduler will run an aperiodic task if one is ready.
*/
static void ddSchedulerTask(void *pvParameters) {
	struct TaskListItem* activeTasks = NULL;
	struct TaskListItem* overdueTasks = NULL;
	struct TaskListItem* aperiodicTasks = NULL;
	struct SchedulerMessage receivedMessage;
	int aperiodicTaskCounter = 0;
	uint32_t taskStart = 0;
	EventBits_t uxBits;

	// clear timer before we start
	TIM2->CNT = ((uint32_t)0x0);

	// infinite loop
	for(;;) {
		// block on xSchedulerMessageQueue
		xQueueReceive(xSchedulerMessageQueue, &receivedMessage, portMAX_DELAY);

		// record time in task (if task flag has been set)
		uxBits = xEventGroupGetBits(xTaskExecutedFlag);
		if (uxBits == 1) {
			runTimeStats.timeExecutingTasks += ( TIM2->CNT  - taskStart );
			// clear the flag
			xEventGroupClearBits(xTaskExecutedFlag, 1);
		}

		// update total exec time up to this point (in microseconds)
		runTimeStats.totalRunTime += TIM2->CNT;

		// clear timer
		TIM2->CNT = ((uint32_t)0x0);

		// parse type of message
		struct TaskListItem taskListItem = receivedMessage.taskListItem;
		struct TaskListItem responseMessage = {};
		switch(taskListItem.taskType) {
			case CREATION_MESSAGE:
				if(taskListItem.deadline != 0) {
					// This is a periodic task; add it to the active tasks list
					activeTasks = addTaskToEndOfList(activeTasks, &taskListItem);
				} else {
					// This is an aperiodic task. Check how many we have, reject if too many -
					// this is to prevent memory from being eaten up by unresolved aperiodic tasks
					if (aperiodicTaskCounter > 2) {
						printf("Too many aperiodic tasks! Not scheduled.\n");
					} else {
						// Add it to the aperiodic tasks list
						aperiodicTaskCounter++;
						aperiodicTasks = addTaskToEndOfList(aperiodicTasks, &taskListItem);
					}
				}
				// Respond to caller
				xQueueSend(receivedMessage.responseQueueHandle, &responseMessage, 0);
				break;

			case DELETION_MESSAGE:
				// Delete task from appropriate task list, then respond to caller
				if(taskListItem.deadline != 0) {
					activeTasks = deleteTaskByHandle(activeTasks, taskListItem.tHandle);
				} else {
					aperiodicTasks = deleteTaskByHandle(aperiodicTasks, taskListItem.tHandle);
					aperiodicTaskCounter--;
				}
				xQueueSend(receivedMessage.responseQueueHandle, &responseMessage, 0);
				break;

			case ACTIVE_REQUEST:
				// Send the current active tasks list to the caller
				xQueueSend(receivedMessage.responseQueueHandle, &activeTasks, 0);
				break;

			case OVERDUE_REQUEST:
				// Send the current overdue tasks list to the caller
				xQueueSend(receivedMessage.responseQueueHandle, &overdueTasks, 0);
				break;

			default:
				printf("In default case. This should not happen!\n");
				break;
		}

		// Sort task list by deadline
		activeTasks = sortTasksEDF(activeTasks);

		// Check for overdue tasks - add these to overdue tasks list and remove from active tasks list, then delete
		TickType_t currentTimeTicks = xTaskGetTickCount();
		while(activeTasks != NULL && activeTasks->deadline < currentTimeTicks) {
			TaskListItem newOverdue = *activeTasks;
			overdueTasks = addTaskToEndOfList(overdueTasks, &newOverdue);
			activeTasks = deleteTaskByHandle(activeTasks, newOverdue.tHandle);
			vTaskDelete(newOverdue.tHandle);
		}

		// Record the time spent in the scheduler
		runTimeStats.timeInScheduler += TIM2->CNT;

		// Run periodic task at the front of the list, if list isn't empty
		if(activeTasks != NULL) {
			uxBits = xEventGroupSetBits(xTaskExecutedFlag, 1);
			taskStart = TIM2->CNT;
			xTaskNotifyGive(activeTasks->tHandle);
		} else {
			// If there are no periodic tasks to run, try an aperiodic task
			if(aperiodicTasks != NULL) {
				uxBits = xEventGroupSetBits(xTaskExecutedFlag, 1);
				taskStart = TIM2->CNT;
				xTaskNotifyGive(aperiodicTasks->tHandle);
			}
		}
	}
}

/* This is one of the three user tasks that is generated from a generator task.
*  It has a period and execution time defined at the top of the file. */
static void userTask1(void *pvParameters) {
	// Block indefinitely until notified by scheduler
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	// Turn off other LEDS (in case of pre-emption)
	turnOffLEDs();
	// Turn on amber LED
	STM_EVAL_LEDOn(0);

	// busywork for defined execution time
	TickType_t xDelayTicks = TASK1_EXECUTION_MS / portTICK_PERIOD_MS;
	TickType_t targetTimeTicks = xTaskGetTickCount() + xDelayTicks;
	while(xTaskGetTickCount() < targetTimeTicks) { };

	// Turn off amber LED
	STM_EVAL_LEDOff(0);

	// Request scheduler to delete this task from active task list
	const char* taskName = "Task_1";
	TaskHandle_t tHandle = xTaskGetHandle(taskName);
	ddTDelete(tHandle, taskName);

	// Delete task
	vTaskDelete(NULL);
}

/* This is one of the three user tasks that is generated from a generator task.
*  It has a period and execution time defined at the top of the file. */
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

	// Turn off green LED
	STM_EVAL_LEDOff(1);

	// Request scheduler to delete this task from active task list
	const char* taskName = "Task_2";
	TaskHandle_t tHandle = xTaskGetHandle(taskName);
	ddTDelete(tHandle, taskName);

	// Delete task
	vTaskDelete(NULL);
}

/* This is one of the three user tasks that is generated from a generator task.
*  It has a period and execution time defined at the top of the file. */
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

	// Turn off blue LED
	STM_EVAL_LEDOff(3);

	// Request scheduler to delete this task from active task list
	const char* taskName = "Task_3";
	TaskHandle_t tHandle = xTaskGetHandle(taskName);
	ddTDelete(tHandle, taskName);

	// Delete task
	vTaskDelete(NULL);
}

/* The aperiodic task is generated from the aperiodic task generator task.
*  It has an arbitrary execution time with no hard deadline. The aperiodic
*  task also fetches and prints the active and overdue task lists from the
*  scheduler. Since this task will be pre-empted by any periodic task, the
*  lists printed are representative of the list at some point in the
*  program's execution and not necessarily at the time of calling. */
static void aperiodicTask(void *pvParameters) {
	// Block indefinitely until notified by scheduler
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	// Turn off other LEDS
	turnOffLEDs();
	// Turn on red LED
	STM_EVAL_LEDOn(2);

	// busywork for execution time
	TickType_t xDelayTicks = 250 / portTICK_PERIOD_MS;
	TickType_t targetTimeTicks = xTaskGetTickCount() + xDelayTicks;
	while(xTaskGetTickCount() < targetTimeTicks) { };

	struct TaskListItem* activeTasks = ddReturnActiveList();
	struct TaskListItem* overdueTasks = ddReturnOverdueList();
	printActiveTasks(activeTasks);
	printOverdueTasks(overdueTasks);

	// Turn off red LED
	STM_EVAL_LEDOff(2);

	// Request scheduler to delete this task from active task list
	const char* taskName = "ap_task";
	TaskHandle_t tHandle = xTaskGetHandle(taskName);
	ddTDelete(tHandle, taskName);
	vTaskDelete(NULL);
}

/* This task generates the aperiodic task. It is triggered by an interrupt
*  generated by the user button on the board. */
static void ddAperiodicTaskGeneratorTask(void *pvParameters) {
	struct TaskParams aperiodicTaskParams;
	aperiodicTaskParams.taskName = "ap_task";
	aperiodicTaskParams.taskCode = aperiodicTask;
	aperiodicTaskParams.relativeDeadline = 0;

	for(;;) {
		// Block indefinitely until notified by interrupt
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		ddTCreate(&aperiodicTaskParams);
	}
}


/* Task generators generate user tasks at specified intervals */
static void ddTask1GeneratorTask(void *pvParameters) {
	portTickType xNextWakeTime = xTaskGetTickCount();

	for(;;) {
		// delay for task period interval
		vTaskDelayUntil(&xNextWakeTime, TASK1_QUEUE_SEND_PERIOD_MS);

		struct TaskParams task1Params;
		task1Params.taskName = "Task_1";
		task1Params.taskCode = userTask1;
		task1Params.relativeDeadline = TASK1_PERIOD_TICKS;
		ddTCreate(&task1Params);
	}
}

/* Task generators generate user tasks at specified intervals */
static void ddTask2GeneratorTask(void *pvParameters) {
	portTickType xNextWakeTime = xTaskGetTickCount();

	for(;;) {
		// delay for task period interval
		vTaskDelayUntil(&xNextWakeTime, TASK2_QUEUE_SEND_PERIOD_MS);

		struct TaskParams task2Params;
		task2Params.taskName = "Task_2";
		task2Params.taskCode = userTask2;
		task2Params.relativeDeadline = TASK2_PERIOD_TICKS;
		ddTCreate(&task2Params);
	}
}

/* Task generators generate user tasks at specified intervals */
static void ddTask3GeneratorTask(void *pvParameters) {
	portTickType xNextWakeTime = xTaskGetTickCount();

	for(;;) {
		// delay for task period interval
		vTaskDelayUntil(&xNextWakeTime, TASK3_QUEUE_SEND_PERIOD_MS);

		struct TaskParams task3Params;
		task3Params.taskName = "Task_3";
		task3Params.taskCode = userTask3;
		task3Params.relativeDeadline = TASK3_PERIOD_TICKS;
		ddTCreate(&task3Params);
	}
}

/* The monitor task calculates and prints processor utilization and system overhead.
*  This task has the lowest priority - it only runs when no other task is running. */
static void monitorTask(void *pvParameters) {
	portTickType xNextWakeTime = xTaskGetTickCount();
	uint32_t utilization = 0;
	for(;;) {
		// delay for interval, allow idle task to work to clean up memory
		vTaskDelayUntil(&xNextWakeTime, 5000);

		// calculate processor utilization
		utilization = (uint32_t) (100 * runTimeStats.timeExecutingTasks) / runTimeStats.totalRunTime;

		// Report as integer percentages
		printf("Scheduler overhead: %u microseconds\nProcessor utilization: %u microseconds in tasks / %u microseconds total = %u percent utilization\n",
				runTimeStats.timeInScheduler, runTimeStats.timeExecutingTasks, runTimeStats.totalRunTime, utilization);
	}
}

/* Interrupt handler - awakens aperiodic task generator from button interrupt */
void EXTI0_IRQHandler(void) {
	// Ensure interrupt bit is set
	if(EXTI_GetITStatus(EXTI_Line0) != RESET) {
		// clear interrupt flag
		EXTI_ClearITPendingBit(EXTI_Line0);
		// Send notification to aperiodic generator task
		vTaskNotifyGiveFromISR(aperiodicGeneratorHandle, NULL);
	}
}

/* ---------------------------------- MAIN -------------------------------------- */

int main() {
	// Initialize queues
	createQueues();
	// Setup lights, user input on board
	initBoard();

	// Create tasks
	xTaskCreate(ddSchedulerTask, "DD_SCHEDULER_TASK", configMINIMAL_STACK_SIZE, NULL, 4, NULL);
	xTaskCreate(ddTask1GeneratorTask, "TASK1_GEN_TASK", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
	xTaskCreate(ddTask2GeneratorTask, "TASK2_GEN_TASK", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
	xTaskCreate(ddTask3GeneratorTask, "TASK3_GEN_TASK", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
	xTaskCreate(ddAperiodicTaskGeneratorTask, "AP_G_TASK", configMINIMAL_STACK_SIZE, NULL, 3, &aperiodicGeneratorHandle);
	xTaskCreate(monitorTask, "MON_TASK", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

	/* Start the tasks running. */
	vTaskStartScheduler();

    return 0;
}


/*---------------------------- Hook functions included in demo file ------------------------------*/

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
