
#include "cmsis_os2.h"
#include <stdio.h>

static osSemaphoreId_t mySemaphore;      // 信号量
static osTimerId_t myTimer;              // 定时器
static volatile uint32_t timerTickCount = 0;  // 定时器触发计数

// 等待信号量的任务
void waitTask(void *arg) 
{
    while (1) 
	{
        osSemaphoreAcquire(mySemaphore, osWaitForever);  // 等待信号量
        printf("WaitTask: Got semaphore!\n");
        osDelay(1000);  // 模拟处理延时
    }
}

// 释放信号量的任务
void signalTask(void *arg) 
{
    uint32_t count = 0;
    while (1) 
	{
        osDelay(2000);                     // 每2秒释放一次
        osSemaphoreRelease(mySemaphore);
        printf("SignalTask: Released semaphore (%lu)\n", ++count);
    }
}

// 初始化信号量并创建任务
void test_semaphore_init(void) 
{
    mySemaphore = osSemaphoreNew(1, 0, NULL);   // 二进制信号量，初始0
    if (mySemaphore == NULL) 
	{
        printf("Failed to create semaphore\n");
        return;
    }
    osThreadNew(waitTask, NULL, NULL);
    osThreadNew(signalTask, NULL, NULL);
}

// 定时器回调函数
void timerCallback(void *arg) 
{
    timerTickCount++;
    printf("Timer callback: %lu times\n", timerTickCount);
}

// 初始化定时器并启动
void test_timer_init(void) 
{
    myTimer = osTimerNew(timerCallback, osTimerPeriodic, NULL, NULL);
    if (myTimer == NULL) 
	{
        printf("Failed to create timer\n");
        return;
    }
    osTimerStart(myTimer, 1000U);   // 1秒周期
}