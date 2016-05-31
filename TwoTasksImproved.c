// File: TwoTasks.c 

#include <stdio.h>
#include "includes.h"
#include <string.h>

#define DEBUG 1

/* Definition of Task Stacks */
/* Stack grows from HIGH to LOW memory */
#define   TASK_STACKSIZE       2048
OS_STK    task1_stk[TASK_STACKSIZE];
OS_STK    task2_stk[TASK_STACKSIZE];
OS_STK    stat_stk[TASK_STACKSIZE];

/* Definition of Task Priorities */
#define TASK1_PRIORITY      6  // highest priority
#define TASK2_PRIORITY      7
#define TASK_STAT_PRIORITY 12  // lowest priority 

OS_EVENT *Semaphore1;
OS_EVENT *Semaphore2;
OS_EVENT *Semaphore3;

void printStackSize(INT8U prio)
{
    INT8U err;
    OS_STK_DATA stk_data;
    
    err = OSTaskStkChk(prio, &stk_data);
    if (err == OS_NO_ERR) 
    {
        if (DEBUG == 1)
           printf("Task Priority %d - Used: %d; Free: %d\n", 
                   prio, stk_data.OSFree, stk_data.OSUsed);
    }
    else
    {
        if (DEBUG == 1)
           printf("Stack Check Error!\n");    
    }
}

/* Prints a message and sleeps for given time interval */
void task1(void* pdata)
{
    INT8U err;
    while (1)
    { 
        char text1[] = "Hello from Task1\n";
        int i;
        OSSemPend (Semaphore1, 0, &err);
        for (i = 0; i < strlen(text1); i++)
            putchar(text1[i]);
            
        if (OSSemPost (Semaphore2) != OS_ERR_NONE)
        {
            printf ("Semaphore not signaled\n");
        }
        OSTimeDlyHMSM(0, 0, 0, 11); // Context Switch to next task

                                    // Task will go to the ready state

                                    // after the specified delay
    }
}

/* Prints a message and sleeps for given time interval */
void task2(void* pdata)
{
    INT8U err;      // Hold an error code for semaphore
    while (1)
    { 
        char text2[] = "Hello from Task2\n";
        int i;
        OSSemPend (Semaphore2, 0, &err);        // Trying to access the key
        for (i = 0; i < strlen(text2); i++)
            putchar(text2[i]);
            
        if (OSSemPost (Semaphore3) != OS_ERR_NONE)      // Release the key
        {
            printf ("Semaphore not signaled\n");
        }     
        OSTimeDlyHMSM(0, 0, 0, 4);
    }
}

/* Printing Statistics */
void statisticTask(void* pdata)
{
    INT8U err;      // Hold an error code for semaphore
    while(1)
    {
        OSSemPend (Semaphore3, 0, &err);
        printStackSize(TASK1_PRIORITY);
        printStackSize(TASK2_PRIORITY);
        printStackSize(TASK_STAT_PRIORITY);
        OSSemPost (Semaphore1);
    }
}

/* The main function creates two task and starts multi-tasking */
int main(void)
{
    printf("Lab 3 - Two Tasks\n");
  
    Semaphore1 = OSSemCreate(1);      // Binary semaphore
    Semaphore2 = OSSemCreate(0);
    Semaphore3 = OSSemCreate(0);
  
    if (Semaphore1 == NULL)
    {
        printf("Event Control Block not available\n");
    }
  

    OSTaskCreateExt
        (task1,                        // Pointer to task code
        NULL,                         // Pointer to argument that is passed to task
                                  
        &task1_stk[TASK_STACKSIZE-1], // Pointer to top of task stack
        TASK1_PRIORITY,               // Desired Task priority
        TASK1_PRIORITY,               // Task ID
        &task1_stk[0],                // Pointer to bottom of task stack
        TASK_STACKSIZE,               // Stacksize
        NULL,                         // Pointer to user supplied memory
                                   // (not needed here)
        OS_TASK_OPT_STK_CHK |         // Stack Checking enabled 
        OS_TASK_OPT_STK_CLR           // Stack Cleared                                 
        );
               
    OSTaskCreateExt
        (task2,                        // Pointer to task code
        NULL,                         // Pointer to argument that is passed to task
                                   
        &task2_stk[TASK_STACKSIZE-1], // Pointer to top of task stack
        TASK2_PRIORITY,               // Desired Task priority
        TASK2_PRIORITY,               // Task ID
        &task2_stk[0],                // Pointer to bottom of task stack
        TASK_STACKSIZE,               // Stacksize
        NULL,                         // Pointer to user supplied memory
                                   // (not needed here)
        OS_TASK_OPT_STK_CHK |         // Stack Checking enabled 
        OS_TASK_OPT_STK_CLR           // Stack Cleared                       
        );  
    
    if (DEBUG == 1)
    {
        OSTaskCreateExt
            (statisticTask,                // Pointer to task code
            NULL,                         // Pointer to argument that is passed to task
                                     
            &stat_stk[TASK_STACKSIZE-1],  // Pointer to top of task stack
            TASK_STAT_PRIORITY,           // Desired Task priority
            TASK_STAT_PRIORITY,           // Task ID
            &stat_stk[0],                 // Pointer to bottom of task stack
            TASK_STACKSIZE,               // Stacksize
            NULL,                         // Pointer to user supplied memory
                                     // (not needed here)
            OS_TASK_OPT_STK_CHK |         // Stack Checking enabled 
            OS_TASK_OPT_STK_CLR           // Stack Cleared                              
        );
    }  
    
    OSStart();
    return 0;
}
