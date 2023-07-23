#include "stm32g431xx.h"
#include "FreeRTOS.h"
#include "task.h"

#include "core_cm4.h"

// uCSpecific
#include "ucSpecific/Public/hal.h"

// Used modules
#include "Modules/LCD/Public/lcd.h"

static inline void enterLPR(void)
{
    __ucHAL_SysFreq_function_enterSleepState();
}

static inline void exitLPR(void)
{
    __ucHAL_SysFreq_function_exitSleepState();
}

void vPortSuppressTicksAndSleep(TickType_t xExpectedIdleTime)
{
    static uint8_t functionRunForTheFirstTime = 1;
    if (functionRunForTheFirstTime)
    {
        __ucHAL_Sleeptimer_configure();
        functionRunForTheFirstTime = 0;
    }

    // Formula used to determine ARR value:
    //  #xExpectedIdleTime * (1 second/#Ticks) * (1 timer_clock_cycle/time_in_seconds)

    // Formula used to determine elapsed ticks
    //  timCounter * (time_in_seconds/1 timer_clock_cycle) * (#ticks/1second)

    eSleepModeStatus eSleepStatus;

    __ucHAL_RtosTimer_function_pause();
    __ucHAL_Interrupts_function_disableInterrupts();

    eSleepStatus = eTaskConfirmSleepModeStatus();

    if (eSleepStatus == eAbortSleep)
    {
        __ucHAL_RtosTimer_function_resume();
        __ucHAL_Interrupts_function_reenableInterrupts();
    }
    else
    {
        lcdPutsBlinkFreeAtPos("SLEEP", 4);

        if (eSleepStatus == eNoTasksWaitingTimeout)
        {
            enterLPR();
            __ucHAL_Sleeptimer_startOneShot(0, 99);
            __ucHAL_Interrupts_function_WaitForInterrupt();
        }
        else
        {
            enterLPR();
            __ucHAL_Sleeptimer_startOneShot(0, (xExpectedIdleTime * (1000 / configTICK_RATE_HZ)) / 51);
            __ucHAL_Interrupts_function_WaitForInterrupt();
        }

        SleepTimerValues sleepTimer = __ucHAL_Sleeptimer_stopOneShot();
        exitLPR();

        lcdPutsBlinkFreeAtPos("", 4);

        (sleepTimer.updateInterruptOcurred &&
         (sleepTimer.counter == 0))
            ? vTaskStepTick(xExpectedIdleTime)
            : vTaskStepTick((sleepTimer.counter * 51 * configTICK_RATE_HZ) / 1000);

        __ucHAL_Interrupts_function_reenableInterrupts();
        __ucHAL_RtosTimer_function_resume();
    }
}
