
#include "sys_timer.hpp"
#include "stm32f4xx.h"

using namespace AirD;

__IO uint32_t tick;
uint32_t tickPriority = (1UL << __NVIC_PRIO_BITS); /* Invalid PRIO */
TickInterval tickInterval = ms1;

Status SysTimer::init(uint32_t priority)
{
    /* Configure the SysTimer to have interrupt in 1ms time basis*/
    if (SysTick_Config(SystemCoreClock / (1000U / tickInterval)) > 0U)
    {
        return error;
    }

    uint32_t prioritygroup = NVIC_GetPriorityGrouping();

    /* Configure the SysTimer IRQ priority */
    if (priority < (1UL << __NVIC_PRIO_BITS))
    {
        NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(prioritygroup, priority, 0U));
        tickPriority = priority;
    }
    else
    {
        return error;
    }

    /* Return function status */
    return ok;
}

void SysTimer::incrementTick()
{
    tick += tickInterval;
}

uint32_t SysTimer::getTick()
{
    return tick;
}

uint32_t SysTimer::getTickNVICPriority()
{
    return tickPriority;
}

Status SysTimer::setTickInterval(TickInterval interval)
{
    if (tickInterval == interval)
    {
        return ok;
    }

    TickInterval prevousInterval = tickInterval;
    tickInterval = interval;

    if (init(tickPriority) != ok)
    {
        tickInterval = prevousInterval;
        init(tickPriority);
        return error;
    }

    return ok;
}

TickInterval SysTimer::getTickInterval()
{
    return tickInterval;
}

// blocking delay
void SysTimer::delay(uint32_t timeMs)
{
    uint32_t tickstart = getTick();
    while ((getTick() - tickstart) <= timeMs)
    {
    }
}

void SysTimer::suspendTick()
{
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
}

void SysTimer::resumeTick()
{
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
}