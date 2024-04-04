#pragma once

#include "types_and_data.hpp"
#include <cstdint>

namespace AirD
{
    class SysTimer
    {
    private:
        uint32_t tickPriority = (1UL << __NVIC_PRIO_BITS); /* Invalid PRIO */
        SysTimer(){};
        ~SysTimer(){};

    public:
        static Status init(uint32_t tickPriority);

        static void incrementTick();

        static uint32_t getTick();

        static uint32_t getTickNVICPriority();

        static Status setTickInterval(TickInterval interval);

        static TickInterval getTickInterval();

        static void suspendTick();

        static void resumeTick();

        // blocking delay
        static void delay(uint32_t timeMs);
    };
} // namespace AirD
