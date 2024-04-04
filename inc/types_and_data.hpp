#pragma once
#include <cstdint>

namespace AirD
{
    enum Status
    {
        ok,
        error,
        busy,
        timeout
    };

    enum TickInterval
    {
        ms1 = 1U,
        ms10 = 10U,
        ms100 = 100U,
    };
} // namespace AirD