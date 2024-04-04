#pragma once

#include "types_and_data.hpp"

namespace AirD
{
    class RccController
    {
    public:
        enum OscType
        {
            HSI,
            HSE,
            LSI,
            LSE
        };

    private:
        /* data */
    public:
        RccController(){};
        ~RccController(){};

        Status configurateOsc();
        Status configurateClock();
    };
} // namespace AirD
