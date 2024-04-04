#pragma once

#include "stm32f4xx.h"
#include "types_and_data.hpp"

namespace AirD
{
    class GPIOPin;

    class GPIOPort
    {
    public:
        enum ClockMask : uint32_t
        {
            PortA = 0x1UL << 0U,
            PortC = 0x1UL << 2U,
            PortD = 0x1UL << 3U,
            PortH = 0x1UL << 7U,
        };

    private:
        GPIO_TypeDef *port;
        ClockMask clockMask;
        bool isPortEnabled;
        GPIOPin *pins[16] = {nullptr};

    public:
        GPIOPort(GPIO_TypeDef *port, ClockMask clockMask) : port(port), clockMask(clockMask) {}
        Status enable();
        Status disable();
        GPIOPin *getPin(uint32_t pinNumber);
    };

    static GPIOPort *const GPIOPortA = new GPIOPort(GPIOA, GPIOPort::ClockMask::PortA);
    static GPIOPort *const GPIOPortD = new GPIOPort(GPIOD, GPIOPort::ClockMask::PortD);

    class GPIOPin
    {
    private:
        friend GPIOPort;

    public:
        enum PullMode : uint32_t
        {
            none = 0U,
            up = 1U,
            down = 2U
        };

        enum GPIOMode : uint32_t
        {
            input = 0x0UL,
            gpio = 0x3UL,
            output = 0x1UL,
            AF = 0x2UL,
            analog = 0x3UL,
        };

        enum OutputType : uint32_t
        {
            position = 4U,
            outputType = 0x1UL << 4U,
            pushPull = 0x0UL << 4U,
            openDrain = 0x1UL << 4U
        };

        enum Speed : uint32_t
        {
            low = 0U,
            medium = 1U,
            high = 2U,
            veryHigh = 3U
        };

    private:
        GPIO_TypeDef *port;
        uint32_t pinNumber;
        PullMode pullMode;
        GPIOMode mode;

        GPIOPin(GPIO_TypeDef *port, uint32_t pinNumber) : port(port), pinNumber(pinNumber) {}
        ~GPIOPin() {}
        void configureDirectionMode(GPIOMode mode);
        void configurePullMode(PullMode pullMode);

    public:
        Status configureAnalog();
        Status configureDigitalInput(PullMode pullMode);
        Status configureDigitalOutput(OutputType outputType, Speed speed, PullMode pullMode);
        Status alternativeFunction(Status (*configurationFunction)(uint32_t pin, GPIO_TypeDef *prot));
        void writeValue(bool isHigh);
        bool readValue();
    };
}
