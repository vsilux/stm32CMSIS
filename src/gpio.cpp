#include "gpio.hpp"
#include "stm32f4xx.h"

using namespace AirD;

Status GPIOPort::enable()
{
    SET_BIT(RCC->AHB1ENR, clockMask); /* Delay after an RCC peripheral clock enabling */
    if ((READ_BIT(RCC->AHB1ENR, clockMask) & clockMask) != 0)
    {
        isPortEnabled = true;
        return ok;
    }

    return error;
}

Status GPIOPort::disable()
{
    CLEAR_BIT(RCC->AHB1ENR, clockMask);
    if ((READ_BIT(RCC->AHB1ENR, clockMask) & clockMask) == 0)
    {
        isPortEnabled = false;
        return ok;
    }
    return error;
}

GPIOPin *GPIOPort::getPin(uint32_t pinNumber)
{
    GPIOPin *pin = pins[pinNumber];
    if (pin == nullptr)
    {
        pin = new GPIOPin(port, pinNumber);
        pins[pinNumber] = pin;
    }
    return pin;
}

/*** GPIOPin */

void GPIOPin::writeValue(bool isHigh)
{
    uint32_t pin = ((uint16_t)(1U << pinNumber));
    if (isHigh)
    {
        port->BSRR = pin;
    }
    else
    {
        port->BSRR = (uint32_t)pin << 16U;
    }
}

bool GPIOPin::readValue()
{
    volatile uint16_t pin = ((uint16_t)(1U << pinNumber));
    volatile uint16_t value = (port->IDR & pin);
    return value != 0;
}

void GPIOPin::configureDirectionMode(GPIOMode mode)
{
    this->mode = mode;
    uint32_t configuration = port->MODER;
    configuration &= ~(GPIO_MODER_MODE0 << (pinNumber * 2U));
    configuration |= ((mode & GPIOMode::gpio) << (pinNumber * 2U));
    port->MODER = configuration;
}

void GPIOPin::configurePullMode(PullMode pullMode)
{
    this->pullMode = pullMode;
    uint32_t configuration = port->PUPDR;
    configuration &= ~(GPIO_PUPDR_PUPDR0 << (pinNumber * 2U));
    configuration |= ((pullMode) << (pinNumber * 2U));
    port->PUPDR = configuration;
}

Status GPIOPin::configureAnalog()
{
    mode = GPIOMode::analog;
    uint32_t configuration = port->MODER;
    configuration &= ~(GPIO_MODER_MODE0 << (pinNumber * 2U));
    configuration |= ((mode & GPIOMode::gpio) << (pinNumber * 2U));
    port->MODER = configuration;
    return ok;
}

Status GPIOPin::configureDigitalInput(PullMode pullMode)
{
    configurePullMode(pullMode);
    configureDirectionMode(input);
    return ok;
}

Status GPIOPin::configureDigitalOutput(OutputType outputType, Speed speed, PullMode pullMode)
{
    /* Configure the IO Speed */
    uint32_t configuration = port->OSPEEDR;
    configuration &= ~(GPIO_OSPEEDR_OSPEED0 << (pinNumber * 2U));
    configuration |= (speed << (pinNumber * 2U));
    port->OSPEEDR = configuration;

    /* Configure the IO Output Type */
    configuration = port->OTYPER;
    configuration &= ~(GPIO_OTYPER_OT0 << pinNumber);
    configuration |= (((outputType & OutputType::outputType) >> 4U) << pinNumber);
    port->OTYPER = configuration;

    configureDirectionMode(output);

    return ok;
}

Status GPIOPin::alternativeFunction(Status (*configurationFunction)(uint32_t pin, GPIO_TypeDef *prot)) { return ok; }