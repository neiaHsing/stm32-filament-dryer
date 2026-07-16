//
// Created by Neia Hsing on 3/21/25.
//
#include "Relay.h"

Relay::Relay(GPIO_TypeDef* gpio_port, uint16_t gpio_pin, bool activeHigh)
    : gpio_port(gpio_port), gpio_pin(gpio_pin), activeHigh(activeHigh) {}

void Relay::SetState(bool state) {
    const GPIO_PinState pinState = (state == activeHigh) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(gpio_port, gpio_pin, pinState);
}

bool Relay::GetState() {
    const bool pinHigh = HAL_GPIO_ReadPin(gpio_port, gpio_pin) == GPIO_PIN_SET;
    return pinHigh == activeHigh;
}
