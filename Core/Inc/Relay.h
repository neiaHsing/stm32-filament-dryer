//
// Created by Neia Hsing on 3/21/25.
//

#ifndef RELAY_H
#define RELAY_H

#include "stm32f1xx_hal.h"

class Relay {
public:
    Relay(GPIO_TypeDef* gpio_port, uint16_t gpio_pin, bool activeHigh = true);
    void SetState(bool state);
    bool GetState();
private:
    GPIO_TypeDef* gpio_port;
    uint16_t gpio_pin;
    bool activeHigh;
};

#endif // RELAY_H
