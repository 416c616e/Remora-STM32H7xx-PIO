#include "pin.h"
#include <cstdio>

Pin::Pin(const std::string& portAndPin, int dir) 
    : portAndPin(portAndPin), dir(dir), modifier(NONE) {
    configurePin();
    enableClock();
    initialisePin();
}

Pin::Pin(const std::string& portAndPin, int dir, int modifier) 
    : portAndPin(portAndPin), dir(dir), modifier(modifier) {
    configurePin();
    enableClock();
    initialisePin();
}

void Pin::configurePin() {
    GPIO_TypeDef* gpioPorts[8] = {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH};
    
    if (portAndPin[0] == 'P') {
        portIndex = portAndPin[1] - 'A';
        pinNumber = std::stoi(portAndPin.substr(3));
        pin = 1 << pinNumber;
    } else {
        printf("Invalid port and pin definition\n\r");
        return;
    }
    
    GPIOx = gpioPorts[portIndex];
    
    mode = (dir == INPUT) ? GPIO_MODE_INPUT : GPIO_MODE_OUTPUT_PP;
    pull = (modifier == PULLUP) ? GPIO_PULLUP :
           (modifier == PULLDOWN) ? GPIO_PULLDOWN :
           GPIO_NOPULL;
}

void Pin::enableClock() {
    switch (portIndex) {
        case 0: __HAL_RCC_GPIOA_CLK_ENABLE(); break;
        case 1: __HAL_RCC_GPIOB_CLK_ENABLE(); break;
        case 2: __HAL_RCC_GPIOC_CLK_ENABLE(); break;
        case 3: __HAL_RCC_GPIOD_CLK_ENABLE(); break;
        case 4: __HAL_RCC_GPIOE_CLK_ENABLE(); break;
        case 5: __HAL_RCC_GPIOF_CLK_ENABLE(); break;
        case 6: __HAL_RCC_GPIOG_CLK_ENABLE(); break;
        case 7: __HAL_RCC_GPIOH_CLK_ENABLE(); break;
    }
}

void Pin::initialisePin() {
    HAL_GPIO_WritePin(GPIOx, pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = mode;
    GPIO_InitStruct.Pull = pull;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

bool Pin::get() const {
    return HAL_GPIO_ReadPin(GPIOx, pin);
}

void Pin::set(bool value) {
    HAL_GPIO_WritePin(GPIOx, pin, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Pin::setAsOutput() {
    mode = GPIO_MODE_OUTPUT_PP;
    pull = GPIO_NOPULL;
    initialisePin();
}

void Pin::setAsInput() {
    mode = GPIO_MODE_INPUT;
    pull = GPIO_NOPULL;
    initialisePin();
}

void Pin::setPullNone() {
    pull = GPIO_NOPULL;
    initialisePin();
}

void Pin::setPullUp() {
    pull = GPIO_PULLUP;
    initialisePin();
}

void Pin::setPullDown() {
    pull = GPIO_PULLDOWN;
    initialisePin();
}
