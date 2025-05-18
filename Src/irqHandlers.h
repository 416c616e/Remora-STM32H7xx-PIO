#ifndef IRQ_HANDLERS_H
#define IRQ_HANDLERS_H

#include "remora/interrupt/interrupt.h"
#include "stm32h7xx_hal.h"

extern "C" {

    void EXTI15_10_IRQHandler() {
        if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_10) != RESET) {
            __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_10);
            Interrupt::InvokeHandler(EXTI15_10_IRQn);
        }
    }

    void DMA1_Stream0_IRQHandler() {
        Interrupt::InvokeHandler(DMA1_Stream0_IRQn);
    }

    void DMA1_Stream1_IRQHandler() {
        Interrupt::InvokeHandler(DMA1_Stream1_IRQn);
    }

    void TIM2_IRQHandler() {
        if (TIM2->SR & TIM_SR_UIF) {
            TIM2->SR &= ~TIM_SR_UIF;
            Interrupt::InvokeHandler(TIM2_IRQn);
        }
    }

    void TIM3_IRQHandler() {
        if (TIM3->SR & TIM_SR_UIF) {
            TIM3->SR &= ~TIM_SR_UIF;
            Interrupt::InvokeHandler(TIM3_IRQn);
        }
    }

    void TIM4_IRQHandler() {
        if (TIM4->SR & TIM_SR_UIF) {
            TIM4->SR &= ~TIM_SR_UIF;
            Interrupt::InvokeHandler(TIM4_IRQn);
        }
    }

} // extern "C"

#endif // IRQ_HANDLERS_H
