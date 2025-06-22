#ifndef IR_PINS_H
#define IR_PINS_H

#include "stm32f4xx_hal.h"

// Declare the GPIO pins as extern
extern GPIO_TypeDef *Right_IR_GPIO_Port;
extern uint16_t Right_IR_Pin;

extern GPIO_TypeDef *Left_IR_GPIO_Port;
extern uint16_t Left_IR_Pin;

#endif // IR_PINS_H