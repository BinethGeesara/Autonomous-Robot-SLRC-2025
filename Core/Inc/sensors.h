#ifndef SENSORS_H
#define SENSORS_H

#include "stm32f4xx_hal.h" // Adjust this include according to your STM32 series

extern uint16_t ADC_VAL[10];       // Raw ADC values (DMA buffer)
extern uint16_t thresholds[10];
extern uint8_t IR[10]; // Array to store 1/0 for each sensor

uint8_t get_sensor_state(uint16_t adc_value, uint16_t threshold);
void process_sensors(void);

#endif // SENSORS_H