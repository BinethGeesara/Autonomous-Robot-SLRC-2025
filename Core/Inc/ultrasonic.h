#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "main.h"

// Function prototypes
void Ultrasonic_Init(void);
uint32_t Ultrasonic_GetDistance(uint8_t sensor);

#endif // ULTRASONIC_H