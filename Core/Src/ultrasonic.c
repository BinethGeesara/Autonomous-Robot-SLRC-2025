#include "ultrasonic.h"
#include "Timer_Delay.h"

extern TIM_HandleTypeDef htim5;

static uint32_t timeout_us = 50000;  // 10 ms timeout in microseconds
static uint32_t elapsed_time = 0;

double kalman(double U) {
    static const double R = 40;       // Measurement noise covariance
    static const double H = 1.00;     // Observation matrix
    static double Q = 10;             // Process noise covariance
    static double P = 0;              // Estimation error covariance
    static double U_hat = 0;          // Estimated state
    static double K = 0;              // Kalman gain

    K = P * H / (H * P * H + R);     // Calculate Kalman gain
    U_hat += K * (U - H * U_hat);     // Update estimate
    P = (1 - K * H) * P + Q;         // Update estimation error covariance

    return U_hat;                     // Return filtered value
}

void Ultrasonic_Init(void) {
    HAL_TIM_Base_Start(&htim5);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
}


uint32_t Ultrasonic_GetDistance(uint8_t sensor) {
    uint32_t Value1 = 0, Value2 = 0, Distance = 0;
    uint16_t echo_pin;

    switch (sensor) {
        case 0:
            echo_pin = GPIO_PIN_2;
            break;
        case 1:
            echo_pin = GPIO_PIN_12;
            break;
        case 2:
            echo_pin = GPIO_PIN_13;
            break;
        default:
            return 0;
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);  // Pull the TRIG pin HIGH
    delay_us(10);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    
    // Wait for the echo pin to go HIGH
    elapsed_time = 0;
    while (!(HAL_GPIO_ReadPin(GPIOB, echo_pin)) && (elapsed_time < timeout_us)) {
        delay_us(1);
        elapsed_time++;
    }
    Value1 = __HAL_TIM_GET_COUNTER(&htim5);

    // Wait for the echo pin to go LOW
    elapsed_time = 0;
    while ((HAL_GPIO_ReadPin(GPIOB, echo_pin)) && (elapsed_time < timeout_us)) {
        delay_us(1);
        elapsed_time++;
    }
    Value2 = __HAL_TIM_GET_COUNTER(&htim5);
    Distance = (Value2 - Value1) * 0.34 / 2; // Calculate distance in cm
    double filtered_distance = kalman((double)Distance);

    return (uint32_t)filtered_distance; // Return the filtered distance
    return Distance;
}