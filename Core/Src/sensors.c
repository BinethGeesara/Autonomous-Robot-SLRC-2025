#include "sensors.h"
#include "main.h"

extern int junction_detected;
uint16_t ADC_VAL[10];       // Raw ADC values (DMA buffer)
uint16_t thresholds[10] = {600, 600, 1000, 1000, 1000, 1000, 1000, 400, 400 , 1500};
uint8_t IR[10]; // Array to store 1/0 for each sensor
//{1500, 2500, 1800, 2500, 2500, 2500, 1700, 600, 600};
uint8_t get_sensor_state(uint16_t adc_value, uint16_t threshold) {
  if (adc_value > threshold) {
    return 1; // Sensor detects a line (or white surface)
  } else {
    return 0; // Sensor detects background (or black surface)
  }
}

void process_sensors(void) {
  for (int i = 0; i < 10; i++) {
    IR[i] = get_sensor_state(ADC_VAL[i], thresholds[i]);
  }
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1) {
  if (hadc1->Instance == ADC1) {
      process_sensors();
      if (( IR[1] == 1 || IR[2] == 1 || IR[3] == 1 || IR[4] == 1 || IR[5] == 1 ) && (IR[7] == 1 && IR[8] == 1)) {
        junction_detected = 1;
      } else {
        junction_detected = 0;
      }
  }
}
