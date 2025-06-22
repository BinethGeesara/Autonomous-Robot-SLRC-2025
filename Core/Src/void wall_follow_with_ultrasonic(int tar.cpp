void wall_follow_with_ultrasonic(int target_distance, int base_speed, float Kp, float Ki, float Kd)
{
    int last_error = 0;                       // Previous error for derivative calculation
    float integral = 0;                       // Integral term
    const int min_speed = 55;                 // Minimum motor speed to overcome friction
    const int max_speed = 80;                 // Maximum motor speed
    const int ultrasonic_check_interval = 20; // Check every 20ms (50Hz)
    uint32_t last_ultrasonic_check = 0;

    while (1)
    {
        // Get the current distance from the wall
        uint32_t current_time = HAL_GetTick();
        if ((current_time - last_ultrasonic_check) >= ultrasonic_check_interval)
        {
            last_ultrasonic_check = current_time;
            int current_distance = Ultrasonic_GetDistance(0);

            // Calculate the error
            errorp = target_distance - current_distance;

            integral += errorp;
            if (integral > 1000)
                integral = 1000;
            if (integral < -1000)
                integral = -1000;

            // Calculate the derivative term
            int derivative = errorp - last_error;

            // Calculate the PID correction
            int correction = (int)(Kp * errorp + Ki * integral + Kd * derivative);

            // Adjust motor speeds
            int left_speed = base_speed + correction;
            int right_speed = base_speed - correction;

            // Ensure motor speeds are within bounds
            if (left_speed > max_speed)
                left_speed = max_speed;
            if (left_speed < min_speed)
                left_speed = min_speed;
            if (right_speed > max_speed)
                right_speed = max_speed;
            if (right_speed < min_speed)
                right_speed = min_speed;

            // Set motor speeds
            setMotorSpeedL(right_speed);
            setMotorSpeedR(left_speed);

            // Update last error
            last_error = errorp;
        }
        delay_ms(1);
    }
    setMotorSpeedL(0);
    setMotorSpeedR(0);
}
