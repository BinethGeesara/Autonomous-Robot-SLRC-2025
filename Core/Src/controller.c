#include "controller.h"
#include "motor_control.h"
#include "sensors.h"
#include "encoders.h"
#include "Timer_Delay.h"
#include "ultrasonic.h"
#include <stdlib.h>
#include <limits.h>
#include <math.h>

// Constants
#define TICKS_PER_DEGREE 9.6 // Derived from calculations

int attempts = 0;
int teta_3 = 0;

extern int junction_count;
extern int difference_;
extern int enc_dist;
int base_pwm = 85;
int base_pwm_fast = 130;
float Kp1 = 1; // Proportional gain (tune this value)
float Ki1 = 0.001;
float Kp_fast = 2; // Proportional gain (tune this value)
float Ki_fast = 0.002;
int integral_limit = 5000; // Prevent windup
float correction;
int difference_ = 0;
int derivative = 0;
float integral = 0;
int enc_dist = 0;
int cylinder_detected = 0;
int cylinder_Distance = 0;
int frontwall_detected = 0;
int frontwall_Distance = 0;
int sidewall_Distance = 0;
int wall = 0;
int errorp = 0; // Current error

int min_sidewall_Distance = INT_MAX;
extern int wall;
extern int sidewall_Distance;
extern int min_sidewall_Distance;
extern int frontwall_Distance;
extern int frontwall_detected;
extern int cylinder_Distance;
/////
float_t angleDISTmm = 0;          // Variable to store the angle in degrees
int averageEncoderDifference = 0; // Variable to store the average difference between left and right encoders
int x = 0;                        // Variable to store the x-coordinate
// extern int averageEncoderDifference;
double teta = 0; // Variable to store the angle in radians
///
extern int leftIRTriggered;  // Declare the variable as extern
extern int rightIRTriggered; // Declare the variable as extern
////

#define LINE_THRESHOLD 500 // Adjust based on your IR sensor values
#define MIN_WORKING_PWM 40 // Minimum speed that keeps motors moving

void enc_drive_accel(int target_distance, const int maxSpeed, const int minSpeed)
{
    resetEncoders();

    const int accelSteps = target_distance / 4; // Distance for acceleration
    int currentSpeed = minSpeed;                // Start at minimum speed

    while (1)
    {
        // Read encoder counts
        int left_count = getLeftEncoderCounts();
        int right_count = getRightEncoderCounts();

        // Compute the distance (average of both wheels)
        int enc_dist = (left_count + right_count) / 2;

        // Break the loop if the target distance is reached
        if (enc_dist >= target_distance)
        {
            break;
        }

        // Acceleration phase
        if (enc_dist < accelSteps)
        {
            currentSpeed = minSpeed + (maxSpeed - minSpeed) * enc_dist / accelSteps;
        }
        else
        {
            currentSpeed = maxSpeed; // Maintain max speed after acceleration
        }

        // Calculate the error (difference in encoder counts)
        int difference_ = left_count - right_count;

        // Integral term with anti-windup
        integral += difference_;
        if (integral > integral_limit)
            integral = integral_limit;
        if (integral < -integral_limit)
            integral = -integral_limit;

        // Compute correction using PI controller
        int correction = (difference_ * Kp1) + (integral * Ki1);

        // Adjust motor speeds with correction
        int speedL = currentSpeed + correction;
        int speedR = currentSpeed - correction;

        // Ensure minimum speed is maintained
        if (abs(speedL) < minSpeed)
            speedL = speedL < 0 ? -minSpeed : minSpeed;
        if (abs(speedR) < minSpeed)
            speedR = speedR < 0 ? -minSpeed : minSpeed;

        setMotorSpeedL(speedL);
        setMotorSpeedR(speedR);
    }

    // Stop the motors after reaching the target
    setMotorSpeedR(0);
    setMotorSpeedL(0);
    resetEncoders();
}
void enc_drive_decel(int target_distance, const int maxSpeed, const int minSpeed)
{
    resetEncoders();

    const int decelStart = target_distance - (target_distance / 1.8); // Start decelerating early

    int currentSpeed = maxSpeed; // Start at max speed

    while (1)
    {
        // Read encoder counts
        int left_count = getLeftEncoderCounts();
        int right_count = getRightEncoderCounts();

        // Compute the distance (average of both wheels)
        int enc_dist = (left_count + right_count) / 2;

        // Break the loop if the target distance is reached
        if (enc_dist >= target_distance)
        {
            break;
        }

        // Deceleration phase
        if (enc_dist > decelStart)
        {
            int remaining = target_distance - enc_dist;
            if (remaining > 0)
            {
                currentSpeed = minSpeed + (maxSpeed - minSpeed) * remaining / (target_distance - decelStart);
            }
            else
            {
                currentSpeed = 0;
            }
        }

        // Calculate the error (difference in encoder counts)
        int difference_ = left_count - right_count;

        // Integral term with anti-windup
        integral += difference_;
        if (integral > integral_limit)
            integral = integral_limit;
        if (integral < -integral_limit)
            integral = -integral_limit;

        // Compute correction using PI controller
        int correction = (difference_ * Kp1) + (integral * Ki1);

        // Adjust motor speeds with correction
        int speedL = currentSpeed + correction;
        int speedR = currentSpeed - correction;

        // Ensure minimum speed is maintained
        if (abs(speedL) < minSpeed)
            speedL = speedL < 0 ? -minSpeed : minSpeed;
        if (abs(speedR) < minSpeed)
            speedR = speedR < 0 ? -minSpeed : minSpeed;

        setMotorSpeedL(speedL);
        setMotorSpeedR(speedR);
    }

    // Stop the motors after reaching the target
    setMotorSpeedR(0);
    setMotorSpeedL(0);
    resetEncoders();
}
void enc_driveR_decel(int target_distance, const int maxSpeed, const int minSpeed)
{
    resetEncoders();

    const int decelStart = target_distance - (target_distance / 1.8); // Start decelerating early
    int currentSpeed = maxSpeed;                                      // Start at max speed

    while (1)
    {
        // Read encoder counts (will be negative in reverse)
        int left_count = getLeftEncoderCounts();
        int right_count = getRightEncoderCounts();

        // Compute the distance (average of absolute values)
        int enc_dist = (abs(left_count) + abs(right_count)) / 2;

        // Break the loop if the target distance is reached
        if (enc_dist >= target_distance)
        {
            break;
        }

        // Deceleration phase
        if (enc_dist > decelStart)
        {
            int remaining = target_distance - enc_dist;
            if (remaining > 0)
            {
                currentSpeed = minSpeed + (maxSpeed - minSpeed) * remaining / (target_distance - decelStart);
            }
            else
            {
                currentSpeed = 0;
            }
        }

        // Calculate the error (difference in encoder counts)
        int difference_ = left_count - right_count;

        // Integral term with anti-windup
        integral += difference_;
        if (integral > integral_limit)
            integral = integral_limit;
        if (integral < -integral_limit)
            integral = -integral_limit;

        // Compute correction using PI controller
        int correction = (difference_ * Kp1) + (integral * Ki1);

        // Adjust motor speeds with correction (negative for reverse)
        int speedL = -(currentSpeed + correction);
        int speedR = -(currentSpeed - correction);

        // Ensure minimum speed is maintained
        if (abs(speedL) < minSpeed)
            speedL = speedL < 0 ? -minSpeed : minSpeed;
        if (abs(speedR) < minSpeed)
            speedR = speedR < 0 ? -minSpeed : minSpeed;

        setMotorSpeedL(speedL);
        setMotorSpeedR(speedR);
    }

    // Stop the motors after reaching the target
    setMotorSpeedL(0);
    setMotorSpeedR(0);
    resetEncoders();
}
void enc_drive_T(int target_distance, const int maxSpeed, const int minSpeed)
{
    resetEncoders();

    const int accelSteps = target_distance / 4;                       // Acceleration distance
    const int decelStart = target_distance - (target_distance / 1.9); // When to start decelerating

    int currentSpeed = minSpeed; // Starting speed

    while (1)
    {
        // Read encoder counts
        int left_count = getLeftEncoderCounts();
        int right_count = getRightEncoderCounts();

        // Compute the distance (average of both wheels)
        int enc_dist = (left_count + right_count) / 2;

        // Break the loop if the target distance is reached
        if (enc_dist >= target_distance)
        {
            break;
        }

        // Acceleration phase
        if (enc_dist < accelSteps)
        {
            currentSpeed = minSpeed + (maxSpeed - minSpeed) * enc_dist / accelSteps;
        }
        // Deceleration phase
        else if (enc_dist > decelStart)
        {
            int remaining = target_distance - enc_dist;
            if (remaining > 0)
            {
                currentSpeed = minSpeed + (maxSpeed - minSpeed) * remaining / (target_distance - decelStart);
            }
            else
            {
                currentSpeed = 0;
            }
        }
        // Constant speed phase
        else
        {
            currentSpeed = maxSpeed;
        }

        // Calculate the error (difference in encoder counts)
        int difference_ = left_count - right_count;

        // Integral term with anti-windup
        integral += difference_;
        if (integral > integral_limit)
            integral = integral_limit;
        if (integral < -integral_limit)
            integral = -integral_limit;

        // Compute correction using PI controller
        int correction = (difference_ * Kp1) + (integral * Ki1);

        // Adjust motor speeds with correction
        int speedL = currentSpeed + correction;
        int speedR = currentSpeed - correction;

        // Ensure minimum speed is maintained
        if (abs(speedL) < minSpeed)
            speedL = speedL < 0 ? -minSpeed : minSpeed;
        if (abs(speedR) < minSpeed)
            speedR = speedR < 0 ? -minSpeed : minSpeed;

        setMotorSpeedL(speedL);
        setMotorSpeedR(speedR);
    }

    // Stop the motors after reaching the target
    setMotorSpeedR(0);
    setMotorSpeedL(0);
    resetEncoders();
}
void enc_drive()
{
    // Read encoder counts
    int left_count = getLeftEncoderCounts();
    int right_count = getRightEncoderCounts();

    // Compute the distance (average of both wheels)
    enc_dist = (left_count + right_count) / 2;

    // Calculate the error (difference in encoder counts)
    difference_ = left_count - right_count;

    // Integral term with anti-windup
    integral += difference_;
    if (integral > integral_limit)
        integral = integral_limit;
    if (integral < -integral_limit)
        integral = -integral_limit;

    // Compute correction using PI controller
    correction = (difference_ * Kp1) + (integral * Ki1);

    // Adjust motor speeds
    setMotorSpeedR(base_pwm - correction);
    setMotorSpeedL(base_pwm + correction);
}
void enc_drive_fast()
{
    // Read encoder counts
    int left_count = getLeftEncoderCounts();
    int right_count = getRightEncoderCounts();

    // Compute the distance (average of both wheels)
    enc_dist = (left_count + right_count) / 2;

    // Calculate the error (difference in encoder counts)
    difference_ = left_count - right_count;

    // Integral term with anti-windup
    integral += difference_;
    if (integral > integral_limit)
        integral = integral_limit;
    if (integral < -integral_limit)
        integral = -integral_limit;

    // Compute correction using PI controller
    correction = (difference_ * Kp_fast) + (integral * Ki_fast);

    // Adjust motor speeds
    setMotorSpeedR(base_pwm_fast - correction);
    setMotorSpeedL(base_pwm_fast + correction);
}
void enc_drive2(int speed)
{
    // Read encoder counts
    int left_count = getLeftEncoderCounts();
    int right_count = getRightEncoderCounts();

    // Compute the distance (average of both wheels)
    enc_dist = (left_count + right_count) / 2;

    // Calculate the error (difference in encoder counts)
    difference_ = left_count - right_count;

    // Integral term with anti-windup
    integral += difference_;
    if (integral > integral_limit)
        integral = integral_limit;
    if (integral < -integral_limit)
        integral = -integral_limit;

    // Compute correction using PI controller
    correction = (difference_ * 1.3) + (integral * 0.0013);

    // Adjust motor speeds
    setMotorSpeedR(speed - correction);
    setMotorSpeedL(speed + correction);
}
void enc_driveR(int speed)
{
    int left_count = getLeftEncoderCounts();
    int right_count = getRightEncoderCounts();
    // Compute the distance (average of both wheels)
    enc_dist = (left_count + right_count) / 2;
    // Calculate the error (difference in encoder counts)
    difference_ = left_count - right_count;
    // Integral term with anti-windup
    integral += difference_;
    if (integral > integral_limit)
        integral = integral_limit;
    if (integral < -integral_limit)
        integral = -integral_limit;
    // Compute correction using PI controller
    correction = (difference_ * Kp1) + (integral * Ki1);
    setMotorSpeedL(-speed - 15 + correction);
    setMotorSpeedR(-speed - correction);
}
void enc_driveR_T(int target_distance, const int maxSpeed, const int minSpeed)
{
    resetEncoders();

    const int accelSteps = target_distance / 4;                       // Acceleration distance
    const int decelStart = target_distance - (target_distance / 1.9); // When to start decelerating

    int currentSpeed = minSpeed; // Starting speed

    while (1)
    {
        // Read encoder counts
        int left_count1 = getLeftEncoderCounts();
        int right_count1 = getRightEncoderCounts();
        int left_count = -left_count1;
        int right_count = -right_count1;
        // Compute the distance (average of both wheels)
        int enc_dist = (abs(left_count) + abs(right_count)) / 2;

        // Break the loop if the target distance is reached
        if (enc_dist >= target_distance)
        {
            break;
        }

        // Acceleration phase
        if (enc_dist < accelSteps)
        {
            currentSpeed = minSpeed + (maxSpeed - minSpeed) * enc_dist / accelSteps;
        }
        // Deceleration phase
        else if (enc_dist > decelStart)
        {
            int remaining = target_distance - enc_dist;
            if (remaining > 0)
            {
                currentSpeed = minSpeed + (maxSpeed - minSpeed) * remaining / (target_distance - decelStart);
            }
            else
            {
                currentSpeed = 0;
            }
        }
        // Constant speed phase
        else
        {
            currentSpeed = maxSpeed;
        }

        // Calculate the error (difference in encoder counts)
        int difference_ = left_count - right_count;

        // Integral term with anti-windup
        integral += difference_;
        if (integral > integral_limit)
            integral = integral_limit;
        if (integral < -integral_limit)
            integral = -integral_limit;

        // Compute correction using PI controller
        int correction = (difference_ * Kp1) + (integral * Ki1);

        // Adjust motor speeds with correction (negative for reverse)
        int speedL = (currentSpeed + correction);
        int speedR = (currentSpeed - correction);

        // Ensure minimum speed is maintained
        if (abs(speedL) < minSpeed)
            speedL = speedL < 0 ? -minSpeed : minSpeed;
        if (abs(speedR) < minSpeed)
            speedR = speedR < 0 ? -minSpeed : minSpeed;

        setMotorSpeedL(-speedL);
        setMotorSpeedR(-speedR);
    }

    // Stop the motors after reaching the target
    setMotorSpeedL(0);
    setMotorSpeedR(0);
    resetEncoders();
}
void enc_drive_fast_to_target(int target_distance, const int maxSpeed, const int minSpeed)
{
    resetEncoders();

    const int decelStart = target_distance - (target_distance / 2); // Start decelerating at 2/3 of the target distance
    int currentSpeed = maxSpeed;                                    // Start at maximum speed

    while (1)
    {
        // Read encoder counts
        int left_count = getLeftEncoderCounts();
        int right_count = getRightEncoderCounts();

        // Compute the distance (average of both wheels)
        int enc_dist = (left_count + right_count) / 2;

        // Break the loop if the target distance is reached
        if (enc_dist >= target_distance)
        {
            break;
        }

        // Deceleration phase
        if (enc_dist > decelStart)
        {
            int remaining = target_distance - enc_dist;
            if (remaining > 0)
            {
                currentSpeed = minSpeed + (maxSpeed - minSpeed) * remaining / (target_distance - decelStart);
            }
            else
            {
                currentSpeed = minSpeed;
            }
        }

        // Calculate the error (difference in encoder counts)
        int difference_ = left_count - right_count;

        // Integral term with anti-windup
        integral += difference_;
        if (integral > integral_limit)
            integral = integral_limit;
        if (integral < -integral_limit)
            integral = -integral_limit;

        // Compute correction using PI controller
        int correction = (difference_ * Kp_fast) + (integral * Ki_fast);

        // Adjust motor speeds with correction
        int speedL = currentSpeed + correction;
        int speedR = currentSpeed - correction;

        // Ensure minimum speed is maintained
        if (abs(speedL) < minSpeed)
            speedL = speedL < 0 ? -minSpeed : minSpeed;
        if (abs(speedR) < minSpeed)
            speedR = speedR < 0 ? -minSpeed : minSpeed;

        setMotorSpeedL(speedL);
        setMotorSpeedR(speedR);
    }

    // Stop the motors after reaching the target
    setMotorSpeedL(0);
    setMotorSpeedR(0);
    resetEncoders();
}
/// @brief ////////////////////////////////
void align_to_line()
{
    // Variables to store encoder counts
    static int stripeStartLeftCount = 0; // Left encoder count at the start of the stripe
    static int stripeStartRightCount = 0;
    double MPI = 3.14159265358979323846;
    int maxAttempts = 3;           // Maximum alignment attempts to prevent infinite loops
    enc_drive();                   // Call the enc_drive function to adjust motor speeds
    while (attempts < maxAttempts) // Limit the number of alignment attempts
    {

        int currentLeftCount = getLeftEncoderCounts();
        int currentRightCount = getRightEncoderCounts();
        int happen = 1;
        enc_drive(); // Call the enc_drive function to adjust motor speeds

        if (leftIRTriggered == 1 && happen == 1)
        {
            leftIRTriggered = 1;
            stripeStartLeftCount = currentLeftCount;
            stripeStartRightCount = getRightEncoderCounts();
            // Start driving until the right IR sensor is triggered
            while (rightIRTriggered == 0)
            {
                enc_drive();
            }
            setMotorSpeedL(0);
            setMotorSpeedR(0);
            delay_ms(100);
            // Get encoder counts at the end
            int stripeEndLeftCount = currentLeftCount;
            int stripeEndRightCount = getRightEncoderCounts();

            // Calculate the encoder counts for the stripe
            int leftStripeCount = stripeEndLeftCount - stripeStartLeftCount;
            int rightStripeCount = stripeEndRightCount - stripeStartRightCount;
            // Calculate the average stripe width
            averageEncoderDifference = (leftStripeCount + rightStripeCount) / 2;
            teta = ((atan2(averageEncoderDifference * 10 / 31, 133)) * 180 / MPI);

            // Exit the function if teta < 3
            if (teta <= 3 && teta != 0)
            {
                teta_3 = 1;
                return;
            }
            {
                teta_3 = 1;
                return;
            }

            // If theta is still > 3, perform correction
            resetEncoders(); // Reset encoders before moving
            while (1)
            {
                int currentRightCount = getRightEncoderCounts();
                int currentLeftCount = getLeftEncoderCounts();
                x = (teta * TICKS_PER_DEGREE);
                // Check if the right wheel has moved the required distance
                if (currentLeftCount <= -x)
                {
                    break;
                }
                setMotorSpeedR(-75); // Keep the left wheel stationary
            }

            setMotorSpeedR(0);
            setMotorSpeedL(0); // Stop the left wheel as well
            delay_ms(100);     // Optional delay for stabilization

            // Reverse and move forward to correct again
            enc_driveR_T(320, 100, 60); // Adjust speed as needed

            resetEncoders();
            attempts++; // Increment attempt counter
        }
        else if (rightIRTriggered == 1 && happen == 1)
        {
            rightIRTriggered = 0;
            stripeStartRightCount = currentRightCount;
            stripeStartLeftCount = getLeftEncoderCounts();
            // Start driving until the right IR sensor is triggered
            while (leftIRTriggered == 0)
            {
                enc_drive();
            }
            setMotorSpeedL(0);
            setMotorSpeedR(0);
            delay_ms(100); // Optional delay for stabilization
            int stripeEndRightCount = currentRightCount;
            int stripeEndLeftCount = getLeftEncoderCounts();
            int leftStripeCount = stripeEndLeftCount - stripeStartLeftCount;
            int rightStripeCount = stripeEndRightCount - stripeStartRightCount;
            averageEncoderDifference = (leftStripeCount + rightStripeCount) / 2;
            teta = ((atan2(averageEncoderDifference * 10 / 31, 133)) * 180 / MPI);

            // Exit the function if teta < 3
            if (teta < 3)
            {
                teta_3 = 1;
                return;
            }

            // If theta is still > 3, perform correction
            resetEncoders(); // Reset encoders before moving
            while (1)
            {
                int currentRightCount = getRightEncoderCounts();
                int currentLeftCount = getLeftEncoderCounts();
                x = (teta * TICKS_PER_DEGREE);
                // Check if the right wheel has moved the required distance
                if (currentRightCount <= -x)
                {
                    break;
                }
                setMotorSpeedL(-75); // Adjust speed as needed
            }
            setMotorSpeedR(0);
            setMotorSpeedL(0);          // Stop the left wheel as well
            delay_ms(200);              // Optional delay for stabilization
            enc_driveR_T(320, 100, 60); // Adjust speed as needed

            resetEncoders();
            attempts++; // Increment attempt counter
        }
        else
        {
            enc_drive(); // Continue driving if no IR sensor is triggered
        }
    }

    // If we reach here, we've exceeded max attempts without getting theta < 3
    // You might want to handle this case (e.g., stop motors, signal error, etc.)
    setMotorSpeedL(0);
    setMotorSpeedR(0);
}

void turn_Left_align()
{ // target
    resetEncoders();

    // Ramp parameters - adjust these for your robot
    const int maxSpeed = 70; // Maximum motor speed
    const int minSpeed = 60; // Minimum speed to overcome friction
    const int target = teta * TICKS_PER_DEGREE;
    const int accelSteps = target / 2.5;        // Counts to accelerate
    const int decelStart = target - accelSteps; // When to start decelerating

    int currentSpeed = minSpeed; // Starting speed

    while (1)
    {
        int left_counts = abs(getLeftEncoderCounts());
        int right_counts = abs(getRightEncoderCounts());
        int avg_counts = (left_counts + right_counts) / 2;

        // Acceleration phase
        if (avg_counts < accelSteps)
        {
            currentSpeed = minSpeed + (maxSpeed - minSpeed) * avg_counts / accelSteps;
        }
        // Deceleration phase
        else if (avg_counts > decelStart)
        {
            int remaining = target - avg_counts;
            if (remaining > 0)
            {
                currentSpeed = minSpeed + (maxSpeed - minSpeed) * remaining / (target - decelStart);
            }
            else
            {
                currentSpeed = 0;
            }
        }
        // Constant speed phase
        else
        {
            currentSpeed = maxSpeed;
        }

        // Balance correction
        int error = left_counts - right_counts;
        int correction = error * 1;

        // Apply speeds with minimum guaranteed
        int speedL = currentSpeed + correction;
        int speedR = -(currentSpeed - correction);

        // Ensure minimum speed is maintained
        if (abs(speedL) < minSpeed)
            speedL = speedL < 0 ? -minSpeed : minSpeed;
        if (abs(speedR) < minSpeed)
            speedR = speedR < 0 ? -minSpeed : minSpeed;

        setMotorSpeedL(speedL);
        setMotorSpeedR(speedR);

        // Exit condition
        if (left_counts >= target && right_counts >= target)
        {
            break;
        }
    }

    m_stopLR();
    delay_ms(8880);
    resetEncoders();
}
/////////////////////////////////////////////////

/////////////////////////////////////////////////
void line_follow_to_target(int target_distance, int maxSpeed, int minSpeed)
{
    resetEncoders();

    const int accelSteps = target_distance / 3;                     // Distance for acceleration
    const int decelStart = target_distance - (target_distance / 2); // Start deceleration

    int currentSpeed = minSpeed; // Initial speed
    static int last_error = 0;   // Previous error for derivative calculation

    while (1)
    {
        // Read encoder counts
        int left_count = getLeftEncoderCounts();
        int right_count = getRightEncoderCounts();

        // Compute the distance (average of both wheels)
        int enc_dist = (left_count + right_count) / 2;

        // Break the loop if the target distance is reached
        if (enc_dist >= target_distance)
        {
            break;
        }

        // Acceleration phase
        if (enc_dist < accelSteps)
        {
            currentSpeed = minSpeed + (maxSpeed - minSpeed) * enc_dist / accelSteps;
        }
        // Deceleration phase
        else if (enc_dist > decelStart)
        {
            int remaining = target_distance - enc_dist;
            if (remaining > 0)
            {
                currentSpeed = minSpeed + (maxSpeed - minSpeed) * remaining / (target_distance - decelStart);
            }
            else
            {
                currentSpeed = 0;
            }
        }
        // Constant speed phase
        else
        {
            currentSpeed = maxSpeed;
        }

        // Line-following logic
        int error = (IR[0] * -38.1) +
                    (IR[1] * -25.4) +
                    (IR[2] * -10.16) +
                    (IR[3] * 0) +
                    (IR[4] * 10.16) +
                    (IR[5] * 25.4) +
                    (IR[6] * 38.1);

        int derivative = error - last_error;

        // PD control constants
        const float Kp = 0.7; // Proportional gain
        const float Kd = 0.3; // Derivative gain

        // Calculate control output
        int control_output = (int)(Kp * error + Kd * derivative);

        // Adjust motor speeds based on control output
        int left_speed = currentSpeed - control_output;
        int right_speed = currentSpeed + control_output;

        // Ensure minimum PWM for motors
        if (left_speed < MIN_WORKING_PWM)
            left_speed = MIN_WORKING_PWM;
        if (right_speed < MIN_WORKING_PWM)
            right_speed = MIN_WORKING_PWM;

        // Set motor speeds
        setMotorSpeedL(left_speed);
        setMotorSpeedR(right_speed);

        // Update last error
        last_error = error;
    }

    // Stop the motors after reaching the target
    setMotorSpeedL(0);
    setMotorSpeedR(0);
    resetEncoders();
}
void line_follow()
{
    static int last_error = 0; // Previous error value for derivative calculation
    int error = 0;             // Error value for line deviation
                               // Derivative of the error

    // Calculate weighted error
    error = (IR[0] * -38.1) +
            (IR[1] * -25.4) +
            (IR[2] * -10.16) +
            (IR[3] * 0) +
            (IR[4] * 10.16) +
            (IR[5] * 25.4) +
            (IR[6] * 38.1);

    // Calculate derivative
    derivative = error - last_error;

    // PD control constants
    const float Kp = 0.7; // Proportional gain
    const float Kd = 0.3;
    // Calculate control output
    int control_output = (int)(Kp * error + Kd * derivative);

    // Adjust motor speeds based on control output
    int left_speed = BASE_SPEED - control_output;
    int right_speed = BASE_SPEED + control_output;

    // Ensure minimum PWM for motors
    if (left_speed < MIN_WORKING_PWM)
        left_speed = MIN_WORKING_PWM;
    if (right_speed < MIN_WORKING_PWM)
        right_speed = MIN_WORKING_PWM;

    // Set motor speeds
    setMotorSpeedL(left_speed);
    setMotorSpeedR(right_speed);

    // Update last error
    last_error = error;
}
void line_follow_Last_Column()
{
    static int last_error = 0; // Previous error value for derivative calculation
    int error = 0;             // Error value for line deviation
                               // Derivative of the error

    // Calculate weighted error
    error = (IR[2] * -10.16) +
            (IR[3] * 0) +
            (IR[4] * 10.16);

    // Calculate derivative
    derivative = error - last_error;

    // PD control constants
    const float Kp = 0.7; // Proportional gain
    const float Kd = 0.7;
    // Calculate control output
    int control_output = (int)(Kp * error + Kd * derivative);

    // Adjust motor speeds based on control output
    int left_speed = BASE_SPEED - control_output;
    int right_speed = BASE_SPEED + control_output;

    // Ensure minimum PWM for motors
    if (left_speed < MIN_WORKING_PWM)
        left_speed = MIN_WORKING_PWM;
    if (right_speed < MIN_WORKING_PWM)
        right_speed = MIN_WORKING_PWM;

    // Set motor speeds
    setMotorSpeedL(left_speed);
    setMotorSpeedR(right_speed);

    // Update last error
    last_error = error;
}
void line_follow_NoJunctions()
{
    static int last_error = 0; // Previous error value for derivative calculation
    int error = 0;             // Error value for line deviation
                               // Derivative of the error

    // Calculate weighted error
    error = (IR[0] * -38.1) +
            (IR[1] * -25.4) +
            (IR[2] * -10.16) +
            (IR[3] * 0) +
            (IR[4] * 10.16) +
            (IR[5] * 25.4) +
            (IR[6] * 38.1);

    // Calculate derivative
    derivative = error - last_error;

    // PD control constants
    const float Kp = 0.7; // Proportional gain
    const float Kd = 0.3;
    // Calculate control output
    if (IR[3] == 1 && IR[6] == 1)
    {
        error = 0;
        derivative = 0;
    }
    else if (IR[2] == 1 && IR[3] == 1 && IR[4] == 1 && IR[5] == 1 && IR[6] == 1)
    {
        error = 0;
        derivative = 0;
    }
    else if (IR[3] == 1 && IR[5] == 1 && IR[6] == 1)
    {
        error = 0;
        derivative = 0;
    }

    int control_output = (int)(Kp * error + Kd * derivative);

    // Adjust motor speeds based on control output
    int left_speed = BASE_SPEED - control_output;
    int right_speed = BASE_SPEED + control_output;

    // Ensure minimum PWM for motors
    if (left_speed < MIN_WORKING_PWM)
        left_speed = MIN_WORKING_PWM;
    if (right_speed < MIN_WORKING_PWM)
        right_speed = MIN_WORKING_PWM;

    // Set motor speeds
    setMotorSpeedL(left_speed);
    setMotorSpeedR(right_speed);

    // Update last error
    last_error = error;
}
////////////////////////////////////////////
void line_follow_with_ultrasonic_check(const uint32_t duration_ms)
{
    uint32_t start_time = HAL_GetTick();
    const int ultrasonic_check_interval = 20; // Check every 20ms (50Hz)
    uint32_t last_ultrasonic_check = 0;

    static int last_error = 0;

    while ((HAL_GetTick() - start_time) < duration_ms && !cylinder_detected)
    {
        // Line following logic
        int error = (IR[0] * -38.1) +
                    (IR[1] * -25.4) +
                    (IR[2] * -10.16) +
                    (IR[3] * 0) +
                    (IR[4] * 10.16) +
                    (IR[5] * 25.4) +
                    (IR[6] * 38.1);

        int derivative = error - last_error;

        // PD control constants
        const float Kp = 0.7;
        const float Kd = 0.3;

        // Calculate control output
        int control_output = (int)(Kp * error + Kd * derivative);

        // Adjust motor speeds
        int left_speed = BASE_SPEED - control_output;
        int right_speed = BASE_SPEED + control_output;

        // Ensure minimum PWM
        if (left_speed < MIN_WORKING_PWM)
            left_speed = MIN_WORKING_PWM;
        if (right_speed < MIN_WORKING_PWM)
            right_speed = MIN_WORKING_PWM;

        setMotorSpeedL(left_speed);
        setMotorSpeedR(right_speed);
        last_error = error;

        // Check ultrasonic periodically
        uint32_t current_time = HAL_GetTick();
        if ((current_time - last_ultrasonic_check) >= ultrasonic_check_interval)
        {
            last_ultrasonic_check = current_time;
            cylinder_Distance = Ultrasonic_GetDistance(0);

            if (cylinder_Distance < 120)
            {
                cylinder_detected = 1;
                break;
            }

            // Small delay to prevent overwhelming the system
            delay_ms(1);
        }
    }
}
void enc_drive_with_ultrasonic_check()
{
    resetEncoders();

    static int last_error = 0; // Previous error value for encoder correction
    integral = 0;              // Reset integral term

    const int ultrasonic_check_interval = 20; // Check every 20ms (50Hz)
    uint32_t last_ultrasonic_check = 0;
    int minSpeed = 60;  // Minimum speed to overcome friction
    int baseSpeed = 75; // Base speed for motors
    while (1)
    {
        // Read encoder counts
        int left_count = getLeftEncoderCounts();
        int right_count = getRightEncoderCounts();

        // Compute the distance (average of both wheels)
        enc_dist = (left_count + right_count) / 2;

        // Calculate the error (difference in encoder counts)
        difference_ = left_count - right_count;

        // Integral term with anti-windup
        integral += difference_;
        if (integral > integral_limit)
            integral = integral_limit;
        if (integral < -integral_limit)
            integral = -integral_limit;

        // Compute correction using PI controller
        correction = (difference_ * Kp1) + (integral * Ki1);

        // Adjust motor speeds
        int speedL = baseSpeed + correction;
        int speedR = baseSpeed - correction;

        // Ensure minimum speed is maintained
        if (abs(speedL) < minSpeed)
            speedL = speedL < 0 ? -minSpeed : minSpeed;
        if (abs(speedR) < minSpeed)
            speedR = speedR < 0 ? -minSpeed : minSpeed;

        setMotorSpeedL(speedL);
        setMotorSpeedR(speedR);

        // Ultrasonic distance check
        uint32_t current_time = HAL_GetTick();
        if ((current_time - last_ultrasonic_check) >= ultrasonic_check_interval)
        {
            last_ultrasonic_check = current_time;
            frontwall_Distance = Ultrasonic_GetDistance(1);

            if (frontwall_Distance < 110)
            {
                frontwall_detected = 1;
                break;
            }

            // Small delay to prevent overwhelming the system
            delay_ms(1);
        }
    }

    // Stop the motors after the condition is met
    setMotorSpeedL(0);
    setMotorSpeedR(0);
    resetEncoders();
}
void enc_drive_with_ultrasonic_check_85(int target, int sensor)
{
    resetEncoders();

    static int last_error = 0; // Previous error value for encoder correction
    integral = 0;              // Reset integral term

    const int ultrasonic_check_interval = 20; // Check every 20ms (50Hz)
    uint32_t last_ultrasonic_check = 0;
    int minSpeed = 60;  // Minimum speed to overcome friction
    int baseSpeed = 85; // Base speed for motors
    while (1)
    {
        // Read encoder counts
        int left_count = getLeftEncoderCounts();
        int right_count = getRightEncoderCounts();

        // Compute the distance (average of both wheels)
        enc_dist = (left_count + right_count) / 2;

        // Calculate the error (difference in encoder counts)
        difference_ = left_count - right_count;

        // Integral term with anti-windup
        integral += difference_;
        if (integral > integral_limit)
            integral = integral_limit;
        if (integral < -integral_limit)
            integral = -integral_limit;

        // Compute correction using PI controller
        correction = (difference_ * Kp1) + (integral * Ki1);

        // Adjust motor speeds
        int speedL = baseSpeed + correction;
        int speedR = baseSpeed - correction;

        // Ensure minimum speed is maintained
        if (abs(speedL) < minSpeed)
            speedL = speedL < 0 ? -minSpeed : minSpeed;
        if (abs(speedR) < minSpeed)
            speedR = speedR < 0 ? -minSpeed : minSpeed;

        setMotorSpeedL(speedL);
        setMotorSpeedR(speedR);

        // Ultrasonic distance check
        uint32_t current_time = HAL_GetTick();
        if ((current_time - last_ultrasonic_check) >= ultrasonic_check_interval)
        {
            last_ultrasonic_check = current_time;
            frontwall_Distance = Ultrasonic_GetDistance(sensor);

            if (frontwall_Distance < target)
            {
                wall = 1;
                break;
            }

            // Small delay to prevent overwhelming the system
            delay_ms(1);
        }
    }

    // Stop the motors after the condition is met
    setMotorSpeedL(0);
    setMotorSpeedR(0);
    resetEncoders();
}
void enc_drive_with_ultrasonic_Target(const int target_encoder_counts)
{
    resetEncoders();

    static int last_error = 0; // Previous error value for encoder correction
    integral = 0;              // Reset integral term

    const int ultrasonic_check_interval = 20; // Check every 20ms (50Hz)
    uint32_t last_ultrasonic_check = 0;
    int minSpeed = 60;  // Minimum speed to overcome friction
    int baseSpeed = 75; // Base speed for motors
    sidewall_Distance = 0;
    min_sidewall_Distance = INT_MAX; // Variable to store the minimum sidewall distance

    while (1)
    {
        // Read encoder counts
        int left_count = getLeftEncoderCounts();
        int right_count = getRightEncoderCounts();

        // Compute the distance (average of both wheels)
        enc_dist = (left_count + right_count) / 2;

        // Stop if the target encoder counts are reached
        if (enc_dist >= target_encoder_counts)
        {
            break;
        }

        // Calculate the error (difference in encoder counts)
        difference_ = left_count - right_count;

        // Integral term with anti-windup
        integral += difference_;
        if (integral > integral_limit)
            integral = integral_limit;
        if (integral < -integral_limit)
            integral = -integral_limit;

        // Compute correction using PI controller
        correction = (difference_ * Kp1) + (integral * Ki1);

        // Adjust motor speeds
        int speedL = baseSpeed + correction ;
        int speedR = baseSpeed - correction ;

        // Ensure minimum speed is maintained
        if (abs(speedL) < minSpeed)
            speedL = speedL < 0 ? -minSpeed : minSpeed;
        if (abs(speedR) < minSpeed)
            speedR = speedR < 0 ? -minSpeed : minSpeed;

        setMotorSpeedL(speedL);
        setMotorSpeedR(speedR);

        // Ultrasonic distance check
        uint32_t current_time = HAL_GetTick();
        if ((current_time - last_ultrasonic_check) >= ultrasonic_check_interval)
        {
            last_ultrasonic_check = current_time;
            sidewall_Distance = Ultrasonic_GetDistance(2);

            // Update the minimum sidewall distance
            if (sidewall_Distance < min_sidewall_Distance && sidewall_Distance > 0)
            {
                min_sidewall_Distance = sidewall_Distance;
            }
        }
    }

    // Stop the motors after the condition is met
    setMotorSpeedL(0);
    setMotorSpeedR(0);
    resetEncoders();
}
/*
Normal Wall following
 */
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
        if (IR[7] = 1 && IR[8] == 1)
        {
            break; // Exit the function if both IR sensors are triggered
        }
    }
}
/*
 Exits from the function at the minuimum error position
 */
void wall_follow_with_ultrasonic2(int target_distance, int base_speed, float Kp, float Ki, float Kd)
{
    int last_error = 0;                       // Previous error for derivative calculation
    float integral = 0;                       // Integral term
    const int min_speed = 55;                 // Minimum motor speed to overcome friction
    const int max_speed = 80;                 // Maximum motor speed
    const int ultrasonic_check_interval = 20; // Check every 20ms (50Hz)
    uint32_t last_ultrasonic_check = 0;

    uint32_t error_start_time = 0; // Timer to track error duration
    int error_within_range = 0;    // Flag to track if error is within range

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

            // Check if error is within the range of 10 to -15
            if (errorp >= -11 && errorp <= 0)
            {
                if (!error_within_range)
                {
                    error_within_range = 1;
                    error_start_time = current_time;
                }
                else if ((current_time - error_start_time) >= 500) // 2 seconds
                {
                    break; // Exit the function
                }
            }
            else
            {
                error_within_range = 0; // Reset the flag if error goes out of range
            }

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
/*
 Exits from the function for enc counts
 */
void wall_follow_with_ultrasonic_encoders(int target_distance, int base_speed, float Kp, float Ki, float Kd, int target_encoder_counts)
{
    int last_error = 0;                       // Previous error for derivative calculation
    float integral = 0;                       // Integral term
    const int min_speed = 55;                 // Minimum motor speed to overcome friction
    const int max_speed = 80;                 // Maximum motor speed
    const int ultrasonic_check_interval = 20; // Check every 20ms (50Hz)
    uint32_t last_ultrasonic_check = 0;

    resetEncoders(); // Reset encoders before starting

    while (1)
    {
        // Get the current encoder counts
        int left_count = getLeftEncoderCounts();
        int right_count = getRightEncoderCounts();
        int avg_encoder_counts = (left_count + right_count) / 2;

        // Exit the loop if the target encoder count is reached
        if (avg_encoder_counts >= target_encoder_counts)
        {
            break;
        }

        // Get the current distance from the wall
        uint32_t current_time = HAL_GetTick();
        if ((current_time - last_ultrasonic_check) >= ultrasonic_check_interval)
        {
            last_ultrasonic_check = current_time;
            int current_distance = Ultrasonic_GetDistance(0);

            // Calculate the error
            int errorp = target_distance - current_distance;

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

        delay_ms(1); // Small delay to prevent overwhelming the system
    }

    // Stop the motors after reaching the target encoder count
    setMotorSpeedL(0);
    setMotorSpeedR(0);
    resetEncoders();
}
/// @brief /////////////////////////
void turn_90_degreesR(int targetCounts)
{ // 790
    // Reset encoders
    resetEncoders();

    while (1)
    {
        int left_counts = abs(getLeftEncoderCounts());
        int right_counts = abs(getRightEncoderCounts());

        // Compute error
        int error = left_counts - right_counts;

        // Adjust motor speeds dynamically to keep counts balanced
        int baseSpeed = 85;
        int correction = error * 1; // Simple proportional control

        setMotorSpeedR(baseSpeed - correction);
        setMotorSpeedL(-(baseSpeed + correction));

        // Check if both wheels reached the target
        if (left_counts >= targetCounts && right_counts >= targetCounts)
        {
            break;
        }
    }

    m_stopLR();
    delay_ms(50);
    resetEncoders();
}
void turn_90_degreesR_TS2()
{
    // Reset encoders
    resetEncoders();

    // Set motor speeds for turning (e.g., left motor forward, right motor backward)
    setMotorSpeedL(-90);
    setMotorSpeedR(90);

    while (1)
    {
        int current_left_counts = getLeftEncoderCounts();
        int current_right_counts = getRightEncoderCounts();

        // Check if the turn is complete
        if (current_left_counts >= 730 && current_right_counts <= -730)
        {
            break;
        }
    }

    // Stop the motors
    setMotorSpeedL(0);
    setMotorSpeedR(0);
    resetEncoders();
}
void turn_90_degreesR_After_PadColor()
{
    // Reset encoders
    resetEncoders();

    // Set motor speeds for turning (e.g., left motor forward, right motor backward)
    setMotorSpeedL(-90);
    setMotorSpeedR(90);

    while (1)
    {
        int current_left_counts = getLeftEncoderCounts();
        int current_right_counts = getRightEncoderCounts();

        // Check if the turn is complete
        if (current_left_counts >= 747 && current_right_counts <= -747)
        {
            break;
        }
    }

    // Stop the motors
    setMotorSpeedL(0);
    setMotorSpeedR(0);
    resetEncoders();
}
void turn_90_degreesR1_reverse(int targetCounts)
{
    // Reset encoders
    resetEncoders();

    // Ramp parameters
    const int maxSpeed = 95;                   // Maximum motor speed
    const int minSpeed = 70;                   // Minimum speed to overcome friction
    const int accelSteps = 400;                // Counts to accelerate
    const int decelStart = targetCounts - 900; // When to start decelerating

    int currentSpeed = minSpeed; // Starting speed

    while (1)
    {
        int current_right_counts = abs(getRightEncoderCounts());

        // Acceleration phase
        if (current_right_counts < accelSteps)
        {
            currentSpeed = minSpeed + (maxSpeed - minSpeed) * current_right_counts / accelSteps;
        }
        // Deceleration phase
        else if (current_right_counts > decelStart)
        {
            int remaining = targetCounts - current_right_counts;
            if (remaining > 0)
            {
                currentSpeed = minSpeed + (maxSpeed - minSpeed) * remaining / (targetCounts - decelStart);
            }
            else
            {
                currentSpeed = 0;
            }
        }
        // Constant speed phase
        else
        {
            currentSpeed = maxSpeed;
        }

        // Set motor speeds
        setMotorSpeedL(-currentSpeed); // Left motor reverse
        setMotorSpeedR(0);             // Right motor stationary

        // Exit condition
        if (current_right_counts >= targetCounts)
        {
            break;
        }
    }

    // Stop the motors
    setMotorSpeedL(0);
    setMotorSpeedR(0);
    resetEncoders();
}
void turn_90_degreesR1_reverse_FOR_PAD()
{
    // Reset encoders
    resetEncoders();
    // Set motor speeds for turning (e.g., left motor forward, right motor backward)
    setMotorSpeedL(-95);
    setMotorSpeedR(0);

    while (1)
    {
        int current_right_counts = getRightEncoderCounts();

        // Check if the turn is complete
        if (current_right_counts <= -1700)
        {
            break;
        }
    }

    // Stop the motors
    setMotorSpeedL(0);
    setMotorSpeedR(0);
    resetEncoders();
}
void turn_90_degreesL_fromLastColumn()
{
    // Reset encoders
    resetEncoders();

    // Set motor speeds for turning (e.g., left motor forward, right motor backward)
    setMotorSpeedL(90);
    setMotorSpeedR(-90);

    while (1)
    {
        int current_left_counts = getLeftEncoderCounts();
        int current_right_counts = getRightEncoderCounts();

        // Check if the turn is complete
        if (current_left_counts <= -790 && current_right_counts >= 790)
        {
            break;
        }
    }

    // Stop the motors
    setMotorSpeedL(0);
    setMotorSpeedR(0);
    delay_ms(50);
    resetEncoders();
}
void turn_90_degreesL1_reverse_FROM_PAD()
{
    // Reset encoders
    resetEncoders();
    // Set motor speeds for turning (e.g., left motor forward, right motor backward)
    setMotorSpeedL(85);
    setMotorSpeedR(0);

    while (1)
    {
        int current_right_counts = getRightEncoderCounts();

        // Check if the turn is complete
        if (current_right_counts >= 1680)
        {
            break;
        }
    }

    // Stop the motors
    setMotorSpeedL(0);
    setMotorSpeedR(0);
    resetEncoders();
}
void turn_20_degrees_L()
{
    resetEncoders();
    setMotorSpeedL(80);

    while (1)
    {

        int current_right_counts = getRightEncoderCounts();

        // Check if the turn is complete
        if (current_right_counts >= 255)
        {
            break;
        }
    }
    setMotorSpeedL(0);
    setMotorSpeedR(0);
    delay_ms(50);
    resetEncoders();
}
void turn_20_degrees_R()
{
    resetEncoders();
    setMotorSpeedL(-80);

    while (1)
    {

        int current_right_counts = getRightEncoderCounts();

        // Check if the turn is complete
        if (current_right_counts <= -300)
        {
            break;
        }
    }
    setMotorSpeedL(0);
    setMotorSpeedR(0);
    delay_ms(50);
    resetEncoders();
}
/// ControlledTurns
//////////////////////
void turn_90_degreesR1()
{
    // Reset encoders
    resetEncoders();
    // Ramp parameters
    const int maxSpeed = 95;                   // Maximum motor speed
    const int minSpeed = 75;                   // Minimum speed to overcome friction
    const int accelSteps = 400;                // Counts to accelerate
    const int targetCounts = 1700;             // Target encoder counts for 90-degree turn
    const int decelStart = targetCounts - 600; // When to start decelerating

    int currentSpeed = minSpeed; // Starting speed

    while (1)
    {
        int current_left_counts = getLeftEncoderCounts();

        // Acceleration phase
        if (current_left_counts < accelSteps)
        {
            currentSpeed = minSpeed + (maxSpeed - minSpeed) * current_left_counts / accelSteps;
        }
        // Deceleration phase
        else if (current_left_counts > decelStart)
        {
            int remaining = targetCounts - current_left_counts;
            if (remaining > 0)
            {
                currentSpeed = minSpeed + (maxSpeed - minSpeed) * remaining / (targetCounts - decelStart);
            }
            else
            {
                currentSpeed = 0;
            }
        }
        // Constant speed phase
        else
        {
            currentSpeed = maxSpeed;
        }

        // Set motor speeds
        setMotorSpeedL(0);            // Left motor stationary
        setMotorSpeedR(currentSpeed); // Right motor turning

        // Exit condition
        if (current_left_counts >= targetCounts)
        {
            break;
        }
    }

    // Stop the motors
    setMotorSpeedL(0);
    setMotorSpeedR(0);
    resetEncoders();
}
void turn_90_degreesR1_Controlled(int targetCounts)
{
    // Reset encoders
    resetEncoders();
    // Ramp parameters
    const int maxSpeed = 95;                   // Maximum motor speed
    const int minSpeed = 75;                   // Minimum speed to overcome friction
    const int accelSteps = 400;                // Counts to accelerate
    const int decelStart = targetCounts - 600; // When to start decelerating

    int currentSpeed = minSpeed; // Starting speed

    while (1)
    {
        int current_left_counts = getLeftEncoderCounts();

        // Acceleration phase
        if (current_left_counts < accelSteps)
        {
            currentSpeed = minSpeed + (maxSpeed - minSpeed) * current_left_counts / accelSteps;
        }
        // Deceleration phase
        else if (current_left_counts > decelStart)
        {
            int remaining = targetCounts - current_left_counts;
            if (remaining > 0)
            {
                currentSpeed = minSpeed + (maxSpeed - minSpeed) * remaining / (targetCounts - decelStart);
            }
            else
            {
                currentSpeed = 0;
            }
        }
        // Constant speed phase
        else
        {
            currentSpeed = maxSpeed;
        }

        // Set motor speeds
        setMotorSpeedR(0);            // Left motor stationary
        setMotorSpeedL(-currentSpeed); // Right motor turning

        // Exit condition
        if (current_left_counts >= targetCounts)
        {
            break;
        }
    }

    // Stop the motors
    setMotorSpeedL(0);
    setMotorSpeedR(0);
    resetEncoders();
}

void turn_90_degreesL1()
{
    // Reset encoders
    resetEncoders();
    // Ramp parameters
    const int maxSpeed = 95;                   // Maximum motor speed
    const int minSpeed = 75;                   // Minimum speed to overcome friction
    const int accelSteps = 400;                // Counts to accelerate
    const int targetCounts = 1638;             // Target encoder counts for 90-degree turn
    const int decelStart = targetCounts - 600; // When to start decelerating

    int currentSpeed = minSpeed; // Starting speed

    while (1)
    {
        int current_right_counts = getRightEncoderCounts();

        // Acceleration phase
        if (current_right_counts < accelSteps)
        {
            currentSpeed = minSpeed + (maxSpeed - minSpeed) * current_right_counts / accelSteps;
        }
        // Deceleration phase
        else if (current_right_counts > decelStart)
        {
            int remaining = targetCounts - current_right_counts;
            if (remaining > 0)
            {
                currentSpeed = minSpeed + (maxSpeed - minSpeed) * remaining / (targetCounts - decelStart);
            }
            else
            {
                currentSpeed = 0;
            }
        }
        // Constant speed phase
        else
        {
            currentSpeed = maxSpeed;
        }

        // Set motor speeds
        setMotorSpeedL(currentSpeed); // Left motor turning
        setMotorSpeedR(0);            // Right motor stationary

        // Exit condition
        if (current_right_counts >= targetCounts)
        {
            break;
        }
    }

    // Stop the motors
    setMotorSpeedL(0);
    setMotorSpeedR(0);
    resetEncoders();
}
void turn_left_45_degrees_right_Wh()
{
    resetEncoders();

    // Ramp parameters
    const int maxSpeed = 90; // Maximum motor speed
    const int minSpeed = 75; // Minimum speed to overcome friction
    const int accelSteps = 200;
    const int targetCounts = 670;              // Counts to accelerate
    const int decelStart = targetCounts - 300; // When to start decelerating
                                               // Target encoder counts for 45-degree turn

    int currentSpeed = minSpeed; // Starting speed

    while (1)
    {
        int current_right_counts = getRightEncoderCounts();

        // Acceleration phase
        if (current_right_counts < accelSteps)
        {
            currentSpeed = minSpeed + (maxSpeed - minSpeed) * current_right_counts / accelSteps;
        }
        // Deceleration phase
        else if (current_right_counts > decelStart)
        {
            int remaining = targetCounts - current_right_counts;
            if (remaining > 0)
            {
                currentSpeed = minSpeed + (maxSpeed - minSpeed) * remaining / (targetCounts - decelStart);
            }
            else
            {
                currentSpeed = 0;
            }
        }
        // Constant speed phase
        else
        {
            currentSpeed = maxSpeed;
        }

        // Set motor speeds
        setMotorSpeedL(currentSpeed);

        // Exit condition
        if (current_right_counts >= targetCounts)
        {
            break;
        }
    }

    // Stop the motors
    setMotorSpeedL(0);
    setMotorSpeedR(0);
    delay_ms(20);
    resetEncoders();
}
void turn_right_45_degrees_right_Wh()
{
    resetEncoders();

    // Ramp parameters
    const int maxSpeed = 90; // Maximum motor speed
    const int minSpeed = 70; // Minimum speed to overcome friction
    const int accelSteps = 200;
    const int targetCounts = 660;              // Counts to accelerate
    const int decelStart = targetCounts - 300; // When to start decelerating

    int currentSpeed = minSpeed; // Starting speed

    while (1)
    {
        int current_counts = getRightEncoderCounts();
        int current_right_counts = -(current_counts); // Use absolute value for right turn
        // Acceleration phase
        if (current_right_counts < accelSteps)
        {
            currentSpeed = minSpeed + (maxSpeed - minSpeed) * current_right_counts / accelSteps;
        }
        // Deceleration phase
        else if (current_right_counts > decelStart)
        {
            int remaining = targetCounts - current_right_counts;
            if (remaining > 0)
            {
                currentSpeed = minSpeed + (maxSpeed - minSpeed) * remaining / (targetCounts - decelStart);
            }
            else
            {
                currentSpeed = 0;
            }
        }
        // Constant speed phase
        else
        {
            currentSpeed = maxSpeed;
        }

        // Set motor speeds for right turn (opposite direction to left turn)
        setMotorSpeedL(-currentSpeed);

        // Exit condition
        if (current_right_counts >= targetCounts)
        {
            break;
        }
    }

    // Stop the motors
    setMotorSpeedL(0);
    setMotorSpeedR(0);
    delay_ms(20);
    resetEncoders();
}
///////////////////////////////
void turn_Right_45_TASK2_LR()
{
    resetEncoders();

    // Ramp parameters - adjust these for your robot
    const int maxSpeed = 90; // Maximum motor speed
    const int minSpeed = 80; // Minimum speed to overcome friction
    const int accelSteps = 150;
    const int target = 600;              // Counts to accelerate
    const int decelStart = target - 280; // When to start decelerating

    int currentSpeed = minSpeed; // Starting speed

    while (1)
    {
        int left_counts = abs(getLeftEncoderCounts());
        int right_counts = abs(getRightEncoderCounts());
        int avg_counts = (left_counts + right_counts) / 2;

        // Acceleration phase
        if (avg_counts < accelSteps)
        {
            currentSpeed = minSpeed + (maxSpeed - minSpeed) * avg_counts / accelSteps;
        }
        // Deceleration phase
        else if (avg_counts > decelStart)
        {
            int remaining = target - avg_counts;
            if (remaining > 0)
            {
                currentSpeed = minSpeed + (maxSpeed - minSpeed) * remaining / (target - decelStart);
            }
            else
            {
                currentSpeed = 0;
            }
        }
        // Constant speed phase
        else
        {
            currentSpeed = maxSpeed;
        }

        // Balance correction
        int error = left_counts - right_counts;
        int correction = error * 1;

        // Apply speeds with minimum guaranteed
        int speedL = -(currentSpeed + correction);
        int speedR = currentSpeed - correction;

        // Ensure minimum speed is maintained
        if (abs(speedL) < minSpeed)
            speedL = speedL < 0 ? -minSpeed : minSpeed;
        if (abs(speedR) < minSpeed)
            speedR = speedR < 0 ? -minSpeed : minSpeed;

        setMotorSpeedL(speedL);
        setMotorSpeedR(speedR);

        // Exit condition
        if (left_counts >= target && right_counts >= target)
        {
            break;
        }
    }

    m_stopLR();
    delay_ms(20);
    resetEncoders();
}
void turn_Left_45_TASK2_LR()
{
    resetEncoders();

    // Ramp parameters - adjust these for your robot
    const int maxSpeed = 90; // Maximum motor speed
    const int minSpeed = 85; // Minimum speed to overcome friction
    const int accelSteps = 150;
    const int target = 600;              // Counts to accelerate
    const int decelStart = target - 280; // When to start decelerating

    int currentSpeed = minSpeed; // Starting speed

    while (1)
    {
        int left_counts = abs(getLeftEncoderCounts());
        int right_counts = abs(getRightEncoderCounts());
        int avg_counts = (left_counts + right_counts) / 2;

        // Acceleration phase
        if (avg_counts < accelSteps)
        {
            currentSpeed = minSpeed + (maxSpeed - minSpeed) * avg_counts / accelSteps;
        }
        // Deceleration phase
        else if (avg_counts > decelStart)
        {
            int remaining = target - avg_counts;
            if (remaining > 0)
            {
                currentSpeed = minSpeed + (maxSpeed - minSpeed) * remaining / (target - decelStart);
            }
            else
            {
                currentSpeed = 0;
            }
        }
        // Constant speed phase
        else
        {
            currentSpeed = maxSpeed;
        }

        // Balance correction
        int error = left_counts - right_counts;
        int correction = error * 1;

        int speedL = currentSpeed + correction;
        int speedR = -(currentSpeed - correction);

        // Ensure minimum speed is maintained
        if (abs(speedL) < minSpeed)
            speedL = speedL < 0 ? -minSpeed : minSpeed;
        if (abs(speedR) < minSpeed)
            speedR = speedR < 0 ? -minSpeed : minSpeed;

        setMotorSpeedL(speedL);
        setMotorSpeedR(speedR);

        // Exit condition
        if (left_counts >= target && right_counts >= target)
        {
            break;
        }
    }

    m_stopLR();
    delay_ms(20);
    resetEncoders();
}
void turn_Right_45_TASK2_R1()
{
    // Reset encoders
    resetEncoders();
    // Ramp parameters
    const int maxSpeed = 90;                   // Maximum motor speed
    const int minSpeed = 85;                   // Minimum speed to overcome friction
    const int accelSteps = 400;                // Counts to accelerate
    const int targetCounts = 1350;             // Target encoder counts for 90-degree turn
    const int decelStart = targetCounts - 600; // When to start decelerating

    int currentSpeed = minSpeed; // Starting speed

    while (1)
    {
        int current_left_counts = getLeftEncoderCounts();

        // Acceleration phase
        if (current_left_counts < accelSteps)
        {
            currentSpeed = minSpeed + (maxSpeed - minSpeed) * current_left_counts / accelSteps;
        }
        // Deceleration phase
        else if (current_left_counts > decelStart)
        {
            int remaining = targetCounts - current_left_counts;
            if (remaining > 0)
            {
                currentSpeed = minSpeed + (maxSpeed - minSpeed) * remaining / (targetCounts - decelStart);
            }
            else
            {
                currentSpeed = 0;
            }
        }
        // Constant speed phase
        else
        {
            currentSpeed = maxSpeed;
        }

        // Set motor speeds
        setMotorSpeedL(0);            // Left motor stationary
        setMotorSpeedR(currentSpeed); // Right motor turning

        // Exit condition
        if (current_left_counts >= targetCounts)
        {
            break;
        }
    }

    // Stop the motors
    setMotorSpeedL(0);
    setMotorSpeedR(0);
    resetEncoders();
}
/// @brief //////////////////////////////
void turn_Right_180_LR()
{ // target
    resetEncoders();

    // Ramp parameters - adjust these for your robot
    const int maxSpeed = 95; // Maximum motor speed
    const int minSpeed = 65; // Minimum speed to overcome friction
    const int accelSteps = 500;
    const int target = 1661;             // Counts to accelerate
    const int decelStart = target - 800; // When to start decelerating

    int currentSpeed = minSpeed; // Starting speed

    while (1)
    {
        int left_counts = abs(getLeftEncoderCounts());
        int right_counts = abs(getRightEncoderCounts());
        int avg_counts = (left_counts + right_counts) / 2;

        // Acceleration phase
        if (avg_counts < accelSteps)
        {
            currentSpeed = minSpeed + (maxSpeed - minSpeed) * avg_counts / accelSteps;
        }
        // Deceleration phase
        else if (avg_counts > decelStart)
        {
            int remaining = target - avg_counts;
            if (remaining > 0)
            {
                currentSpeed = minSpeed + (maxSpeed - minSpeed) * remaining / (target - decelStart);
            }
            else
            {
                currentSpeed = 0;
            }
        }
        // Constant speed phase
        else
        {
            currentSpeed = maxSpeed;
        }

        // Balance correction
        int error = left_counts - right_counts;
        int correction = error * 1;

        // Apply speeds with minimum guaranteed
        int speedL = -(currentSpeed + correction);
        int speedR = currentSpeed - correction;

        // Ensure minimum speed is maintained
        if (abs(speedL) < minSpeed)
            speedL = speedL < 0 ? -minSpeed : minSpeed;
        if (abs(speedR) < minSpeed)
            speedR = speedR < 0 ? -minSpeed : minSpeed;

        setMotorSpeedL(speedL);
        setMotorSpeedR(speedR);

        // Exit condition
        if (left_counts >= target && right_counts >= target)
        {
            break;
        }
    }

    m_stopLR();
    delay_ms(20);
    resetEncoders();
}
void turn_Right_90_LR()
{ // target
    resetEncoders();

    // Ramp parameters - adjust these for your robot
    const int maxSpeed = 95; // Maximum motor speed
    const int minSpeed = 66; // Minimum speed to overcome friction
    const int accelSteps = 300;
    const int target = 835;              // Counts to accelerate
    const int decelStart = target - 480; // When to start decelerating

    int currentSpeed = minSpeed; // Starting speed

    while (1)
    {
        int left_counts = abs(getLeftEncoderCounts());
        int right_counts = abs(getRightEncoderCounts());
        int avg_counts = (left_counts + right_counts) / 2;

        // Acceleration phase
        if (avg_counts < accelSteps)
        {
            currentSpeed = minSpeed + (maxSpeed - minSpeed) * avg_counts / accelSteps;
        }
        // Deceleration phase
        else if (avg_counts > decelStart)
        {
            int remaining = target - avg_counts;
            if (remaining > 0)
            {
                currentSpeed = minSpeed + (maxSpeed - minSpeed) * remaining / (target - decelStart);
            }
            else
            {
                currentSpeed = 0;
            }
        }
        // Constant speed phase
        else
        {
            currentSpeed = maxSpeed;
        }

        // Balance correction
        int error = left_counts - right_counts;
        int correction = error * 1;

        // Apply speeds with minimum guaranteed
        int speedL = -(currentSpeed + correction);
        int speedR = currentSpeed - correction;

        // Ensure minimum speed is maintained
        if (abs(speedL) < minSpeed)
            speedL = speedL < 0 ? -minSpeed : minSpeed;
        if (abs(speedR) < minSpeed)
            speedR = speedR < 0 ? -minSpeed : minSpeed;

        setMotorSpeedL(speedL);
        setMotorSpeedR(speedR);

        // Exit condition
        if (left_counts >= target && right_counts >= target)
        {
            break;
        }
    }

    m_stopLR();
    delay_ms(20);
    resetEncoders();
}
void turn_Left_180_LR()
{ // target
    resetEncoders();

    // Ramp parameters - adjust these for your robot
    const int maxSpeed = 95; // Maximum motor speed
    const int minSpeed = 66;              // Minimum speed to overcome friction
    const int accelSteps = 400;           // Counts to accelerate
    const int target = 1650;              // Target encoder counts for 180-degree turn
    const int decelStart = target - 1000; // When to start decelerating

    int currentSpeed = minSpeed; // Starting speed

    while (1)
    {
        int left_counts = abs(getLeftEncoderCounts());
        int right_counts = abs(getRightEncoderCounts());
        int avg_counts = (left_counts + right_counts) / 2;

        // Acceleration phase
        if (avg_counts < accelSteps)
        {
            currentSpeed = minSpeed + (maxSpeed - minSpeed) * avg_counts / accelSteps;
        }
        // Deceleration phase
        else if (avg_counts > decelStart)
        {
            int remaining = target - avg_counts;
            if (remaining > 0)
            {
                currentSpeed = minSpeed + (maxSpeed - minSpeed) * remaining / (target - decelStart);
            }
            else
            {
                currentSpeed = 0;
            }
        }
        // Constant speed phase
        else
        {
            currentSpeed = maxSpeed;
        }

        // Balance correction
        int error = left_counts - right_counts;
        int correction = error * 1;

        // Apply speeds with minimum guaranteed
        int speedL = currentSpeed + correction;
        int speedR = -(currentSpeed - correction);

        // Ensure minimum speed is maintained
        if (abs(speedL) < minSpeed)
            speedL = speedL < 0 ? -minSpeed : minSpeed;
        if (abs(speedR) < minSpeed)
            speedR = speedR < 0 ? -minSpeed : minSpeed;

        setMotorSpeedL(speedL);
        setMotorSpeedR(speedR);

        // Exit condition
        if (left_counts >= target && right_counts >= target)
        {
            break;
        }
    }

    m_stopLR();
    delay_ms(20);
    resetEncoders();
}
void turn_Left_90_LR()
{ // target
    resetEncoders();

    // Ramp parameters - adjust these for your robot
    const int maxSpeed = 95; // Maximum motor speed
    const int minSpeed = 66; // Minimum speed to overcome friction
    const int accelSteps = 300;
    const int target = 835;              // Counts to accelerate
    const int decelStart = target - 480; // When to start decelerating

    int currentSpeed = minSpeed; // Starting speed

    while (1)
    {
        int left_counts = abs(getLeftEncoderCounts());
        int right_counts = abs(getRightEncoderCounts());
        int avg_counts = (left_counts + right_counts) / 2;

        // Acceleration phase
        if (avg_counts < accelSteps)
        {
            currentSpeed = minSpeed + (maxSpeed - minSpeed) * avg_counts / accelSteps;
        }
        // Deceleration phase
        else if (avg_counts > decelStart)
        {
            int remaining = target - avg_counts;
            if (remaining > 0)
            {
                currentSpeed = minSpeed + (maxSpeed - minSpeed) * remaining / (target - decelStart);
            }
            else
            {
                currentSpeed = 0;
            }
        }
        // Constant speed phase
        else
        {
            currentSpeed = maxSpeed;
        }

        // Balance correction
        int error = left_counts - right_counts;
        int correction = error * 1;

        // Apply speeds with minimum guaranteed
        int speedL = currentSpeed + correction;
        int speedR = -(currentSpeed - correction);

        // Ensure minimum speed is maintained
        if (abs(speedL) < minSpeed)
            speedL = speedL < 0 ? -minSpeed : minSpeed;
        if (abs(speedR) < minSpeed)
            speedR = speedR < 0 ? -minSpeed : minSpeed;

        setMotorSpeedL(speedL);
        setMotorSpeedR(speedR);

        // Exit condition
        if (left_counts >= target && right_counts >= target)
        {
            break;
        }
    }

    m_stopLR();
    delay_ms(20);
    resetEncoders();
}
void turn_Right_45_LR()
{
    resetEncoders();

    // Ramp parameters - adjust these for your robot
    const int maxSpeed = 90; // Maximum motor speed
    const int minSpeed = 70; // Minimum speed to overcome friction
    const int accelSteps = 120;
    const int target = 310;              // Counts to accelerate
    const int decelStart = target - 210; // When to start decelerating

    int currentSpeed = minSpeed; // Starting speed

    while (1)
    {
        int left_counts = abs(getLeftEncoderCounts());
        int right_counts = abs(getRightEncoderCounts());
        int avg_counts = (left_counts + right_counts) / 2;

        // Acceleration phase
        if (avg_counts < accelSteps)
        {
            currentSpeed = minSpeed + (maxSpeed - minSpeed) * avg_counts / accelSteps;
        }
        // Deceleration phase
        else if (avg_counts > decelStart)
        {
            int remaining = target - avg_counts;
            if (remaining > 0)
            {
                currentSpeed = minSpeed + (maxSpeed - minSpeed) * remaining / (target - decelStart);
            }
            else
            {
                currentSpeed = 0;
            }
        }
        // Constant speed phase
        else
        {
            currentSpeed = maxSpeed;
        }

        // Balance correction
        int error = left_counts - right_counts;
        int correction = error * 1;

        // Apply speeds with minimum guaranteed
        int speedL = -(currentSpeed + correction);
        int speedR = currentSpeed - correction;

        // Ensure minimum speed is maintained
        if (abs(speedL) < minSpeed)
            speedL = speedL < 0 ? -minSpeed : minSpeed;
        if (abs(speedR) < minSpeed)
            speedR = speedR < 0 ? -minSpeed : minSpeed;

        setMotorSpeedL(speedL);
        setMotorSpeedR(speedR);

        // Exit condition
        if (left_counts >= target && right_counts >= target)
        {
            break;
        }
    }

    m_stopLR();
    delay_ms(20);
    resetEncoders();
}
void turn_Right_90_LR_Controlled(int target, int minSpeed)
{ // target
    resetEncoders();

    // Ramp parameters - adjust these for your robot
    const int maxSpeed = 90; // Maximum motor speed
    // Minimum speed to overcome friction
    const int accelSteps = 300;
    // Counts to accelerate
    const int decelStart = target - 480; // When to start decelerating

    int currentSpeed = minSpeed; // Starting speed

    while (1)
    {
        int left_counts = abs(getLeftEncoderCounts());
        int right_counts = abs(getRightEncoderCounts());
        int avg_counts = (left_counts + right_counts) / 2;

        // Acceleration phase
        if (avg_counts < accelSteps)
        {
            currentSpeed = minSpeed + (maxSpeed - minSpeed) * avg_counts / accelSteps;
        }
        // Deceleration phase
        else if (avg_counts > decelStart)
        {
            int remaining = target - avg_counts;
            if (remaining > 0)
            {
                currentSpeed = minSpeed + (maxSpeed - minSpeed) * remaining / (target - decelStart);
            }
            else
            {
                currentSpeed = 0;
            }
        }
        // Constant speed phase
        else
        {
            currentSpeed = maxSpeed;
        }

        // Balance correction
        int error = left_counts - right_counts;
        int correction = error * 1;

        // Apply speeds with minimum guaranteed
        int speedL = -(currentSpeed + correction);
        int speedR = currentSpeed - correction;

        // Ensure minimum speed is maintained
        if (abs(speedL) < minSpeed)
            speedL = speedL < 0 ? -minSpeed : minSpeed;
        if (abs(speedR) < minSpeed)
            speedR = speedR < 0 ? -minSpeed : minSpeed;

        setMotorSpeedL(speedL);
        setMotorSpeedR(speedR);

        // Exit condition
        if (left_counts >= target && right_counts >= target)
        {
            break;
        }
    }

    m_stopLR();
    delay_ms(20);
    resetEncoders();
}
//////////////task4////////////////
void T4_turn_Right_90_LR()
{ // target
    resetEncoders();

    // Ramp parameters - adjust these for your robot
    const int maxSpeed = 95; // Maximum motor speed
    const int minSpeed = 65; // Minimum speed to overcome friction
    const int accelSteps = 300;
    const int target = 823;              // Counts to accelerate
    const int decelStart = target - 480; // When to start decelerating

    int currentSpeed = minSpeed; // Starting speed

    while (1)
    {
        int left_counts = abs(getLeftEncoderCounts());
        int right_counts = abs(getRightEncoderCounts());
        int avg_counts = (left_counts + right_counts) / 2;

        // Acceleration phase
        if (avg_counts < accelSteps)
        {
            currentSpeed = minSpeed + (maxSpeed - minSpeed) * avg_counts / accelSteps;
        }
        // Deceleration phase
        else if (avg_counts > decelStart)
        {
            int remaining = target - avg_counts;
            if (remaining > 0)
            {
                currentSpeed = minSpeed + (maxSpeed - minSpeed) * remaining / (target - decelStart);
            }
            else
            {
                currentSpeed = 0;
            }
        }
        // Constant speed phase
        else
        {
            currentSpeed = maxSpeed;
        }

        // Balance correction
        int error = left_counts - right_counts;
        int correction = error * 1;

        // Apply speeds with minimum guaranteed
        int speedL = -(currentSpeed + correction);
        int speedR = currentSpeed - correction;

        // Ensure minimum speed is maintained
        if (abs(speedL) < minSpeed)
            speedL = speedL < 0 ? -minSpeed : minSpeed;
        if (abs(speedR) < minSpeed)
            speedR = speedR < 0 ? -minSpeed : minSpeed;

        setMotorSpeedL(speedL);
        setMotorSpeedR(speedR);

        // Exit condition
        if (left_counts >= target && right_counts >= target)
        {
            break;
        }
    }

    m_stopLR();
    delay_ms(20);
    resetEncoders();
}
void T4_turn_Left_180_LR()
{ // target
    resetEncoders();

    // Ramp parameters - adjust these for your robot
    const int maxSpeed = 95; // Maximum motor speed
    const int minSpeed = 65;              // Minimum speed to overcome friction
    const int accelSteps = 400;           // Counts to accelerate
    const int target = 1630;              // Target encoder counts for 180-degree turn
    const int decelStart = target - 1000; // When to start decelerating

    int currentSpeed = minSpeed; // Starting speed

    while (1)
    {
        int left_counts = abs(getLeftEncoderCounts());
        int right_counts = abs(getRightEncoderCounts());
        int avg_counts = (left_counts + right_counts) / 2;

        // Acceleration phase
        if (avg_counts < accelSteps)
        {
            currentSpeed = minSpeed + (maxSpeed - minSpeed) * avg_counts / accelSteps;
        }
        // Deceleration phase
        else if (avg_counts > decelStart)
        {
            int remaining = target - avg_counts;
            if (remaining > 0)
            {
                currentSpeed = minSpeed + (maxSpeed - minSpeed) * remaining / (target - decelStart);
            }
            else
            {
                currentSpeed = 0;
            }
        }
        // Constant speed phase
        else
        {
            currentSpeed = maxSpeed;
        }

        // Balance correction
        int error = left_counts - right_counts;
        int correction = error * 1;

        // Apply speeds with minimum guaranteed
        int speedL = currentSpeed + correction;
        int speedR = -(currentSpeed - correction);

        // Ensure minimum speed is maintained
        if (abs(speedL) < minSpeed)
            speedL = speedL < 0 ? -minSpeed : minSpeed;
        if (abs(speedR) < minSpeed)
            speedR = speedR < 0 ? -minSpeed : minSpeed;

        setMotorSpeedL(speedL);
        setMotorSpeedR(speedR);

        // Exit condition
        if (left_counts >= target && right_counts >= target)
        {
            break;
        }
    }

    m_stopLR();
    delay_ms(20);
    resetEncoders();
}
void T4_turn_Left_90_LR()
{ // target
    resetEncoders();

    // Ramp parameters - adjust these for your robot
    const int maxSpeed = 95; // Maximum motor speed
    const int minSpeed = 65; // Minimum speed to overcome friction
    const int accelSteps = 300;
    const int target = 825;              // Counts to accelerate
    const int decelStart = target - 480; // When to start decelerating

    int currentSpeed = minSpeed; // Starting speed

    while (1)
    {
        int left_counts = abs(getLeftEncoderCounts());
        int right_counts = abs(getRightEncoderCounts());
        int avg_counts = (left_counts + right_counts) / 2;

        // Acceleration phase
        if (avg_counts < accelSteps)
        {
            currentSpeed = minSpeed + (maxSpeed - minSpeed) * avg_counts / accelSteps;
        }
        // Deceleration phase
        else if (avg_counts > decelStart)
        {
            int remaining = target - avg_counts;
            if (remaining > 0)
            {
                currentSpeed = minSpeed + (maxSpeed - minSpeed) * remaining / (target - decelStart);
            }
            else
            {
                currentSpeed = 0;
            }
        }
        // Constant speed phase
        else
        {
            currentSpeed = maxSpeed;
        }

        // Balance correction
        int error = left_counts - right_counts;
        int correction = error * 1;

        // Apply speeds with minimum guaranteed
        int speedL = currentSpeed + correction;
        int speedR = -(currentSpeed - correction);

        // Ensure minimum speed is maintained
        if (abs(speedL) < minSpeed)
            speedL = speedL < 0 ? -minSpeed : minSpeed;
        if (abs(speedR) < minSpeed)
            speedR = speedR < 0 ? -minSpeed : minSpeed;

        setMotorSpeedL(speedL);
        setMotorSpeedR(speedR);

        // Exit condition
        if (left_counts >= target && right_counts >= target)
        {
            break;
        }
    }

    m_stopLR();
    delay_ms(20);
    resetEncoders();
}
