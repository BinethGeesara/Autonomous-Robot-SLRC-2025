resetEncoders(); // Reset encoders before moving
        while (1)
        {
            int currentRightCount = getRightEncoderCounts();
            int currentLeftCount = getLeftEncoderCounts();
            int x = ((teta * 2 * MPI * 18 * 31 ) / 360);
            // Check if the right wheel has moved the required distance
            if (currentRightCount >= x && currentLeftCount <= -x)
            {
                break;
            }

            // Move the right wheel
            setMotorSpeedR(-65); // Adjust speed as needed
            setMotorSpeedL(65);  // Keep the left wheel stationary
        }






        //////////////////////////////
        // Wait for the left IR sensor to trigger
    if (leftIRTriggered == 1 && happen == 1)
    {
        happen = 0;
        stripeStartLeftCount = currentLeftCount;
        stripeStartRightCount = getRightEncoderCounts();

        // Start driving until the right IR sensor is triggered
        while (rightIRTriggered == 0)
        {
            enc_drive();
        }
        setMotorSpeedL(0);
        setMotorSpeedR(0);

        // Get encoder counts at the end
        int stripeEndLeftCount = currentLeftCount;
        int stripeEndRightCount = getRightEncoderCounts();

        // Calculate the encoder counts for the stripe
        int leftStripeCount = stripeEndLeftCount - stripeStartLeftCount;
        int rightStripeCount = stripeEndRightCount - stripeStartRightCount;
        // Calculate the average stripe width
        averageEncoderDifference = (leftStripeCount + rightStripeCount) / 2;
        // angleDISTmm = averageEncoderDifference * 10 / 31; // Adjust this value based on your robot's configuration
        teta = ((atan2(averageEncoderDifference * 10 / 31, 133)) * 180 / MPI);

        resetEncoders(); // Reset encoders before moving
        while (1)
        {
            int currentRightCount = getRightEncoderCounts();
            int currentLeftCount = getLeftEncoderCounts();
            int x = (teta * TICKS_PER_DEGREE);
            // Check if the right wheel has moved the required distance
            if (currentRightCount >= x && currentLeftCount <= -x)
            {
                break;
            }

            // Move the right wheel
            setMotorSpeedR(-65); // Adjust speed as needed
            setMotorSpeedL(65);  // Keep the left wheel stationary
        }

        // Stop the right wheel after moving
        setMotorSpeedR(0);
        setMotorSpeedL(0); // Stop the left wheel as well
        delay_ms(9000);    // Optional delay for stabilization

        resetEncoders();
    }
    else 




    /////////////////////////////
    void align_to_line()
{
    // Variables to store encoder counts
    int leftEncoderStart = 0;
    int rightEncoderStart = 0;
    int leftEncoderEnd = 0;
    int rightEncoderEnd = 0;
    static int stripeStartLeftCount = 0; // Left encoder count at the start of the stripe
    static int stripeStartRightCount = 0;
    int currentLeftCount = getLeftEncoderCounts();
    int currentRightCount = getRightEncoderCounts();
    int happen = 1;
    double MPI = 3.14159265358979323846;

    if (leftIRTriggered == 1 && happen == 1)
    {
        leftIRTriggered = 1;
        happen = 0;
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
        // angleDISTmm = averageEncoderDifference * 10 / 31; // Adjust this value based on your robot's configuration
        teta = ((atan2(averageEncoderDifference * 10 / 31, 133)) * 180 / MPI);

        resetEncoders(); // Reset encoders before moving
        while (1)
        {
            int currentRightCount = getRightEncoderCounts();
            int currentLeftCount = getLeftEncoderCounts();
            x = (teta * TICKS_PER_DEGREE);
            // Check if the right wheel has moved the required distance
            if (currentLeftCount <= -x )
            {
                break;
            }
            setMotorSpeedR(-75);  // Keep the left wheel stationary
        }

        // Stop the right wheel after moving
        setMotorSpeedR(0);
        setMotorSpeedL(0); // Stop the left wheel as well
        delay_ms(9000);    // Optional delay for stabilization

        resetEncoders();
    }
    else if (rightIRTriggered == 1 && happen == 1)
    {
        rightIRTriggered = 0;
        happen = 0;
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
        //angleDISTmm = averageEncoderDifference * 10 / 31; // Adjust this value based on your robot's configuration
        teta = ((atan2(averageEncoderDifference * 10 / 31, 133)) * 180 / MPI);

        resetEncoders(); // Reset encoders before moving
        while (1)
        {
            int currentRightCount = getRightEncoderCounts();
            int currentLeftCount = getLeftEncoderCounts();
            x = (teta * TICKS_PER_DEGREE);
            // Check if the right wheel has moved the required distance
            if (currentRightCount <= -x )
            {
                break;
            }
            setMotorSpeedL(-75);  // Adjust speed as needed
             // Keep the left wheel stationary
        }
        setMotorSpeedR(0);
        setMotorSpeedL(0); // Stop the left wheel as well
        delay_ms(9000);    // Optional delay for stabilization
        resetEncoders();
    }
}
\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
void align_to_line()
{
    // Variables to store encoder counts
    int leftEncoderStart = 0;
    int rightEncoderStart = 0;
    int leftEncoderEnd = 0;
    int rightEncoderEnd = 0;
    static int stripeStartLeftCount = 0; // Left encoder count at the start of the stripe
    static int stripeStartRightCount = 0;
    int currentLeftCount = getLeftEncoderCounts();
    int currentRightCount = getRightEncoderCounts();
    int happen = 1;
    double MPI = 3.14159265358979323846;

    if (leftIRTriggered == 1 && happen == 1)
    {
        leftIRTriggered = 1;
        happen = 0;
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
        // angleDISTmm = averageEncoderDifference * 10 / 31; // Adjust this value based on your robot's configuration
        teta = ((atan2(averageEncoderDifference * 10 / 31, 133)) * 180 / MPI);

        // Exit if teta < 3
        if (teta < 3)
        {
            return;
        }

        resetEncoders(); // Reset encoders before moving
        while (1)
        {
            int currentRightCount = getRightEncoderCounts();
            int currentLeftCount = getLeftEncoderCounts();
            x = (teta * TICKS_PER_DEGREE);
            // Check if the right wheel has moved the required distance
            if (currentLeftCount <= -x )
            {
                break;
            }
            setMotorSpeedR(-75);  // Keep the left wheel stationary
        }

        // Stop the right wheel after moving
        setMotorSpeedR(0);
        setMotorSpeedL(0); // Stop the left wheel as well
        delay_ms(9000);    // Optional delay for stabilization

        resetEncoders();
    }
    else if (rightIRTriggered == 1 && happen == 1)
    {
        rightIRTriggered = 0;
        happen = 0;
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
        //angleDISTmm = averageEncoderDifference * 10 / 31; // Adjust this value based on your robot's configuration
        teta = ((atan2(averageEncoderDifference * 10 / 31, 133)) * 180 / MPI);

        // Exit if teta < 3
        if (teta < 3)
        {
            return;
        }

        resetEncoders(); // Reset encoders before moving
        while (1)
        {
            int currentRightCount = getRightEncoderCounts();
            int currentLeftCount = getLeftEncoderCounts();
            x = (teta * TICKS_PER_DEGREE);
            // Check if the right wheel has moved the required distance
            if (currentRightCount <= -x )
            {
                break;
            }
            setMotorSpeedL(-75);  // Adjust speed as needed
             // Keep the left wheel stationary
        }
        setMotorSpeedR(0);
        setMotorSpeedL(0); // Stop the left wheel as well
        delay_ms(9000);    // Optional delay for stabilization
        resetEncoders();
    }
}
///////////////////////////
void align_to_line()
{
    // Variables to store encoder counts
    int leftEncoderStart = 0;
    int rightEncoderStart = 0;
    int leftEncoderEnd = 0;
    int rightEncoderEnd = 0;
    static int stripeStartLeftCount = 0; // Left encoder count at the start of the stripe
    static int stripeStartRightCount = 0;
    int currentLeftCount = getLeftEncoderCounts();
    int currentRightCount = getRightEncoderCounts();
    int happen = 1;
    double MPI = 3.14159265358979323846;

    while (1) // Loop until teta < 3
    {
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

            // Exit the loop if teta < 3
            if (teta < 3)
            {
                return;
            }

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

            // Stop the right wheel after moving
            setMotorSpeedR(0);
            setMotorSpeedL(0); // Stop the left wheel as well
            delay_ms(200);    // Optional delay for stabilization

            // Reverse and move forward to correct again
            setMotorSpeedL(-75);
            setMotorSpeedR(-75);
            delay_ms(500);
            setMotorSpeedL(75);
            setMotorSpeedR(75);
            delay_ms(500);

            resetEncoders();
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

            // Exit the loop if teta < 3
            if (teta < 3)
            {
                return;
            }

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
            setMotorSpeedL(0); // Stop the left wheel as well
            delay_ms(200);    // Optional delay for stabilization

            // Reverse and move forward to correct again
            setMotorSpeedL(-75);
            setMotorSpeedR(-75);
            delay_ms(500);
            setMotorSpeedL(75);
            setMotorSpeedR(75);
            delay_ms(500);

            resetEncoders();
        }
    }
}

