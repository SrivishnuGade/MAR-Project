#include <Servo.h>

// Define 5 servo objects
Servo leftHand, rightHand, head, leftLeg, rightLeg;

// Define servo pins
const int leftHandPin = 3;
const int rightHandPin = 4;
const int headPin = 5;
const int leftLegPin = 6;
const int rightLegPin = 7;

// Angle limits
const int lowerLimit[5] = {120, 0, 0, 130, 0};
const int upperLimit[5] = {180, 60, 15, 180, 50};

// Store current positions
int currentAngles[5] = {180, 0, 0, 180, 0};

// Delay per interpolation step in ms
const int delayPerStep = 50;

void setup() {
    Serial.begin(9600);
    Serial.println("Arduino Ready! Waiting for angles and duration...");

    // Attach servos
    leftHand.attach(leftHandPin);
    rightHand.attach(rightHandPin);
    head.attach(headPin);
    leftLeg.attach(leftLegPin);
    rightLeg.attach(rightLegPin);

    // Move to initial positions
    moveAllServosSmoothly(currentAngles, delayPerStep * 20); // Default 1s
}

void loop() {
    if (Serial.available() > 0) {
        String receivedData = Serial.readStringUntil('\n');
        Serial.print("Received: ");
        Serial.println(receivedData);

        int targetAngles[5];
        int duration = 1000;  // Default duration
        int index = 0;

        char *token = strtok((char *)receivedData.c_str(), ",");

        while (token != NULL && index < 6) {
            int value = atoi(token);

            if (index < 5) {
                // Clamp angles within limits
                if (value < lowerLimit[index]) value = lowerLimit[index];
                if (value > upperLimit[index]) value = upperLimit[index];
                targetAngles[index] = value;
            } else {
                duration = max(100, value); // Minimum duration 100ms
            }

            token = strtok(NULL, ",");
            index++;
        }

        if (index == 6) {
            moveAllServosSmoothly(targetAngles, duration);
            Serial.println("Servos moved smoothly!");
        } else {
            Serial.println("Error: Expected 6 values (5 angles + duration).");
        }
    }
}

// Smooth movement function with variable duration
void moveAllServosSmoothly(int targetAngles[5], int duration) {
    Servo* servos[] = {&leftHand, &rightHand, &head, &leftLeg, &rightLeg};

    int startAngles[5];  
    float stepSizes[5];
    int steps = duration / delayPerStep;

    if (steps < 1) steps = 1;

    // Calculate step sizes
    for (int i = 0; i < 5; i++) {
        startAngles[i] = currentAngles[i];
        stepSizes[i] = (float)(targetAngles[i] - startAngles[i]) / steps;
    }

    // Perform interpolation
    for (int step = 0; step <= steps; step++) {
        for (int i = 0; i < 5; i++) {
            int interpolatedAngle = startAngles[i] + (stepSizes[i] * step);
            servos[i]->write(interpolatedAngle);
        }
        delay(delayPerStep);
    }

    // Update current angles
    for (int i = 0; i < 5; i++) {
        currentAngles[i] = targetAngles[i];
    }
}
