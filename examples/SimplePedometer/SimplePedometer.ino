/*
  LSM6DSOX Pedometer Example

  This example demonstrates how to use the built-in pedometer
  feature of the LSM6DSOX IMU. The pedometer counts the number of
  steps taken based on the accelerometer data and can generate
  interrupts when a step is detected.

  Circuit:
   - Arduino with LSM6DSOX (e.g. Arduino Nano RP2040 Connect, Arduino Nano 33 BLE)
     or external LSM6DSOX module.
   - For interrupts, connect the IMU's INT1 pin to an interrupt-capable pin
     on your Arduino, and set `interruptPin` accordingly.
*/

#include <Arduino_LSM6DSOX.h>

// Change this to the pin connected to INT1 if using an external module.
// On some boards with built-in LSM6DSOX there is an internal connection,
// but it might require a specific pin depending on the board architecture.
// For basic testing without interrupts, you can leave it undefined or ignore the interrupt logic.
const int interruptPin = 2; 

volatile bool stepDetected = false;
int lastSteps = 0;

void onStepDetected() {
  stepDetected = true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.println("IMU initialized.");

  // Enable pedometer, debounce set to 5 steps, route interrupt to INT1
  // If you don't want interrupts, you can just do: IMU.configurePedometer(true);
  if (!IMU.configurePedometer(true, 5, true, false)) {
    Serial.println("Failed to configure pedometer!");
    while (1);
  }

  // Set interrupt pin as input with pullup (adjust depending on your board/module wiring)
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), onStepDetected, FALLING); 

  Serial.println("Pedometer enabled. Start walking!");
}

void loop() {
  if (stepDetected) {
    Serial.println("Step interrupt fired!");
    stepDetected = false;
  }

  int steps = IMU.pedometerSteps();
  if (steps >= 0) {
    if (steps != lastSteps) {
      Serial.print("Steps: ");
      Serial.println(steps);
      lastSteps = steps;
    }
  }

  delay(100);
}