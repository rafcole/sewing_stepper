#include "FastAccelStepper.h"
#include "AVRStepperPins.h" // Only required for AVR controllers

#define stepPinStepper   6 // must be 6, 7, 8 for MEGA 2560
#define dirPinStepper    7
#define enablePinStepper 8

int pedalPin = A0;

// pedal ranges from 3.5-5v 
unsigned int pedalPinMin = 700;
unsigned int pedalPinMax = 1010;

unsigned int minSpeedInHz = 1700;
unsigned int maxSpeedInHz = 50000;

// not accessible in setup?
unsigned int absAccelerationRate = 60000;
unsigned int linAccelRate = 30;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  engine.init();
  stepper = engine.stepperConnectToPin(stepPinStepper);
  if (stepper) {
    stepper->setDirectionPin(dirPinStepper, false);
    stepper->setEnablePin(enablePinStepper);
    stepper->setAutoEnable(true);
    stepper->setLinearAcceleration(linAccelRate);
    stepper->setSpeedInHz(0);       // min value is 1 wrt runForward(), won't respond to 0
    stepper->setAcceleration(absAccelerationRate);
  }

}

void loop() {
  int sensorValue = analogRead(pedalPin);

  unsigned int newSpeedInHz = map(sensorValue, pedalPinMax, pedalPinMin, minSpeedInHz, maxSpeedInHz);

  if (newSpeedInHz <= minSpeedInHz) {
    stepper->stopMove();
  } else {
    stepper->setSpeedInHz(newSpeedInHz);
    stepper->runForward();
  }

  delay(50);

  Serial.println(newSpeedInHz);
}