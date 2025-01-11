#include "FastAccelStepper.h"
#include <Encoder.h>

// As in StepperDemo for Motor 1 on AVR
#define dirPinStepper    5
#define enablePinStepper 6
#define stepPinStepper   9  // OC1A in case of AVR

// As in StepperDemo for Motor 1 on ESP32
// #define dirPinStepper 18
// #define enablePinStepper 26
// #define stepPinStepper 17

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;




int encoderPinA = 3;
int encoderPinB = 2;
int encoderPulsesPerRotation = 2400;
long oldPosition = 0;
Encoder myEnc(encoderPinA, encoderPinB);




void setup() {
  Serial.begin(115200);
  engine.init();
  stepper = engine.stepperConnectToPin(stepPinStepper);
  if (stepper) {
    stepper->setDirectionPin(dirPinStepper, false);
    stepper->setEnablePin(enablePinStepper);
    stepper->setAutoEnable(true);

    // If auto enable/disable need delays, just add (one or both):
    // stepper->setDelayToEnable(50);
    // stepper->setDelayToDisable(1000);

    stepper->setSpeedInHz(20000);  // the parameter is us/step !!!
    stepper->setAcceleration(10000);

    
   
    stepper->move(100, true);
    Serial.print("\nPulses per 100 steps: ");
    Serial.println(myEnc.readAndReset());

    stepper->move(1000, true);
    Serial.print("\nPulses per 1000 steps: ");
    Serial.println(myEnc.readAndReset());

    stepper->move(2000, true);
    Serial.print("\nPulses per 2000 steps: ");
    Serial.println(myEnc.readAndReset());

    stepper->move(10000, true);
    Serial.print("\nPulses per 10000 steps: ");
    Serial.println(myEnc.readAndReset());
  }
}

void loop() {}
