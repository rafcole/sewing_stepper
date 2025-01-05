#include "FastAccelStepper.h"
#include "AVRStepperPins.h" // Only required for AVR controllers

#define stepPinStepper   6 // must be 6, 7, 8 for MEGA 2560
#define dirPinStepper    7
#define enablePinStepper 8


// If using an AVR device use the definitons provided in AVRStepperPins
//    stepPinStepper1A
//
// or even shorter (for 2560 the correct pin on the chosen timer is selected):
//    stepPinStepperA

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

void setup() {
   engine.init();
   stepper = engine.stepperConnectToPin(stepPinStepper);
   if (stepper) {
      stepper->setDirectionPin(dirPinStepper);
      stepper->setEnablePin(enablePinStepper);
      stepper->setAutoEnable(true);

      stepper->setSpeedInHz(20000);       // 500 steps/s
      stepper->setAcceleration(17000);    // 100 steps/s²
      // stepper->move(100000);
   }

}

void loop() {
}