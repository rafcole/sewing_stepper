#include "FastAccelStepper.h"
#include "AVRStepperPins.h" // Only required for AVR controllers

// // Mega
// #define stepPinStepper   6 // must be 6, 7, 8 for MEGA 2560
// #define dirPinStepper    7
// #define enablePinStepper 8


// Uno R3
#define dirPinStepper    5
#define enablePinStepper 6
#define stepPinStepper   9

// external io
int pedalPin = A1;

// pedal ranges from 3.5-5v 
unsigned int pedalPinMin = 350;
unsigned int pedalPinMax = 10;

unsigned int minSpeedInHz = 5000; // 1700 good
unsigned int maxSpeedInHz = 49000;
unsigned int homingSpeed = 7000;

// not accessible in setup?
unsigned int absAccelerationRate = 40000;
unsigned int linAccelRate = 50;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

// debounce
int counter = 0;
long last_micros = 0;

// button press logic tester
// normally closed momentary
int interuptPin = 3;
// volatile bool lastMoveHomed = false;
volatile bool foundHome = true;
volatile bool firstHomeFound = false;

long stepsPerRotation = 0;
bool firstRotationComplete = false;

// add single sensor
// Track current position in steps 
// Need global variable 
//   "getCurrentPosition() is counting monotonously up or down respectively."
//   setCurrentPosition(num)

// stepsPerRotation, intialize to 0
// firstPass = initialize to false, set to true when first detection hits

// interupt on ND sensor
  // calibration
    // set steps per rotation if not already set
      // if firstPass is false
        // turn to true, set current position to 0
      // if stepPerRotation is not set
        // stepsPerRotation = current position
        // halfStep
        // cushion

  // if currentSpeed > minimum
    // do nothing
  // still sewing

// TODO - ======================================== Do this first
  // interupt on pedal goes low
    // set this first, should work immediately?
    // refactor main loop to run forward based on pin read

  // interupts on pedal control runForward and forceStop?


// calibrationPass
  
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

  // ISR for external sensors
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interuptPin), needleDownInterrupt, RISING);
}

void loop() {
  int pedalReading = analogRead(pedalPin);
  // Serial.println(pedalReading);

  unsigned int newSpeedInHz = map(pedalReading, pedalPinMax, pedalPinMin, minSpeedInHz, maxSpeedInHz);
  // Serial.println(newSpeedInHz);



  if (!stepsPerRotation){
    setStepsPerRotation();
  }

  // pedal is up
  // home to needle down and stop
  if (foundHome && newSpeedInHz <= minSpeedInHz) {
    stepper->forceStop();
    foundHome = false;

  } else if (newSpeedInHz < minSpeedInHz) {
    // normal 
    // TODO 
    // switch to step counting routine

    // primative single sensor homing
    stepper->setSpeedInHz(homingSpeed);
  } else {
    // run as normal
    stepper->setSpeedInHz(newSpeedInHz);
    stepper->runForward();
  }

  // Serial.print("Steps per rotation: ");
  // Serial.print(stepsPerRotation);
  // Serial.print("/n");

  // if (newSpeedInHz <= minSpeedInHz) {
    
  //   stepper->forceStop();

  //   // stepper->runForward();
  // } 

  delay(80);

  
}

void setStepsPerRotation() {
  if (firstHomeFound && !firstRotationComplete) {
    stepper->setCurrentPosition(0);
  } else if (firstRotationComplete) {
    stepsPerRotation = stepper->getCurrentPosition();
    Serial.println("steps per rotation");
    Serial.print("\t\t");
    Serial.print(stepsPerRotation);
    Serial.println("\n");
  }
}

void needleDownInterrupt() {
  // debounce
  if(micros() - last_micros > 400) {
    foundHome = true;
    if (!firstHomeFound) {
      firstHomeFound = true;
    } else if (!firstRotationComplete) {
      firstRotationComplete = true;
    }
  }
}
