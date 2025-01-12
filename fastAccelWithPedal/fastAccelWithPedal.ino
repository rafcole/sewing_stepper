#include "FastAccelStepper.h"
#include "AVRStepperPins.h" // Only required for AVR controllers
#include <Encoder.h>


// // Mega
// #define stepPinStepper   6 // must be 6, 7, 8 for MEGA 2560
// #define dirPinStepper    7
// #define enablePinStepper 8


// Uno R3
#define dirPinStepper    5
#define enablePinStepper 6
#define stepPinStepper   9

// external io
#define pedalPin A1

//
int driverMicroSteps = 3200;
int stepperSpecsStepsPerRevolution = 200;
int microStepsPerRevolution = driverMicroSteps/stepperSpecsStepsPerRevolution;

// pedal ranges from 3.5-5v 
unsigned int pedalPinMin = 350;
unsigned int pedalPinMax = 10;

unsigned int minSpeedInHz = 5000; // 1700 good
unsigned int maxSpeedInHz = 49000;
unsigned int homingSpeed = 7000;

// not accessible in setup?
unsigned int absAccelerationRate = 40000;
unsigned int linAccelRate = 10000;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

// debounce
int debounceCounterMicros = 0;
long last_micros = 0;

// encoder
int encoderPinA = 3;
int encoderPinB = 2;
long oldPosition  = -999;
int encoderPulsesPerRotation = 2400;

Encoder myEnc(encoderPinA, encoderPinB);

// button press logic tester
// normally closed momentary
int needleDownSensorPin = 4;
// volatile bool lastMoveHomed = false;
volatile bool foundHome = false;
volatile bool firstHomeFound = false;
volatile bool machineIsHoming = false;


long stepsPerRotation = 0;
volatile bool firstRotationComplete = false;
volatile bool makingFirstRotation = false;
long firstHomeTimeStamp = 0;
long secondHomeTimeStamp = 0;
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
  Serial.begin(115200);

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
  pinMode(needleDownSensorPin, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(needleDownSensorPin), needleDownInterrupt, RISING);

  // default value gets erased
  foundHome = false;
}

void loop() {
  // if (stepper->isRunning()) {
  //   Serial.print("Motor running -> Current position: ");
  //   Serial.println(stepper->getCurrentPosition());
  // }

  int pedalReading = analogRead(pedalPin);
  // Serial.println(pedalReading);
  bool atZero = digitalRead(needleDownSensorPin);

  unsigned int pedalSpeedInHz = map(pedalReading, pedalPinMax, pedalPinMin, minSpeedInHz, maxSpeedInHz);
  // Serial.println(pedalSpeedInHz);



  bool pedalUp = pedalSpeedInHz < minSpeedInHz;
  bool pedalDown = pedalSpeedInHz >= minSpeedInHz;

  // foundHome = digitalRead(needleDownSensorPin);

  if (pedalUp && stepper->isRunning() && firstRotationComplete) {
    machineIsHoming = true;
    Serial.println("Machine is homing set to true");
  }

  if (!stepsPerRotation && atZero){
    Serial.println("found home");
    setStepsPerRotation();
    foundHome = false;
  }

  // no homing information
  // default to stopping
  if (pedalUp) {
    if (stepsPerRotation && machineIsHoming) {
      // TODO
      // HOMING
      // set to homing speed
      int stepsOfRotationCompleted = stepper->getCurrentPosition() % stepsPerRotation;
      stepper->setCurrentPosition(0);
      stepper->moveTo(stepsOfRotationCompleted, true);
      // stepper->forceStop();

      Serial.print("Steps of rotation: ");
      Serial.print(stepsOfRotationCompleted);
      Serial.println();

      // this presumes that the blocking call of moveTo wins out?
      machineIsHoming = false;

    } else if (stepper->isRunning() && !stepsPerRotation) {
      stepper->forceStop();
      Serial.println("No spr data, forced stop");
    }
  } else {
    if (foundHome) {
      stepper->setCurrentPosition(0);
      Serial.println("Found home while running continuously");
    }
    // Pedal down
    // run as normal
    // Serial.println("running as normal");
    stepper->setSpeedInHz(pedalSpeedInHz);
    stepper->runForward();
  }
}

void setStepsPerRotation() {
 if (foundHome && firstHomeFound && !firstHomeTimeStamp) {
  firstHomeTimeStamp = millis();
  stepper->setCurrentPosition(0);
 } else if (foundHome && firstRotationComplete && !secondHomeTimeStamp) {
  secondHomeTimeStamp = millis();
  stepsPerRotation = stepper->getCurrentPosition();
  Serial.println("Completed first rotation");
  Serial.print("\tsteps per rotation : ");
  Serial.println(stepsPerRotation);
 }
}

void needleDownInterrupt() {
  // debounce
  if(micros() - last_micros > 800) {
    foundHome = true;
    if (!firstHomeFound) {
      firstHomeFound = true;
      makingFirstRotation = true;
    } else if (firstHomeFound && !firstRotationComplete) {
      firstRotationComplete = true;
      makingFirstRotation = false;
    }
  }
  last_micros = micros();
}

