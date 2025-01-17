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

// encoder

// key points in the stitch cycles
struct MachinePositions {
  int bottomDeadCenter;
  int needleDown;       // the point at which the hook has captured the needle thread
  int needleExit;
  int topDeadCenter;
  int needleUp;         // when the take up lever has reached apex
  int needleEnter;      // important to track esp re:needle strikes on the throat plate
};

struct MachinePositions keyPositions = {0, 50, 85, 180, 240, 275};

// pedal ranges from 3.5-5v 
unsigned int pedalPinMax = 290;
unsigned int pedalPinMin = 20;

unsigned int reservedSingleStitchMin = 20;
unsigned int reservedSingleStitchMax = 40;

unsigned int minSpeedInHz = 5000; // 1700 good
unsigned int maxSpeedInHz = 30000; // bog at 45k
unsigned int homingSpeed = 7000;

// not accessible in setup?
unsigned int absAccelerationRate = 40000;
unsigned int linAccelRate = 100;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

// debounce
int debounceCounterMicros = 0;
long last_micros = 0;

// encoder
int encoderPinA = 3;
int encoderPinB = 4;
long oldPosition  = -999;
int encoderPulsesPerRotation = 2400;
float pulsesPerDegree = float (encoderPulsesPerRotation) / 360;

float stepsPerDegree = 5712.00 / 360;

Encoder myEnc(encoderPinA, encoderPinB);

// button press logic tester
// normally closed momentary
int needleDownSensorPin = 2;
// volatile bool lastMoveHomed = false;
volatile bool foundZero = false;
volatile bool foundHome = false;
volatile bool firstHomeFound = false;
volatile bool machineIsHoming = false;
volatile bool machineIsHomingToggle = false;
volatile bool machineIsSewing = false;
volatile bool lastZero = false;
volatile bool lastPedalUp = false;

unsigned long pedalReadCount = 0;
unsigned long pedalReadTotal = 0;
float pedalReadAvg = 0;
int pedalReadMax = 0;

// sampling of pedal signals to quash stray signals on the pedal
int pedalSamples[5] = {0};
bool homingSamples[10] = {false};

long stepsPerRotation = 0;
volatile bool firstRotationComplete = false;
volatile bool makingFirstRotation = false;
long firstHomeTimeStamp = 0;
long secondHomeTimeStamp = 0;
volatile int lastEncoderReading = 0;

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
  attachInterrupt(digitalPinToInterrupt(needleDownSensorPin), needleDownInterrupt, RISING);
  lastZero = !digitalRead(needleDownSensorPin);
}

long pulsesTravelled = 0;

void updatePulsesTravelled() {
  long encoderReading = myEnc.read();
  pulsesTravelled += encoderReading - lastEncoderReading;
  lastEncoderReading = encoderReading;
}

void loop() {
  Serial.println(getCurrentDegrees());
  delay(40);
  int pedalReading = updatePedalReading();

  updatePulsesTravelled();

  bool pedalUp = pedalReading < pedalPinMin - 5;

  unsigned int pedalSpeedInHz;

  if (pedalUp) {
    pedalSpeedInHz = 0;
  } else {
    pedalSpeedInHz = map(pedalReading, pedalPinMin, pedalPinMax, minSpeedInHz, maxSpeedInHz);
  }

  machineIsHoming = debounceHoming(pedalUp) && degreesTravelled() > 40;


  // no homing information
  // default to stopping
  if (!stepper->isRunning() && pedalUp) {
    machineIsSewing = false;
    machineIsHoming = false;
    machineIsHomingToggle = false;
    pulsesTravelled = 0;
    stepper->forceStop();
  } else if (machineIsHoming && !machineIsHomingToggle) {
    machineIsHomingToggle = true;
    Serial.println("Setting homing speed");
    Serial.println(stepper->getAcceleration());
    stepper->setAcceleration(10000000);
    stepper->setLinearAcceleration(10);
    stepper->setSpeedInHz(homingSpeed);
    stepper->applySpeedAcceleration();
    stepper->keepRunning();

    // stepper->stopMove();
    // stepper->forceStopAndNewPosition(0);
    // stepper->move(2000);

    int degreesToZero = getDegreesToZero();
    int degreesToNeedleDown = degreesToZero + keyPositions.needleDown;
    // int degreesToNeedleUp = degreesToZero + keyPositions.needleUp;
    int degreesToNextValidStopPos = degreesToNeedleDown % 180;

    // false rising edge accounts for ~15 degrees of this error
    // remainder probably momentum
    int degreesErrorCorrection = 20;
    while (getCurrentDegrees() != keyPositions.needleDown - degreesErrorCorrection && getCurrentDegrees() != keyPositions.needleUp - degreesErrorCorrection) {

    }
    
    stepper->forceStopAndNewPosition(0);

    delay(300);
    Serial.println(getCurrentDegrees());
    
    // int stepsToTravel = degreesToNextValidStopPos * stepsPerDegree;

    // // arbitrary thresholding for when the machine wants to run in reverse because it overshot
    // // due to hand wheel momentum
    // Serial.print("Steps to Travel: ");
    // Serial.println(stepsToTravel);

    // // reset the steppers current op
    // Serial.println(stepper->getCurrentSpeedInMilliHz(false) / 1000);

    // int stepperSpeedInHz = ((stepper->getCurrentSpeedInMilliHz(false) / 1000) > (maxSpeedInHz * .66));

    // int overshoot = 0;

    // if (stepperSpeedInHz > (maxSpeedInHz * 33)) {
    //   overshoot += 180;
    // } else if (stepperSpeedInHz > (maxSpeedInHz * 66)) {
    //   overshoot += 360;
    // }

    // // more gentle halting
    // if (overshoot) {
    //   Serial.println("overshoot logic");
    //   stepsToTravel += overshoot;
    //   // stepper->forceStopAndNewPosition(0);
    //   // TODO could force stop, recalculate and then move, not so different than how industrial machines hit the brakes
    // } else {
    //   Serial.println("normal logic branch");
    //   // stepper->setCurrentPosition(0);
    // }

    // // stepper->setCurrentPosition(0);
    // // stepper->forceStopAndNewPosition(0);

    // // degrees travel check might make this check obsolete
    // if (stepsToTravel > 100) {
    //   Serial.print("steps to move");
    //   Serial.println(stepsToTravel);
    //   stepper->move(stepsToTravel); // blocking defeats zero rising sensor
    //   // delay(1000);
    // }
    stepper->setAcceleration(absAccelerationRate);
    stepper->applySpeedAcceleration();
  } else if (pedalReading > pedalPinMin && !machineIsHoming) {
    machineIsSewing = true;
    machineIsHomingToggle = false;
    machineIsHoming = false;

    Serial.println("else branch");
    stepper->setSpeedInHz(pedalSpeedInHz);
    Serial.print("return value for runForward(): ");
    Serial.println(stepper->runForward());
    stepper->keepRunning();
  }
  // delayMicroseconds(100);
}

int degreesTravelled() {
  if (pulsesTravelled == 0) {
    return 0;
  } else {
    return pulsesTravelled / pulsesPerDegree;
  }
}

int getCurrentDegrees() {
  int pulsesSoFar = myEnc.read();
  int pulsesToGo = 2400 - pulsesSoFar;

  int rawDegrees = pulsesSoFar / pulsesPerDegree;

  int currentAngle = rawDegrees % 360;

  return currentAngle;
}

int getDegreesToZero() {
  return 360 - getCurrentDegrees();
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



bool debounceHoming(bool pedalStatus) {

  int numSamples = sizeof(homingSamples) / sizeof(homingSamples[0]);

  bool allTrueFlag = true;

  for (int i = numSamples - 1; i > 0; i--) {
    homingSamples[i] = homingSamples[i-1];
    if (allTrueFlag && !homingSamples[i]) {
      allTrueFlag = false;
    }
  }

  homingSamples[0] = pedalStatus;

  return pedalStatus;
}

int updatePedalReading() {
  // TODO change to bit shift

  int pedalReading(analogRead(pedalPin));
  int numSamples = sizeof(pedalSamples) / sizeof(pedalSamples[0]);

  int samplesTotal = 0;

  for (int i = numSamples - 1; i > 0; i--) {
    pedalSamples[i] = pedalSamples[i-1];
    samplesTotal += pedalSamples[i];
  }

  pedalSamples[0] = pedalReading;
  samplesTotal += pedalSamples[0];

  float avg = float(samplesTotal) / numSamples;

  // Serial.print("SampledAvg:");
  // Serial.print(avg);
  // Serial.print(",");
  // if (avg > pedalReadMax) {
  //   pedalReadMax = avg;
  // }

  // Serial.print("Max:");
  // Serial.println(pedalReadMax);

  // -2 offset is to catch false positives
  // TODO globalize
  return avg;
}


void needleDownInterrupt() {
  long encoderReading = myEnc.readAndReset();

  // hand off passing over zero
  if (lastEncoderReading > 2390) {
    pulsesTravelled += (2400 - lastEncoderReading);
  }
  // not accounting for going opposite direction because that would be done by hand, not during sewing
  lastEncoderReading = 0;

  foundZero = true;
  stepper->setCurrentPosition(0);
}

