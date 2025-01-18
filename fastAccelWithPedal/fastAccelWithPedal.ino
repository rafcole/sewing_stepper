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
unsigned int maxSpeedInHz = 40000; // bog at 45k
unsigned int homingSpeed = 7000;

// not accessible in setup?
unsigned int absAccelerationRate = 30000;
unsigned int linAccelRate = 200;

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
// 6.66... for 600 resolution encoder
float pulsesPerDegree = float (encoderPulsesPerRotation) / float(360);

float stepsPerDegree = 6250.00 / 360;

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
volatile unsigned long millisLastZero; 
volatile bool lastPedalUp = false;
volatile long pulsesAtLastZero;

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
volatile int maxSteps = 0;
volatile long pulsesTravelled = 0;

#define thumbButtonPin 12

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

  pinMode(thumbButtonPin, INPUT);

  millisLastZero = millis();
  stepper->setForwardPlanningTimeInMs(16);
}

void loop() {
  bool thumbButtonPressed = digitalRead(thumbButtonPin);

  if (thumbButtonPressed) {
    Serial.println("ButtonPress");
  }

  Serial.println(maxSteps);
  Serial.println(getCurrentDegrees());
  delay(40);
  int pedalReading = updatePedalReading();

  updatePulsesTravelled();

  if (foundZero) {
    stepper->setCurrentPosition(0);
  }

  bool pedalUp = pedalReading < pedalPinMin - 5;

  unsigned int pedalSpeedInHz;

  if (pedalUp) {
    pedalSpeedInHz = 0;
  } else {
    pedalSpeedInHz = map(pedalReading, pedalPinMin, pedalPinMax, minSpeedInHz, maxSpeedInHz);
  }

  machineIsHoming = debounceHoming(pedalUp) && degreesTravelled() > 40 || thumbButtonPressed;


  // no homing information
  // default to stopping
  if (!stepper->isRunning() && pedalUp) {
    machineIsSewing = false;
    machineIsHoming = false;
    machineIsHomingToggle = false;
    pulsesTravelled = 0;
    stepper->forceStop();
  } else if (machineIsHoming && !machineIsHomingToggle) {

    // Serial.println("Setting homing speed");
    // Serial.println(stepper->getAcceleration());
    stepper->setAcceleration(100000);
    stepper->setLinearAcceleration(80);
    stepper->setSpeedInHz(homingSpeed);
    stepper->applySpeedAcceleration();
    stepper->keepRunning();
    
    int currentSpeed = pedalReading;
    // int currentSpeed = stepper->getCurrentSpeedInMilliHz() / 1000;
    if ( pedalReading < maxSpeedInHz * .5 ) {
      if (foundZero) {
        // Serial.print("destination: ");
        // Serial.println(stepper->getPositionAfterCommandsCompleted());
        machineIsHomingToggle = true;

        stepper->forceStop();
        machineIsSewing = false;
        stepper->setAcceleration(absAccelerationRate);
        stepper->applySpeedAcceleration();
      }
      // good needle down routine
       else {
        int currentDegreePosition = getCurrentDegrees();
        int currentStepperPosition = stepper->getCurrentPosition();
        int stepperDestination = stepper->getPositionAfterCommandsCompleted();

        int stepperDestinationDeltaDegrees = (stepperDestination - currentStepperPosition) / stepsPerDegree;

        // go to needle up
        int degreesToAdd = 220 - currentDegreePosition - stepperDestinationDeltaDegrees;
        int stepsToAdd = degreesToAdd * stepsPerDegree;

        stepper->move(stepsToAdd, true);
        stepper->forceStop();
      } 
    }







    // stepper->stopMove();
    // stepper->forceStopAndNewPosition(0);
    // stepper->move(2000);

    // if (stepper->getCurrentSpeedInMilliHz() / 1000 > maxSpeedInHz * .66) {
    //   stepper->move(stepsPerRotation / 2, true);
    // } else {
    //   stepper->forceStop();
    // }

    int degreesToZero = getDegreesToZero();
    int degreesToNeedleDown = degreesToZero + keyPositions.needleDown;
    // int degreesToNeedleUp = degreesToZero + keyPositions.needleUp;
    int degreesToNextValidStopPos = degreesToNeedleDown % 180;

    // false rising edge accounts for ~15 degrees of this error
    // remainder probably momentum
    int degreesErrorCorrection = 0;
    // while (getCurrentDegrees() != keyPositions.needleDown - degreesErrorCorrection && getCurrentDegrees() != keyPositions.needleUp - degreesErrorCorrection) {
    //   Serial.println("stuck here?");
    // }
 

  } else if (pedalReading > pedalPinMin && !machineIsHoming) {
    machineIsSewing = true;
    machineIsHomingToggle = false;
    machineIsHoming = false;

    // Serial.println("else branch");
    stepper->setSpeedInHz(pedalSpeedInHz);
    // Serial.print("return value for runForward(): ");
    // Serial.println(s);
    stepper->runForward();
    // stepper->keepRunning();
        Serial.print("destination: ");
        Serial.println(stepper->getPositionAfterCommandsCompleted());
        Serial.print("Current pos:");
        Serial.println(stepper->getCurrentPosition());
  }

  foundZero = false;
  delayMicroseconds(100);
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

  return pulsesSoFar / pulsesPerDegree;
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

void updatePulsesTravelled() {
  long encoderReading = myEnc.read();
  pulsesTravelled += encoderReading - lastEncoderReading;
  lastEncoderReading = encoderReading;
}

void needleDownInterrupt() {
  // debounce false positive from needle down detector
  // didn't work still getting error
  if (myEnc.read() > 500) {
    // Serial.println("single zero");
    millisLastZero = millis();
    long encoderReading = myEnc.readAndReset();

    // hand off passing over zero
    if (lastEncoderReading > 2390) {
      pulsesTravelled += (2400 - lastEncoderReading);
    }

    int stepperPosition = stepper->getCurrentPosition();

    if (stepperPosition > maxSteps) {
      maxSteps = stepperPosition;
    }
    
    // not accounting for going opposite direction because that would be done by hand, not during sewing
    lastEncoderReading = 0;

    foundZero = true;
  }
  
}

