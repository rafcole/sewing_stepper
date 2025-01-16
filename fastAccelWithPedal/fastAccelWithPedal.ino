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
unsigned int linAccelRate = 1000;

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
float pulsesPerDegree = float (encoderPulsesPerRotation) / 360;

float stepsPerDegree = 5712.00 / 360;

Encoder myEnc(encoderPinA, encoderPinB);

// button press logic tester
// normally closed momentary
int needleDownSensorPin = 4;
// volatile bool lastMoveHomed = false;
volatile bool foundHome = false;
volatile bool firstHomeFound = false;
volatile bool machineIsHoming = false;
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

  lastZero = !digitalRead(needleDownSensorPin);
}

long pulsesTravelled = 0;

void loop() {
  int pedalReading = updatePedalReading(analogRead(pedalPin));

  bool atZero = !digitalRead(needleDownSensorPin);
  bool zeroRising = atZero && !lastZero;
  lastZero = atZero;

  pulsesTravelled += myEnc.read() - pulsesTravelled;

  if (zeroRising) {
    myEnc.write(0);
    // Serial.print("encoder pos:");
    // Serial.println(myEnc.read());
    stepper->setCurrentPosition(0);
  }

  bool pedalUp = pedalReading < pedalPinMin - 5;

  unsigned int pedalSpeedInHz;

  if (pedalUp) {
    pedalSpeedInHz = 0;
  } else {
     pedalSpeedInHz = map(pedalReading, pedalPinMin, pedalPinMax, minSpeedInHz, maxSpeedInHz);
  }

  machineIsHoming = debounceHoming(pedalUp) && degreesTravelled() > 40 && !(stepper->isStopping());

  Serial.print("\n\nDegrees travelled: ");
  Serial.println(degreesTravelled());

  bool machineIsHomingRising = machineIsHoming && !lastPedalUp ;

  lastPedalUp = pedalUp;

  // no homing information
  // default to stopping
  if (pedalUp && !(stepper->isStopping())) {


    Serial.println("Setting homing speed");
    stepper->setSpeedInHz(homingSpeed);

    int pulsesSoFar = myEnc.read();
    int pulsesToGo = 2400 - pulsesSoFar;

    int rawDegrees = pulsesSoFar / pulsesPerDegree;

    int currentAngle = rawDegrees % 360;
    int degreesToZero = 360 - currentAngle;

    Serial.print("Current angle: ");
    Serial.println(currentAngle);

    int degreesToNeedleDown = degreesToZero + keyPositions.needleDown;
    int degreesToNextValidStopPos = degreesToNeedleDown % 180;
    
    int stepsToTravel = degreesToNextValidStopPos * stepsPerDegree;
    Serial.print("Steps to Travel: ");
    Serial.println(stepsToTravel);

    stepper->setCurrentPosition(0);

    if (stepsToTravel > 100) {
      stepper->moveTo(stepsToTravel, true); // blocking defeats zero rising sensor
      while(stepper->getCurrentPosition() != stepsToTravel) {

      }
      Serial.println("confirm non blocking"); // blocking is actually blocking, not just a flag
      stepper->forceStop();

      delay(1000);

      pulsesTravelled = 0;
      delay(300);
    }
  } else {
    Serial.println("else branch");
    stepper->setSpeedInHz(pedalSpeedInHz);
    stepper->runForward();
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

int updatePedalReading(int pedalReading) {
  // TODO change to bit shift

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

