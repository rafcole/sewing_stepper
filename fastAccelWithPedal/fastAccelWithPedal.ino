#include "FastAccelStepper.h"
#include "AVRStepperPins.h" // Only required for AVR controllers
#include <Encoder.h>

#include "AcksenButton.h"


// ==========================
//      Select your controller
// ==========================

// // Mega
// #define stepPinStepper   6 // must be 6, 7, 8 for MEGA 2560
// #define dirPinStepper    7
// #define enablePinStepper 8

// Uno R3
#define dirPinStepper    5
#define enablePinStepper 6
#define stepPinStepper   9

// ==========================
//      external i/o
// ==========================
#define pedalPin A1
#define needleDownSensorPin 2
#define thumbButtonPin 12

int thumbButtonDebounce = 80;
AcksenButton thumbButton = AcksenButton(thumbButtonPin, ACKSEN_BUTTON_MODE_NORMAL, thumbButtonDebounce, INPUT);

// pedal ranges from 3.5-5v 
unsigned int pedalPinMax = 290;
unsigned int pedalPinMin = 20;

// ==========================
//     Machine Settings
// ==========================

unsigned int minSpeedInHz = 5000; // 1700 good
unsigned int maxSpeedInHz = 40000; // bog at 45k
unsigned int homingSpeed = 7000;

// not accessible in setup?
unsigned int absAccelerationRate = 30000;
unsigned int linAccelRate = 200;

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

unsigned int reservedSingleStitchMin = 20;
unsigned int reservedSingleStitchMax = 40;

// ==========================
//        Stepper
// ==========================

int driverMicroSteps = 3200;
int stepperSpecsStepsPerRevolution = 200;
int microStepsPerRevolution = driverMicroSteps/stepperSpecsStepsPerRevolution;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

// debounce
int debounceCounterMicros = 0;
long last_micros = 0;

// ==========================
//          Encoder
// ==========================
int encoderPinA = 3;
int encoderPinB = 4;
long oldPosition  = -999;
int encoderPulsesPerRotation = 2400;
// 6.66... for 600 resolution encoder
float pulsesPerDegree = float (encoderPulsesPerRotation) / float(360);
float stepsPerDegree = 6250.00 / 360;

Encoder myEnc(encoderPinA, encoderPinB);

// ==========================
//        Logic Flags &
//          Counters
// ==========================

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
int lastDegreesTravelled = 0;



// ==========================================   
//  #####  ####### ####### #     # ######  
// #     # #          #    #     # #     # 
// #       #          #    #     # #     # 
//  #####  #####      #    #     # ######  
//       # #          #    #     # #       
// #     # #          #    #     # #       
//  #####  #######    #     #####  #       
// ==========================================      

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

  millisLastZero = millis();
  stepper->setForwardPlanningTimeInMs(16);
}

// ==========================================           
//  ##       #######  #######  ####### 
//  ##            ##       ##       ## 
//  ##       ##   ##  ##   ##  ####### 
//  ##   ##  ##   ##  ##   ##  ##      
//  #######  #######  #######  ##   
// ==========================================   


void loop() {
  // Serial.println(millis());
  // Serial.println(millis() % 4 == 0);
  // if (millis() % 1000 == 0) {
  //   Serial.print("Current Degrees: ");
  //   Serial.println(getCurrentDegrees());
  // }
  
  updatePulsesTravelled();
  thumbButton.refreshStatus();
  resetStepperIfFoundZero();

  bool thumbButtonHigh = thumbButton.getButtonState();
  bool thumbButtonFalling = thumbButton.onReleased();
  
  int pedalReading = updatePedalReading();
  bool pedalUp = pedalReading < pedalPinMin - 5;
  unsigned int pedalSpeedInHz = findPedalSpeedInHz(pedalUp, pedalReading);

  machineIsHoming = (machineIsSewing && debounceHoming(pedalUp) && !thumbButtonHigh) && (degreesTravelled() > 40);

  // ==========================
  //      Main control loop
  // ==========================
  if (thumbButtonHigh && !stepper->isRunning()) { // Thumb button pressed
    machineIsSewing = true;
    machineIsHoming = true;

    // thumb button was able to sneak in a state change after homing burn down but before the motor is disabled
    // which caused a latching effect
    machineIsHomingToggle = false;

    stepper->setSpeedInHz(minSpeedInHz);
    stepper->runForward();

  } else if (!stepper->isRunning() && pedalUp && !thumbButtonHigh && machineIsSewing) { // End of sewing
    // Serial.println("End of sewing branch");
    machineIsSewing = false;
    machineIsHoming = false;
    machineIsHomingToggle = false;

    pulsesTravelled = 0;

    stepper->forceStopAndNewPosition(0);

  } else if (machineIsHoming && !machineIsHomingToggle) { // Homing
    // Serial.println("\n\n=======================================\n");

    // Serial.println("Homing branch");
    stepper->setAcceleration(100000);
    stepper->setLinearAcceleration(80);
    stepper->setSpeedInHz(homingSpeed);
    stepper->applySpeedAcceleration();
    stepper->keepRunning();
    
    // if the speed is too high, let the loop execute as long as it needs to
    if (stepper->getCurrentSpeedInMilliHz() / 1000 < homingSpeed * 1.3) {
      // now that the speed is workable, we're going to perform a homing operation
      // the homing toggle will ensure that we don't run multiple homing operations which ends up in PID esque feedback loop

      int currentPosition = getCurrentDegrees();

      int nextValidStopPosition = keyPositions.needleDown;

      if (currentPosition < keyPositions.needleDown || currentPosition > keyPositions.needleUp) {
        nextValidStopPosition = keyPositions.needleDown;
      } else {
        nextValidStopPosition = keyPositions.needleUp;
      }

      int offset = -10;
      while(getCurrentDegrees() != nextValidStopPosition + offset && updatePedalReading() < pedalPinMin) {
        // burn cycles until reached stop position
        if (millis() % 3 == 0){
          Serial.print("Homing burn down: ");
          Serial.println(getCurrentDegrees());
        }
      }

      // if the user hasn't pressed pedal or button, stop
      // otherwise reset flag and take a lap
      if(updatePedalReading() < pedalPinMin && !thumbButtonHigh) {
        stepper->forceStopAndNewPosition(0);
        machineIsHomingToggle = true;
      } else {
        machineIsHomingToggle = false;
      }

    } else if (pedalReading > pedalPinMin && !machineIsHoming) { // normal sewing
      machineIsSewing = true;
      machineIsHomingToggle = false;

      stepper->setAcceleration(absAccelerationRate);
      stepper->setSpeedInHz(pedalSpeedInHz);
      stepper->runForward();
    }
    foundZero = false;
    delayMicroseconds(100);
  }
}

int findPedalSpeedInHz(bool pedalUp, int pedalReading) {
  int pedalSpeedInHz;

  if (pedalUp) {
    pedalSpeedInHz = 0;
  } else {
    pedalSpeedInHz = map(pedalReading, pedalPinMin, pedalPinMax, minSpeedInHz, maxSpeedInHz);
  }

  return pedalSpeedInHz;
}

void resetStepperIfFoundZero() {
  if (foundZero) {
    stepper->setCurrentPosition(0);
  }
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

