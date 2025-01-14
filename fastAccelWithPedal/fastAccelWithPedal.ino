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
unsigned int pedalPinMin = 14;

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
  lastZero = !digitalRead(needleDownSensorPin);
}

void loop() {
  int pedalReading = analogRead(pedalPin);



  // pedalReadCount++;
  // pedalReadTotal += pedalReading;
  // pedalReadAvg = pedalReadTotal / pedalReadCount;
  // if (pedalReading > pedalReadMax) {
  //   pedalReadMax = pedalReading;
  // }
  // if (millis() % 10 == 0) {
  //   Serial.print("pedal_reading:");
  //   Serial.print(pedalReading);
  //   Serial.print(",");
  //   Serial.print("Max_reading:");
  //   Serial.print(pedalReadMax);
  //   Serial.print(",");
  //   Serial.print("Avg_reading:");
  //   Serial.println(pedalReadAvg);
  // }


  // Serial.println(stepper->getCurrentPosition());
  // delay(500);
  bool atZero = !digitalRead(needleDownSensorPin);
  bool zeroRising = atZero && !lastZero;
  lastZero = atZero;

  if (zeroRising) {
    // Serial.print(stepper->getCurrentPosition());
    // Serial.println("Found Zero");
    stepper->setCurrentPosition(0);
  }

  bool pedalUp = detectPedalUp(pedalReading);

  unsigned int pedalSpeedInHz = map(pedalReading, pedalPinMin, pedalPinMax, minSpeedInHz, maxSpeedInHz);
  // Serial.println(pedalSpeedInHz);



  
  // bool pedalDown = pedalSpeedInHz >= minSpeedInHz;
  if (!pedalUp) {
    // Serial.print("PedalUp: ");
    // Serial.println(pedalUp);
    // Serial.println(pedalReading);
  }

  // Serial.print("PedalUp: ");
  // Serial.println(pedalUp);
  // Serial.println(pedalReading);
  // delay(300);

  machineIsHoming = pedalUp && stepper->isRunning();
  bool machineIsHomingRising = machineIsHoming && !lastPedalUp;
  lastPedalUp = pedalUp;

  // no homing information
  // default to stopping
  if (pedalUp) {
    if (zeroRising) {
      stepper->forceStopAndNewPosition(0);
      delay(500);
    } else if (machineIsHomingRising) {
      // Serial.println("Setting homing speed");
      stepper->setSpeedInHz(homingSpeed);
      stepper->move(5900);
      // stepper->forceStopAndNewPosition(5700);

      // Serial.println(stepper->getCurrentPosition());
    }
  } else {
    stepper->setSpeedInHz(pedalSpeedInHz);
    stepper->runForward();
  }
  delayMicroseconds(100);
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

bool detectPedalUp(int pedalReading) {
  int numSamples = sizeof(pedalSamples) / sizeof(pedalSamples[0]);

  int samplesTotal = 0;

  for (int i = numSamples - 1; i > 0; i--) {
    pedalSamples[i] = pedalSamples[i-1];
    samplesTotal += pedalSamples[i];
  }

  pedalSamples[0] = pedalReading;
  samplesTotal += pedalSamples[0];

  float avg = float(samplesTotal) / numSamples;

  Serial.print("SampledAvg:");
  Serial.print(avg);
  Serial.print(",");
  if (avg > pedalReadMax) {
    pedalReadMax = avg;
  }

  Serial.print("Max:");
  Serial.println(pedalReadMax);

  return (avg < pedalPinMin);
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

