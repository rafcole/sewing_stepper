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


#define zeroSensorPin 4

#define encoderPinA 3
#define encoderPinB 2
long encoderPulsesPerRotation = 2400;
long oldPosition = 0;
Encoder myEnc(encoderPinA, encoderPinB);

#define stepsPerRotation 5712

float stepsPerPulse = 2.38;

// 10 degrees
int minPulsesThreshold = 66;
int minStepsThreshold = 500;

long timerCounter = 0;


void setup() {
  pinMode(zeroSensorPin, INPUT_PULLUP);

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
    stepper->setLinearAcceleration(1000);
    stepper->setSpeedInHz(45000);  // the parameter is us/step !!!
    stepper->setAcceleration(20000);


    // approaching 2.38?
    // doesn't accout for asymptotic decay
    // 5712
    // int stepsPerRotation = encoderPulsesPerRotation * 2.38;
    int chatGPTSPR = 7677;


    // asymptotic decay


    // long testVals[] = {stepsPerRotation};
    // long testVals[] = {chatGPTSPR};
    long testVals[] = {100, 200, 300, 400, 500, 600, 750, 1000, 1250, 1500, 2000, 3000, 10000, 50000, 100000}; 

    int numTests = sizeof(testVals) / sizeof(long);
    // runTest(testVals, numTests);

    // goToZero();
    // manualIntervention();
  }
}

void loop() {
  if (millis() - timerCounter > 3000) {
    timerCounter = millis();
    manualIntervention();
  }
  if (!digitalRead(zeroSensorPin)) {
    myEnc.write(0);
    Serial.println("Passed zero");
  }
}

void goToZero(long offset) {
  // stepper->moveTo(6054, true);
  // delay(300);


  Serial.print("\n\n\n\nencoder position: ");
  Serial.print(myEnc.read());
  Serial.println();

  Serial.print("offset : ");
  Serial.println(offset);
  Serial.println();

  myEnc.write(offset);

  long pulsesToMove = encoderPulsesPerRotation - offset;

  if (pulsesToMove >= encoderPulsesPerRotation) {
    pulsesToMove = 0;
  }

  long stepsToMove = stepsPerPulse * pulsesToMove;

  if (stepsToMove > minStepsThreshold) {
    Serial.print("Steps to move: ");
    Serial.println(stepsToMove);

    stepper->move(stepsToMove);
  }

}

void manualIntervention() {
  long offset = myEnc.read() % 2400;
  goToZero(offset);

}

void runTest(long testValues[], int numTests) {

  for (int idx = 0; idx < numTests; idx++) {
    long currentNumSteps = testValues[idx];

    stepper->move(currentNumSteps, true);

    long numPulses = myEnc.readAndReset();
    float stepsPerPulse = currentNumSteps / float (numPulses);

    Serial.print("\nPulses per ");
    Serial.print(currentNumSteps);
    Serial.print(" steps: ");
    Serial.println(numPulses);

    Serial.print("Steps per pulse: ");
    Serial.println(stepsPerPulse);

    delay(300);
  }
}
