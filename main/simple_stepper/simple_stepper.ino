// testing a stepper motor with a Pololu A4988 driver board or equivalent
// on an Uno the onboard led will flash with each step
// this version uses delay() to manage timing

byte directionPin = 4;
byte stepPin = 3;
int numberOfSteps = 5000;
byte ledPin = 13;
int pulseWidthMicros = 1;  // microseconds
int millisbetweenSteps = 1; // milliseconds - or try 1000 for slower steps


void setup() { 

  Serial.begin(9600);
  Serial.println("Starting StepperTest");
  digitalWrite(ledPin, LOW);
  
  delay(2000);

  pinMode(directionPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  
 
  digitalWrite(directionPin, HIGH);
  for(int n = 0; n < numberOfSteps; n++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(pulseWidthMicros); // this line is probably unnecessary
    digitalWrite(stepPin, LOW);
    
    delay(millisbetweenSteps);
    
    digitalWrite(ledPin, !digitalRead(ledPin));
  }
  
  delay(3000);
  

  digitalWrite(directionPin, LOW);
  for(int n = 0; n < numberOfSteps; n++) {
    digitalWrite(stepPin, HIGH);
    // delayMicroseconds(pulseWidthMicros); // probably not needed
    digitalWrite(stepPin, LOW);
    
    delay(millisbetweenSteps);
    
    digitalWrite(ledPin, !digitalRead(ledPin));
  }
}

void loop() { 
}