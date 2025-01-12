int sensorPin = 18;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(sensorPin, INPUT_PULLUP);
}

void loop() {
  // put your main code here, to run repeatedly:
  bool state = digitalRead(sensorPin);

  Serial.println(state);
}
