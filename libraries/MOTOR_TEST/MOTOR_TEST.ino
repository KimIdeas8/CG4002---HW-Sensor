
void setup() {
  // put your setup code here, to run once:
  pinMode(6, OUTPUT);
    pinMode(5, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(6, 217);
    analogWrite(10, 217);
  analogWrite(5, 217);
  analogWrite(11, 217);

}
