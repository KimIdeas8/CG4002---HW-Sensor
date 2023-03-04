#include <IRremote.hpp>
#define IR_RECEIVE_PIN 0

int points = 100;
int digitPins[] = {A1,A2,A3};
int segPins[] = {5,3,A5,A4,2,4,A0};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  IrReceiver.begin(IR_RECEIVE_PIN); // Start the receiver

  //SET LED SEGMENT PINS AS OUTPUTS:
  for (int i = 0; i < 3; i++) 
  {pinMode(digitPins[i], OUTPUT);}
  for (int i = 0; i < 7; i++) 
  {pinMode(segPins[i], OUTPUT);}
}

void displayPoints(int points){
  if (points==100){
      digitalWrite(digitPins[0], LOW);
      digitalWrite(digitPins[1], HIGH);
      digitalWrite(digitPins[2], HIGH);
      
      digitalWrite(segPins[0], LOW);
      digitalWrite(segPins[1], HIGH);
      digitalWrite(segPins[2], HIGH);
      digitalWrite(segPins[3], LOW);
      digitalWrite(segPins[4], LOW);
      digitalWrite(segPins[5], LOW);
      digitalWrite(segPins[6], LOW);
      delay(1);

      digitalWrite(digitPins[0], HIGH);
      digitalWrite(digitPins[1], LOW);
      digitalWrite(digitPins[2], HIGH);
      
      digitalWrite(segPins[0], HIGH);
      digitalWrite(segPins[1], HIGH);
      digitalWrite(segPins[2], HIGH);
      digitalWrite(segPins[3], HIGH);
      digitalWrite(segPins[4], HIGH);
      digitalWrite(segPins[5], HIGH);
      digitalWrite(segPins[6], LOW);
      delay(1);

      digitalWrite(digitPins[0], HIGH);
      digitalWrite(digitPins[1], HIGH);
      digitalWrite(digitPins[2], LOW);
      
      digitalWrite(segPins[0], HIGH);
      digitalWrite(segPins[1], HIGH);
      digitalWrite(segPins[2], HIGH);
      digitalWrite(segPins[3], HIGH);
      digitalWrite(segPins[4], HIGH);
      digitalWrite(segPins[5], HIGH);
      digitalWrite(segPins[6], LOW);
      delay(1);
  }
  else if (points == 90) { 

      digitalWrite(digitPins[0], HIGH);
      digitalWrite(digitPins[1], LOW);
      digitalWrite(digitPins[2], HIGH);
      
      digitalWrite(segPins[0], HIGH);
      digitalWrite(segPins[1], HIGH);
      digitalWrite(segPins[2], HIGH);
      digitalWrite(segPins[3], LOW);
      digitalWrite(segPins[4], LOW);
      digitalWrite(segPins[5], HIGH);
      digitalWrite(segPins[6], HIGH);
      delay(1);

      digitalWrite(digitPins[0], HIGH);
      digitalWrite(digitPins[1], HIGH);
      digitalWrite(digitPins[2], LOW);
      
      digitalWrite(segPins[0], HIGH);
      digitalWrite(segPins[1], HIGH);
      digitalWrite(segPins[2], HIGH);
      digitalWrite(segPins[3], HIGH);
      digitalWrite(segPins[4], HIGH);
      digitalWrite(segPins[5], HIGH);
      digitalWrite(segPins[6], LOW);
      delay(1);
  }
  else if (points == 80) {
    
      digitalWrite(digitPins[0], HIGH);
      digitalWrite(digitPins[1], LOW);
      digitalWrite(digitPins[2], HIGH);
      
      digitalWrite(segPins[0], HIGH);
      digitalWrite(segPins[1], HIGH);
      digitalWrite(segPins[2], HIGH);
      digitalWrite(segPins[3], HIGH);
      digitalWrite(segPins[4], HIGH);
      digitalWrite(segPins[5], HIGH);
      digitalWrite(segPins[6], HIGH);
      delay(1);

      digitalWrite(digitPins[0], HIGH);
      digitalWrite(digitPins[1], HIGH);
      digitalWrite(digitPins[2], LOW);
      
      digitalWrite(segPins[0], HIGH);
      digitalWrite(segPins[1], HIGH);
      digitalWrite(segPins[2], HIGH);
      digitalWrite(segPins[3], HIGH);
      digitalWrite(segPins[4], HIGH);
      digitalWrite(segPins[5], HIGH);
      digitalWrite(segPins[6], LOW);
      delay(1);
  }
  else if (points == 70) {
    
      digitalWrite(digitPins[0], HIGH);
      digitalWrite(digitPins[1], LOW);
      digitalWrite(digitPins[2], HIGH);
      
      digitalWrite(segPins[0], HIGH);
      digitalWrite(segPins[1], HIGH);
      digitalWrite(segPins[2], HIGH);
      digitalWrite(segPins[3], LOW);
      digitalWrite(segPins[4], LOW);
      digitalWrite(segPins[5], LOW);
      digitalWrite(segPins[6], LOW);
      delay(1);
  
      digitalWrite(digitPins[0], HIGH);
      digitalWrite(digitPins[1], HIGH);
      digitalWrite(digitPins[2], LOW);
      
      digitalWrite(segPins[0], HIGH);
      digitalWrite(segPins[1], HIGH);
      digitalWrite(segPins[2], HIGH);
      digitalWrite(segPins[3], HIGH);
      digitalWrite(segPins[4], HIGH);
      digitalWrite(segPins[5], HIGH);
      digitalWrite(segPins[6], LOW);
      delay(1);
  }
  else if (points == 60) {
  
    digitalWrite(digitPins[0], HIGH);
    digitalWrite(digitPins[1], LOW);
    digitalWrite(digitPins[2], HIGH);
    
    digitalWrite(segPins[0], HIGH);
    digitalWrite(segPins[1], LOW);
    digitalWrite(segPins[2], HIGH);
    digitalWrite(segPins[3], HIGH);
    digitalWrite(segPins[4], HIGH);
    digitalWrite(segPins[5], HIGH);
    digitalWrite(segPins[6], HIGH);
    delay(1);

    digitalWrite(digitPins[0], HIGH);
    digitalWrite(digitPins[1], HIGH);
    digitalWrite(digitPins[2], LOW);
    
    digitalWrite(segPins[0], HIGH);
    digitalWrite(segPins[1], HIGH);
    digitalWrite(segPins[2], HIGH);
    digitalWrite(segPins[3], HIGH);
    digitalWrite(segPins[4], HIGH);
    digitalWrite(segPins[5], HIGH);
    digitalWrite(segPins[6], LOW);
    delay(1);
  }
  else if (points == 50) {

    digitalWrite(digitPins[0], HIGH);
    digitalWrite(digitPins[1], LOW);
    digitalWrite(digitPins[2], HIGH);
    
    digitalWrite(segPins[0], HIGH);
    digitalWrite(segPins[1], LOW);
    digitalWrite(segPins[2], HIGH);
    digitalWrite(segPins[3], HIGH);
    digitalWrite(segPins[4], LOW);
    digitalWrite(segPins[5], HIGH);
    digitalWrite(segPins[6], HIGH);
    delay(1);
  
    digitalWrite(digitPins[0], HIGH);
    digitalWrite(digitPins[1], HIGH);
    digitalWrite(digitPins[2], LOW);
    
    digitalWrite(segPins[0], HIGH);
    digitalWrite(segPins[1], HIGH);
    digitalWrite(segPins[2], HIGH);
    digitalWrite(segPins[3], HIGH);
    digitalWrite(segPins[4], HIGH);
    digitalWrite(segPins[5], HIGH);
    digitalWrite(segPins[6], LOW);
    delay(1);
  }
  else if (points == 40) {

    digitalWrite(digitPins[0], HIGH);
    digitalWrite(digitPins[1], LOW);
    digitalWrite(digitPins[2], HIGH);
    
    digitalWrite(segPins[0], LOW);
    digitalWrite(segPins[1], HIGH);
    digitalWrite(segPins[2], HIGH);
    digitalWrite(segPins[3], LOW);
    digitalWrite(segPins[4], LOW);
    digitalWrite(segPins[5], HIGH);
    digitalWrite(segPins[6], HIGH);
    delay(1);
  
    digitalWrite(digitPins[0], HIGH);
    digitalWrite(digitPins[1], HIGH);
    digitalWrite(digitPins[2], LOW);
    
    digitalWrite(segPins[0], HIGH);
    digitalWrite(segPins[1], HIGH);
    digitalWrite(segPins[2], HIGH);
    digitalWrite(segPins[3], HIGH);
    digitalWrite(segPins[4], HIGH);
    digitalWrite(segPins[5], HIGH);
    digitalWrite(segPins[6], LOW);
    delay(1);
  }
  else if (points == 30) {

    digitalWrite(digitPins[0], HIGH);
    digitalWrite(digitPins[1], LOW);
    digitalWrite(digitPins[2], HIGH);
    
    digitalWrite(segPins[0], HIGH);
    digitalWrite(segPins[1], HIGH);
    digitalWrite(segPins[2], HIGH);
    digitalWrite(segPins[3], HIGH);
    digitalWrite(segPins[4], LOW);
    digitalWrite(segPins[5], LOW);
    digitalWrite(segPins[6], HIGH);
    delay(1);
  
    digitalWrite(digitPins[0], HIGH);
    digitalWrite(digitPins[1], HIGH);
    digitalWrite(digitPins[2], LOW);
    
    digitalWrite(segPins[0], HIGH);
    digitalWrite(segPins[1], HIGH);
    digitalWrite(segPins[2], HIGH);
    digitalWrite(segPins[3], HIGH);
    digitalWrite(segPins[4], HIGH);
    digitalWrite(segPins[5], HIGH);
    digitalWrite(segPins[6], LOW);
    delay(1);
  }
  else if (points == 20) {
  
    digitalWrite(digitPins[0], HIGH);
    digitalWrite(digitPins[1], LOW);
    digitalWrite(digitPins[2], HIGH);
    
    digitalWrite(segPins[0], HIGH);
    digitalWrite(segPins[1], HIGH);
    digitalWrite(segPins[2], LOW);
    digitalWrite(segPins[3], HIGH);
    digitalWrite(segPins[4], HIGH);
    digitalWrite(segPins[5], LOW);
    digitalWrite(segPins[6], HIGH);
    delay(1);
  
    digitalWrite(digitPins[0], HIGH);
    digitalWrite(digitPins[1], HIGH);
    digitalWrite(digitPins[2], LOW);
    
    digitalWrite(segPins[0], HIGH);
    digitalWrite(segPins[1], HIGH);
    digitalWrite(segPins[2], HIGH);
    digitalWrite(segPins[3], HIGH);
    digitalWrite(segPins[4], HIGH);
    digitalWrite(segPins[5], HIGH);
    digitalWrite(segPins[6], LOW);
    delay(1);
  }
  else if (points == 10) {

    digitalWrite(digitPins[0], HIGH);
    digitalWrite(digitPins[1], LOW);
    digitalWrite(digitPins[2], HIGH);
    
    digitalWrite(segPins[0], LOW);
    digitalWrite(segPins[1], HIGH);
    digitalWrite(segPins[2], HIGH);
    digitalWrite(segPins[3], LOW);
    digitalWrite(segPins[4], LOW);
    digitalWrite(segPins[5], LOW);
    digitalWrite(segPins[6], LOW);
    delay(1);
  
    digitalWrite(digitPins[0], HIGH);
    digitalWrite(digitPins[1], HIGH);
    digitalWrite(digitPins[2], LOW);
    
    digitalWrite(segPins[0], HIGH);
    digitalWrite(segPins[1], HIGH);
    digitalWrite(segPins[2], HIGH);
    digitalWrite(segPins[3], HIGH);
    digitalWrite(segPins[4], HIGH);
    digitalWrite(segPins[5], HIGH);
    digitalWrite(segPins[6], LOW);
    delay(1);
  }
  else if (points == 0) {  
    digitalWrite(digitPins[0], HIGH);
    digitalWrite(digitPins[1], HIGH);
    digitalWrite(digitPins[2], LOW);
    
    digitalWrite(segPins[0], HIGH);
    digitalWrite(segPins[1], HIGH);
    digitalWrite(segPins[2], HIGH);
    digitalWrite(segPins[3], HIGH);
    digitalWrite(segPins[4], HIGH);
    digitalWrite(segPins[5], HIGH);
    digitalWrite(segPins[6], LOW);
    delay(1);
  }
}
void loop() {

  // put your main code here, to run repeatedly:
  if(IrReceiver.decode()){
    //if recv signal from opponent's IR transmitter, gun shot at opponent and minus opponent's healthpoints:
    Serial.print("command:");
    Serial.println(IrReceiver.decodedIRData.command); 
    
    if(IrReceiver.decodedIRData.command == 0x02) {                                                                                                      
      Serial.println("gun shot");
      if (points > 0) { 
        points -= 10;     
      }                                                                    
    }
    else {
      Serial.println("missed");  
    }
    displayPoints(points);
    delay(25);
    IrReceiver.resume();
  }
  else{
    displayPoints(points);
  }
}
