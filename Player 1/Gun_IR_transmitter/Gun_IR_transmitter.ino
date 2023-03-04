#include <IRremote.hpp>

const int buttonPin = 2;  
const int irLED = 3; 
byte segPins[] = {A3, A4, 4, 5, A5, A2, A1}; //7-seg LED segment display

//CHANGEABLE VARIABLE:
int ammo = 6; //6 bullets at start of game

// Variables will change:
int buttonStateNew;     // the current reading from the input pin
int actualButtonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

void setup() {
  Serial.begin(9600);
  pinMode(buttonPin, INPUT);
  pinMode(irLED, OUTPUT);
  pinMode(13,OUTPUT);
  //enable IR LED:
  IrSender.begin(irLED, ENABLE_LED_FEEDBACK, USE_DEFAULT_FEEDBACK_LED_PIN); //enable IR LED
  //enable LED 7 seg display:
  for(int i =0; i< 7; i++) {
    pinMode(segPins[i], OUTPUT);
  }
  //initialise led seg display to display 6 bullets:
  sevsegSetNumber(6);
}

uint16_t sAddress = 0x0102;

void sevsegSetNumber(int num){
  if (num == 0) {
      digitalWrite(segPins[0], LOW);
      digitalWrite(segPins[1], LOW);
      digitalWrite(segPins[2], LOW);
      digitalWrite(segPins[3], LOW);  
      digitalWrite(segPins[4], LOW);
      digitalWrite(segPins[5], LOW);      
      digitalWrite(segPins[6], HIGH);   
}
  if (num == 1) {
        digitalWrite(segPins[1], LOW);
        digitalWrite(segPins[2], LOW);
        digitalWrite(segPins[0], HIGH);
        digitalWrite(segPins[3], HIGH);  
        digitalWrite(segPins[4], HIGH);
        digitalWrite(segPins[5], HIGH);      
        digitalWrite(segPins[6], HIGH);   
  }
  if (num == 2) {
        digitalWrite(segPins[2], HIGH);
        digitalWrite(segPins[5], HIGH);
        digitalWrite(segPins[0], LOW);
        digitalWrite(segPins[1], LOW);  
        digitalWrite(segPins[3], LOW);
        digitalWrite(segPins[4], LOW);      
        digitalWrite(segPins[6], LOW);  
  }
  if (num == 3){
        digitalWrite(segPins[5], HIGH);
        digitalWrite(segPins[4], HIGH);
        digitalWrite(segPins[1], LOW);
        digitalWrite(segPins[0], LOW);
        digitalWrite(segPins[2], LOW);
        digitalWrite(segPins[3], LOW);  
        digitalWrite(segPins[6], LOW);
    
  }
  if (num == 4){
        digitalWrite(segPins[0], HIGH);
        digitalWrite(segPins[4], HIGH);
        digitalWrite(segPins[3], HIGH);
        digitalWrite(segPins[1], LOW);  
        digitalWrite(segPins[2], LOW);
        digitalWrite(segPins[5], LOW);      
        digitalWrite(segPins[6], LOW);      
  }
  if (num == 5){
        digitalWrite(segPins[1], HIGH);
        digitalWrite(segPins[4], HIGH);
        digitalWrite(segPins[0], LOW);
        digitalWrite(segPins[2], LOW);  
        digitalWrite(segPins[3], LOW);
        digitalWrite(segPins[5], LOW);      
        digitalWrite(segPins[6], LOW);      
  }
  if (num == 6){
      digitalWrite(segPins[1], HIGH);
      digitalWrite(segPins[0], LOW);
      digitalWrite(segPins[2], LOW);
      digitalWrite(segPins[3], LOW);  
      digitalWrite(segPins[4], LOW);
      digitalWrite(segPins[5], LOW);      
      digitalWrite(segPins[6], LOW);      
  }
  
}
void loop() {

  // read the state of the switch into a local variable:
  buttonStateNew = digitalRead(buttonPin);

  // If the switch changed, due to noise or pressing:
  if (buttonStateNew != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }
  // whatever the reading is at, it's been there for longer than the debounce
  // delay, so take it as the actual current state:
  if ((millis() - lastDebounceTime) > debounceDelay) {

    // if the button state has changed:
    if (buttonStateNew != actualButtonState) {
      actualButtonState = buttonStateNew;

      // only toggle the LED if the new button state is HIGH
      if (actualButtonState == HIGH) {
                
        IrSender.sendNEC(sAddress, 0x02, 0); //Command sent: 0x02
        digitalWrite(13,HIGH); //test IR LED
        if(ammo > 0) {
          ammo-=1.; //minus one for ammo and display this number on led segment display + send this info to visualiser
        }
        //ammo = 0; if user reload, reset ammo to 6
        else {
          ammo = 6;
        } 
        sevsegSetNumber(ammo);
      }
      else {
        digitalWrite(13, LOW);
      }
    }
  }
  
  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = buttonStateNew;
}
