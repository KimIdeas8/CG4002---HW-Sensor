#pragma pack(1)
#include <IRremote.hpp>
const int SWITCH_PIN = 2;
const int IR_SEND_PIN = 3;
byte segPins[] = {A3, A4, 4, 5, A5, A2, A1}; //7-seg LED segment display

//CHANGEABLE VARIABLE:
int ammo = 6; //6 bullets at start of game

// Variables will change:
int buttonStateNew;     // the current reading from the input pin
int actualButtonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

uint16_t sAddress = 0x0102;
enum PacketType
{
  HELLO,
  ACK,
  NACK,
  DATA
};

typedef struct
{
  uint8_t header;           // contains beetle id and packet type
  uint8_t padding;          // Padding header to 2 bytes
  int euler_x;              // contains IR data for data packet for IR sensors
  int euler_y;              // all other fields padded with 0 for data packet for IR sensors
  int euler_z;
  int acc_x;
  int acc_y;
  int acc_z;
  int flex_1;
  int flex_2;
  uint16_t crc;             // Cyclic redundancy check (CRC-16)
} BLEPacket;

const unsigned int PACKET_SIZE = 20;

uint8_t serial_buffer[PACKET_SIZE];
BLEPacket* curr_packet;

BLEPacket default_packets[3];

uint16_t crcCalc(uint8_t* data)
{
  uint16_t curr_crc = 0x0000;
  uint8_t sum1 = (uint8_t) curr_crc;
  uint8_t sum2 = (uint8_t) (curr_crc >> 8);

  for (int i = 0; i < PACKET_SIZE; i++)
  {
    sum1 = (sum1 + data[i]) % 255;
    sum2 = (sum2 + sum1) % 255;
  }
  return (sum2 << 8) | sum1;
}

bool crcCheck()
{
  uint16_t crc = curr_packet->crc;
  curr_packet->crc = 0;
  return (crc == crcCalc((uint8_t*)curr_packet));
}

bool packetCheck(uint8_t node_id, PacketType packet_type)
{
  uint8_t header = curr_packet->header;
  uint8_t curr_node_id = (header & 0xf0) >> 4;
  PacketType curr_packet_type = PacketType(header & 0xf);
  return curr_node_id == node_id && curr_packet_type == packet_type;
}

void waitForData()
{
  unsigned int buf_pos = 0;
  while (buf_pos < PACKET_SIZE)
  {
    if (Serial.available())
    {
      uint8_t in_data = Serial.read();
      serial_buffer[buf_pos] = in_data;
      buf_pos++;
    }
  }
  curr_packet = (BLEPacket*)serial_buffer;
}

BLEPacket generatePacket(PacketType packet_type, int* data)
{
  BLEPacket p;
  p.header = (1 << 4) | packet_type;
  p.padding = 0;
  p.euler_x = data[0];
  p.euler_y = data[1];
  p.euler_z = data[2];
  p.acc_x = data[3];
  p.acc_y = data[4];
  p.acc_z = data[5];
  p.flex_1 = data[6];
  p.flex_2 = data[7];
  p.crc = 0;
  uint16_t calculatedCRC = crcCalc((uint8_t*)&p);
  p.crc = calculatedCRC;
  return p;
}

void generateDefaultPackets()
{
  int data[] = {0, 0, 0, 0, 0, 0, 0, 0};
  for (int i = 0; i < 3; i++)
  {
    default_packets[i] = generatePacket(PacketType(i), data);
  }
}

void sendDefaultPacket(PacketType packet_type)
{
  Serial.write((byte*)&default_packets[packet_type], PACKET_SIZE);
}

void sendDataPacket(int* data)
{
  BLEPacket p = generatePacket(DATA, data);
  Serial.write((byte*)&p, PACKET_SIZE);
}

void threeWayHandshake()
{
  bool is_connected = false;
  while (!is_connected)
  {
    // wait for hello from laptop
    waitForData();

    if (!crcCheck() || !packetCheck(0, HELLO))
    {
      sendDefaultPacket(NACK);
      continue;
    }
    sendDefaultPacket(HELLO);

    // wait for ack from laptop
    waitForData();

    if (crcCheck() && packetCheck(0, ACK))
    {
      is_connected = true;
    }
  }
}

// ================================================================
// ===               FUNCTIONS            ===
// ================================================================
void display_0() {
  digitalWrite(segPins[0], LOW);
  digitalWrite(segPins[1], LOW);
  digitalWrite(segPins[2], LOW);
  digitalWrite(segPins[3], LOW);
  digitalWrite(segPins[4], LOW);
  digitalWrite(segPins[5], LOW);
  digitalWrite(segPins[6], HIGH);
}
void display_1() {
  digitalWrite(segPins[1], LOW);
  digitalWrite(segPins[2], LOW);
  digitalWrite(segPins[0], HIGH);
  digitalWrite(segPins[3], HIGH);
  digitalWrite(segPins[4], HIGH);
  digitalWrite(segPins[5], HIGH);
  digitalWrite(segPins[6], HIGH);
}
void display_2() {
  digitalWrite(segPins[2], HIGH);
  digitalWrite(segPins[5], HIGH);
  digitalWrite(segPins[0], LOW);
  digitalWrite(segPins[1], LOW);
  digitalWrite(segPins[3], LOW);
  digitalWrite(segPins[4], LOW);
  digitalWrite(segPins[6], LOW);
}
void display_3() {
  digitalWrite(segPins[5], HIGH);
  digitalWrite(segPins[4], HIGH);
  digitalWrite(segPins[1], LOW);
  digitalWrite(segPins[0], LOW);
  digitalWrite(segPins[2], LOW);
  digitalWrite(segPins[3], LOW);
  digitalWrite(segPins[6], LOW);
}
void display_4() {
  digitalWrite(segPins[0], HIGH);
  digitalWrite(segPins[4], HIGH);
  digitalWrite(segPins[3], HIGH);
  digitalWrite(segPins[1], LOW);
  digitalWrite(segPins[2], LOW);
  digitalWrite(segPins[5], LOW);
  digitalWrite(segPins[6], LOW);
}
void display_5() {
  digitalWrite(segPins[1], HIGH);
  digitalWrite(segPins[4], HIGH);
  digitalWrite(segPins[0], LOW);
  digitalWrite(segPins[2], LOW);
  digitalWrite(segPins[3], LOW);
  digitalWrite(segPins[5], LOW);
  digitalWrite(segPins[6], LOW);
}
void display_6() {
  digitalWrite(segPins[1], HIGH);
  digitalWrite(segPins[0], LOW);
  digitalWrite(segPins[2], LOW);
  digitalWrite(segPins[3], LOW);
  digitalWrite(segPins[4], LOW);
  digitalWrite(segPins[5], LOW);
  digitalWrite(segPins[6], LOW);
}
void sevsegSetNumber(int num) {
  if (num == 0) {
    display_0();
  }
  if (num == 1) {
    display_1();
  }
  if (num == 2) {
    display_2();
  }
  if (num == 3) {
    display_3();
  }
  if (num == 4) {
    display_4();
  }
  if (num == 5) {
    display_5();
  }
  if (num == 6) {
    display_6();
  }
}

int data[8];
//This function sends data pkt to relay node till ack:
void send_data_pkt(){
    
    data[0] = 1; //set data pkt 1st bit to '1' - ie. player is shooting
    bool is_ack = false;
    while (!is_ack) // packet will keep sending until it is acknowledged by laptop
    {
      sendDataPacket(data);
      waitForData();
      if (!crcCheck()) continue;
      if (packetCheck(0, ACK))
      {
        is_ack = true;
      }
      else if (packetCheck(0, HELLO))
      {
        sendDefaultPacket(HELLO);

        // wait for ack from laptop
        waitForData();

        if (crcCheck() && packetCheck(0, ACK))
        {
          is_ack = true;
        }
      }
    }
}

void setup() {

  Serial.begin(115200);

  pinMode(SWITCH_PIN, INPUT);
  pinMode(IR_SEND_PIN, OUTPUT);
  pinMode(13, OUTPUT);

  //enable IR LED:
  IrSender.begin(IR_SEND_PIN, ENABLE_LED_FEEDBACK, USE_DEFAULT_FEEDBACK_LED_PIN); //enable IR LED

  //enable LED 7 seg display:
  for (int i = 0; i < 7; i++) {
    pinMode(segPins[i], OUTPUT);
  }
  //initialise led seg display to display 6 bullets:
  sevsegSetNumber(6);

  //handshake:
  generateDefaultPackets();
  threeWayHandshake();
}

void loop() {

  // ===               DATA PACKET              ===
  for (int i = 0; i < 8; i++)
  {
    data[i] = 0;
  }

  // ===               MAIN CODE              ===
  // read the state of the switch into a local variable:
  buttonStateNew = digitalRead(SWITCH_PIN);

  // If the switch changed, due to noise or pressing/release:
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

      // only send IR LED if the new button state is HIGH
      if (actualButtonState == HIGH) {

        IrSender.sendNEC(sAddress, 0x01, 0); //Command sent: 0x01
        //modify data pkt sent to SW Visualiser:
        data[0] = 1;
        // ===               DATA PACKET              ===
        send_data_pkt();
        //change ammo on led:
        if (ammo > 0) {
          ammo -= 1; //minus one for ammo and display this number on led segment display + send this info to visualiser
        }
        //ammo = 0; if user reload, reset ammo to 6
        else {
          ammo = 6;
        }
        sevsegSetNumber(ammo);
      }
    }
    //either: time limit has not passed 50ms and gun shot
    //or: no shooting action
  }
  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = buttonStateNew;


  // ===               END              ===
}
