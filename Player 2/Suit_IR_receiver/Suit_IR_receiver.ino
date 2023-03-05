// ================================================================
// ===               INTERNAL COMMS CODE              ===
// ================================================================
#pragma pack(1)
// ================================================================
// ===               LIBRARIES + VARIABLES              ===
// ================================================================
#include <IRremote.hpp>
#define IR_RECEIVE_PIN 0

int points = 100;
int digitPins[] = {A1,A2,A3};
int segPins[] = {5,3,A5,A4,2,4,A0};
// ================================================================
// ===               INTERNAL COMMS CODE              ===
// ================================================================
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
  p.header = (2 << 4) | packet_type;
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
// ===               SETUP CODE              ===
// ================================================================
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  IrReceiver.begin(IR_RECEIVE_PIN); // Start the receiver

  //SET LED SEGMENT PINS AS OUTPUTS:
  for (int i = 0; i < 3; i++) 
  {pinMode(digitPins[i], OUTPUT);}
  for (int i = 0; i < 7; i++) 
  {pinMode(segPins[i], OUTPUT);}
  
  //INTERNAL COMMS:
  generateDefaultPackets();
  threeWayHandshake();
}

// ================================================================
// ===               FUNCTIONS            ===
// ================================================================
void turnOnDigit1(){
      digitalWrite(digitPins[0], LOW);
      digitalWrite(digitPins[1], HIGH);
      digitalWrite(digitPins[2], HIGH);
}
void turnOnDigit2(){
      digitalWrite(digitPins[0], HIGH);
      digitalWrite(digitPins[1], LOW);
      digitalWrite(digitPins[2], HIGH);
}
void turnOnDigit3(){
      digitalWrite(digitPins[0], HIGH);
      digitalWrite(digitPins[1], HIGH);
      digitalWrite(digitPins[2], LOW);
}
void display_0(){
      digitalWrite(segPins[0], HIGH);
      digitalWrite(segPins[1], HIGH);
      digitalWrite(segPins[2], HIGH);
      digitalWrite(segPins[3], HIGH);
      digitalWrite(segPins[4], HIGH);
      digitalWrite(segPins[5], HIGH);
      digitalWrite(segPins[6], LOW);
}
void display_1(){
      digitalWrite(segPins[0], LOW);
      digitalWrite(segPins[1], HIGH);
      digitalWrite(segPins[2], HIGH);
      digitalWrite(segPins[3], LOW);
      digitalWrite(segPins[4], LOW);
      digitalWrite(segPins[5], LOW);
      digitalWrite(segPins[6], LOW);
}
void display_2(){
      digitalWrite(segPins[0], HIGH);
      digitalWrite(segPins[1], HIGH);
      digitalWrite(segPins[2], LOW);
      digitalWrite(segPins[3], HIGH);
      digitalWrite(segPins[4], HIGH);
      digitalWrite(segPins[5], LOW);
      digitalWrite(segPins[6], HIGH);
}
void display_3(){
    digitalWrite(segPins[0], HIGH);
    digitalWrite(segPins[1], HIGH);
    digitalWrite(segPins[2], HIGH);
    digitalWrite(segPins[3], HIGH);
    digitalWrite(segPins[4], LOW);
    digitalWrite(segPins[5], LOW);
    digitalWrite(segPins[6], HIGH);
}
void display_4(){
    digitalWrite(segPins[0], LOW);
    digitalWrite(segPins[1], HIGH);
    digitalWrite(segPins[2], HIGH);
    digitalWrite(segPins[3], LOW);
    digitalWrite(segPins[4], LOW);
    digitalWrite(segPins[5], HIGH);
    digitalWrite(segPins[6], HIGH);
}
void display_5(){
    digitalWrite(segPins[0], HIGH);
    digitalWrite(segPins[1], LOW);
    digitalWrite(segPins[2], HIGH);
    digitalWrite(segPins[3], HIGH);
    digitalWrite(segPins[4], LOW);
    digitalWrite(segPins[5], HIGH);
    digitalWrite(segPins[6], HIGH);
}
void display_6(){
    digitalWrite(segPins[0], HIGH);
    digitalWrite(segPins[1], LOW);
    digitalWrite(segPins[2], HIGH);
    digitalWrite(segPins[3], HIGH);
    digitalWrite(segPins[4], HIGH);
    digitalWrite(segPins[5], HIGH);
    digitalWrite(segPins[6], HIGH);
}
void display_7(){
      digitalWrite(segPins[0], HIGH);
      digitalWrite(segPins[1], HIGH);
      digitalWrite(segPins[2], HIGH);
      digitalWrite(segPins[3], LOW);
      digitalWrite(segPins[4], LOW);
      digitalWrite(segPins[5], LOW);
      digitalWrite(segPins[6], LOW);
}
void display_8(){
      digitalWrite(segPins[0], HIGH);
      digitalWrite(segPins[1], HIGH);
      digitalWrite(segPins[2], HIGH);
      digitalWrite(segPins[3], HIGH);
      digitalWrite(segPins[4], HIGH);
      digitalWrite(segPins[5], HIGH);
      digitalWrite(segPins[6], HIGH);
}
void display_9(){
      digitalWrite(segPins[0], HIGH);
      digitalWrite(segPins[1], HIGH);
      digitalWrite(segPins[2], HIGH);
      digitalWrite(segPins[3], LOW);
      digitalWrite(segPins[4], LOW);
      digitalWrite(segPins[5], HIGH);
      digitalWrite(segPins[6], HIGH);
}
void displayPoints(int points){
  if (points==100){

      turnOnDigit1();
      display_1();
      delay(1);

      turnOnDigit2();
      display_0();
      delay(1);

      turnOnDigit3();
      display_0();
      delay(1);
  }
  else if (points == 90) { 

      turnOnDigit2();
      display_9();
      delay(1);

      turnOnDigit3();
      display_0();
      delay(1);
  }
  else if (points == 80) {
    
      turnOnDigit2();
      display_8();
      delay(1);
      
      turnOnDigit3();
      display_0();
      delay(1);
  }
  else if (points == 70) {
    
      turnOnDigit2();
      display_7();
      delay(1);
      
      turnOnDigit3();
      display_0();
      delay(1);
  }
  else if (points == 60) {
  
      turnOnDigit2();
      display_6();
      delay(1);
      
      turnOnDigit3();
      display_0();
      delay(1);
  }
  else if (points == 50) {

      turnOnDigit2();
      display_5();
      delay(1);
      
      turnOnDigit3();
      display_0();
      delay(1);
  }
  else if (points == 40) {

      turnOnDigit2();
      display_4();
      delay(1);
      
      turnOnDigit3();
      display_0();
      delay(1);
  }
  else if (points == 30) {

      turnOnDigit2();
      display_3();
      delay(1);
      
      turnOnDigit3();
      display_0();
      delay(1);
  }
  else if (points == 20) {
      
      turnOnDigit2();
      display_2();
      delay(1);
      
      turnOnDigit3();
      display_0();
      delay(1);
  }
  else if (points == 10) {

      turnOnDigit2();
      display_1();
      delay(1);
      
      turnOnDigit3();
      display_0();
      delay(1);
  }
  else if (points == 0) {  
      
      turnOnDigit3();
      display_0();
      delay(1);
  }
}

// ================================================================
// ===               MAIN LOOP              ===
// ================================================================
void loop() {
  // ===               DATA PACKET              ===
  int data[8];
  for (int i = 0; i < 8; i++)
  {
    data[i] = 0; 
  }
  
  // ===               MAIN CODE              ===
  //CHECK: Whenever healthpoints of player hits 0, reset back to 100 in next iteration - Unlimited Rebirth (to be replaced with proper hps coming from SW Visualiser)
  if(points == 0) {
    points = 100;
  }
  displayPoints(points);
  if(IrReceiver.decode()){
    //Action: if recv signal from opponent's IR transmitter (ie. gun shot at opponent) 
    //Result: minus opponent's healthpoints (display points on led)
    //Player 2's suit will receive a 0x02
    if(IrReceiver.decodedIRData.command == 0x02) {                                                                                                      
      // ===               DATA PACKET              ===
      data[0] = 1; //indicate a shot at player (ie. 1 - means shot, 0 - means no shot)
      // ===               MAIN CODE              ===
      if (points > 0) { 
        points -= 10;     
      }
    }
    displayPoints(points);
    delay(25);
    IrReceiver.resume();
  }

  // ===               DATA PACKET              ===
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
  
  // ===               END              ===
}
