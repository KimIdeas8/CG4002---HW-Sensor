// ================================================================
// ===               INTERNAL COMMS CODE              ===
// ================================================================
#pragma pack(1)
// ================================================================
// ===               LIBRARIES + VARIABLES              ===
// ================================================================
#include <TM1637Display.h>
#include <Arduino.h>
#include "TinyIRReceiver.hpp"

//IR RECEIVER:
#define IR_RECEIVE_PIN    2   // INT0

#define GRENADE 1
#define SHOT 2

//7-SEG LED:
#define CLK 5
#define DIO 4
TM1637Display display = TM1637Display(CLK, DIO);
int points = 100;
int num_Shots_Stored = 0;
// ================================================================
// ===               INTERNAL COMMS CODE              ===
// ================================================================
int data[8];
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

  //LED display:
  display.clear();
  display.setBrightness(5);
  display.showNumberDec(points); //display 100 points

  //IR LED:
  //IrReceiver.begin(IR_RECEIVE_PIN); // Start the receiver
  initPCIInterruptForTinyReceiver();

  //INTERNAL COMMS:
  generateDefaultPackets();
  threeWayHandshake();
}

// ================================================================
// ===               FUNCTIONS            ===
// ================================================================
void displayPoints(int points){
    display.showNumberDec(points); //show the points
}

void changeDisplayPoints(int points){
    display.clear(); //clear the screen
    display.showNumberDec(points); //show the points
}


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

volatile int num_Shots_Detected = 0;
/*
 * This is the function is called if a complete command was received
 */
void handleReceivedTinyIRData(uint8_t aAddress, uint8_t aCommand, uint8_t aFlags) {
  //command received for player 2 is '2'
  if (255-aCommand == 2){
   num_Shots_Detected++;
  }
}

void updatePoints(int Action){
  if (Action == GRENADE){
    points -= 30; //to be multipled by num of grenade throws from FPGA
  }
  else if (Action == SHOT){
    points -= (10 * num_Shots_Stored);
  }
}
// ================================================================
// ===               MAIN LOOP              ===
// ================================================================
void loop() {
  
  // ===               DATA PACKET              ===
  for (int i = 0; i < 8; i++)
  {
    data[i] = 0; 
  }
  
  // ===               MAIN CODE              ===
  //CHECK: Whenever hps hits 0, reset back to 100 in next iteration - (to be replaced with proper hps coming from SW Visualiser)
  if(points == 0) {
    delay(1000); //can see 0[player loses all hps] changing to 100 on LED
    points = 100;
    changeDisplayPoints(points);
  }
  else if (points == 100){
    delay(1000); //can see 100 changing to 90 on LED
  }

  //if hit by grenade, minus 30 hps + change points displayed on led:
   //updatePoints(GRENADE);
  
  //change points displayed on led+ send pkt to relay node
  if (num_Shots_Detected > 0) {
    num_Shots_Stored = num_Shots_Detected; //store the num of shots detected
    num_Shots_Detected -= num_Shots_Stored; 
    
    //if shot, minus 10 hps:
    updatePoints(SHOT);
    changeDisplayPoints(points);

    // ===               DATA PACKET              ===
    send_data_pkt(); //Send pkt to relay node
  }
  // ===               END              ===
}
