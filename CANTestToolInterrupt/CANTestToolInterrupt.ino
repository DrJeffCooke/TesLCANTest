/*
  TesLorean Battery Control Test
  2018
  Jeff Cooke

  Addresses the dropped CAN packets with the MCP2515_CAN interface
  Uses interrupts to capture the data and outputs and other low priority tasks in the loop()
  
*/

#include <mcp2515.h>
#include <SPI.h>

// Relay Connections - Connector X357 (data connection)
#define Relay_X357_4  7  // X357-3 to 12v??? accessory wake up
#define Relay_X357_5  8  // X357-14 to 5v HV energy mgt communication enable

// CAN board Chip Select (CS) pins
#define BATTERY_CAN_CS 10
#define TESLOREAN_CAN_CS 9

// DEFINE
#define StoredFrames 100

// Array for preprepared CAN frames
can_frame CANFrames[StoredFrames];

// Counters tracking the add point and read point in the circular array
volatile uint8_t addPointFrames = 0;
volatile uint8_t readPointFrames = 0;
volatile uint16_t netAddReadCount = 0;   // Adds increment, Reads decrement, only Read if <> 0

// Create the CAN objects, specific the 'Chip Select' line
MCP2515 can0(BATTERY_CAN_CS);    // Battery CAN Bus - CS on line 10
MCP2515 can1(TESLOREAN_CAN_CS);   // TesLorean CAN bus - CS on line 9

struct can_frame frame;
	
void irqBATHandler()
{
  uint8_t irq = can0.getInterrupts();

  // check channel 0
  if (irq & MCP2515::CANINTF_RX0IF)
  {
    if (can0.readMessage(MCP2515::RXB0, &CANFrames[addPointFrames]) == MCP2515::ERROR_OK)
    {
        // frame contains received from RXB0 message
        addPointFrames = (addPointFrames + 1) % StoredFrames;
        netAddReadCount++;
    }
  }
            
  // check channel 1
  if (irq & MCP2515::CANINTF_RX1IF)
  {
    if (can0.readMessage(MCP2515::RXB1, &CANFrames[addPointFrames]) == MCP2515::ERROR_OK)
    {
      // frame contains received from RXB1 message
      addPointFrames = (addPointFrames + 1) % StoredFrames;
      netAddReadCount++;
    }
  } 
}
	
void setup()
{
  Serial.begin(115200);
  SPI.begin();

  // Initialize the array pointers
  addPointFrames = 0;
  readPointFrames = 0;
  netAddReadCount = 0;

  can0.reset();
  can0.setBitrate(CAN_1000KBPS);
  can0.setNormalMode();

  //// SETUP OUTPUT LINES - BMS Data Communications
  pinMode(Relay_X357_4, OUTPUT);
  pinMode(Relay_X357_5, OUTPUT);
  digitalWrite(Relay_X357_4, HIGH); // X357-3 to 12v accessory wake up
  digitalWrite(Relay_X357_5, HIGH); // X357-14 to 5v HV energy mgt communication enable
  
  attachInterrupt(digitalPinToInterrupt(2), irqBATHandler, LOW);

  Serial.println("Starting...");
}

// Send the frame to the Serial port
void outputCANframe(can_frame & frame)
{
  // Send frame to the Serial Port
  Serial.print("CAN frame : 0x");
  Serial.print(frame.can_id,HEX);
  Serial.print(" - ");
  for (int i=0;i<frame.can_dlc;i++)
  {
    Serial.print(" 0x");
    Serial.print(frame.data[i],HEX);
  }
  Serial.println();  
}

void loop()
{
  // Output a captured frame
  if (netAddReadCount > 0)
  {
    if (readPointFrames != addPointFrames)
    {
      outputCANframe(CANFrames[readPointFrames]);
      readPointFrames = (readPointFrames + 1) % StoredFrames;
      if(netAddReadCount > 0){netAddReadCount--;}
    }
  }
}
