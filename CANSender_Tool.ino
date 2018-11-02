/*
 * TesLorean CAN Test Interface
 * 2018
 * Jeff Cooke
*/

#include <mcp2515.h>
#include <SPI.h>

// Create the CAN object for transmission
MCP2515 can0(10);     // CS on digitial pin 10

// Array for preprepared CAN frames
can_frame CANPkg[5];

// Define the debug variable
#define debug 1

void setup() {
  //// SERIAL PORT
  // init the serial port - for status/debug
  while (!Serial);
  Serial.begin(115200);

  #ifdef debug 
    Serial.println("Initialized serial port (115200 baud)");
  #endif

  // init the SPI communications
  SPI.begin();

  // Startup CAN  TesLorean bus
  can0.reset();
  can0.setBitrate(CAN_500KBPS);
  can0.setNormalMode();
  #ifdef debug
  Serial.println("Test CANbus initialized (500 kbps)");
  #endif

  // Populate a number of pre-built CAN frames
  buildCANframe(CANPkg[0],0x000,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07);
  buildCANframe(CANPkg[1],0x001,0x01,0x01,0x02,0x03,0x04,0x05,0x06,0x07);
  buildCANframe(CANPkg[2],0x002,0x02,0x01,0x02,0x03,0x04,0x05,0x06,0x07);
  buildCANframe(CANPkg[3],0x003,0x03,0x01,0x02,0x03,0x04,0x05,0x06,0x07);
  buildCANframe(CANPkg[4],0x004,0x04,0x01,0x02,0x03,0x04,0x05,0x06,0x07);
  
}

// Populate the CAN frame with the data specified
void buildCANframe(can_frame & frame, uint16_t cid, uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3,  uint8_t b4, uint8_t b5, uint8_t b6, uint8_t b7){
  frame.can_id = cid;
  frame.can_dlc = 8; 
  frame.data[0] = b0;
  frame.data[1] = b1;
  frame.data[2] = b2;
  frame.data[3] = b3;
  frame.data[4] = b4;
  frame.data[5] = b5;
  frame.data[6] = b6;
  frame.data[7] = b7;
}

// Send the frame to the Serial port
void outputCANframe(can_frame & frame){

  // Send frame to the Serial Port
  Serial.print("CAN frame : ");
  Serial.print(frame.can_id,HEX);
  Serial.print(" - ");
  for (int i=0;i<frame.can_dlc;i++){
    Serial.print(frame.data[i],HEX);
    Serial.print(" ");
  }
  Serial.println();  
}

void loop() {
  
  // Define CAN Frame variable structure
  struct can_frame outgoing;

  // Populate a test frame
  buildCANframe(outgoing, 0x0000, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);

  // Check serial port for instructions
  char receivedChar;
  int receivedNum;
  if (Serial.available() > 0) {

    receivedChar = Serial.read();
    
    // Check for preprepared frame
    if (receivedChar == 'p')
    {
      delay(1);   // Without delay (the next Serial Available may not see anything)
      if (Serial.available() > 0)
      {
        receivedNum = Serial.parseInt();

        if ((receivedNum >= 0) && (receivedNum <= 4))
        {
          // Send the selected prepackaged CAN frame
          can0.sendMessage(&CANPkg[receivedNum]);

          // Send frame to the Serial Port
          Serial.println("Preprepared CAN Frame sent...");
          outputCANframe(CANPkg[receivedNum]);
        }
      }
    }

    // Check for full specified frame
    if (receivedChar == 'f')
    {

      bool success = false;
      delay(1);
      
      if (Serial.available() > 0)
      {
        // Grab the ID for the frame
        outgoing.can_id = Serial.parseInt();
        // There should another 8 data entries for the frame
        for (int i = 0; i < 8;i++)
        {
          delay(1);
          if (Serial.available() > 0)
          {
            outgoing.data[i] = Serial.parseInt();
            if (i==7){success = true;}
          }
        }
      }
      
      // Output status of the frame input
      if (success)
      {
        Serial.println("Full CAN Frame sent...");
      }
      else
      {
        Serial.println("Full CAN Frame NOT sent...");
      }

      // Send the frame
      if (success)
      {
        can0.sendMessage(&outgoing);
      }

      // Print out the frame (even if not successfully input
      outputCANframe(outgoing);      
    }
  
  }

}
