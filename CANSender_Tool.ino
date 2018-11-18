/*
 * TesLorean CAN Test Interface
 * 2018
 * Jeff Cooke
 
 Instructions
 - 'p n' where n is 0..4  // Sends the predefined frame number n immediately once
 - 'f id b1 b2 b3 b4 b5 b6 b7 b8' send a frame with ID and 0..8 bytes, HEX entries should be 0xNN
 - 'q n id b1 b2 b3 b4 b5 b6 b7 b8' load a frame into predefined n with ID and 0..8 bytes, HEX entries should be 0xNN
 - 'r n nnnn' start repeating predefined frame n every nnnn miliseconds
 - 'r n 0' stop repeating predefined frame n
 - 'c 0/1' 1 to start CAN frame capture (and CSV output), 0 to stop

 Coding Todos
 - interpretation of CAN codes heard on the TesLorean CAN network (predefined subsets of CAN traffic on/off)

 Coding Notes
 - delay(1) needed to allow the Serial port to catch up with the microcontroller
 
*/

#include <mcp2515.h>
#include <SPI.h>

// DEFINE
#define PreDefinedFrames 5

// Create the CAN object for transmission
MCP2515 can0(10);     // CS on digital pin 10

// Array for preprepared CAN frames
can_frame CANPkg[PreDefinedFrames];
unsigned long CANDurs[PreDefinedFrames];       // Delay duration between frame sends
unsigned long CANTimes[PreDefinedFrames];       // Last time that a frame was sent

// Flags
bool CANread = false;     // Indicates if CAN frames should be captured and output to Serial as CSV

// Define the debug variable
#define debug 1

void setup()
{
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
  buildCANframe(CANPkg[0],0x000,8,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07);
  buildCANframe(CANPkg[1],0x001,8,0x01,0x01,0x02,0x03,0x04,0x05,0x06,0x07);
  buildCANframe(CANPkg[2],0x002,8,0x02,0x01,0x02,0x03,0x04,0x05,0x06,0x07);
  buildCANframe(CANPkg[3],0x003,8,0x03,0x01,0x02,0x03,0x04,0x05,0x06,0x07);
  buildCANframe(CANPkg[4],0x004,8,0x04,0x01,0x02,0x03,0x04,0x05,0x06,0x07);
  
  // Initialize the timers for the pre-built frames
  for (int t=0; t < PreDefinedFrames; t++)
  {
    CANDurs[t] = 0;        // Zero denotes an inactive timer
    CANTimes[t] = 0;       // init to 0 so always triggers immediately if set
  }
  
}

// Populate the CAN frame with the data specified
// When less that 8 bytes in frame, just fill out remainder with whatever...
void buildCANframe(can_frame & frame, uint16_t cid, uint8_t bcnt, uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3,  uint8_t b4, uint8_t b5, uint8_t b6, uint8_t b7)
{
  frame.can_id = cid;
  frame.can_dlc = bcnt;
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
void outputCANframe(can_frame & frame)
{
  // Send frame to the Serial Port
  Serial.print("CAN frame : ");
  Serial.print(frame.can_id,HEX);
  Serial.print(" - ");
  for (int i=0;i<frame.can_dlc;i++)
  {
    Serial.print(frame.data[i],HEX);
    Serial.print(" ");
  }
  Serial.println();  
}

#define PARSE_TIMEOUT 10
int parseHexInt()
{
    int result = 0;     // accumulate the parse int result here

    // arrays to help parse and convert the text numbers to values
    char hexes[23] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F','x','a','b','c','d','e','f'};
    int hexnm[23] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,0,10,11,12,13,14,15};
    char deces[10] = {'0','1','2','3','4','5','6','7','8','9'};

    char token[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    int numtok = 0;             // number of token chars
    unsigned long timesincechar = 0;
    bool timedout = false;      // timeout since last received character has occurred
    bool numchar = true;        // last char was numeric (either HEX or DEC)
    char nextchar;
    bool hextok = true;        // Does this comply with HEX notation?
    bool dectok = true;        // Does this comply with DEC notation?
    bool hexnot = false;        // Found hex 'x' notation
    bool inttok = true;        // Does this comply with DEC notation?
    bool spctok = true;         // still stripping leading spaces

    while (!timedout && numchar)
    {
        if (Serial.available() > 0)
        {
            nextchar = Serial.read();
            
            // test for hex compliance
            bool hexcom = false;
            for (int i=0;i<23;i++)
            {
                if (nextchar == hexes[i]){hexcom = true;}
            }
            if (!hexcom) {hextok = false;}

            // test for positive integer compliance
            bool deccom = false;
            for (int j=0;j<10;j++)
            {
                if (nextchar == deces[j]){deccom = true;}
            }
            if (!deccom) {dectok = false;}
            
            // If compliant, store the character
            if (hexcom || deccom)
            {
                token[numtok++] = nextchar;
                
                if (nextchar == 'x'){hexnot = true;}
            }
            else
            {
                // Test if a leading space (ignored)
                if ((spctok && nextchar != ' ') || (!spctok && nextchar == ' '))
                {
                    // don't read any more characters
                    numchar = false;
                }
          
            }
        }
        else
        {
            // Test timeout
            if (timesincechar < (millis() - PARSE_TIMEOUT)){timedout = true;}
        }
    }
    
    // Process if HEX was captured in the scan, must have the 0xNN notation to be recognized as HEX
    if (hexnot && hextok)
    {
        result = 0;
        for (int k=0;k<numtok;k++)
        {
            if (token[k] != 'x')
            {
                int d = 0;
                for (int n=0;n<23;n++)
                {
                    if (hexes[n] == token[k])
                    {
                        d = n;
                    }
                }
                result = (result * 16) + d;
            }
        }
    }
    
    // Process if DEC was captured in the scan
    if (dectok)
    {
        result = 0;
        for (int k=0;k<numtok;k++)
        {
            int d = 0;
            for (int n=0;n<10;n++)
            {
                if (deces[n] == token[k])
                {
                    d = n;
                }
            }
            result = (result * 10) + d;
        }
    }  
    return result;
}

void loop()
{
  // Define CAN Frame variable structure
  struct can_frame outgoing;
  struct can_frame incoming;

  // Check serial port for instructions
  char receivedChar;
  int receivedNum;
  if (Serial.available() > 0)
  {

    receivedChar = Serial.read();
    
    // Check for preprepared frame
    if (receivedChar == 'p')
    {
      delay(1);
      if (Serial.available() > 0)
      {
        receivedNum = Serial.parseInt();

        if ((receivedNum >= 0) && (receivedNum < PreDefinedFrames))
        {
          // Send the selected prepackaged CAN frame
          can0.sendMessage(&CANPkg[receivedNum]);

          // Send frame to the Serial Port
          Serial.println("Preprepared CAN Frame sent...");
          outputCANframe(CANPkg[receivedNum]);
        }
        else
        {
          Serial.print("Error: Predefined frame number not within 0..");
          Serial.print(PreDefinedFrames,DEC);
          Serial.println(" range.");
        }
      }
      else
      {
        Serial.println("Error: Need to specify which predefined frame to send.");
      }
    }    // end of 'p'

    // Start or Stop the CAN frame capture
    if (receivedChar == 'c')
    {
      delay(1);
      if (Serial.available() > 0)
      {
        receivedNum = Serial.parseInt();

        if (receivedNum == 0)
        {
          CANread = false;
          Serial.println("Stop catching received CAN frames...");
        }
        if (receivedNum == 1)
        {
          CANread = true;
          Serial.println("Start catching received CAN frames...");
        }
      }
      else
      {
        Serial.println("Error: Need to specify 0 - stop or 1 - start frame capture.");
      }
    }    // end of 'c'

    // Check for repeat start/stop requests
    if (receivedChar == 'r')
    {
      delay(1);
      if (Serial.available() > 0)
      {
        receivedNum = Serial.parseInt();

        if ((receivedNum >= 0) && (receivedNum < PreDefinedFrames))
        {

          // Fetch the millisecond delay duration
          unsigned long fdur = 0;
          delay(1);
          if (Serial.available() > 0)
          {
            fdur = Serial.parseInt();
            
            // Update the delay duration for this predefined frame
            CANDurs[receivedNum] = fdur;        // Zero denotes an inactive timer
            CANTimes[receivedNum] = 0;       // init to 0 so always triggers immediately if set

            if (fdur == 0)
            {
              // Send frame to the Serial Port
              Serial.println("Repeat CAN request cleared...");
            }
            else
            {
              // Send frame to the Serial Port
              Serial.println("Repeat CAN request set...");
            }
          }
        }
        else
        {
          Serial.print("Error: Predefined frame number not within 0..");
          Serial.print(PreDefinedFrames,DEC);
          Serial.println(" range.");
        }
      }
    }    // end of 'r'

    // Check for full specified frame
    if (receivedChar == 'f')
    {
      bool fsuc = false;    // at least some frame ID found
      bool bfin = false;   // No more byte data available
      int bcnt = 0;         // number of data bytes found
      delay(1);
      
      if (Serial.available() > 0)
      {
        // Grab the ID for the frame
        outgoing.can_id = parseHexInt();
        fsuc = true;
        
        // There could another 8 data entries for the frame
        for (int i = 0; i < 8; i++)
        {
          if (!bfin)
          {
            delay(1);
            if (Serial.available() > 0)
            {
              outgoing.data[i] = parseHexInt();
              bcnt = i + 1;
            }
            else
            {
              bfin = true;      // Flag that no more bytes were found
            }
          }
        }
        outgoing.can_dlc = bcnt;        // Set the number of data bytes in the CAN frame
      }
      
      // Output status of the frame input
      if (fsuc)
      {
        Serial.println("CAN Frame sent...");
        can0.sendMessage(&outgoing);
        outputCANframe(outgoing);      
      }
      else
      {
        Serial.println("No CAN Frame sent...");
      }

    }  // end of 'f'

    // Check for frame to load into predefined array
    if (receivedChar == 'q')
    {
      bool fsuc = false;    // at least some frame ID found
      bool bfin = false;   // No more byte data available
      int bcnt = 0;         // number of data bytes found
      int rNum = 0;         // number of predefined entry to replace
      delay(1);
      
      // Test for the predefined record num
      if (Serial.available() > 0)
      {
        rNum = Serial.parseInt();

        // Check that entry is within range
        if ((rNum >= 0) && (rNum < PreDefinedFrames))
        {

          delay(1);
          if (Serial.available() > 0)
          {
            // Grab the ID for the frame
            CANPkg[rNum].can_id = parseHexInt();
            fsuc = true; // Sufficient frame data found
        
            // There could another 8 data entries for the frame
            for (int i = 0; i < 8; i++)
            {
              if (!bfin)
              {
                delay(1);
                if (Serial.available() > 0)
                {
                  CANPkg[rNum].data[i] = parseHexInt();
                  bcnt = i + 1;
                }
                else
                {
                  bfin = true;      // Flag that no more bytes were found
                }
              }
            }
            CANPkg[rNum].can_dlc = bcnt;        // Set the number of data bytes in the CAN frame
          }
        }
        else
        {
          Serial.print("Error: Predefined frame number not within 0..");
          Serial.print(PreDefinedFrames,DEC);
          Serial.println(" range.");
        }
      }
      
      // Output status of the frame input
      if (fsuc)
      {
        Serial.println("CAN Frame loaded...");
        outputCANframe(outgoing);      
      }
      else
      {
        Serial.println("No CAN Frame loaded...");
      }

    }  // end of 'q'

  }  // end of serial port commands check
  

  // Check for repeating timed frame sending
  for (int t = 0; t < PreDefinedFrames; t++)
  {
    // Duration must be > 0 for the timed frames to be active
    if (CANDurs[t] != 0)
    {
      // Check if the duration has passed for this repeated frame
      if (CANTimes[t] < (millis() - CANDurs[t]))
      {
        // Update the timer
        CANTimes[t] = millis();
      
        // Send the selected prepackaged CAN frame
        can0.sendMessage(&CANPkg[t]);

        // Send frame to the Serial Port
        Serial.println("Preprepared CAN Frame repeated...");
        outputCANframe(CANPkg[t]);
      }
    }
  }  // end of repeat check
  
  // Receive CAN frame check
  if (CANread)
  {
    // check if frame waiting
    if (can0.readMessage(&incoming) == MCP2515::ERROR_OK)
    {

      // Output FRAME ID and bytes in CSV format
      Serial.print("IN,");
      Serial.print(incoming.can_id, HEX); // print ID
      Serial.print(","); 
      Serial.print(incoming.can_dlc, DEC); // print DLC
      Serial.print(",");

      for (int i = 0; i < incoming.can_dlc; i++)
      {
        Serial.print(incoming.data[i],HEX);
        Serial.print(",");
      }
      Serial.println();      
    }
  }    // end of message capture  
}
