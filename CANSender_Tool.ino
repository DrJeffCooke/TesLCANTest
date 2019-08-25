/*
 * TesLorean CAN Test Interface
 * 2018
 * Jeff Cooke
 
 Instructions
 - 'p n' where n is 0..4  // Sends the predefined frame number n immediately once
 - 's id b1 b2 b3 b4 b5 b6 b7 b8' send a frame with ID and 0..8 bytes, HEX entries should be 0xNN
 - 'q n id b1 b2 b3 b4 b5 b6 b7 b8' load a frame into predefined n with ID and 0..8 bytes, HEX entries should be 0xNN
 - 'r n nnnn' start repeating predefined frame n every nnnn miliseconds
 - 'r n 0' stop repeating predefined frame n
 - 'c 0/1' 1 to start CAN frame capture (and CSV output), 0 to stop
 - '?' to reshow the instructions
 
 Commented Out
 - 'm 0/1 0xnnnn'  to set ID mask 0 or 1 to 0xNNNN, with value nnnn
 - 'm 0/1 0' to clear mask 0 or 1
 - 'f 0..5 0xnnnn' to set the filter 0..1 for Mask 0, and 2..5 for Mask 1, with value nnnn
 - 'f 0..5 0' to clear filter 0..5
 - 'b nnnn' to set baud rate in KBPS, so 1000 = 1MBPS, 500 = 500KBPS, 250, 200, 125, 100, etc.

 Coding Todos
 - interpretation of CAN codes heard on the TesLorean CAN network (predefined subsets of CAN traffic on/off)
 - Change the Baud rate

 Coding Notes
 - delay(1) needed to allow the Serial port to catch up with the microcontroller

 //check
*/

struct can_frame {
    uint32_t  can_id;
    uint8_t   can_dlc;
    uint8_t   data[8];
};

// Variables for the CAN traffic
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];

// Timestamp for stay alone message
unsigned long timekeepalive;

//#include <mcp2515.h>
#include <mcp_can.h>
#include <SPI.h>

// CAN board Chip Select (CS) pins
#define TESLOREAN_CAN_CS 10
#define TESLOREAN_CAN_INT 2

// DEFINE
#define StoredFrames 50

// Array for preprepared CAN frames
can_frame CANFrames[StoredFrames];

// Counters tracking the add point and read point in the circular array
volatile uint8_t addPointFrames = 0;
volatile uint8_t readPointFrames = 0;
volatile uint16_t netAddReadCount = 0;   // Adds increment, Reads decrement, only Read if <> 0
volatile bool activeAccess = false;

// DEFINE
#define PreDefinedFrames 20

// Array for preprepared CAN frames
can_frame CANPkg[PreDefinedFrames];
unsigned long CANDurs[PreDefinedFrames];       // Delay duration between frame sends
unsigned long CANTimes[PreDefinedFrames];       // Last time that a frame was sent

// Flags
bool CANread = false;     // Indicates if CAN frames should be captured and output to Serial as CSV

// Define the debug variable
#define debug 1

// Create the CAN object for transmission
MCP_CAN can0(TESLOREAN_CAN_CS);      // TESLOREAN CAN - CS on digital pin 9/10

struct can_frame frame;

/*
void irqBATHandler()
{
//  uint8_t irq = can0.getInterrupts();

  if(!digitalRead(TESLOREAN_CAN_INT))   // pin TESLOREAN_CAN_INT low if data on can0
  {
    if (!activeAccess)
    {
      can0.readMsgBuf(&CANFrames[addPointFrames].can_dlc, CANFrames[addPointFrames].data);
      CANFrames[addPointFrames].can_id = can0.getCanId();
  
      // frame contains received from RXB0 message
      addPointFrames = (addPointFrames + 1) % StoredFrames;
      netAddReadCount++;    
    }
  }
}
*/
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

  // Prepare the interrupt pins
  pinMode(TESLOREAN_CAN_INT, INPUT);

  // Startup CAN  Battery bus
  can0.begin(CAN_500KBPS);
//  can0.begin(CAN_1000KBPS);
  #ifdef debug
    Serial.println("Test CANbus 0 initialized");
  #endif

  // Initialize the array pointers
  addPointFrames = 0;
  readPointFrames = 0;
  netAddReadCount = 0;

  // Set up the interrupt to capture incoming CAN frames
//  attachInterrupt(digitalPinToInterrupt(TESLOREAN_CAN_INT), irqBATHandler, LOW);
  
  // Output the instructions for frames
  outputInstructions();
  timekeepalive = millis();

  //// Populate a number of pre-built CAN frames

  // Battery Pack Contactors control
  buildCANframe(CANPkg[0],0x132,8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00); // Turn off battery high voltage contactors
  buildCANframe(CANPkg[1],0x132,8,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00); // Turn on battery high voltage contactors
  buildCANframe(CANPkg[2],0x142,8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00); // Turn off battery pack accessories
  buildCANframe(CANPkg[3],0x142,8,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00); // Turn on battery pack accessories
  buildCANframe(CANPkg[4],0x151,8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00); // Initiate send of all battery cell voltages

  // Charger control
  buildCANframe(CANPkg[5],0x181,8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00); // Stop charging
  buildCANframe(CANPkg[6],0x181,8,0x01,0x90,0x01,0x05,0x00,0x90,0x01,0x01); // Start the charger with 400v, 5amps, module 1

  // Available
  buildCANframe(CANPkg[7],0x151,8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
  buildCANframe(CANPkg[8],0x151,8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
  buildCANframe(CANPkg[9],0x151,8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
  buildCANframe(CANPkg[10],0x151,8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
  buildCANframe(CANPkg[11],0x151,8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
  buildCANframe(CANPkg[12],0x151,8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
  buildCANframe(CANPkg[13],0x151,8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
  buildCANframe(CANPkg[14],0x151,8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
  buildCANframe(CANPkg[15],0x151,8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
  buildCANframe(CANPkg[16],0x151,8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
  buildCANframe(CANPkg[17],0x151,8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
  buildCANframe(CANPkg[18],0x151,8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
  buildCANframe(CANPkg[19],0x151,8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);
  
  // Initialize the timers for the pre-built frames
  for (int t=0; t < PreDefinedFrames; t++)
  {
    CANDurs[t] = 0;        // Zero denotes an inactive timer
    CANTimes[t] = 0;       // init to 0 so always triggers immediately if set
  }

  Serial.println("Ready...");
}  // end of Setup

// Print out the instructions
void outputInstructions()
{
  Serial.println("Instructions");  
  Serial.println("'p n' where n is 0..4  // Sends the predefined frame number n immediately once");
  Serial.println("'s id b1 b2 b3 b4 b5 b6 b7 b8' send a frame with ID and 0..8 bytes, HEX entries should be 0xNN");
  Serial.println("'q n id b1 b2 b3 b4 b5 b6 b7 b8' load a frame into predefined n with ID and 0..8 bytes, HEX entries should be 0xNN");
  Serial.println("'r n nnnn' start repeating predefined frame n every nnnn miliseconds");
  Serial.println("'r n 0' stop repeating predefined frame n");
  Serial.println("'c 0/1' 1 to start CAN frame capture (and CSV output), 0 to stop");
/*
  Serial.println("'m 0/1 0xnnnn'  to set ID mask 0 or 1 to 0xNNNN, with value nnnn");
  Serial.println("'m 0/1 0' to clear mask 0 or 1");
  Serial.println("'f 0..5 0xnnnn' to set the filter 0..1 for Mask 0, and 2..5 for Mask 1, with value nnnn");
  Serial.println("'f 0..5 0' to clear filter 0..5");
  Serial.println("'b nnnn' to set baud rate in KBPS, so nnnn = 1000 is 1MBPS, 500 is 500KBPS, 250, 200, 125, 100, etc.");
*/
  Serial.println("'?' to reshow the instructions");
  Serial.println("Preprogrammed Frames...");
  Serial.println("'p0' 'p1' switch off/on HV battery contactors");
  Serial.println("'p2' 'p3' switch off/on battery accessories");
  Serial.println("'p4' initiate cell voltage output");
  Serial.println("'p5' 'p6' stop/start battery charging");
  unsigned long time;
  time = millis();
  Serial.print("Time : ");
  Serial.println(time);
  delay(1000);  
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

// For some frames (i.e. warnings) output a full text explanation
uint8_t interpretCANframe(can_frame & frame)
{
  // Temp vars
  uint8_t b0,b1,b2,b3,b4,b5,b6,b7;
  bool frameidknown;
  frameidknown = false;

  if (frame.can_dlc > 0){b0 = frame.data[0];}
  if (frame.can_dlc > 1){b1 = frame.data[1];}
  if (frame.can_dlc > 2){b2 = frame.data[2];}
  if (frame.can_dlc > 3){b3 = frame.data[3];}
  if (frame.can_dlc > 4){b4 = frame.data[4];}
  if (frame.can_dlc > 5){b5 = frame.data[5];}
  if (frame.can_dlc > 6){b6 = frame.data[6];}
  if (frame.can_dlc > 7){b7 = frame.data[7];}

  switch (frame.can_id)
  {
    case 0x111: // BattWarnTFrameID

      Serial.print("BCM: 0x111 - Battery Temperature Warning [");
      if (b0 == 0xFF){Serial.print("HH ");}
      if (b0 == 0xF0){Serial.print("H ");}
      if (b0 == 0x0F){Serial.print("L ");}
      if (b0 == 0x00){Serial.print("LL ");}

      int warntemp;
      //warntemp = ((uint16_t)b2 << 8) + b1;
      warntemp = b1;
      int tempC;
      tempC = (int)((warntemp >> 1) - 40);
      Serial.print(warntemp,DEC);
      
      if (b3 == 0)
      {
        Serial.print("C in Pack ");
      }
      if (b3 == 1)
      {
        Serial.print("C in Module");
      }
      Serial.print(b4,DEC);
      Serial.println("]");

      frameidknown = true;

      break;

    case 0x110: // BattWarnVFrameID

      Serial.print("BCM: 0x110 - Battery Voltage Warning [");
      if (b0 == 0xFF){Serial.print("HH ");}
      if (b0 == 0xF0){Serial.print("H ");}
      if (b0 == 0x0F){Serial.print("L ");}
      if (b0 == 0x00){Serial.print("LL ");}

      uint16_t warnvolt;
      uint16_t volts;
      volts = b2;
      warnvolt = (uint16_t)((volts << 8) + b1);
      Serial.print(warnvolt,DEC);

      if (b3 == 0)
      {
        Serial.print("V for ");
        Serial.print("Total Pack");
      }
      if (b3 == 1)
      {
        Serial.print("V/100th for ");
        Serial.print("Singe Cell");
      }
      Serial.println("]");

      frameidknown = true;

      break;

    case 0x141: // Battery Status Frame

      Serial.print("BCM: 0x141 - Battery Status Message [");
      Serial.print("Cont:");
      Serial.print(frame.data[0],DEC);
      Serial.print(" BMS:");
      Serial.print(frame.data[1],DEC);
      
      uint16_t totalpackvolts;
      uint16_t mincellvolts;
      uint16_t maxcellvolts;

      totalpackvolts = b3;
      totalpackvolts = (uint16_t)((totalpackvolts << 8) + b2);
      Serial.print(" Pack(V):");
      Serial.print(totalpackvolts,DEC);

      mincellvolts = b5;
      mincellvolts = (uint16_t)((mincellvolts << 8) + b4);
      Serial.print(" Min Cell V/100:");
      Serial.print(mincellvolts,DEC);

      maxcellvolts = b7;
      maxcellvolts = (uint16_t)((maxcellvolts << 8) + b6);
      Serial.print(" Max Cell V/100:");
      Serial.print(maxcellvolts,DEC);

      Serial.println("]");
      frameidknown = true;

      break;
      
    case 0x131: // Battery Status Frame

      Serial.print("BCM: 0x131 - Battery Status Message 2 [");
     
      int voltchangerate;    // volts per 100th of a minute
      int packtemp0;
      int packtemp1;

      voltchangerate = b5;
      voltchangerate = (int16_t)((voltchangerate << 8) + b4);
      Serial.print(" Delta (V/100thMin):");
      Serial.print(voltchangerate,DEC);

      if (b0 !=0)
      {
        packtemp0 =  b0; //((uint16_t)b1 << 8) + b0;
        int temp0C;
        temp0C = (int)((packtemp0 >> 1) - 40);
        Serial.print(", Pack0: ");
        Serial.print(temp0C,DEC);
      }
      else
      {
        Serial.print(", Pack0: ##");        
      }

      if (b0 !=0)
      {
        packtemp1 = b2; //((uint16_t)b3 << 8) + b2;
        int temp1C;
        temp1C = (int)((packtemp1 >> 1) - 40);
        Serial.print("C, Pack1: ");
        Serial.print(temp1C,DEC);
        Serial.print("C");
      }
      else
      {
        Serial.print("C, Pack1: ##C");
      }

      Serial.println("]");
      frameidknown = true;

      break;
      
    case 0x133: // Battery Module Temps

      Serial.print("BCM: 0x133 - Battery Modules Temps [");
      for (int i = 0 ; i < 8; i++)
      {
        // Convert temp to C
        int tempC;
        tempC = (frame.data[i]>>1)-40;
        Serial.print(tempC,DEC);
        Serial.print("C ");
      }
      Serial.println("]");
      frameidknown = true;

      break;

    default:
      // if nothing else matches, do the default
      break;

   }

   if (frameidknown)
   {
      return 1;
   }
   else
   {
      return 0;
   }
}

void sendMessage(MCP_CAN canx, can_frame & frame)
{
  canx.sendMsgBuf(frame.can_id, 0, frame.can_dlc, frame.data);
}

#define PARSE_TIMEOUT 10
int parseHexInt()
{
    int result = 0;     // accumulate the parse int result here

    // arrays to help parse and convert the text numbers to values
    char hexes[23] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F','x','a','b','c','d','e','f'};
    int hexnm[23] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,0,10,11,12,13,14,15};
    char deces[10] = {'0','1','2','3','4','5','6','7','8','9'};
    int decnm[10] = {0,1,2,3,4,5,6,7,8,9};

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
    bool errtok = false;        // Indicates if an error was detected, i.e. a non-DEC or HEX value

    while (!timedout && numchar)
    {
      if (Serial.available() > 0)
      {
        nextchar = Serial.read();
        timesincechar = millis();   // Set the timer for when the character was received
        
        // If newline found, just end the token read
        if (nextchar == 10)
        {
          numchar = false;
        }
        else
        {
          // If not clearing leading spaces, and see a space, stop reading characters
          if (!spctok && nextchar == ' ')
          {
            numchar = false;
          }
          else
          {
            // If stripping spaces, and space found, just skip the character checks
            if (spctok && nextchar == ' ')
            {
              // Skipping the space by not processing
            }
            else
            {
              // If stripping leading spaces, and you find a non-space, stop ignoring leading spaces
              if (spctok && nextchar != ' ')
              {
                // Stop ignoring leading spaces
                spctok = false;
              }
    
              // test for hex compliance
              bool hexcom = false;
              for (int i=0;i<23;i++)
              {
                  if (nextchar == hexes[i]){hexcom = true;}
              }
              if (!hexcom)
              {
                hextok = false;
              }
              else
              {
                // Test if this could be a hex notation value '0xNN'
                if (numtok == 1 && nextchar == 'x')
                {
                  hexnot = true;
                }

                // Throw an error if the 'x' is in the wrong place
                if (numtok != 1 && nextchar == 'x')
                {
                  hexnot = false;
                  errtok = true;
                }
              }
                  
              // test for positive integer compliance
              bool deccom = false;
              for (int j=0;j<10;j++)
              {
                  if (nextchar == deces[j]){deccom = true;}
              }
              if (!deccom)
              {
                dectok = false;
              }
              
              // If compliant, store the character
              if (hexcom || deccom)
              {
                  // Capture the character
                  token[numtok] = nextchar;
                  numtok = numtok + 1;
              }
              else
              {
                // Flag an error as a non-valid character was located
                errtok = true;
              }
            }
          }
        }
      }
      else
      {
          // Test timeout
          if (millis() > timesincechar + PARSE_TIMEOUT)
          {timedout = true;}
      }
    } // end of While

    // Check that a valid hex notation was received
    if (hextok && !hexnot && !dectok)
    {
      // Looks like a HEX notation, but no 'x' found
      errtok = true;
    }

    if (!errtok)
    {
    
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
                          d = hexnm[n];
                      }
                  }
                  result = (result * 16) + d;
              }
          }
      }
      else
      {
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
                        d = decnm[n];
                    }
                }
                result = (result * 10) + d;
            }
        }
      }
    }
    else
    {
      // Error detected (non-DEC and non-HEX compliance)
      result = -1;
    }

    return result;
}

void loop()
{

  // Set a flag to indicate if a CAN frame was just read
  bool readCANSuccess;
  readCANSuccess = false;

  // Check if there is a CAN frame to read
  if(!digitalRead(TESLOREAN_CAN_INT))   // pin TESLOREAN_CAN_INT low if data on can0
  {
    if (!activeAccess)
    {
      can0.readMsgBuf(&CANFrames[addPointFrames].can_dlc, CANFrames[addPointFrames].data);
      CANFrames[addPointFrames].can_id = can0.getCanId();
      readCANSuccess = true;
  
      // frame contains received from RXB0 message
      addPointFrames = (addPointFrames + 1) % StoredFrames;
      netAddReadCount++;    
    }
  }

  // Receive CAN frame check
  if (CANread)
  {
    // Output a captured frame
    if (netAddReadCount > 0)
    {
      if (readPointFrames != addPointFrames)
      {
        // Copy data into temp store
        activeAccess = true;
        can_frame frame0;
        frame0.can_id = CANFrames[readPointFrames].can_id;
        frame0.can_dlc = CANFrames[readPointFrames].can_dlc;
        for (int i = 0; i < 8; i++)
        {
          frame0.data[i] = CANFrames[readPointFrames].data[i];
        }
        readPointFrames = (readPointFrames + 1) % StoredFrames;
        if(netAddReadCount > 0){netAddReadCount--;}
        activeAccess = false;
        
        // Interpret the frame
        uint8_t foundid;
        foundid = interpretCANframe(frame0);
        // Output the CAN data if the frame was not recognized
        if (foundid==0){outputCANframe(frame0);}

      }
    }
  }    // end of message capture

  // No CAN to catch so just look for Serial commands
  if(!readCANSuccess)
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
      if (receivedChar == '?')
      {
        // Output the instructions for frames
        outputInstructions();
      } // end of '?'
      
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
            sendMessage(can0, CANPkg[receivedNum]);
  
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
  /*
      // Set the baud rate
      if (receivedChar == 'b')
      {
        delay(1);
        if (Serial.available() > 0)
        {
          receivedNum = Serial.parseInt();
  
          if (receivedNum == 1000 || receivedNum == 500 || receivedNum == 250 || receivedNum == 200 || receivedNum == 125 || receivedNum == 100)
          {
            MCP2515::ERROR canError;
            canError = MCP2515::ERROR_OK;
            can0.setConfigMode();
            
            if(receivedNum == 1000){canError = can0.setBitrate(CAN_1000KBPS);}
            if(receivedNum == 500){canError = can0.setBitrate(CAN_500KBPS);}
            if(receivedNum == 250){canError = can0.setBitrate(CAN_250KBPS);}
            if(receivedNum == 200){canError = can0.setBitrate(CAN_200KBPS);}
            if(receivedNum == 125){canError = can0.setBitrate(CAN_125KBPS);}
            if(receivedNum == 100){canError = can0.setBitrate(CAN_100KBPS);}
            
            if (canError == MCP2515::ERROR_OK)
            {
              Serial.print("Baud rate set to ");
              Serial.println(receivedNum, DEC);
            }
            else
            {
              Serial.print("Error : Baud rate unchanged.");
            }
            can0.setNormalMode();
          }
          else
          {
            Serial.println("Enter a valid baud rate (BPS). Values are 1000, 500, 250, 200, 125, and 100.");
          }
        }
        else
        {
          Serial.println("Error: Need to specify a baud rate in BPS.");
        }
      }    // end of 'b'
  */
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
  /*
      // Check for Mask setup request
      if (receivedChar == 'm')
      {
        delay(1);
        if (Serial.available() > 0)
        {
          receivedNum = Serial.parseInt();
  
          // Only masks are 0 and 1
          if ((receivedNum == 0) || (receivedNum == 1))
          {
            // Fetch the mask value
            unsigned long mask = 0;
            delay(1);
            if (Serial.available() > 0)
            {
              mask = parseHexInt();
  
              // Check that mask is valid and assign to CANBUS
              if (mask != -1)
              {
                MCP2515::MASK rxMask;
                if (receivedNum==0){rxMask = MCP2515::MASK0;}
                else {rxMask = MCP2515::MASK1;}
  
                can0.setConfigMode();
                if (can0.setFilterMask(rxMask,0,(uint32_t)mask) != MCP2515::ERROR_OK)
                {
                  Serial.println("Mask was not set successfully.");
                  mask = -1;
                }
                can0.setNormalMode();
              }
              else
              {
                Serial.println("Mask value not recognized as a valid value."); 
              }
  
              if (mask == 0)
              {
                Serial.print("Mask ");
                Serial.print(receivedNum, DEC);
                Serial.println(" cleared.");
              }
              if (mask != -1 && mask > 0)
              {
                // Send frame to the Serial Port
                Serial.print("Mask ");
                Serial.print(receivedNum, DEC);
                Serial.print(" set to 0x");
                Serial.println(mask, HEX);
              }
            }
          }
          else
          {
            Serial.print("Error: Only Mask 0 or 1 can be defined.");
          }
        }
      }    // end of 'm'
  */
  /*    
      // Check for Filter setup request
      if (receivedChar == 'f')
      {
        delay(1);
        if (Serial.available() > 0)
        {
          receivedNum = Serial.parseInt();
  
          // Only masks are 0 and 1
          if ((receivedNum >= 0) || (receivedNum <= 5))
          {
            // Fetch the filter value
            unsigned long filter = 0;
            delay(1);
            if (Serial.available() > 0)
            {
              filter = parseHexInt();
  
              // Check that mask is valid and assign to CANBUS
              if (filter != -1)
              {
                MCP2515::RXF rxFilter;
                if (receivedNum == 0){rxFilter = MCP2515::RXF0;}
                if (receivedNum == 1){rxFilter = MCP2515::RXF1;}
                if (receivedNum == 2){rxFilter = MCP2515::RXF2;}
                if (receivedNum == 3){rxFilter = MCP2515::RXF3;}
                if (receivedNum == 4){rxFilter = MCP2515::RXF4;}
                if (receivedNum == 5){rxFilter = MCP2515::RXF5;}
  
                can0.setConfigMode();
                if (can0.setFilter(rxFilter, 0,(uint32_t)filter) != MCP2515::ERROR_OK)
                {
                  Serial.println("Filter setting returned an error.");
                  filter = -1;
                }
                can0.setNormalMode();
              }
              else
              {
                Serial.println("Filter value not recognized as a valid value.");
              }
  
              if (filter == 0)
              {
                Serial.print("Filter ");
                Serial.print(receivedNum, DEC);
                Serial.println(" cleared.");
              }
              if (filter != -1 && filter > 0)
              {
                // Send frame to the Serial Port
                Serial.print("Filter ");
                Serial.print(receivedNum, DEC);
                Serial.print(" set to 0x");
                Serial.println(filter, HEX);
              }
            }
          }
          else
          {
            Serial.print("Error: Only Filters 0..5 can be defined.");
          }
        }
      }    // end of 'f'
  */    
      // Check for full specified frame
      if (receivedChar == 's')
      {
        bool fsuc = false;    // at least some frame ID found
        bool bfin = false;   // No more byte data available
        bool ferr = false;    // error detected in the input data
        int bcnt = 0;         // number of data bytes found
        delay(1);
        
        if (Serial.available() > 0)
        {
          // Fetch value and check for error
          int canidtemp = 0;
          canidtemp = parseHexInt();
          if (canidtemp >= 0)
          {
            // Grab the ID for the frame
            outgoing.can_id = canidtemp;
            fsuc = true;
            
            // There could another 8 data entries for the frame
            for (int i = 0; i < 8; i++)
            {
              if (!bfin)
              {
                delay(1);
                if (Serial.available() > 0)
                {
                  int datavaluetemp = 0;
                  datavaluetemp = parseHexInt();
                  if (datavaluetemp >= 0)
                  {
                    outgoing.data[i] = datavaluetemp;
                    bcnt = i + 1;
                  }
                  else
                  {
                    // Record error
                    ferr = true;
                    Serial.print("Error in frame byte ");
                    Serial.println(i,DEC);
                  }
                }
                else
                {
                  bfin = true;      // Flag that no more bytes were found
                }
              }
            }
            outgoing.can_dlc = bcnt;        // Set the number of data bytes in the CAN frame
          }
          else
          {
            ferr = true;
            Serial.println("Error detected in input CAN ID.");
          }  // end of error check
        }
        
        // Output status of the frame input
        if (fsuc and !ferr)
        {
          Serial.println("CAN Frame sent...");
          sendMessage(can0, outgoing);
          outputCANframe(outgoing);      
        }
        else
        {
          Serial.println("No CAN Frame sent...");
        }
  
      }  // end of 's'
  
      // Check for frame to load into predefined array
      if (receivedChar == 'q')
      {
        bool fsuc = false;    // at least some frame ID found
        bool bfin = false;   // No more byte data available
        bool ferr = false;    // indicates an error found in input
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
              int canidtemp = 0;
              canidtemp = parseHexInt();
              if (canidtemp >= 0)
              {
                CANPkg[rNum].can_id = canidtemp;
                fsuc = true; // Sufficient frame data found
            
                // There could another 8 data entries for the frame
                for (int i = 0; i < 8; i++)
                {
                  if (!bfin)
                  {
                    delay(1);
                    if (Serial.available() > 0)
                    {
                      int datavaluetemp = 0;
                      datavaluetemp = parseHexInt();
                      if (datavaluetemp >= 0)
                      {
                        CANPkg[rNum].data[i] = datavaluetemp;
                        bcnt = i + 1;
                      }
                      else
                      {
                        // Record error
                        ferr = true;
                        Serial.print("Error in frame byte ");
                        Serial.println(i,DEC);
                      }
                    }
                    else
                    {
                      bfin = true;      // Flag that no more bytes were found
                    }
                  }
                }
                CANPkg[rNum].can_dlc = bcnt;        // Set the number of data bytes in the CAN frame
              }
              else
              {
                ferr = true;
                Serial.println("Error detected in input CAN ID.");
              }  // end of error check
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
        if (fsuc && !ferr)
        {
          Serial.println("CAN Frame loaded...");
          outputCANframe(CANPkg[rNum]);      
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
          sendMessage(can0, CANPkg[t]);
  
          // Send frame to the Serial Port
          Serial.println("Preprepared CAN Frame repeated...");
          outputCANframe(CANPkg[t]);
        }
      }
    }  // end of repeat check
    
  } // end of !readCANSuccess
}
