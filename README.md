# TesLCANTest
TesLorean CAN Sender/Receiving Testing Tool

I needed a simple CAN tool that would run through the Serial Monitor and allow an Arduino to send CAN frames (pre-programmed or defined on the fly).  This tool works with the MCP2515_CAN breakout board.  I use it to send test frames to the other TesLorean controller modules to test their responses.

Commands (entered into Serial Monitor)
 Instructions
 - 'p n' where n is 0..4  // Sends the predefined frame number n immediately once
 - 's id b1 b2 b3 b4 b5 b6 b7 b8' send a frame with ID and 0..8 bytes, HEX entries should be 0xNN
 - 'q n id b1 b2 b3 b4 b5 b6 b7 b8' load a frame into predefined n with ID and 0..8 bytes, HEX entries should be 0xNN
 - 'r n nnnn' start repeating predefined frame n every nnnn miliseconds
 - 'r n 0' stop repeating predefined frame n
 - 'c 0/1' 1 to start CAN frame capture (and CSV output), 0 to stop
 [Filter and Mask Functionality disabled
 - 'm 0/1 0xnnnn'  to set ID mask 0 or 1 to 0xNNNN, with value nnnn
 - 'm 0/1 0' to clear mask 0 or 1
 - 'f 0..5 0xnnnn' to set the filter 0..1 for Mask 0, and 2..5 for Mask 1, with value nnnn
 - 'f 0..5 0' to clear filter 0..5- '?' to reshow the instructions]
 
ALSO....

int parseHexInt()
=================
Works just like parseInt(), except that it can also accept values input as hex values, i.e. '0xNN'.  Values can be any mix of decimal or hex, but hex values must use the '0x00' notation to be recognized.  Returns -1 if the input cannot be interpreted as a number in either a decimal or hexidecimal notation.

Notes of Masks and Filters in the MCP2515 CAN chip
==================================================
Best explanation of how to set Masks and Filters, here... https://www.microchip.com/forums/m456043.aspx

DEVELOPMENT NOTES
=================
- FIXED : The capture command "c 1" will miss frames when the data transmission rate is very high (e.g. 1000KBPS).  This is because the frames come in faster than the microcontroller can capture and handle them - especially if Serial.print() is used. The test code in the CANTestToolInterrupt addresses the problem by using Interrupts and Circular Queue.  The CANSender_Tool.ino still needs to be updated to use the interrupt/queue method. SOLUTION : Implemented a circular array with overwrite.  This allows frames to get captured and processed at (somewhat) leisure.  Incoming frames are added to the circular array overwriting the oldest frame, separately, the reader progresses through frames as quickly as possible.  Some frames may not get processed.  This is a minor risk so long as 1) frames of interest are sent repeatedly (i.e. just updating a previous reading), or 2) instruction frames are resent if they do not get a confirmed arrival.
