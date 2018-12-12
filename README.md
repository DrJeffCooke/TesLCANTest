# TesLCANTest
TesLorean CAN Sender/Receiving Testing Tool

I needed a simple CAN tool that would run through the Serial Monitor and allow an Arduino to send CAN frames (pre-programmed or defined on the fly).  This tool works with the MCP2515_CAN breakout board.  I use it to send test frames to the other TesLorean controller modules to test their responses.

Commands (entered into Serial Monitor)
 Instructions
 - 'p n' where n is 0..4  // Sends the predefined frame number n immediately once
 - 'f id b1 b2 b3 b4 b5 b6 b7 b8' send a frame with ID and 0..8 bytes, HEX entries should be 0xNN
 - 'q n id b1 b2 b3 b4 b5 b6 b7 b8' load a frame into predefined n with ID and 0..8 bytes, HEX entries should be 0xNN
 - 'r n nnnn' start repeating predefined frame n every nnnn miliseconds
 - 'r n 0' stop repeating predefined frame n
 - 'c 0/1' 1 to start CAN frame capture (and CSV output), 0 to stop
 - '?' to reshow the instructions
 
ALSO....

int parseHexInt()
=================
Works just like parseInt(), except that it can also accept values input hex values, i.e. '0xNN'.  Values can be any mix of decimal or hex, but hex values must use the '0x00' notation to be recognized.  
