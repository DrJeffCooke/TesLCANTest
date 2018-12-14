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
 - 'm 0/1 0xnnnn'  to set ID mask 0 or 1 to 0xNNNN, with value nnnn
 - 'm 0/1 0' to clear mask 0 or 1
 - 'f 0..5 0xnnnn' to set the filter 0..1 for Mask 0, and 2..5 for Mask 1, with value nnnn
 - 'f 0..5 0' to clear filter 0..5- '?' to reshow the instructions
 
ALSO....

int parseHexInt()
=================
Works just like parseInt(), except that it can also accept values input as hex values, i.e. '0xNN'.  Values can be any mix of decimal or hex, but hex values must use the '0x00' notation to be recognized.  Returns -1 if the input cannot be interpreted as a number in either a decimal or hexidecimal notation.

Notes of Masks and Filters in the MCP2515 CAN chip
==================================================
Best explanation of how to set Masks and Filters, here... https://www.microchip.com/forums/m456043.aspx
