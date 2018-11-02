# TesLCANTest
TesLorean CAN Sender Testing Tool

I needed a simple CAN tool that would run through the Serial Monitor and allow an Arduino to send CAN frames (pre-programmed or defined on the fly).  This tool works with the MCP2515_CAN breakout board.  I use it to send test frames to the other TesLorean controller modules to test their responses.

Commands (entered into Serial Monitor)

p followed by 1-4
'p1', 'p2', 'p3', 'p4'
Send out one of the four predefined frames

f <frame_id> <eight byte values>
f 128 1 2 3 4 5 6 7 8

Planned developments
- r <1-4> <nnnn> (for repeat transmission of a pre-defined frame every nnnn 1/1000ths/sec
- f (with hex value inputs)
- c <frame_id>[,<frame_id>] or <*> (output received listed frame ids or all (*) frames details to the serial monitor)
  
