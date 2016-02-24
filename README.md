# Rephone-KL02-swd
Program Freescale KL02 via SWD , ESP8266 does the bitbanging

Program Rephone's GPS/NFC modules vith ESP8266 and ARM SWD protocol!
Why ESP8266, very cheap and it is 3.3V
(maybe a port for the GSM/BLE/Breakout module would be smart)

----------
This directory is an Arduino sketch.

The Serial Wire Debug client here is a slightly adapted version of the one Micah Elizabeth Scott originally wrote for Fadecandy's factory test infrastructure. 
It has optional extensions for the Freescale Kinetis microcontrollers, but the lower-level SWD interface should be compatible with any ARM microcontroller.

Just look at the ingenious code at:

https://github.com/scanlime/esp8266-arm-swd 
https://github.com/scanlime/fadecandy/tree/master/testjig/production 

Installing
----------

Software you'll need:

* The latest [Arduino IDE](http://www.arduino.cc/en/Main/Software)
* Install the [ESP8266 Arduino core](https://github.com/esp8266/Arduino)
* Python 2.7

[i am using Visual Studio 2015 and Visual Micro plugin as Arduino IDE]

1. Put your Freescale KL02 hex-file in the 'HexFile' catalog
2. Open 'python/firmwareprep.py' in Python Idle and check the hex-file name, and RUN
   This will generate the 'firmware_data.h', which you must compile into the ESP8266
3. Now open `production.ino` in the IDE:
4. In the Tools menu, select your ESP8266 board and serial port info
5. Sketch -> Upload

Usage
-----
Commands in  Terminal: 115200 (no CR or CR/LF)

1#   = Memory Dump - check that you get:  Found ARM processor debug port (IDCODE: 0bc11477)

E#   = MassErase

P#   = Program/Verify

Okay, now your Rephone Thing is programmed!


Solder 4 pind to the testpads on the backside of the module

Hookups:
-----

| Rephone     | ESP8266 pin | 
| ----------- | ----------- | 
| SWIO        | GPIO2       | 
| SWCLK       | GPIO0       | 
| Ground      | GND         | 
| 3V3 Power   | VCC         | 

