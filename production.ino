// Default pins:
//    ESP-01          GPIO0 = swdclk, GPIO2 = swdio
//    NodeMCU devkit  D3 = swdclk, D4 = swdio
//
// And for reference:
//    SWD header      pin1 = 3.3v, pin2 = swdio, pin3 = gnd, pin4 = swdclk
//    SWD over JTAG   TCLK = swdclk, TMS = swdio



#include "arm_debug.h"
#include "arm_kinetis_debug.h"
#include "arm_kinetis_reg.h"
#include "fc_remote.h"


const int swd_clock_pin = 0;
const int swd_data_pin = 2;
const int buttonPin = 4;


ARMKinetisDebug target(swd_clock_pin, swd_data_pin, ARMDebug::LOG_NORMAL);//LOG_TRACE_DP

FcRemote remote(target);


void setup()
{
	pinMode(BUILTIN_LED, OUTPUT);
	digitalWrite(BUILTIN_LED, HIGH);
	//pinMode(buttonPin, INPUT_PULLUP);
	//analogReference(INTERNAL);
	Serial.begin(115200);
}

int iSelector = 0;
String strInputString = "";         // A string to hold incoming data.
boolean bStringComplete = false;	// Whether the string is complete.

void ReadCommand()
{
	while (Serial.available())
	{
		char inChar = (char)Serial.read();  // Get the new byte.    
											// say what you got:
		Serial.print("I received: ");
		Serial.println(inChar, HEX);
		
		strInputString += inChar;  // Add it to the strInputString.

		if (strInputString.length() > 2)
		{
			strInputString = "";  // Clear the string.
			return;
		}
		// .  
		if (strInputString.length() == 2)
		{
			if (strInputString == "1#")
				iSelector = 1;
			if (strInputString == "2#")
				iSelector = 2;
			if (strInputString == "3#")
				iSelector = 3;
			if (strInputString == "E#")	//Erase
				iSelector = 4;
			if (strInputString == "P#")	//Program
				iSelector = 5;

			bStringComplete = true;
			strInputString = "";  // Clear the string.
		}
	}
}

void waitForCommand()
{
	Serial.println("");
	Serial.println("--------------------------------------------");
	Serial.println(" KL02 SWD Programmer : Press a Cmd to start");
	Serial.println("--------------------------------------------");
	Serial.println("");
	Serial.flush();

	iSelector = 0;
	while (Serial.available() < 2 ){
		// While we're waiting, blink the LED to indicate we're alive
		yield();	// feed the watchdog
		digitalWrite(BUILTIN_LED, (millis() % 1000) < 150);
	}
	ReadCommand();

	digitalWrite(BUILTIN_LED, HIGH);
}

void success()
{
	Serial.println("");
	Serial.println("#### Tests Passed! ####");
	Serial.println("");
}

void loop()
{
	char buffer[32];
	
	waitForCommand();

	// Start debugging the target
	if (iSelector > 0)
	{
		if (!target.begin())
			return;
		if (!target.startup())
			return;
		if (!target.initKL02())
			return;
	}
	
	if (iSelector == 1)
		target.hexDump(0x1040, 4);  // GPS bug line
	if (iSelector == 2)
		target.hexDump(0, 20);
	if (iSelector == 3) {
		uint32_t value;
		if (target.memLoad(REG_SIM_SDID, value)) {
			snprintf(buffer, sizeof buffer, " %08x", value);
			Serial.println(buffer);
		}
		else {
			Serial.println(" REG_SIM_SDID(error )");
		}
	
		if (target.memLoad(REG_SIM_COPC, value)) {
			snprintf(buffer, sizeof buffer, " %08x", value);
			Serial.println(buffer);
		}
		else {
			Serial.println(" REG_SIM_COPC(error )");
		}
			//target.hexDump(REG_SIM_SDID, 1);
	}
	if (iSelector == 4)
	{
		target.flashMassErase();
	}
	if (iSelector == 5)
	{
		remote.installFirmware();
	}
		
	
	// Run an electrical test, to verify that the target board is okay
/*	if (!etest.runAll())
		return;
*/
	// Program firmware, blinking both LEDs in unison for status.
//	if (!remote.installFirmware())
//		return;
/*
	// Boot the target
	if (!remote.boot())
		return;

	// Disable interpolation, since we only update fbNext
	if (!remote.setFlags(CFLAG_NO_INTERPOLATION))
		return;

	// Set a default color lookup table
	if (!remote.initLUT())
		return;

	// Pixel pattern to display while running the frame rate test (white / green)
	if (!remote.setPixel(0, 16, 16, 16)) return;
	if (!remote.setPixel(1, 0, 24, 0)) return;

	// Check the frame rate; make sure the firmware is going fast enough
	if (!remote.testFrameRate())
		return;
*/
	success();
}
