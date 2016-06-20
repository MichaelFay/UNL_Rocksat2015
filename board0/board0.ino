/*****************************************************************************
*********************Syncronization Library for Atmega 2560*******************
******************************************************************************

Developed By: Michael Fay for Rocksat-X 2016

File Name: board0.ino
Last revised:
Version Number:

Tested By:
Last Tested:
******************************************************************************/


#define __ARDUINO__
				//Builds the arduino header 
#define __ATMEGA_2560__
				//Builds the support package for the atmega 2560
#define RELEASE
				//Defines this build as a release canidate build


#define UTL_ASSERT_CONTINUE(x) logWrite(x)
/******************************* Includes ************************************/

#include <Arduino.h>
#include <Servo.h>
#include <EEPROM.h>
#include <Spi.h>
#include <Time.h>

#include "bsp_prj.h"

#include "stdio.h"
#include "TimerOne.h"
#include "packet_def.hpp"
#include "SoftwareSerial.h"



 
#define HARD_RESET_LOG
/*********************** Memory Constants ************************************/


/***************** Macro's and Definitions************************************/
typedef unsigned char byte;  


/***************** Global Variables*******************************************/
bool resync;

byte RX_byte_buffer[RX_BUFFER_SIZE] = {0};
byte TX_byte_buffer[TX_BUFFER_SIZE] = {0};

byte proc_data;

uint16_t 	EEPROM_address;

NERI RX_buffer[MAX_RX_DATA_STREAM]  = {0};
NERI TX_buffer[MAX_TX_DATA_STREAM]  = {0};

RX_buffer_location = 0;
RX_byte_buffer_location = 0;
/*********************** Function Proto's ************************************/

SoftwareSerial Serial(RX_PIN, TX_PIN);



void setup()
{
	initSerial();
	initTimerSync();
	initEEProm();
}


void loop()
{
	if(Serial.avaliable())
	{
		proc_data = false;
		RX_byte_buffer[RX_buffer_location] = Serial.read();
		RX_buffer_location ++;
		if(RX_buffer_location > RX_BUFFER_SIZE)
		{
			UTL_ASSERT_CONTINUE(getCurrentTime());
			UTL_ASSERT_CONTINUE(':');
			UTL_ASSERT_CONTINUE(0x01);
		}
	}
	else
	{
		//Once we have recieved all the datawe need process it 
		proc_serial(&RX_byte_buffer[RX_byte_buffer_location], &RX_buffer[RX_buffer_location]);
	}

	if(proc_data == true)
	{
		NERI tmp = RX_buffer[]
		switch()
	}


}

void initTimerSync()
{
	Timer1.initialize(MICRO_TO_MILLI(SYNC_TIME_MS) );        // initialize timer1, and set a 1/2 second period
  	Timer1.attachInterrupt(sync_cb);  						// attaches callback() as a timer overflow interrupt
}

void initSerial()
{
	Serial.begin(9600);
}

void initEEProm()
{
	if(HARD_RESET_LOG == true) 
	{
		EEPROM_address = logWrite(0x00);
	}
	else
	{
		EEPROM_address = logWrite(BASE_ADDRESS_EEPROM);
	}
}

void sync_cb()
{
	//this resets the timer reg. to one before it rolls over. 
	SREG = 65535;
}


void readCurrentSensor()
{

}

void proc_serial(byte * bytes, NERI * packet)
{
	packet->packet_len = bytes[0];
	packet->reciever_id = bytes[1];
	packet->sender_id = bytes[2];
	packet->command = bytes[3];
	packet->data_len = bytes[4];
	uint8_t i;
	for(i = 0; i < packet->data_len && i < MAX_DATA_SIZE; i++)
	{
		packet->data[i] = bytes[5 + i];
	}
	packet->crc = bytes[5 + packet->data_len];

	proc_data = true;

}

void sendPacket(NERI * pkt)
{
	Serial.write(*pkt)
}


void logWrite(byte val)
{

	EEPROM.write(EEPROM_address, val );
	addr = addr + 1;
  	if (addr == EEPROM.length()) {
    	addr = 0;
  	}
}
/*********************** Note's ************************************/
/*







*/
