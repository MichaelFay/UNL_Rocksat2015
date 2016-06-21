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
uint32_t 	CurrentTime;

NERI RX_buffer[MAX_RX_DATA_STREAM]  = {0};
NERI TX_buffer[MAX_TX_DATA_STREAM]  = {0};

RX_buffer_location = 0;
RX_byte_buffer_location = 0;
/*********************** Function Proto's ************************************/

//TODO Define RX_PIN AND TX_PIN

SoftwareSerial Serial(RX_PIN, TX_PIN);



void setup()
{
	// TODO INIT pins 
	// TODO INIT Servo Code

	initSerial();
	initTimerSync();
	initEEProm();

}


void loop()
{
	if(Serial.avaliable())
	{
		proc_data = false;
		RX_byte_buffer[RX_byte_buffer_location] = Serial.read();
		RX_byte_buffer_location ++;
		if(RX_byte_buffer_location > RX_BUFFER_SIZE)
		{
			UTL_ASSERT_CONTINUE(Timer1.read()/2);
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
		proc_data = false; 

		NERI tmp = RX_buffer[RX_buffer_location];
		switch(tmp.command)
		{
			case SYNC_CLOCK:
			syncClock();
			break; 

			case START_SERVO:
				if(tmp.data[0] == SERVO_ARM)
				{
					#ifdef PKT_WRITE
						Arm.write(tmp.data[1]);
					#else
						Arm.write(SERVO_ARM_SPEED_FWD);
					#endif 
				}
				else if(tmp.data[0] == SERVO_SP)
				{
					#ifdef PKT_WRITE
						SP.write(tmp.data[1]);
					#else
						SP.write(SERVO_SP_SPEED_FWD);
					#endif 
				}
				break;
		
			case STOP_SERVO:
				if(tmp.data[0] == SERVO_ARM)
				{
					#ifdef PKT_WRITE
						Arm.write(tmp.data[1]);
					#else
						Arm.write(SERVO_ARM_SPEED_STOP);
					#endif 
				}
				else if(tmp.data[0] == SERVO_SP)
				{
					#ifdef PKT_WRITE
						SP.write(tmp.data[1]);
					#else
						SP.write(SERVO_SP_SPEED_STOP);
					#endif 
				}
				break;

			case REVERSE_SERVO:
				if(tmp.data[0] == SERVO_ARM)
				{
					#ifdef PKT_WRITE
						Arm.write(tmp.data[1]);
					#else
						Arm.write(SERVO_ARM_SPEED_STOP);
					#endif 
				}
				else if(tmp.data[0] == SERVO_SP)
				{
					#ifdef PKT_WRITE
						SP.write(tmp.data[1]);
					#else
						SP.write(SERVO_SP_SPEED_STOP);
					#endif 
				}
				break;

			case SHUTDOWN:
				logUpdate(BASE_ADDRESS_EEPROM, EEPROM_address);

		
			default: 
				UTL_ASSERT_CONTINUE(Timer1.read());
				UTL_ASSERT_CONTINUE(':');
				UTL_ASSERT_CONTINUE(0x02);
				break;

			RX_buffer_location --;
		}
	}


}

void initServos()
{

}

void initTimerSync()
{
	Timer1.initialize(MICRO_TO_MILLI(SYNC_TIME_MS) );        // initialize timer1, and set a 1/2 second period
  	//Timer1.attachInterrupt(sync_cb);  						// attaches callback() as a timer overflow interrupt
  	//DO NOT TURN THIS ON ^ it will consistantly intrrupt
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

void syncClock()
{
	//this resets the timer reg. to one before it rolls over. 
	SREG = 65535;
}

//TODO: IDK where the pins go. We may not need to implement this. 
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

	RX_byte_buffer_location -= (5 + packet->data_len);

	proc_data = true;

}

void sendPacket(NERI * pkt)
{
	Serial.write(*pkt)
}

void logUpdate(uint16_t address, uint8_t val)
{
	  EEPROM.update(address, val);
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

For the love of god please done compile this after all the TODO items are
done.





*/
