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


#define BOARD_ADDRESS 0x01
 
#define HARD_RESET_LOG
/*********************** Memory Constants ************************************/


/***************** Macro's and Definitions************************************/
typedef unsigned char byte;  


/***************** Global Variables*******************************************/
bool send_packet;
bool not_shutdown;
long int time; 

byte boot_sequence; 

byte RX_byte_buffer[RX_BUFFER_SIZE] = {0};
byte TX_byte_buffer[TX_BUFFER_SIZE] = {0};

byte proc_data;

uint16_t 	EEPROM_address;
uint32_t 	CurrentTime;

NERI RX_buffer[MAX_RX_DATA_STREAM]  = {0};
NERI TX_buffer[MAX_TX_DATA_STREAM]  = {0};

RX_buffer_location = 0;
RX_byte_buffer_location = 0

TX_buffer_size = 0;


/*********************** Function Proto's ************************************/

//TODO Define RX_PIN AND TX_PIN

SoftwareSerial Serial(RX_PIN, TX_PIN);



void setup()
{
	// TODO INIT pins 
	// TODO INIT Servo Code
	initPins();
	initSerial();
	initTimerSync();
	initEEProm();


}

void initPins()
{
	pinMode(CAMERA_PIN, OUTPUT);
	send_packet 	= false; 
	not_shutdown	= false; 
}

void loop()
{
	readAllPins();
	time = Timer1.read();

	if(time < START_TIME)
	{
		switch(boot_sequence)
		{
			case BOOT_CAMERA:
				startCamera();
				boot_sequence++;
				break; 
			case SERVO_LOCK:				
				TX_buffer[TX_buffer_size].reciever_id 	= BOARD_ADDRESS;
				TX_buffer[TX_buffer_size].sender_id 	= 0x01;
				TX_buffer[TX_buffer_size].command		= STOP_SERVO;
				TX_buffer[TX_buffer_size].data_len		= 0x01;
				TX_buffer[TX_buffer_size].data[0]		= SERVO_ARM;
				TX_buffer[TX_buffer_size].crc			= NO_CRC;
				boot_sequence++;
				TX_buffer_size ++;

			case SERVO_LOCK:				
				TX_buffer[TX_buffer_size].reciever_id 	= BOARD_ADDRESS;
				TX_buffer[TX_buffer_size].sender_id 	= 0x01;
				TX_buffer[TX_buffer_size].command		= STOP_SERVO;
				TX_buffer[TX_buffer_size].data_len		= 0x01;
				TX_buffer[TX_buffer_size].data[0]		= SERVO_SP;
				TX_buffer[TX_buffer_size].crc			= NO_CRC;
				boot_sequence++;
				TX_buffer_size ++;

		}	
	}

	else if(time >=START_TIME && time < DEPLOY_ARM_TIME)
	{
		TX_buffer[TX_buffer_size].reciever_id 	= BOARD_ADDRESS;
		TX_buffer[TX_buffer_size].sender_id 	= 0x01;
		TX_buffer[TX_buffer_size].command		= START_SERVO;
		TX_buffer[TX_buffer_size].data_len		= 0x01;
		TX_buffer[TX_buffer_size].data[0]		= SERVO_ARM;
		TX_buffer[TX_buffer_size].crc			= NO_CRC;
		TX_buffer_size ++;

		Arm.write(SERVO_ARM_SPEED_FWD);
	}
	else if(time >=DEPLOY_ARM_TIME && time < DEPLOY_SP_TIME)
	{


		SP.write(SERVO_SP_SPEED_FWD);

		TX_buffer[TX_buffer_size].reciever_id 	= BOARD_ADDRESS;
		TX_buffer[TX_buffer_size].sender_id 	= 0x01;
		TX_buffer[TX_buffer_size].command		= START_SERVO;
		TX_buffer[TX_buffer_size].data_len		= 0x01;
		TX_buffer[TX_buffer_size].data[0]		= SERVO_SP;
		TX_buffer[TX_buffer_size].crc			= NO_CRC;
		TX_buffer_size ++;
	}
	else if(time >= DEPLOY_SP_TIME && time >= EXTEND_DELAY)
	{
		SP.write(SERVO_SP_SPEED_STOP);
		Arm.write(SERVO_ARM_SPEED_STOP);
		readAllPins();
	}

	else if(time >= EXTEND_DELAY && time >= RETRACT_SP_TIME)
	{
		SP.write(SERVO_SP_SPEED_STOP_REVERSE);

		TX_buffer[TX_buffer_size].reciever_id 	= BOARD_ADDRESS;
		TX_buffer[TX_buffer_size].sender_id 	= 0x01;
		TX_buffer[TX_buffer_size].command		= REVERSE_SERVO;
		TX_buffer[TX_buffer_size].data_len		= 0x01;
		TX_buffer[TX_buffer_size].data[0]		= SERVO_SP;
		TX_buffer[TX_buffer_size].crc			= NO_CRC;
		TX_buffer_size ++;
	}

	else if(time >= RETRACT_SP_TIME && time >= RETRACT_ARM_TIME)
	{
		Arm.write(SERVO_ARM_SPEED_REVERSE);

		TX_buffer[TX_buffer_size].reciever_id 	= BOARD_ADDRESS;
		TX_buffer[TX_buffer_size].sender_id 	= 0x01;
		TX_buffer[TX_buffer_size].command		= REVERSE_SERVO;
		TX_buffer[TX_buffer_size].data_len		= 0x01;
		TX_buffer[TX_buffer_size].data[0]		= SERVO_ARM;
		TX_buffer[TX_buffer_size].crc			= NO_CRC;
		TX_buffer_size ++;
	}

	else if(time >= RETRACT_ARM_TIME && time <= SHUTDOWN_TIME && not_shutdown)
	{
		boot_sequence = 0;
		not_shutdown = true; 
		if(time < START_TIME)
		{
			switch(boot_sequence)
			{
				case BOOT_CAMERA:
					stopCamera();
					boot_sequence++;
					break; 
				case SERVO_LOCK:
					Arm.write(SERVO_ARM_SPEED_STOP);

					TX_buffer[TX_buffer_size].reciever_id 	= BOARD_ADDRESS;
					TX_buffer[TX_buffer_size].sender_id 	= 0x01;
					TX_buffer[TX_buffer_size].command		= STOP_SERVO;
					TX_buffer[TX_buffer_size].data_len		= 0x01;
					TX_buffer[TX_buffer_size].data[0]		= SERVO_ARM;
					TX_buffer[TX_buffer_size].crc			= NO_CRC;
					boot_sequence++;
					TX_buffer_size ++;

				case SERVO_LOCK:
					SP.write(SERVO_SP_SPEED_STOP);			
						
					TX_buffer[TX_buffer_size].reciever_id 	= BOARD_ADDRESS;
					TX_buffer[TX_buffer_size].sender_id 	= 0x01;
					TX_buffer[TX_buffer_size].command		= STOP_SERVO;
					TX_buffer[TX_buffer_size].data_len		= 0x01;
					TX_buffer[TX_buffer_size].data[0]		= SERVO_SP;
					TX_buffer[TX_buffer_size].crc			= NO_CRC;
					boot_sequence++;
					TX_buffer_size ++;
			
			}	
		}
	}

	else
	{
		//do nothing
		//defensive programming
	}

	if(TX_buffer_size >= 0 && send_packet == true)
	{
		TX_buffer_size --;

		sendPacket(&TX_buffer[TX_buffer_size]);
		if(TX_buffer_size == 0 )
		{
			send_packet = false; 
		}

	}

}




void startCamera()
{
	//TODO start camera code
}

void stopCamera()
{
	//TODO stop camera code
}

void sendPacket(NERI * pkt)
{
	Serial.write(*pkt)
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
