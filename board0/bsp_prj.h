//#include <avr/io.h>  //to use portx, pinx, and all the other bullshit

#define MICRO_TO_MILLI(x) x * 1000

#define PIN1 ( 1 << 0)
#define PIN2 ( 1 << 1)
#define PIN3 ( 1 << 2)
#define PIN4 ( 1 << 3)
#define PIN5 ( 1 << 4)
#define PIN6 ( 1 << 5)
#define PIN7 ( 1 << 6)
#define PIN8 ( 1 << 7)

#define STROBE_LINE_PORT	PORTJ
#define STROBE_LINE_PIN		PIN6


#define COMM_BANK 		PORTL
#define COMM_BANK_READ 	PORTL

#define COMM_DDR DDRL

#define COM_1 PIN1
#define COM_2 PIN2
#define COM_3 PIN3
#define COM_4 PIN4
#define COM_5 PIN5
#define COM_6 PIN6
#define COM_7 PIN7
#define COM_8 PIN8

#define SOFTSERIAL_RX 43
#define SOFTSERIAL_TX 42

#define NERI_LINE_0	COM1
#define NERI_LINE_1 COM2

#define INTERRUPTS_ON()  sei()

#define INTERRUPTS_OFF() cli()

//TODO: Define TX and RX buffer max

#define MAX_RX_DATA_STREAM 24
#define MAX_TX_DATA_STREAM 24

#define RX_BUFFER_SIZE 1024
#define TX_BUFFER_SIZE 1024

#define LED_SHIFT_PORT 	PORTA

#define LED_SHIFT_DATA 	PIN3
#define LED_SHIFT_CLOCK	PIN4
#define LED_SHIFT_LATCH	PIN5
#define LED_SHIFT_OE 	PIN6

#define LED0 0
#define LED1 1
#define LED2 2
#define LED3 3
#define	LED4 4 
#define LED5 5
#define LED6 6
#define LED7 7
#define LED8 8

#define MAX_LED_ON 5

#define SYNC_TIME_MS 500

#define UNHOUSED

#define BASE_ADDRESS_EEPROM 0x00

#define I2C_B_PORT PORTD
#define SCL_B_PIN PIN1
#define SDA_B_PIN PIN2

#define CURS_PORT	PORTF
#define CURS_MAIN_PIN	PIN1
#define CURS_SERV_1_PIN	PIN2
#define CURS_SERV_2_PIN	PIN3

#define MAX_DATA_SIZE[256]

#define RX_PIN SOMETHING
#define TX_PIN SOMETHING
