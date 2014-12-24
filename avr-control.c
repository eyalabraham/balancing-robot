/* avr-control.c
 *
 * this program is the control program for a 2-wheel balancing robot.
 * the robot is controlled by a RaspberryPi connected to an ATmega328P,
 * which runs this code.
 * the AVR performs 2 tasks:
 * AVR: (1) RaspberryPi send PWN setting commands via I2C to AVR, and
 *          AVR controls motors
 *      (2) AVR collects switch and ADC inputs and forwards to RPi upon request
 *
 * RPi: read IMU tilt/gyro sensor (I2C interface), and
 *      runs PID algorithm to control 2 servo motors via PWM
 *
 *                               +-----------+
 *                  +------------+ MPU-6050A |
 * +-----+          |  +-----+   +-----------+
 * |     |          |  |     |
 * | RPi +--< I2C >-+--+ AVR |
 * |     |             |     |
 * +-----+             +--+--+
 *                        |
 *              2x PWM channels to Servo
 *
 * ATmega AVR IO
 * ---------------
 * right motor PWM			OC0A		pin 12		out
 * right motor direction	PB0..PB1	pin 14..15	out
 * left motor PWM			OC0B		pin 11		out
 * left motor direction		PB2..PB3	pin 16..16	out
 * go/no-go select			PB4			pin 18		in
 * Batt low/ok				PB5			pin 19		out
 * run/not-run				PB6			pin 9		out
 * TWI						SCL			pin 28		out
 * TWI						SDA			pin 27		in/out
 * Batt sense				ADC0		pin 23		analog in
 *
 * Port B bit assignment
 *
 * b7 b6 b5 b4 b3 b2 b1 b0
 * |  |  |  |  |  |  |  |
 * |  |  |  |  |  |  |  +--- 'o' right motor dir \ 0, 3=stop, 1=fwd, 2=rev
 * |  |  |  |  |  |  +------ 'o' right motor dir /
 * |  |  |  |  |  +--------- 'o' left motor dir  \ 0, 3=stop, 1=fwd, 2=rev
 * |  |  |  |  +------------ 'o' left motor dir  /
 * |  |  |  +--------------- 'i' go/no-go          PUP enabled, 0=no-go, 1=go
 * |  |  +------------------ 'o' batt. ok/low      0=batt.ok,1=batt.low (red LED)
 * |  +--------------------- 'o' run/not-run       0=not running,1=running (green LED)
 * +------------------------ 'o' cycle test point
 *
 * note: all references to data sheet are for ATmega 328P Rev. 8161D–AVR–10/09
 */

#include	<stdint.h>
#include	<stdlib.h>
#include	<stdio.h>

#include	<avr/pgmspace.h>
#include	<avr/io.h>
#include	<avr/interrupt.h>

#include	"i2c_drv.h"

// debug print to UART port definition
// UART is assumed to be defined and initialized
//#define		__DEBUG_PRINT__

// TWI (I2C) definitions
#define		TWI_ADD			0x10 // I2C bus address 0x10 with General Call bit enabled
#define		TWI_GEN_CALL	0

#define		TWI_BUFFER      TWI_BUFF_LEN
#define		TWI_CMD_LEN		4    // commands are 4 bytes long

#define		TWI_CMD_PWM_R 	1    // PWM data for right motor follows, data:0 to 255
#define		TWI_CMD_PWM_L  	2    // PWM data for left motor follows, data: 0 to 255
#define		TWI_CMD_DIR		3    // motor turn direction follows, data: see Port B definition for b0 to b3
#define		TWI_CMD_RD_BATT 4    // return battery voltage from ADC, data: 0 to 255
#define		TWI_CMD_RD_BITS 5    // return switch bit state, data=see Port B definition for b4 and b7 (all other bits also returned)
#define		TWI_CMD_WR_BITS	6    // set indicator bits, data=see Port B definitions for b5 and b6
#define		TWI_CMD_MOTORS	7    // receive 3 bytes: PWM right, PWM left, direction bits all at once

// port B initialization
#define		PB_DDR_INIT		0xef // port data direction
#define		PB_PUP_INIT		0x10 // port input pin pull-up
#define		PB_INIT			0x00 // port initial values

// motor direction/state bit masks
#define		MOTOR_CLR_RIGHT	0xfc // clear right motor direction bits (AND mask)
#define		MOTOR_CLR_LEFT	0xf3 // clear left motor direction bits (AMD mask)
#define		MOTOR_REV_RIGHT	0x01 // right motor reverse (OR masks)
#define		MOTOR_FWD_RIGHT	0x02 // right motor forward
#define		MOTOR_REV_LEFT	0x04 // left motor reverse
#define		MOTOR_FWD_LEFT	0x08 // left motor forward
#define		MOTOR_PWM_INIT	0xd0 // initial PWM value

// misc masks
#define		STAT_TOGG_BATT	0x20 // toggle batt. status (XOR mask)
#define		STAT_RUN		0x40 // toggle running status (XOR mask)

/****************************************************************************
  Globals
****************************************************************************/
volatile	uint8_t	uBattery = 0; // battery voltage read from ADC

/* ----------------------------------------------------------------------------
 * ioinit()
 *  initialize IO interfaces
 *  timer and data rates calculated based on 4MHz internal clock
 *
 */
void ioinit(void)
{
	// reconfigure system clock prescaler to 4MHz and/or source
	CLKPR = 0x80;   // change clock prescaler to div by 2 (sec 8.12.2 p.37)
	CLKPR = 0x01;

	// initialize the TWI interface as a 'slave' device with address TWI_ADD

	i2c_s_initialize(TWI_ADD, TWI_GEN_CALL); // initialize

	// initialize UART interface to 19200 BAUD, 8 bit, 1 stop, no parity

	UCSR0A = _BV(U2X0);                 // double baud rate (sec 19.10 p.195)
	UCSR0B = _BV(RXEN0) | _BV(TXEN0);   // enable Tx and Rx
	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); // 8 data bits, 1 stop, no parity
	UBRR0 = 25;                         // 19200 baud with the lowest error

	// initialize Timer0 to provide 2 independent PWM signals for motor control
	// (section 14.7.3 p.101)
	// - base PWM frequency at 15625Hz
	// - OC0A -> right motor
	// - OC0B -> left motor

	OCR0A  = MOTOR_PWM_INIT; // initialize output compare registers
	OCR0B  = MOTOR_PWM_INIT;
	TCNT0  = 0;    // initialize counter register
	TIMSK0 = 0;    // no interrupts
	TCCR0A = 0xF3; // 'fast PWM' set OC0x on compare (signal going through 7414)
	TCCR0B = 0x01; // compare on OCRx, use clock with prescale=1 and start timer

	DDRD   = 0x60; // enable PD5 and PD6 as outputs so that OCRx PWM signals are "visible"

	/*
	// initialize Timer1 to provide a periodic interrupt every 50mSec
	// with Clear Timer on Compare Match (CTC) Mode (sec 15.9.2 p.125)
	// the interrupt routine will drive the PID control loop:
	// - read tilt/gyro
	// - calculate PID values
	// - drive PWM

	TCNT1  = 0;     // zero initial counter value
	OCR1A  = 195;   // OCR1A value for 20Hz
	TIMSK1 = 0x02;  // interrupt on OCR1A match
	TCCR1A = 0x00;  // CTC mode with OC1x pins kept in normal IO port mode (not used by timer)
	TCCR1C = 0;
	TCCR1B = 0x0D;  // use OCR1A for compare and internal clock with prescalar=1024 (256uSec resolution) and start timer
	*/

	// initialize ADC converter input ADC0
	ADMUX = 0x60;   // external AVcc reference, left adjusted result, ADC0 source
	ADCSRA = 0xEF;  // enable auto-triggered conversion and interrupts, ADC clock 31.25KHz
	ADCSRB = 0x00;  // auto trigger source is free-running

	// initialize general IO pins for output
	// - PB0, PB1: output, no pull-up, right and left motor fwd/rev control
	DDRB  = PB_DDR_INIT; // PB pin directions
	PORTB = PB_INIT | PB_PUP_INIT; // initial value of pins is '0', and input with pull-up
}

#ifdef __DEBUG_PRINT__
/* ----------------------------------------------------------------------------
 * putchr()
 * Send character c down the UART Tx, wait until tx holding register is empty
 *
 */
static void putchr(char c)
{
  loop_until_bit_is_set(UCSR0A, UDRE0);
  UDR0 = c;
}

/* ----------------------------------------------------------------------------
 * printstr()
 * Send a NULL-terminated string down the UART Tx
 *
 */
static void printstr(const char *s)
{

  while (*s)
    {
      if (*s == '\n')
	putchr('\r');
      putchr(*s++);
    }
}

/* ----------------------------------------------------------------------------
 * printstr_p()
 * Same as printstr(), but the string is located in program memory,
 * so "lpm" instructions are needed to fetch it.
 *
 */
static void printstr_p(const char *s)
{
  char c;

  for (c = pgm_read_byte(s); c; ++s, c = pgm_read_byte(s))
    {
      if (c == '\n')
    	  putchr('\r');
      putchr(c);
    }
}
#endif /* __DEBUG_PRINT__ */

/* ----------------------------------------------------------------------------
 * This ISR will trigger when Timer 1 compare indicates the time interval
 * for running the PID tasks:
 * - read tilt/gyro
 * - compute PID
 * - set motor PWMs
 *

ISR(TIMER1_COMPA_vect)
{
}
*/

/* ----------------------------------------------------------------------------
 * This ISR will trigger when the ADC completes a conversion
 * conversions are auto-triggered and this ISR will trigger at 31.25KHz
 * ADC result is left adjusted, so only ADCH needs to be read
 *
 */
ISR(ADC_vect)
{
	uBattery = ADCH;  // read battery voltage from ADC register
}

/* ----------------------------------------------------------------------------
 * main() control functions
 *
 * initialize IO: TWI, UART, timer and IO pins for H-bridge control
 * - TWI interface in slave mode to get PWM/motor commands from RPi host
 * - timer0 provides 2 PWM signals for motor control
 * - IO pins to control H-bridge direction
 * - (optional) UART to send/receive commands from host RaspberryPi
 *
 */
int main(void)
{
	static uint8_t messageBuf[TWI_BUFFER] = {0};  // I2C receive buffer
	static uint8_t temp;
	static int nOperStatus = 0;
#ifdef __DEBUG_PRINT__
	static char text[24] = {0};
#endif

	ioinit();                 // initialize IO devices
	sei();                    // enable interrupts

	for (;;)
	{
		// main loop
	    // poll the TWI Transceiver for a completed operation.
		nOperStatus = i2c_s_getData(messageBuf, TWI_CMD_LEN);

	    if ( nOperStatus > 0 )
    	{
    		// interpret a command and respond
    		switch ( messageBuf[0] )
    		{
				// read ADC value and send back to host
				case TWI_CMD_RD_BATT:
					messageBuf[0] = uBattery;
					i2c_s_setData(messageBuf, 1);
					break;

				// read various switch status, pack and send to host
				case TWI_CMD_RD_BITS:
					messageBuf[0] = PINB;
					i2c_s_setData(messageBuf, 1);
					break;

				// set PWM value for right motor and update
    			case TWI_CMD_PWM_R:
    				// ** must check if TCNT0 < OCR0x before changing **
    				OCR0A = messageBuf[1];   // update PWM
    				break;

    			// set PWM value for left motor and update
    			case TWI_CMD_PWM_L:
    				// ** must check if TCNT0 < OCR0x before changing **
    				OCR0B = messageBuf[1];   // update PWM
    				break;

    			// set motor forward-reverse port bits
    			case TWI_CMD_DIR:
    				temp = PORTB;
    				temp &= 0xf0;
    				temp |= (messageBuf[1] & 0x0f);
    				PORTB = temp;
    				break;

    			// set indicator bits
    			case TWI_CMD_WR_BITS:
    				temp = PORTB;
    				temp &= 0x9f;
    				temp |= (messageBuf[1] & 0x60);
    				PORTB = temp;
    				break;

    			// set both PWM values and direction bits in one packet
        		case TWI_CMD_MOTORS:
        			PORTB ^= 0x80;           // toggle b7 to output a cycle-test signal

        			OCR0A = messageBuf[1];   // update right PWM
        			OCR0B = messageBuf[2];   // update left PWM
        			temp = PORTB;            // update direction bits
        			temp &= 0xf0;
        			temp |= (messageBuf[3] & 0x0f);
        			PORTB = temp;

        			PORTB ^= 0x80;
        			break;

   				// ignore bad commands
   				default:;
   			} /* end of message processing */
#ifdef __DEBUG_PRINT__
    		printstr_p(PSTR("command "));
    		snprintf(text, 24, "0x%x 0x%x 0x%x 0x%x\n", messageBuf[0], messageBuf[1], messageBuf[2], messageBuf[3]);
    		printstr(text);
    		text[0]=0;
#endif /* __DEBUG_PRINT__ */
	    }

	    //
	    // something else to do
	    //

	} /* end for-loop */

	return 0;
}
