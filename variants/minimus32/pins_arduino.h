/*
  pins_arduino.h - Pin definition functions for Arduino
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2007 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA

  $Id: wiring.h 249 2007-02-03 16:52:51Z mellis $
*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

#define NUM_ANALOG_INPUTS 0
#define NUM_DIGITAL_PINS 21

// We only have a small USB buffer
#define USB_EP_SIZE 16

// FIXME probably wrong
#if 0
#define TX_RX_LED_INIT	DDRD |= (1<<5), DDRB |= (1<<0)
#define TXLED0			PORTD |= (1<<5)
#define TXLED1			PORTD &= ~(1<<5)
#define RXLED0			PORTB |= (1<<0)
#define RXLED1			PORTB &= ~(1<<0)
#else
#define TX_RX_LED_INIT do {} while(0)
#define TXLED0 do {} while(0)
#define TXLED1 do {} while(0)
#define RXLED0 do {} while(0)
#define RXLED1 do {} while(0)
#endif

// Map SPI port to 'new' pins D14..D17
// FIXME Probably not true
static const uint8_t SS   = 19;
static const uint8_t MOSI = 0;
static const uint8_t MISO = 1;
static const uint8_t SCK  = 20;

// FIXME probably wrong
#define digitalPinToPCICR(p)    (((uint8_t *)0))
#define digitalPinToPCICRbit(p) 0
#define digitalPinToPCMSK(p)    (((uint8_t *)0))
#define digitalPinToPCMSKbit(p) ( ((p) >= 8 && (p) <= 11) ? (p) - 4 : ((p) == 14 ? 3 : ((p) == 15 ? 1 : ((p) == 16 ? 2 : ((p) == 17 ? 0 : (p - A8 + 4))))))

#define analogPinToChannel(P)  ( -1 )

#define digitalPinToTimer(P) (NOT_ON_TIMER)

#ifdef ARDUINO_MAIN

// On the Arduino board, digital pins are also used
// for the analog output (software PWM).  Analog input
// pins are a separate set.

// ATMEL ATMEGA32U2 / Minimus 32
//
// D0				PB2				MOSI
// D1				PB3				MISO
// D2				PB4
// D3				PB5
// D4				PB6
// D5#				PB7
// D6				PC7
// D7#				PC6
// !RESET
// D8#				PC5
// D9				PC4
// VCC
// --USB--
// D10				PC2
// D11				PD0
// D12				PD1
// D13				PD2				RX
// D14				PD3				TX
// D15				PD4
// D16				PD5				LEBB (Blue)
// D17				PD6				LEDA (Red)
// D18				PD7				HWB
// D19				PB0
// D20				PB1				SCK
// GND
//

// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
const uint16_t PROGMEM port_to_mode_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &DDRB,
	(uint16_t) &DDRC,
	(uint16_t) &DDRD,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PORT,
	NOT_A_PORT,
	(uint16_t) &PINB,
	(uint16_t) &PINC,
	(uint16_t) &PIND,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[30] = {
	PB, // D0 - PB2
	PB,	// D1 - PB3
	PB, // D2 - PB4
	PB,	// D3 - PB5
	PB,	// D4 - PB6
	PB, // D5 - PB7
	PC, // D6 - PC7
	PC, // D7 - PC6

	PC, // D8 - PC5
	PC,	// D9 - PC4

	PC, // D10 - PC2
	PD,	// D11 - PD0
	PD, // D12 - PD1
	PD, // D13 - PD2
	PD,	// D14 - PD3
	PD,	// D15 - PD4
	PD,	// D16 - PD5
	PD,	// D17 - PD6
	PD,	// D18 - PD7
	PB, // D19 - PB0
	PB, // D20 - PB1
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[30] = {
	_BV(2), // D0 - PB2
	_BV(3),	// D1 - PB3
	_BV(4), // D2 - PB4
	_BV(5),	// D3 - PB5
	_BV(6),	// D4 - PB6
	_BV(7), // D5 - PB7
	_BV(7), // D6 - PC7
	_BV(6), // D7 - PC6

	_BV(5), // D8 - PC5
	_BV(4),	// D9 - PC4

	_BV(2), // D10 - PC2
	_BV(0),	// D11 - PD0
	_BV(1), // D12 - PD1
	_BV(2), // D13 - PD2
	_BV(3),	// D14 - PD3
	_BV(4),	// D15 - PD4
	_BV(5),	// D16 - PD5
	_BV(6),	// D17 - PD6
	_BV(7),	// D18 - PD7
	_BV(0), // D19 - PB0
	_BV(1), // D20 - PB1
};

#if 0
// FIXME: needs updating
const uint8_t PROGMEM digital_pin_to_timer_PGM[16] = {
	NOT_ON_TIMER,	
	NOT_ON_TIMER,
	NOT_ON_TIMER,
	TIMER0B,		/* 3 */
	NOT_ON_TIMER,
	TIMER3A,		/* 5 */
	TIMER4D,		/* 6 */
	NOT_ON_TIMER,	
	
	NOT_ON_TIMER,	
	TIMER1A,		/* 9 */
	TIMER1B,		/* 10 */
	TIMER0A,		/* 11 */
	
	NOT_ON_TIMER,	
	TIMER4A,		/* 13 */
	
	NOT_ON_TIMER,	
	NOT_ON_TIMER,
};
#endif

#endif /* ARDUINO_MAIN */

#if 0
#if (ARDUINO <= 101)
#ifdef __cplusplus

// Ugly hacks to workaround broken arduino USB core
class _minimus_UHWCON {
	public:
	operator uint8_t() const {return 0;}
	void operator= (const int a)
	{
	  USBCON=0;
	}
};
#define UHWCON ({__extension__ _minimus_UHWCON minimus_HWCON; minimus_HWCON;})

class _minimus_PLLCSR {
	public:
	operator uint8_t() const
	{
		uint8_t val = PLLCSR;
		if (val & (1 << PLLP0))
			val |= 0x10;
		return val & 0x13;
	}
	void operator= (const uint8_t val)
	{
		uint8_t flags = 0;
		if (val & 0x10)
			flags |= (1 << PLLP0);
		PLLCSR = (1 << PLLP0) | (1 << PLLE);//flags | (val & 3);
	}
};
#undef PLLCSR
#define PLLCSR ({__extension__ _minimus_PLLCSR minimus_PLLCSR; minimus_PLLCSR;})

// Define this to something we know we will be enabling at the same time
#define OTGPADE USBE

#else

#undef PLLCSR

#endif
#endif
#endif
#endif /* Pins_Arduino_h */
