/*
 *    Filename: umrichter.c
 *     Version: 0.0.1
 * Description: Ansteuerung eines A4960-Drehstromwandlers
 *     License: GPLv3 or later
 *     Depends: global.h, io.h, interrupt.h
 *
 *      Author: Copyright (C) Philipp Hörauf
 *        Date: 2015-12-22
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */


#include "umrichter.h"

#define UBRRH_VALUE 0
#define UBRRL_VALUE 51
#define BAUD 19200
#include "../../AtmelLib/io/serial/uart.h"


// FUNKTIONEN

// TIMER
// Taktfrequenz = fCPU, Ausgabefrequenz 15,625kHz
// clear OC1A on compare match, top=1023
void timerInit (void) {
	TCCR1A = 1<<COM1A1 | 1<<WGM11 | 1<<WGM10;
	TCCR1B = 1<<WGM12 | 1<<CS10;
	OCR1A = 250; // Startwert 10%
}

// Hardware SPI Speed <= 20MHz. (hier = 1MHz)
// MSB first, 8 Data Bits per Transmission
// SCK idle high, sample on trailing edge
void spiInit (void) {
	SPCR = 1<<SPE | 1<<MSTR | 1<<SPR0 | 1<<CPOL | 1<<CPHA;
}

uint8_t spiRx (void) {
	return SPDR;
}

uint8_t spiTxReady (void) {
	if (SPSR & (1<<SPIF)) {
		return 1;
	}
	return 0;
}

void spiTx (uint8_t datenwort) {
	SPDR = datenwort;
}

uint16_t a4960Com (uint8_t adresse, uint16_t datenwort) {
	CS(0);
	delayus(1);
	spiTx((adresse<<4) | (datenwort>>8));
	while(!spiTxReady());
	uint16_t temp = (spiRx()<<8);
	spiTx(datenwort & 0xFF);
	while(!spiTxReady());
	temp |= spiRx();
	delayus(1);
	CS(1);
	return temp;
}

// lese diagnostics aus.
uint16_t a4960ReadStatus (void) {
	return a4960Com(0b1100, 0);
}

void a4960Init (void) {
	a4960Com (0b0001, BT3 | DT4 | DT2);
	a4960Com (0b0011, VR3 | VT5);
	a4960Com (0b0101, PT4);
	a4960Com (0b0111, HQ3 | HQ0 | HT2);
	a4960Com (0b1001, EC2 | EC1 | EC0 | SC1);
	a4960Com (0b1011, RQ3 | RR0);
	a4960Com (0b1101, 0x0FFF);
	a4960Com (0b1111, BW2 | BW1 | RSC);
}

void a4960MotorOn (void) {
	a4960Com (0b1111, BW2 | RSC | RUN);
}

void a4960MotorOff (void) {
	a4960Com (0b1111, BW2 | RSC);
}




// MAIN ROUTINE

int main(void) {
	
	// Umrichteranschluss:
	// PB1/OC1A = PWM
	// PD2/INT0 = Tacho
	// PD3/INT1 = Diag
	// PD4 = Reset (active low)
	// SPI Daten über SPI, CS ist PB2
	DDRB |= 1<<PB1 | 1<<PB2 | 1<<PB3 | 1<<PB5;
	sbi(PORTD, PB5); // SCK ist active low
	
	// Sonstiges:
	// ADC0 = 
	// ADC1 = 
	// PD0/RXI = 
	// PD1/TXO = 
	DDRD |= 1<<PD1 | 1<<PD4;
	PORTD |= 1<<PD2 | 1<<PD3; // Pullups an
	
	// LEDs
	DDRD |= 1<<PD6 | 1<<PD7;
	
	delayms(10);
	
	// Aktiviere PWM für Motordrehzahl
	timerInit();
	
	spiInit();
	sbi(PORTD, PD4); // aktiviere den a4960
	
	uartInit();
	
	delayms(100);
	uartTxNewline();
	uartTxPstrln(PSTR("Motorcontroller für kleine Drehstrommaschinen"));
	uartTxNewline();
	char buf[20];
	uint16_t tempvar = 0;
	
	a4960Init();
	delayms(10);
	a4960MotorOn();
	uartTxStrln("aktiviere Motor");
	ON(1);
// 	delayms(5000);
// 	a4960MotorOff();
// 	ON(0);
// 	uartTxStrln("deaktiviere Motor");
// 	delayms(1000);
	
	
// 	sei(); // und es seien Interrupts :D
	
	
	while(1) {
		PORTD ^= 1<<PD6;
		delayms(100);
		PORTD ^= 1<<PD7;
		delayms(100);
		
		PORTD ^= 1<<PD6;
		delayms(100);
		PORTD ^= 1<<PD7;
		delayms(100);
		
		tempvar = a4960ReadStatus();
		sprintf(buf, "Status: %x", tempvar);
		uartTx(tempvar<<8);
		uartTx((uint8_t)tempvar);
	}
	return 0;
}
