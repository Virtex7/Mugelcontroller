/*
 *    Filename: umrichter.c
 *     Version: 0.0.3
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

// #define DEBUG



// FUNKTIONEN

// TIMER
// Taktfrequenz = fCPU, Ausgabefrequenz 15,625kHz
// clear OC1A on compare match, top=1023
void timerInit (void) {
	// TimerCounter 1: FastPWM, 10 Bit, no Prescaler
	TCCR1A = 1<<COM1A1 | 1<<WGM11 | 1<<WGM10;
	TCCR1B = 1<<WGM12 | 1<<CS10;
	OCR1A = 100; // Startwert 10% Power
	
	// TimerCounter 2: CTC Mode für CRON (compareMatch alle 10ms)
	TCCR2 = 1<<WGM21 | 1<<CS22 | 1<<CS21 | 1<<CS20;
	OCR2 = 156;
	TIMSK |= 1<<OCIE2;
	
}

void isrInit (void) {
	// INT1 und INT0 rising edge
	MCUCR = 1<<ISC01 | 1<<ISC00 | 1<<ISC11 | 1<<ISC10;
	GICR = 1<<INT1 | 1<<INT0;
	UCSRB |= 1<<RXCIE;
}

void adcInit(void) {
	ADMUX = 1<<REFS1 | 1<<REFS0; // Referenz auf interne Referenz (==2,56V)
	ADCSRA = 1<<ADEN | 1<<ADSC | 7<<ADPS0; // ADC an, Prescaler=128
}

void adcStart(uint8_t channel) {
	ADMUX = 1<<REFS1 | 1<<REFS0 | channel;
	ADCSRA |= 1<<ADSC;
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

volatile int8_t motorstatus = 0; // 0 heißt OFF, 1 RECHTS, -1 LINKS
void a4960Init (void) {
	sbi(PORTD, PD4); // aktiviere den a4960
	delayms(10);
	a4960Com (0b0001, BT3 | DT4 | DT2);
	a4960Com (0b0011, VR3 | VT5);
	a4960Com (0b0101, PT4);
	a4960Com (0b0111, HQ1 | HQ0 | HT2);
	a4960Com (0b1001, EC2 | EC1 | EC0 | SC1);
	a4960Com (0b1011, RQ3 | RR1);
	a4960Com (0b1101, 0x0FFF);
	a4960Com (0b1111, BW2 | BW1 | RSC);
}

void a4960MotorOn (int8_t drehrichtung) {
	if (drehrichtung > 0) {
		a4960Com (0b1111, BW2 | RSC | RUN);
		motorstatus = 1;
	} else if (drehrichtung < 0) {
		a4960Com (0b1111, BW2 | RSC | RUN | DIR);
		motorstatus = -1;
	}
}

void a4960MotorOff (void) {
	a4960Com (0b1111, BW2 | RSC);
	motorstatus = 0;
}



// ISR

// INT0 wird durch eine steigende Flanke vom TACHO-Ausgang getriggert.
volatile uint16_t anzahlWellen = 0;
ISR (INT0_vect, ISR_BLOCK) {
	ERROR(1);
	anzahlWellen ++;
	ERROR(0);
}

// ERROR Handler
ISR (INT1_vect, ISR_BLOCK) {
// 	ERROR(1);
}

uint16_t tacho = 0, sollfrequenz = 1200; // Tachofrequenz...

//0 = Warten auf neue; 1 = msg start; 2 = motorCW; 3 = motorCCW; 4 = motor stop; 5 = rpm; 200 = ERROR
uint8_t msgStatus = 0;

//
ISR (USART_RXC_vect, ISR_BLOCK) {
	char empfangen = UDR;

	static uint8_t rpm_checksum = 0;
	static uint8_t rpm_lastDigit = 0;
	static uint16_t rpm_setTo = 0;

	uartTx(empfangen); // remote echo

	if (empfangen == '>') { //
		if(msgStatus == 99){
			//Nothing
		} else if(msgStatus <= 1){
			uartTxStrln("<NoM>"); //Empty Message
		} else if(msgStatus == 2){
			if(motorstatus != -1) {
				a4960MotorOn(1);
				uartTxStrln("<ACK>");
			} else {
				uartTxStrln("<ERX>");
			}
		} else if(msgStatus == 3){
			if(motorstatus != 1) {
				a4960MotorOn(-1);
				uartTxStrln("<ACK>");
			} else {
				uartTxStrln("<ERX>");
			}
		} else if(msgStatus == 4){
			a4960MotorOff();
			uartTxStrln("<ACK>");
		} else if(msgStatus == 5){
			if (rpm_lastDigit == rpm_checksum) {
				cli();
				sollfrequenz = rpm_setTo;
				sei();
				uartTxStrln("<ACK>");
			} else {
				uartTxStrln("<ERC>");
			}
		} else {
			uartTxStrln("<ERR>");
		}
		msgStatus = 0;
	} else if (msgStatus == 0) { //Erwarte Start der Nachricht
		if (empfangen == '<') {
			msgStatus = 1
		} else {
			msgStatus = 200;
		}
	} else if (msgStatus == 1) { //waehle nachrichten-modus
		if(empfangen == 's') {
			msgStatus = 2;
		} else if(empfangen == 'z') {
			msgStatus = 3;
		} else if(empfangen == 'x') {
			msgStatus = 4;
		} else if(empfangen == 'r') {
			msgStatus = 5;
			rpm_checksum = 0;
			rpm_lastDigit = 222;
			rpm_setTo = 0;
		} else {
			msgStatus = 99; //Unbekannte anweisung -> ignorieren
		}
	} else if (msgStatus == 5) {
		if(empfangen < '0' || empfangen > '9') {
			msgStatus = 200;
		} else if(empfangen == 'r') {
			msgStatus = 6;
		} else {
			rpm_setTo *= 10;
			rpmSetTo += empfangen - '0';
			rpm_checksum++;
		}
	} else if (msgStatus == 6) {
		if(empfangen < '0' || empfangen > '9') {
			msgStatus = 200;
		}
		rpm_lastDigit += empfangen - '0';
	} else if (msgStatus == 99) {
		//ignorieren
	} else {
		msgStatus = 200;
	}
}

// CRON und Regelung
float kp=0.05, ki=0.01;
volatile uint16_t eingangsspannung = 0, logikspannung = 0;
#define IMAX 100
#define PWMMAX 1023 // Vollgas fuer 10Bit PWM
#define VREF 2560 // in mV

ISR (TIMER2_COMP_vect, ISR_BLOCK) {
	ERROR(1);
	static uint8_t systick=0;
	static uint8_t adcKanal=0; // ch0 = Vin, ch1 = +5V
	systick ++;
	if (systick == 10) { // 100ms vergangen, regelzeit!
		systick = 0;
		tacho = anzahlWellen*10;
		anzahlWellen = 0;
		
		uint16_t adcWert = ADC;
		if(adcKanal == 0) {
			adcKanal = 1;
			eingangsspannung = (((uint32_t)adcWert * VREF * 11) / 1024 / 1);
			
		} else if (adcKanal == 1) {
			adcKanal = 0;
			logikspannung = (((uint32_t)adcWert * VREF * 39) / 1024 / 14);
			
		} else { // Da war ein Fehler - keine Auswertung des gemessenen Wertes.
			adcKanal = 0;
		}
		
		adcStart(adcKanal);
		
		// Regler: Konzept PI, D folgt
		// Vorsteuerung: PWM-Grad von 100 entspricht 1kHz
		
		float ek = (float)sollfrequenz - (float)tacho;
		static float yk = 0;
		
// 		RampUp-Schutz für I-Anteil
		if ((yk + (ki * ek)) < IMAX) {
			yk += (ki * ek);
		}
		int out = 0;
		if (motorstatus) {
			out = (100 + yk + (kp*ek));
		} else {
			out = 0;
		}
		
#ifdef DEBUG
		char buf[50];
		sprintf(buf, "ek: %f, yk: %f, out: %i\r\n", ek, yk, out);
		uartTxStr(buf);
#endif
		
		// Setze PWM-Grad
		if (out > PWMMAX) {
			OCR1A = PWMMAX;
		} else if (out < 0) {
			OCR1A = 0;
		} else {
			OCR1A = (out);
		}
	}
	ERROR(0);
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
	
	// Aktiviere Programmmodule
	timerInit();
	uartInit();
	spiInit();
	adcInit();
	isrInit();
	
	delayms(100);
	uartTxNewline();
	uartTxPstrln(PSTR("<Motorcontroller fuer kleine Drehstrommaschinen>"));
	uartTxNewline();
	char buf[100];
// 	uint16_t tempvar = 0;
	
	a4960Init();
	uartTxStrln("<...> aktiviere Motor <s>; Drehzahl <r500r3>; stop <x>; rueckwaerts <z>");
	
	
	sei(); // und es seien Interrupts :D
	
	uint8_t i = 0;
	while(1) {
		sprintf(buf, "<t F(soll): %u Hz, F(ist): %u Hz, UMotor: %#2.2u mV, ULogik: %#2.2u mV>\r", sollfrequenz, tacho, eingangsspannung, logikspannung);
		while(msgStatus != 0) {//verhindern von nachrichtenkollision
			delayms(10);
		}
		uartTxStr(buf);
		delayms(250);
	}
	return 0;
}
