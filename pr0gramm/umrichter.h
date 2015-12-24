// header zur umrichter.c

#ifndef _HAVE_HEADER_UMRICHTER_H
#define _HAVE_HEADER_UMRICHTER_H

//Allgemeiner Defines-Block
#include "../../AtmelLib/global.h"
#include "../../AtmelLib/io/io.h"
#include <stdio.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>


// Configregister wie im Datenblatt auf Seite 24 angegeben:

#define A4960_CONFIG0 0x00 // Blanktime, Deadtime
#define WR 1<<12
#define CB1 1<<11
#define CB0 1<<10
#define BT3 1<<9
#define BT2 1<<8
#define BT1 1<<7
#define BT0 1<<6
#define DT5 1<<5
#define DT4 1<<4
#define DT3 1<<3
#define DT2 1<<2
#define DT1 1<<1
#define DT0 1<<0

#define A4960_CONFIG1 0x01 // Vref, Vosth
#define VR3 1<<9
#define VR2 1<<8
#define VR1 1<<7
#define VR0 1<<6
#define VT5 1<<5
#define VT4 1<<4
#define VT3 1<<3
#define VT2 1<<2
#define VT1 1<<1
#define VT0 1<<0

#define A4960_CONFIG2 0x02 // PWM control
#define PT4 1<<4
#define PT3 1<<3
#define PT2 1<<2
#define PT1 1<<1
#define PT0 1<<0

#define A4960_CONFIG3 0x03 // Hold
#define IDS 1<<8
#define HQ3 1<<7
#define HQ2 1<<6
#define HQ1 1<<5
#define HQ0 1<<4
#define HT3 1<<3
#define HT2 1<<2
#define HT1 1<<1
#define HT0 1<<0

#define A4960_CONFIG4 0x04 // Start Com
#define EC3 1<<7
#define EC2 1<<6
#define EC1 1<<5
#define EC0 1<<4
#define SC3 1<<3
#define SC2 1<<2
#define SC1 1<<1
#define SC0 1<<0

#define A4960_CONFIG5 0x05 // Ramp
#define PA3 1<<11
#define PA2 1<<10
#define PA1 1<<9
#define PA0 1<<8
#define RQ3 1<<7
#define RQ2 1<<6
#define RQ1 1<<5
#define RQ0 1<<4
#define RR3 1<<3
#define RR2 1<<2
#define RR1 1<<1
#define RR0 1<<0

#define A4960_MASK 0x06 // Diagnostics Flag Mask
#define TW 1<<11
#define TS 1<<10
#define LOS 1<<9
#define VA 1<<8
#define VB 1<<7
#define VC 1<<6
#define AH 1<<5
#define AL 1<<4
#define BH 1<<3
#define BL 1<<2
#define CH 1<<1
#define CL 1<<0

#define A4960_RUN 0x07 // Run Register
#define BH1 1<<11
#define BH0 1<<10
#define BW2 1<<9
#define BW1 1<<8
#define BW0 1<<7
#define ESF 1<<6
#define DG1 1<<5
#define DG0 1<<4
#define RSC 1<<3
#define BRK 1<<2
#define DIR 1<<1
#define RUN 1<<0

#define A4960_DIAGNOSTIC 0x08 // Diagnostics Register - read only
#define FF 1<<15
#define POR 1<<14
#define VR 1<<13

// DEFINES für einfachen Pin-Zugriff
// CS vom A4960 (active high)
#define CS(x) out(PORTB,PB2,0,x)
#define ON(x) out(PORTD,PD7,0,x)
#define ERROR(x) out(PORTD,PD6,0,x)


// Funktionsprototypen
int main(void);

void spiInit(void);
uint8_t spiRx (void);
uint8_t spiTxReady (void);
void spiTx(uint8_t datenwort);
	


#endif
