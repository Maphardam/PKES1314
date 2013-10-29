#include <avr/io.h>
#define F_CPU 16000000    // Systemtakt in Hz
#define BAUD 9600      // Baudrate

#include <inttypes.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/setbaud.h>

#include <stdint.h>

volatile int8_t counter_trigger;
volatile int8_t counter_dir;
volatile int8_t counter_value;

#include "def.h"

extern void counter_func(void);
void init();
int uart_putc(char);
void uart_puts (const char*);
void writetoDisplay(char, char , char );

enum number {
	ZERO    = 0b11111101,
	ONE     = 0b01100001,
	TWO     = 0b11011011,
	THREE   = 0b11110011,
	FOUR    = 0b01100111,
	FIVE    = 0b10110111,
	SIX     = 0b10111111,
	SEVEN   = 0b11100001,
	EIGHT   = 0b11111111,
	NINE    = 0b11110111,
	NOTHING = 0b00000001,
	MINUS   = 0b00000010
};

void init()
{
	// -------------------------------------------------------------
	// Single Onboard LED configuration
	// -------------------------------------------------------------
	// set leds LED_X as output
	// alternative
	//     - intermediate
	//       DDRA=1+2+4+8;
	//     - using processor makros
	//       DDRA=((1<<DDA0) | (1<<DDA1) | (1<<DDA2) | (1<<DDA3));
	//     - using hardware specific makros
	DDRB |= (1<<DDA7);
	// disable leds
	PORTB &= ~(1<<7);
	// -------------------------------------------------------------
	// Serial bus lines
	// -------------------------------------------------------------
	// Pin 5 = PORT E 3 = clock
	DDRE |= (1<<DDE3);
	// Pin 6 = PORT H 3 = data
	DDRH |= (1<<DDH3);
	// Pin 7 = PORT H 4 = enable
	DDRH |= (1<<DDH4);
	// -------------------------------------------------------------
	// Button configuration
	// -------------------------------------------------------------
	// not necessary but for completion
	// S1 as input
	DDRF &=~(1<<DDG4);
	// S2 as input
	DDRG &=~(1<<DDG5);

	// --------------------------------------------------------------
	// UART 0 - Debug
	// --------------------------------------------------------------
	UBRR0H = UBRRH_VALUE;;    //Define Baud rate
	UBRR0L = UBRRL_VALUE;
	#if USE_2X
	UCSR0A |= (1 << U2X0);/* U2X-Modus erforderlich */
	#else
	UCSR0A &= ~(1 << U2X0);/* U2X-Modus nicht erforderlich */
	#endif
	UCSR0B = (1<<TXEN0); //Enable Recceiver and Transmitter
	UCSR0C = (0<<USBS0) | (1<<UCSZ00) | (1<<UCSZ01);
}

int main() {

	init();
	char s[20];
	int delay = 10; // main clock [ms]
	float version = 0.2;
	char disp [] = {0b10011111,0b11111101,0b10110111};

	uart_puts("----------------------------------\r\n");
	uart_puts("PKES Wintersemester 2013/14\r\n");
	uart_puts("Vorlage 1. Aufgabe - Version ");
	uart_puts( dtostrf( version, 1, 2, s ) );
	uart_puts("\r\n----------------------------------\r\n");

	writetoDisplay(disp[0], disp[1], disp[2]);

	counter_trigger=COUNTER_CYCLE;
	counter_value=0;
	counter_dir=0;

	uart_puts( "Taste druecken \r\n" );
	while(1) {
		if ((PINF & (1<<4)) || (PING & (1<<5)))
			break;
	}


	while(1)
	{
		_delay_ms (delay);
		PORTB &= ~(1<<7);
		_delay_ms (delay);
		PORTB |= (1<<7);

		counter_trigger=COUNTER_CYCLE;
		while (counter_trigger > 0) {
			counter_func();
			sprintf( s, "trigger: %3d", counter_trigger );
			uart_puts( s );
			uart_puts("\r\n");
			sprintf( s, "| count: %2d | ", counter_value );
			uart_puts( s );
			uart_puts("\r\n");
		}

		if (counter_dir==1)
			uart_puts(" up " );
		else
			uart_puts(" sdown" );
		uart_puts( " | \r\n" );

		char negative = 0;
		if (counter_value < 0) {
			negative = 1;
			counter_value *= -1;
		}

		// last digit
		switch (counter_value % 10) {
		case 0: disp[2] = ZERO; break;
		case 1: disp[2] = ONE; break;
		case 2: disp[2] = TWO; break;
		case 3: disp[2] = THREE; break;
		case 4: disp[2] = FOUR; break;
		case 5: disp[2] = FIVE; break;
		case 6: disp[2] = SIX; break;
		case 7: disp[2] = SEVEN; break;
		case 8: disp[2] = EIGHT; break;
		case 9: disp[2] = NINE; break;
		default: break;
		}

		// middle digit
		switch ((counter_value / 10) % 10) {
		case 0: if (counter_value < 100) {
					disp[1] = NOTHING;
				}
				else {
					disp[1] = ZERO;
				}
				break;
		case 1: disp[1] = ONE; break;
		case 2: disp[1] = TWO; break;
		case 3: disp[1] = THREE; break;
		case 4: disp[1] = FOUR; break;
		case 5: disp[1] = FIVE; break;
		case 6: disp[1] = SIX; break;
		case 7: disp[1] = SEVEN; break;
		case 8: disp[1] = EIGHT; break;
		case 9: disp[1] = NINE; break;
		default: break;
		}

		// first digit
		switch ((counter_value / 100) % 10) {
		case 0: disp[0] = NOTHING; break;
		case 1: disp[0] = ONE; break;
		default: break;
		}

		if (negative == 1) {
			if (counter_value < 10) {
				disp[1] |= MINUS;
			}
			else {
				disp[0] |= MINUS;
			}
			counter_value *= -1;
		}

		writetoDisplay(disp[0]^=1, disp[1]^=1, disp[2]^=1);
	}
	return 0;
}

void writetoDisplay(char digit1, char digit2, char digit3){

	char stream[36];
	stream[0]=1;
	int i;
	for ( i=1; i<36; i++ ) {
		stream[i]=0;
	}
	
	for ( i=0; i<8; i++ ) {
		if (digit1 & (1<<(7-i))) stream[i+ 1]=1;
		if (digit2 & (1<<(7-i))) stream[i+9]=1;
		if (digit3 & (1<<(7-i))) stream[i+17]=1;
	}

	// Pin 5 = PORT E 3 = clock
	// Pin 6 = PORT H 3 = data
	// Pin 7 = PORT H 4 = enable

	for ( i=0; i<36; i++ ) {
		// clock low
		PORTE &= ~(1<<3);
		// data enable low
		PORTH &= ~(1<<4);
		_delay_us (1);
		// data
		if (stream[i]==1)
			PORTH |= (1<<3);
		else
			PORTH &=~(1<<3);
		_delay_us (1);
		// clock high - Transmission finished
		PORTE |= (1<<3);
		_delay_us (1);
		// data enable high - ready for next cycle
		PORTH |= (1<<4);
	}
}

int uart_putc(char c)
{
	while (!(UCSR0A & (1<<UDRE0)));
	UDR0 = c;
	return 0;
}

void uart_puts (const char *s)
{
	while (*s) {
		uart_putc(*s);
		s++;
	}
}

