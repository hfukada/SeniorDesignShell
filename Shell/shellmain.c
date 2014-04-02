/*
 * GccApplication1.c
 *
 * Created: 3/31/2014 6:47:36 PM
 *  Author: team9
 */ 

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// Function Prototypes
void SPIInit ();
void initCapSense ();
void initPressureSense ();
int getBlow ();
void mainLoop ();
int baseBreathValue;

int main(void)
{
	cli();
	initSystem();2
    mainLoop();
}
void initSystem()
{
	
}
void SPIInit ()
{
	UBRR1 = 0;
	/* Setting the XCKn port pin as output, enables master mode. */
	DDRD |= _BV(5);
	//XCK1_DDR |= (1<<XCK1);
	/* Set MSPI mode of operation and SPI data mode 0. */
	UCSR1C = (1<<UMSEL11)|(1<<UMSEL10)|(1<< UCSZ11)|(1<< UCSZ10)|(0<<UCPOL1);
	/* Enable receiver and transmitter. */
	UCSR1B = (1<<RXEN1)|(1<<TXEN1);
	/* Set baud rate. */
	/* IMPORTANT: The Baud Rate must be set after the transmitter is
	enabled */
	UBRR1 = 51;
}//
void initCapSense()
{
	//RECEIVE PINS PB0 - PB5
	PCMSK0 = (1<<PCINT5)|(1<<PCINT4)|(1<<PCINT3)|(1<<PCINT2)|(1<<PCINT1)|(1<<PCINT0);//set pin interrupts
	PCICR = 0; //disable pin change interrupts
	//SEND PINS PB6, PC6, PC7, PD4, PD6, PD7
	DDRB |= (1<<DDB6);
	DDRC |= (1<<DDC6)|(1<<DDC7);
	DDRD |= (1<<DDD4)|(1<<DDD6)|(1<<DDD7);
	
	PORTB |= (0<<PINB6);
	PORTC |= (0<<PINC6|(0<<PINC7));
	PORTD |= (0<<PIND4)|(0<<PIND6)|(0<<PIND7);
}
void initPressureSense()
{
	ADMUX = (1<<REFS0)|(1<<REFS1)|(1<<ADLAR)|(1<<4); // 1<<4 => 010000 -> differential input (ADC0+, ADC1-) 1x gain.
	ADCSRA = (1<<ADEN)|(1<<ADIE); //use ADLAR to read
	
	baseBreathValue = ADCH;
}

int getBlow(){
	int currBlow = ADCH;
	return currBlow > baseBreath ? currBlow - baseBreathValue : 0;
}
void mainLoop()
{
}