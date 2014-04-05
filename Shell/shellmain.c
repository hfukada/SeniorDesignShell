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
void senseCap()

void InitUART();
void TransmitByte();


int baseBreathValue;
// Struct Definitions
typedef struct __capTime{
	char prevVal;
    char startTime;
    char capWaiting;
    char time;
} capTime;
// Global variables
capTime globT[6];

int main(void)
{
	cli();
	initSystem();
    mainLoop();
}
void initSystem()
{
	SPIInit();
	initTimer();
	initInterrupts();
	initCapSense();
	initPressureSense();
	InitUART( 51 );
}
void initTimer()
{
        TCCR0B = (1<<CS00); //can change registers to increase frequency
        globT.
}
void initInterrupts()
{
        PCMSK0 |= (1<<PCINT5)|(1<<PCINT4)|(1<<PCINT3)|(1<<PCINT2)|(1<<PCINT1)|(1<<PCINT0); //turn on receive pin interrupts
        PCICR = 1; //turn on digital pin interrupts
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
	//Set the send pins to 0;
	PORTB &= ~(1<<PINB6);
	PORTC &= ~((1<<PINC6)|(1<<PINC7));
	PORTD &= ~((1<<PIND4)|(1<<PIND6)|(1<<PIND7));
	//Init globT struct
	for (i = 0; i < 6; i++)
	{
		globT[i].capWaiting = 1;
		globT[i].prevVal = 0;
	}
}
void initPressureSense()
{
	ADMUX |= (1<<REFS0)|(1<<REFS1)|(1<<ADLAR)|(1<<4); // 1<<4 => 010000 -> differential input (ADC0+, ADC1-) 1x gain.
	ADCSRA |= (1<<ADEN)|(1<<ADIE); //use ADLAR to read
	
	baseBreathValue = ADCH;
}

int getBlow(){
	char currBlow = ADCH;
	return currBlow > baseBreath ? currBlow - baseBreathValue : 0;
}
void mainLoop()
{
	char i;
	while(){
		senseCap();
		//getBlow();
		for (i=0; i<6;i++){
			TransmitByte( globT[i].time );
		}
	}	
}
void senseCap()
{
        int i;
        //toggle send pins and start time
        PIND |= (1<<PORTD4);
        globT[0].startTime = TCNT0;
        PINB |= (1<<PORTB4);
        globT[1].startTime = TCNT0
        PIND |= (1<<PORTD6);
        globT[2].startTime = TCNT0;
        PIND |= (1<<PORTD7);
        globT[3].startTime = TCNT0;
        PINC |= (1<<PORTC6);
        globT[4].startTime = TCNT0;
        PINC |= (1<<PORTC7);
        globT[5].startTime = TCNT0;
        
        //wait for answer from all sources
        for (i = 0; i < 6; i++)
        {
                while(globT[i].capWaiting == 1);
                globT[i].capWaiting = 1;
        }
}
ISR(PCINT0_vect)
{
	int i;
	char val;
	for (i = 0; i < 6; i++)
	{
		val = (PORTB>>i)&1;
		if (globT[i].prevVal != val)
		{
				globT[i].capWaiting = 0;
				globT[i].prevVal = val;
				globT[i].time = TCNT0 - globT.startTime; //endTime-startTime
		}
	}
}
/* Initialize UART */
void InitUART( unsigned char baudrate )
{
	
	UBRR1H = baudrate >> 8;                  /* Set the baud rate */
	UBRR1L = baudrate;
	UCSR1B = (1 << RXEN1) | (1 << TXEN1);  /* Enable UART receiver and transmitter */
	/* Set frame format: 8data, 2stop bit */
	UCSR1C = (1 << USBS1) | (3 << UCSZ10);
}
void TransmitByte( unsigned char data )
{
	while ( !(UCSR1A & (1 << UDRE1)) )
	; 			                /* Wait for empty transmit buffer */
	UDR1 = data; 			        /* Start transmittion */
}