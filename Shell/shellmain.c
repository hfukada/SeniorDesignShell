/*
 * GccApplication1.c
 *
 * Created: 3/31/2014 6:47:36 PM
 *  Author: team9
 */ 

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define CAP_HIGH_THRESH 0x80
#define CAP_LOW_THRESH 0x25
// Function Prototypes
void initSPI ();
void initCapSense ();
void initPressureSense ();
int getBlow ();
void mainLoop ();
void senseCapOneWay();
void senseCapTwoWay();

void InitUART( unsigned char );
void TransmitByte( unsigned char );



// Struct Definitions
struct __capTime{
	char prevVal;
	char startTime;
	volatile char capWaiting;
	char time;
};
// Global variables
struct __capTime globT[6];
char capCalib[6];
int baseBreathValue;
volatile char tempVal = 0;
char intrCnt;

int main(void)
{
	int i = 0;
	cli();
	initSystem();
	_delay_ms(100);
	sei();
	mainLoop();
}
void initSystem()
{
	initSPI();
	initTimer();
	initCapSense();
	initPressureSense();
}
void initTimer()
{
	TCCR0B = (1<<CS00)|(1<<CS01); //can change registers to increase frequency
	WDTCSR &= ~(1<<WDE); //disable watchdog timer
}
void initSPI ()
{
	UBRR1 = 0;
	/* Setting the XCKn port pin as output, enables master mode. */
	DDRD |= _BV(5);
	//XCK1_DDR |= (1<<XCK1);
	/* Set MSPI mode of operation and SPI data mode 0. */
	//UCSR1C = (1<<UMSEL11)|(1<<UMSEL10)|(1<< UCSZ11)|(1<< UCSZ10)|(0<<UCPOL1);
	UCSR1C = (1<<UMSEL11)|(1<<UMSEL10)|(0<<UCPOL1);
	/* Enable receiver and transmitter. */
	UCSR1B = (1<<RXEN1)|(1<<TXEN1);
	/* Set baud rate. */
	/* IMPORTANT: The Baud Rate must be set after the transmitter is
		 enabled */
	UBRR1 = 51;
}//
void initCapSense()
{
	int i;
	//RECEIVE PINS PB0 - PB5
	PCMSK0 = (1<<PCINT5)|(1<<PCINT4)|(1<<PCINT3)|(1<<PCINT2)|(1<<PCINT1)|(1<<PCINT0);//set pin interrupts
	PCICR = 0; //disable pin change interrupts
	//SEND PINS PB6, PC6, PC7, PD4, PD6, PD7
	DDRB |= (1<<DDB6);
	DDRC |= (1<<DDC6)|(1<<DDC7);
	DDRD |= (1<<DDD4)|(1<<DDD6)|(1<<DDD7);
	//Set the send pins to 0 explicitly (not really needed)
	PORTB &= ~(1<<PINB6);
	PORTC &= ~((1<<PINC6)|(1<<PINC7));
	PORTD &= ~((1<<PIND4)|(1<<PIND6)|(1<<PIND7));

	tempVal = DDRB;
	tempVal = PORTB;
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
	ADCSRA |= (1<<ADEN); //use ADLAR to read

	baseBreathValue = ADCH;
}
void mainLoop()
{
	char i;
	char note;
	int vol;
	while(1){
		senseCapOneWay();
		note = genNote();
		getBlow();
		vol = genVolume();
		sendToCore(note,vol);
	}	
}
int getBlow(){
	char currBlow = ADCH;
	return currBlow > baseBreathValue ? currBlow - baseBreathValue : 0;
}
void senseCapOneWay()
{
	int i;
	PCICR = 1; //enable pin change interrupt
	//toggle send pins and start time

	intrCnt = 0;
	PORTD |= (1<<PIND4);
	globT[0].startTime = TCNT0;
	waitForCap(0);
	PORTD &= ~(1<<PIND4);
	waitForCap(0);

	intrCnt = 1;
	PORTB |= (1<<PINB6);
	globT[1].startTime = TCNT0;
	waitForCap(1);
	PORTB &= ~(1<<PINB6);
	waitForCap(1);

	intrCnt = 2;
	PORTD |= (1<<PIND6);
	globT[2].startTime = TCNT0;
	waitForCap(2);
	PORTD &= ~(1<<PIND6);
	waitForCap(2);

	intrCnt = 3;
	PORTD |= (1<<PIND7);
	globT[3].startTime = TCNT0;
	waitForCap(3);
	PORTD &= ~(1<<PIND7);
	waitForCap(3);

	intrCnt = 4;
	PORTC |= (1<<PINC6);
	globT[4].startTime = TCNT0;
	waitForCap(4);
	PORTC &= ~(1<<PINC6);
	waitForCap(4);

	intrCnt = 5;
	PORTC |= (1<<PINC7);
	globT[5].startTime = TCNT0;
	waitForCap(5);
	PORTC &= ~(1<<PINC7);
	waitForCap(5);

	PCICR = 0; //disable pin change interrupts
}
void waitForCap(char i)
{
	while(globT[i].capWaiting == 1);
	globT[i].capWaiting = 1;
}
ISR(PCINT0_vect)
{
	char val;
	val = (PINB>>intrCnt)&1;
	if (globT[intrCnt].prevVal != val)
	{
		globT[intrCnt].capWaiting = 0;
		globT[intrCnt].prevVal = val;
		if (val == 1)
			globT[intrCnt].time = TCNT0 - globT[intrCnt].startTime; //endTime-startTime
	}
}
//generate MIDI
genPitchBend()
{

}
genVolume()
{

}
genNote()
{

}
testSend()
{
	for (i=0; i<6;i++){
		if (globT[i].time >= 0x25)
			TransmitByte(1);
		else
			TransmitByte(0);
		_delay_ms(100);
	}
}
//transmit spi
void TransmitByte( unsigned char data )
{
	while ( !(UCSR1A & (1 << UDRE1)) )
		; 			                /* Wait for empty transmit buffer */
	UDR1 = data; 			        /* Start transmittion */
}
