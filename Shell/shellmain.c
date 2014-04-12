/*
 * GccApplication1.c
 *
 * Created: 3/31/2014 6:47:36 PM
 *  Author: team9
 */ 

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define CAP_HIGH_THRESH 0x25
#define CAP_LOW_THRESH 0x10
// Function Prototypes
void initSPI ();
void initCapSense ();
void initPressureSense ();
int getBlow ();
void mainLoop ();
void senseCapOneWay();
void senseCapTwoWay();
void testSend();

void InitUART( unsigned char );  
void TransmitByte( unsigned char );



// Struct Definitions
struct __capTime{
	char prevVal;
	int startTime;
	volatile char capWaiting;
	volatile unsigned char time;
	volatile unsigned char calibration;
};
struct __midiNoteOff{
	char status;
	char note;
	char velocity;
};
struct __midiNoteOn{
	char status;
	char note;
	char velocity;
};
struct _midiPitchBend{
	char status;
	char lsb;
	char msb;
};
struct __midiVolume{
	char status; //0xB0
	char control; //0x07
	char value;
};
// Global variables
struct __capTime globT[6];
char capCalib[6];
char baseBreathValue;
volatile char tempVal = 0;
char intrCnt;

int main(void)
{
	int i = 0;
	cli();
	initSystem();
	_delay_ms(100);
	sei();
	calibrate();
    mainLoop();
}

//-----------------------------------------------------------------------------------------------------------------
// main loop
//-----------------------------------------------------------------------------------------------------------------
void mainLoop()
{
	char i;
	char note;
	int vol;
	while(1){
		senseCapOneWay();
		//note = genNote();
		i = getBlow();
		//vol = genVolume();
		//sendToCore(note,vol);
		testSend();
	}	
}


//-----------------------------------------------------------------------------------------------------------------
// INITIALIZATIONS
//-----------------------------------------------------------------------------------------------------------------
void initSystem()
{
	initSPI();
	initTimer();
	initCapSense();
	initPressureSense();
}

void initTimer()
{
	TCCR0B = (1<<CS01); //can change registers to increase frequency
	WDTCSR &= ~(1<<WDE); //disable watchdog timer
}

void initSPI ()
{
	UBRR1 = 0;
	/* Setting the XCKn port pin as output, enables master mode. */
	DDRD |= _BV(5);
	/* Set MSPI mode of operation and SPI data mode 0. */
	//UCSR1C = (1<<UMSEL11)|(1<<UMSEL10)|(1<< UCSZ11)|(1<< UCSZ10)|(0<<UCPOL1);
	UCSR1C = (1<<UMSEL11)|(1<<UMSEL10)|(0<<UCPOL1);
	/* Enable receiver and transmitter. */
	UCSR1B = (1<<TXEN1);
	/* Set baud rate. */
	/* IMPORTANT: The Baud Rate must be set after the transmitter is
		 enabled */
	UBRR1 = 51;
}

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
	//ADMUX |= (1<<REFS1)|(1<<ADLAR)|(1<<4); // MUX5:0 = 010000 -> differential input (ADC0{PF0}+, ADC1{PF1}-) 1x gain.
	ADMUX |= (1<<REFS1)|(1<<ADLAR); // MUX5:0 = 000000 -> single input (ADC0{PF0} ) 1x gain.
	ADCSRA |= (1<<ADEN)|(1<<ADSC)|(1<<ADATE); //use ADLAR to read

}
void calibrate()
{
	senseCapOneWay();
	for (int i = 0; i < 6; i++)
	{
		globT[i].calibration = globT[i].time + CAP_LOW_THRESH;
	}
	
	baseBreathValue = ADCH;
}

//-----------------------------------------------------------------------------------------------------------------
// Helper functions
//-----------------------------------------------------------------------------------------------------------------

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
	TCNT0 = 0;
	globT[0].startTime = TCNT0;
	waitForCap(0);
	PORTD &= ~(1<<PIND4);
	waitForCap(0);

	intrCnt = 1;
	PORTB |= (1<<PINB6);
	TCNT0 = 0;
	globT[1].startTime = TCNT0;
	waitForCap(1);
	PORTB &= ~(1<<PINB6);
	waitForCap(1);

	intrCnt = 2;
	PORTD |= (1<<PIND6);
	TCNT0 = 0;
	globT[2].startTime = TCNT0;
	waitForCap(2);
	PORTD &= ~(1<<PIND6);
	waitForCap(2);

	intrCnt = 3;
	PORTD |= (1<<PIND7);
	TCNT0 = 0;
	globT[3].startTime = TCNT0;
	waitForCap(3);
	PORTD &= ~(1<<PIND7);
	waitForCap(3);

	intrCnt = 4;
	PORTC |= (1<<PINC6);
	TCNT0 = 0;
	globT[4].startTime = TCNT0;
	waitForCap(4);
	PORTC &= ~(1<<PINC6);
	waitForCap(4);

	intrCnt = 5;
	PORTC |= (1<<PINC7);
	TCNT0 = 0;
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
void genPitchBend()
{

}
void genVolume()
{

}
void genNote()
{
	//Status
}
void testSend()
{
	int i;
	unsigned char checksum = 0;
	unsigned char transVal;
	for (i=0; i<6;i++){
		if (globT[i].time > CAP_HIGH_THRESH + globT[i].calibration)
			transVal = 0xFF;
		else if (globT[i].time > globT[i].calibration)
			transVal = globT[i].time - globT[i].calibration;
		else
			transVal = 0;
		TransmitByte(transVal);
		checksum += transVal;
		_delay_ms(2);
	}
	TransmitByte(checksum);
}
//transmit spi
void TransmitByte( unsigned char data )
{
	while ( !(UCSR1A & (1 << UDRE1)) )
		; 			                /* Wait for empty transmit buffer */
	UDR1 = data; 			        /* Start transmittion */
}
