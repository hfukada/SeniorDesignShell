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
#define CAP_LOW_THRESH 0x80
#define PRESS_HIGH_THRESH 0x2F
// Function Prototypes
void initSPI ();
void initCapSense ();
void initPressureSense ();
void getBlow ();
void mainLoop ();
void senseCap();
void testSend();

void InitUART( unsigned char );  
void TransmitByte( unsigned char );
char genTouched();



// Struct Definitions
struct __capTime{
	char prevVal;
	unsigned int startTime;
	volatile char capWaiting;
	volatile unsigned int time;
	volatile unsigned int calibration;
};
struct __midiNoteOff{
	char status; //0x80
	char note;
	char velocity; //doesn't matter
};
struct __midiNoteOn{
	char status; //0x90
	char note;
	char velocity;
};
struct __midiPitchBend{
	char status; //0xE0
	char lsb;
	char msb;
};
struct __midiVolume{
	char status; //0xB0
	char control; //0x07
	char val;
};
struct __midiAllNoteOff{
	char status; //0xB0
	char control; //0x07
	char val;
};
// Global variables
struct __capTime globT[6];
struct __midiNoteOn noteOn;
struct __midiPitchBend pitchBend;
struct __midiVolume volume;
struct __midiAllNoteOff allOff;
char prevNote;
char prevVol;
char capCalib[6];
char baseBreathValue;
char breathValue;
volatile char tempVal = 0;
char intrCnt;
volatile char overflow = 0;
int main(void)
{
	int i = 0;
	cli();
	initSystem();
	_delay_ms(100);
	sei();
	_delay_ms(100);
	calibrate();
	mainLoop();
}

//-----------------------------------------------------------------------------------------------------------------
// main loop
//-----------------------------------------------------------------------------------------------------------------
void mainLoop()
{
	char i;
	int vol;
	while(1){
		//_delay_ms(1000);
		senseCap();
		getBlow();
		
		genNoteOn();
		genPitchBend();
		genVolume();
		
		sendToCore();
		//testSend();
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
	initMidi();
}

void initTimer()
{
	TCCR1B = (1<<CS10); //can change registers to increase frequency
	
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
void initMidi()
{
	noteOn.status = 0x90;
	noteOn.velocity = 0x00;
	noteOn.note = 0x00;
	/*old volume status = b0, 07, 00*/
	volume.status = 0xB0; //uncomment this set for volume
	volume.control = 0x07;
	volume.val = 0x00;
	/*volume.status = 0xD0;  // after touch
	volume.control = 0xFF;
	volume.val = 0x00; */
	pitchBend.status = 0xE0;
	pitchBend.lsb = 0x00;
	pitchBend.msb = 0x00;
	allOff.status = 0xB0;
	allOff.control = 0x7B;
	allOff.val = 0x00;
}
void calibrate()
{
	int arr[11][6];
	//int arr[66];
	int k;
	int new;
	for (int i = 0; i < 11; i++) //sort 11 samples of each
	{	
		senseCap();
		for (int j = 0; j < 6; j++)
		{
			new = globT[j].time;
			for (k = i; k > 0 && arr[k-1][j] > new; k--)
			{
				arr[k][j] = arr[k-1][j];
			}
			arr[k][j] = new;
		}
	}
	for (int i = 0; i < 6; i++) //find the median
	{
		globT[i].calibration = arr[5][i] + CAP_LOW_THRESH;
	//	senseCap();
		//globT[i].calibration = globT[i].time;
	}
	baseBreathValue = ADCH;
}

//-----------------------------------------------------------------------------------------------------------------
// Helper functions
//-----------------------------------------------------------------------------------------------------------------

void getBlow(){
	char currBlow = ADCH;
	breathValue = currBlow > baseBreathValue ? currBlow - baseBreathValue : 0;
}

void senseCap()
{
	int i;
	PCICR = 1; //enable pin change interrupt
	//toggle send pins and start time
	TCCR1B = (1<<CS10); //timer enable
	TIMSK1 = ( 1<<TOIE0); //timer overflow interrupt

	overflow = 0;
	intrCnt = 0;
	PORTD |= (1<<PIND4);
	TCNT1 = 0;
	globT[0].startTime = TCNT1;
	waitForCap(0);
	PORTD &= ~(1<<PIND4);
	waitForCap(0);

	overflow = 0;
	intrCnt = 1;
	PORTB |= (1<<PINB6);
	TCNT1 = 0;
	globT[1].startTime = TCNT1;
	waitForCap(1);
	PORTB &= ~(1<<PINB6);
	waitForCap(1);

	overflow = 0;
	intrCnt = 2;
	PORTD |= (1<<PIND6);
	TCNT1 = 0;
	globT[2].startTime = TCNT1;
	waitForCap(2);
	PORTD &= ~(1<<PIND6);
	waitForCap(2);

	overflow = 0;
	intrCnt = 3;
	PORTD |= (1<<PIND7);
	TCNT1 = 0;
	globT[3].startTime = TCNT1;
	waitForCap(3);
	PORTD &= ~(1<<PIND7);
	waitForCap(3);

	overflow = 0;
	intrCnt = 4;
	PORTC |= (1<<PINC6);
	TCNT1 = 0;
	globT[4].startTime = TCNT1;
	waitForCap(4);
	PORTC &= ~(1<<PINC6);
	waitForCap(4);

	overflow = 0;
	intrCnt = 5;
	PORTC |= (1<<PINC7);
	TCNT1 = 0;
	globT[5].startTime = TCNT1;
	waitForCap(5);
	PORTC &= ~(1<<PINC7);
	waitForCap(5);
	
	TIMSK1 &= ~( 1<<TOIE0);
	TCCR1B = 0;
	PCICR = 0; //disable pin change interrupts
}
void waitForCap(char i)
{
	while(globT[i].capWaiting == 1 && overflow == 0);
	globT[i].capWaiting = 1;
	if ( overflow == 1){
		globT[i].time = 0xDEAD;
	}
	
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
			globT[intrCnt].time = TCNT1 - globT[intrCnt].startTime; //endTime-startTime
	}
}
ISR(TIMER1_OVF_vect)
{
	overflow = 1;
}
//generate MIDI
void genPitchBend()
{
	pitchBend.lsb = 0;
	pitchBend.msb = 0;
}
void genVolume()
{
	prevVol = volume.val;
	volume.val = (breathValue >= PRESS_HIGH_THRESH) ? 127 : (breathValue * 127) / PRESS_HIGH_THRESH;
}
void genNoteOn()
{
	prevNote = noteOn.note;
	char touched = genTouched();
	switch (touched)
	{
		case 0x3F: noteOn.note = 62; break;
		case 0x3E: noteOn.note = 64; break;
		case 0x3C: noteOn.note = 66; break;
		case 0x38: noteOn.note = 67; break;
		case 0x30: noteOn.note = 69; break;
		case 0x20: noteOn.note = 71; break;
		case 0x00: noteOn.note = 73; break;
		case 0x1F: noteOn.note = 74; break;
		default: noteOn.note = 27; break;
	}
	noteOn.velocity = (breathValue >= PRESS_HIGH_THRESH) ? 127 : (breathValue * 127) / PRESS_HIGH_THRESH;
}
char genTouched()
{
	char retByte = 0;
	for (int i = 0; i < 6; i++)
	{
		if (globT[i].time > globT[i].calibration) {
			//retByte = retByte << 1 | 0x01;
			retByte |= 1<<i;
		}/*
		else {
			//retByte == retByte << 1;
		}*/
	}
	return retByte;
}
///////////////TRANSMISSION
void sendToCore()
{
	if (volume.val <= 7)
	{
		TransmitByte(allOff.status);
		_delay_ms(2);
		TransmitByte(allOff.control);
		_delay_ms(2);
		TransmitByte(allOff.val);
		_delay_ms(2);
	}
	else
	{
		if (prevNote != noteOn.note || prevVol == 0)
		{
			TransmitByte(allOff.status);
			_delay_ms(2);
			TransmitByte(allOff.control);
			_delay_ms(2);
			TransmitByte(allOff.val);
			_delay_ms(2);
			
			TransmitByte(noteOn.status);
			_delay_ms(2);
			TransmitByte(noteOn.note);
			_delay_ms(2);
			TransmitByte(noteOn.velocity);
			_delay_ms(2);

		}
		TransmitByte(volume.status);
		_delay_ms(2);
		TransmitByte(volume.control);
		_delay_ms(2);
		TransmitByte(volume.val);
		_delay_ms(2);
		
		TransmitByte(pitchBend.status);
		_delay_ms(2);
		TransmitByte(pitchBend.lsb);
		_delay_ms(2);
		TransmitByte(pitchBend.msb);
		_delay_ms(2);
	}
}
void testSend()
{
	int i;
	unsigned int checksum = 0;
	unsigned int transVal;
	for (i=0; i<6;i++){
		if (globT[i].time == 0xDEAD)
			transVal = 0xDEAD;
		//else if (globT[i].time > CAP_HIGH_THRESH + globT[i].calibration)
		//	transVal = 0xFF;
		else if (globT[i].time > globT[i].calibration)
			transVal = globT[i].time - globT[i].calibration;
		else
			transVal = 0;
		TransmitWord(transVal);
		checksum += transVal;
	}
	TransmitWord((unsigned int)breathValue);
	checksum += breathValue;
	TransmitWord(checksum);
}
//transmit spi
void TransmitWord( unsigned int data)
{
	TransmitByte((unsigned char)((data >> 8) & 0x00FF));
	_delay_ms(2);
	TransmitByte((unsigned char)(data & 0x00FF));
	_delay_ms(2);
}
void TransmitByte( unsigned char data )
{
	while ( !(UCSR1A & (1 << UDRE1)) );/* Wait for empty transmit buffer */
	UDR1 = data; 			           /* Start transmittion */
}
