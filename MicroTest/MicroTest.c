// Libraries for register names (DDRD, PORTD) and the delay function
#include <avr/io.h>
#include <util/delay.h>

// Macros to make bit manipulation easier
// Macro tutorial found here: http://www.cprogramming.com/tutorial/cpreprocessor.html
// Bit manipulation tutorial found here: http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=37871
#define set_bit(address,bit) (address |= (1<<bit)) // sets BIT to 1 in the register specified with ADDRESS
#define clear_bit(address,bit) (address &= ~(1<<bit)) // sets BIT to 0 in the register specified with ADDRESS
#define toggle_bit(address,bit) (address ^= (1<<bit)) // sets BIT to the opposite of what it's set to currently in the register specified with ADDRESS

int main(void)
{
	// The following line sets bit 5 high in register DDRD
	set_bit(DDRB,4); // Pin PB5 is now configured as an OUTPUT
	set_bit(PORTB,4); // Pin PB5 is now HIGH
	set_bit(DDRB,5); // Pin PB5 is now configured as an OUTPUT
	set_bit(PORTB,5); // Pin PB5 is now HIGH
	set_bit(DDRB,6); // Pin PB5 is now configured as an OUTPUT
	set_bit(PORTB,6); // Pin PB5 is now HIGH
	_delay_ms(1000);  // Delay for 1 second

	// The following line sets bit 5 low in register PORTD
	clear_bit(PORTB,4); // Pin PB5 is now LOW
	clear_bit(PORTB,5); // Pin PB5 is now LOW
	clear_bit(PORTB,6); // Pin PB5 is now LOW

	while(1)
	{
		_delay_ms(500);
		toggle_bit(PORTB,4); // PB5 switches from LOW to HIGH or vice versa
		
		_delay_ms(500);
		toggle_bit(PORTB,5); // PB5 switches from LOW to HIGH or vice versa
		
		_delay_ms(500);
		toggle_bit(PORTB,6); // PB5 switches from LOW to HIGH or vice versa
	}
	return 0;
}