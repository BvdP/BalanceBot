/*
 * BalanceBot.c
 *
 * Created: 27/05/2015 15:54:15
 *  Author: Bastiaan van der Peet
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>

#define bit_get(p,m) ((p) & (m))
#define bit_set(p,m) ((p) |= (m))
#define bit_clear(p,m) ((p) &= ~(m))
#define bit_flip(p,m) ((p) ^= (m))
#define bit_write(c,p,m) (c ? bit_set(p,m) : bit_clear(p,m))
#define BIT(x) (0x01 << (x))
#define LONGBIT(x) ((unsigned long)0x00000001 << (x))


#define CLOCK_FREQ 8000000
#define UART_BAUDRATE 38400
#define UART_TICKS_PER_BIT (CLOCK_FREQ / UART_BAUDRATE + 9) // +9: hand tuned value for 38400 bps
/*
void InitialiseUSI (void) {
	//pinMode(DataIn, INPUT);         // Define DI as input
	USICR = 0;                      // Disable USI.
	GIFR = 1<<PCIF;                 // Clear pin change interrupt flag.
	GIMSK |= 1<<PCIE1;               // Enable pin change interrupts
	PCMSK1 |= 1<<PCINT0;             // Enable pin change on pin 0
}


// some interrupt handlers for input UART
ISR (PCINT_vect) {	// pin change
	if (!(PINB & 1<<PINB0)) {       // Ignore if DI is high
		GIMSK &= ~(1<<PCIE1);          // Disable pin change interrupts
		TCCR0A = 2<<WGM00;            // CTC mode
		TCCR0B = 0<<WGM02 | 2<<CS00;  // Set prescaler to /8
		TCNT0 = 0;                    // Count up from 0
		OCR0A = 51;                   // Delay (51+1)*8 cycles
		TIFR |= 1<<OCF0A;             // Clear output compare flag
		TIMSK |= 1<<OCIE0A;           // Enable output compare interrupt
	}
}
	
ISR (TIMER0_COMPA_vect) {
	TIMSK &= ~(1<<OCIE0A);          // Disable COMPA interrupt
	TCNT0 = 0;                      // Count up from 0
	OCR0A = 103;                    // Shift every (103+1)*8 cycles
	// Enable USI OVF interrupt, and select Timer0 compare match as USI Clock source:
	USICR = 1<<USIOIE | 0<<USIWM0 | 1<<USICS0;
	USISR = 1<<USIOIF | 8;          // Clear USI OVF flag, and set counter
}
	
ISR (USI_OVF_vect) {	// USI overflow
	USICR = 0;                      // Disable USI
	int temp = USIDR;
	Display(ReverseByte(temp));
	GIFR = 1<<PCIF;                 // Clear pin change interrupt flag.
	GIMSK |= 1<<PCIE1;               // Enable pin change interrupts again
}
*/
unsigned char ReverseByte (unsigned char x) {
	x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
	x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
	x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
	return x;
}



#define UART_TX PORTB0
#define UART_RX PORTB1

//#define COUNT_BITS 0b111
#define START_BIT 1<<0
#define STOP_BIT 1<<1
#define RUN_BIT 1<<2
#define PRECALC_BIT 1<<3

#define UART_BUFFER_SIZE 16

char uart_status = PRECALC_BIT;
volatile char uart_buffer[UART_BUFFER_SIZE];
volatile uint8_t uart_start_idx, uart_buffer_fill;

void precalculate_uart_bit() {
	static uint8_t uart_bit_mask;
	if (uart_status & START_BIT) {
		bit_clear(uart_status, START_BIT | PRECALC_BIT);
		uart_bit_mask = 1;
	}
	else if (uart_status & STOP_BIT) {
		bit_set(uart_status, PRECALC_BIT);
		bit_clear(uart_status, STOP_BIT);
		if (uart_buffer_fill == 0) {
			bit_clear(uart_status, RUN_BIT);
		}
		else {
			uart_buffer_fill--;
			uart_start_idx = (uart_start_idx + 1) % UART_BUFFER_SIZE;
			bit_set(uart_status, START_BIT);
		}
	}
	else {  // data bits
		bit_write(uart_buffer[uart_start_idx] & uart_bit_mask , uart_status, PRECALC_BIT);
		uart_bit_mask <<= 1;
		if(!uart_bit_mask)
		bit_set(uart_status, STOP_BIT);
	}
}


ISR(TIMER0_COMPA_vect){
		OCR0A += UART_TICKS_PER_BIT;
		bit_write(uart_status & PRECALC_BIT, PORTB, 1<<UART_TX);
		if (uart_status & RUN_BIT) {
			sei();
			precalculate_uart_bit();
		}
}

uint8_t putstring(char string[]) {
	int i=0;
	while (string[i] != '\0' && i < UART_BUFFER_SIZE - uart_buffer_fill - 2) {
		uart_buffer[(uart_start_idx + uart_buffer_fill + i) % UART_BUFFER_SIZE] = string[i];
		i++;
	}
	uart_buffer[(uart_start_idx + uart_buffer_fill + i) % UART_BUFFER_SIZE] = '\r';
	i++;
	uart_buffer[(uart_start_idx + uart_buffer_fill + i) % UART_BUFFER_SIZE] = '\n';
	uart_buffer_fill += i;
	bit_set(uart_status, RUN_BIT | START_BIT);
	return i;
}

void uart_setup() {
	// set timer0 clock source
	bit_set(TCCR0B, 1<<CS00);// no prescaling; 8MHz clock

	// configure pins
	bit_set(PORTB, BIT(UART_TX)); // high when idle
	bit_set(DDRB, 1<<UART_TX);
	//bit_clear(DDRB, 1<<UART_RX);

	// setup timer0 compare
	OCR0A = UART_TICKS_PER_BIT;
	bit_clear(TIFR, 1<<OCF0A);	// clear interrupt bit
	bit_set(TIMSK, 1<<OCIE0A);	// enable compare
}

int main(void)
{
	// configuration
	/*	A0 (20): MPU SDA  --> input
		A1 (19): FCSB	  --> analog input
		A2 (18): MPU SCL  --> output
		A3 (17): NC
		A4 (14): MD IB2  --> output
		A5 (13): MD IB1  --> output
		A6 (12): MD IA2  --> output
		A7 (11): MD IA1  --> output
		
		B0  (1): UART TX / ICP MOSI --> output (should be high when idle)
		B1  (2): UART RX / ICP MISO --> input
		B2  (3): ICP SCK
		B3  (4): MD ENB  --> output
		B4  (7): FCSA	 --> analog input
		B5  (8): MD ENA  --> output
		B6  (9): MPU INT --> input
		B7 (10): /RESET
		
	*/
	uart_setup();
	
	// set up IO pins
	DDRA = 0b11110100;
	DDRB = 0b00101001;

	putstring("Booting");
	sei(); // enable interrupts
	//setupUartOut();
	
	
    while(1) // main loop
    {
       
    }
}