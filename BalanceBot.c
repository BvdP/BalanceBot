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

#define UART_TX PORTB0
#define UART_RX PORTB1

//define some uart_status bit masks
#define OUT_START (1<<0)
#define OUT_STOP (1<<1)
#define OUT_RUN (1<<2)
#define OUT_PRECALC (1<<3)
#define IN_START (1<<4)
#define IN_READY (1<<5)

#define UART_BUFFER_SIZE 16

uint8_t putstring(char string[]);

volatile char uart_status = OUT_PRECALC;
volatile char uart_out_buffer[UART_BUFFER_SIZE];
volatile uint8_t uart_out_start_idx = 0, uart_out_buffer_fill = 0;
volatile char uart_in_buffer[UART_BUFFER_SIZE];
volatile uint8_t uart_in_start_idx = 0, uart_in_buffer_fill = 0;

void precalculate_uart_bit() {
	static uint8_t uart_bit_mask;
	if (uart_status & OUT_START) {
		bit_clear(uart_status, OUT_START | OUT_PRECALC);
		uart_bit_mask = 1;
	}
	else if (uart_status & OUT_STOP) {
		bit_set(uart_status, OUT_PRECALC);
		bit_clear(uart_status, OUT_STOP);
		if (uart_out_buffer_fill == 0) {
			bit_clear(uart_status, OUT_RUN);
		}
		else {
			uart_out_buffer_fill--;
			uart_out_start_idx = (uart_out_start_idx + 1) % UART_BUFFER_SIZE;
			bit_set(uart_status, OUT_START);
		}
	}
	else {  // data bits
		bit_write(uart_out_buffer[uart_out_start_idx] & uart_bit_mask , uart_status, OUT_PRECALC);
		uart_bit_mask <<= 1;
		if(!uart_bit_mask) {
			bit_set(uart_status, OUT_STOP);
		}
	}
}


// some interrupt handlers for input UART
// working in a nutshell:
// high->low transition: this is the start bit; triggers interrupt
// timer samples 8 databits at half- clock offset ()
// stop bit is not sampled

ISR (PCINT_vect) {	// pin change
	if (!(PINB & 1<<UART_RX)) {			// we're interested in the falling edge
		bit_clear(PCMSK1, 1<<PCINT9);	// disable pin change interrupt on RX pin
		OCR0B = TCNT0L + UART_TICKS_PER_BIT / 3 ;// delay 1/3 of a uart clock tick
		bit_set(TIFR, 1<<OCF0B);		// clear timer compare flag
		bit_set(uart_status, IN_START);
		bit_set(TIMSK, 1<<OCIE0B);		// enable timer0 compareB interrupt
	}
}

// timer interrupt for UART input
ISR (TIMER0_COMPB_vect) {
	static uint8_t bit_mask;
	OCR0B += UART_TICKS_PER_BIT;
	if(uart_status & IN_START) {	// start bit
			bit_clear(uart_status, IN_START);
			bit_mask = 1;
			return;
	}
	if (bit_mask == 0) {	// bit mask wrapped around --> stop bit
		//bit_clear(uart_status, IN_STOP);
		bit_clear(TIMSK, 1<<OCIE0B);	// disable timer0 compareB
		bit_set(PCMSK1, 1<<PCINT9);	// Enable pin change interrupt on RX pin
		if (uart_in_buffer_fill > 0 && uart_in_buffer[uart_in_buffer_fill] == '\n') {	// strings are \r\n terminated
			uart_in_buffer[uart_in_buffer_fill - 1] = '\0';
			bit_set(uart_status, IN_READY);
		}
		uart_in_buffer_fill++;
		return;
	}
	bit_write(PINB & (1 << UART_RX), uart_in_buffer[uart_in_buffer_fill % UART_BUFFER_SIZE], bit_mask);
	bit_mask <<= 1;
}

// timer interrupt for UART output
ISR(TIMER0_COMPA_vect){
		OCR0A += UART_TICKS_PER_BIT;
		bit_write(uart_status & OUT_PRECALC, PORTB, 1<<UART_TX);
		if (uart_status & OUT_RUN) {
			sei();
			precalculate_uart_bit();
		}
}

uint8_t putstring(char* string) {
	int i=0;
	while (string[i] != '\0' && i < UART_BUFFER_SIZE - uart_out_buffer_fill - 2) {
		uart_out_buffer[(uart_out_start_idx + uart_out_buffer_fill + i) % UART_BUFFER_SIZE] = string[i];
		i++;
	}
	uart_out_buffer[(uart_out_start_idx + uart_out_buffer_fill + i) % UART_BUFFER_SIZE] = '\r';
	i++;
	uart_out_buffer[(uart_out_start_idx + uart_out_buffer_fill + i) % UART_BUFFER_SIZE] = '\n';
	uart_out_buffer_fill += i;
	bit_set(uart_status, OUT_RUN | OUT_START);
	return i;
}

void getstring(char* str) {
	if (uart_status & IN_READY) {
		for (int i=0; i < UART_BUFFER_SIZE; i++) {//TODO: add \0 check
			str[i] = uart_in_buffer[i];
		}
		bit_clear(uart_status, IN_READY);
		uart_in_buffer_fill = 0;
	}
}

void uart_setup() {
	// set timer0 clock source
	bit_set(TCCR0B, 1<<CS00);// no prescaling; 8MHz clock

	// configure pins
	bit_set(PORTB, BIT(UART_TX)); // high when idle
	bit_set(DDRB, 1<<UART_TX);	// TX is output pin
	//bit_clear(DDRB, 1<<UART_RX);

	// setup timer0 compare
	OCR0A = UART_TICKS_PER_BIT;
	bit_clear(TIFR, 1<<OCF0A);	// clear interrupt bit
	bit_set(TIMSK, 1<<OCIE0A);	// enable compare

	// input setup
	GIFR = 1<<PCIF;					// Clear pin change interrupt flag.
	bit_set(GIMSK, 1<<PCIE0);		// Enable pin change interrupts
	PCMSK1 = 1<<PCINT9;				// Enable pin change interrupt on RX pin
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
		B1  (2): UART RX / ICP MISO --> input (pcint9)
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

	sei(); // enable interrupts

	while(1) // main loop
	{
		if (uart_status & IN_READY) {
			char tmp[UART_BUFFER_SIZE];
			getstring(tmp);
			if (tmp[0] == 'B') {
				bit_flip(PORTA, 1<<3); // toggle NC pin
			}
			putstring(tmp);
		}
	}
}
