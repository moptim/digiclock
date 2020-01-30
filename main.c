#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#define FREQ			8000000
#define TMR_FREQ		((FREQ) / 1)
#define IRQ_FREQ		64000
#define ADC_FREQ		100
#define TMR_RELOAD		(256 - ((TMR_FREQ) / (IRQ_FREQ)))

#define ADC_INTERVAL		((IRQ_FREQ) / (ADC_FREQ))

// Stolen from the Linux kernel sources. These will probably not work for
// time scales more than half a second! Assumes 16-bit timestamps.
#define TIME_AFTER(a,b)		(!!(((b) - (a)) & 0x8000))
#define TIME_BEFORE(a,b)	TIME_AFTER(b, a)

static volatile uint16_t ticks;

static uint8_t exponential_filter(uint8_t old, uint8_t new, uint8_t alpha)
{
	uint8_t  inv_alpha, alpha_p1;
	uint16_t old_multd, new_multd;
	uint16_t sum;

	// Doubt alpha + 1 is strictly correct, but it greatly helps with
	// very sluggish filters (think alpha == 3) to reach closer to 255
	// (which they won't reach because of the way small ints work).
	inv_alpha = 255 - alpha;
	alpha_p1  = alpha + 1;

	old_multd = (uint16_t)old * (uint16_t)inv_alpha;
	new_multd = (uint16_t)new * (uint16_t)alpha_p1;

	sum = old_multd + new_multd;
	return sum >> 8;
}

ISR(TIMER0_OVF_vect)
{
	update_display();
	read_digital_io();
	ticks++;
}

static void init_vars(void)
{
	ticks = 0;
}

static void init_timer_irq(void)
{
	TCCR0 = 0x01;	// Prescaler 1
	TCNT0 = TMR_RELOAD;
	TIMSK = 0x02;	// Timer0 Overflow Interrupt Enable
}

static void init_io(void)
{
}

static void init_adc(void)
{
	// Use AVCC, 8-bit ADC, single-ended Channel 0
	ADMUX  = 0x60;

	// Enable ADC, start conversion, use ADC prescaler 128
	ADCSRA = 0xc7;
}

static inline int read_adc(void)
{
	uint8_t brightness;

	// Still converting?
	if (ADCSRA & 0x40)
		return 0;
	
	brightness = 255 - ADCL;
	// TODO: update filters

	// Start reading the next value
	ADCSRA |= 0x40;
	return 1;
}

int main(void)
{
	uint16_t next_adc_read = ADC_INTERVAL;

	cli();
	init_adc();
	init_vars();
	init_timer_irq();
	init_io();
	sei();

	for (;;) {
		// TODO: 

		if (TIME_AFTER(ticks, next_adc_read)) {
			// NOTE: Here you have to filter the reading etc
			if (read_adc())
				next_adc_read = ticks + ADC_INTERVAL;
		}
	}
}
