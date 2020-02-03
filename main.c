#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#define FREQ			8000000
#define TMR_FREQ		((FREQ) / 1)
#define IRQ_FREQ		64000
#define ADC_FREQ		100
#define TMR_RELOAD		(256 - ((TMR_FREQ) / (IRQ_FREQ)))

// 120 for North Americans
#define ZC_FREQ			100

// For 4 kHz
#define BTN_PRESCALER_MASK	0xf
// 32 kHz
#define ZC_PRESCALER_MASK	0x1

// Note that these use different sampling frequencies
#define ZC_ALPHA		32
#define BTN_ALPHA		16

// Thresholds for deciding that a digital IO pin appears to have gone
// low to high, or high to low
#define DIGITAL_LO2HI_THRES	200
#define DIGITAL_HI2LO_THRES	50

#define ADC_INTERVAL		((IRQ_FREQ) / (ADC_FREQ))

// Stolen from the Linux kernel sources. These will probably not work for
// time scales more than half a second! Assumes 16-bit timestamps.
#define TIME_AFTER(a,b)		(!!(((b) - (a)) & 0x8000))
#define TIME_BEFORE(a,b)	TIME_AFTER(b, a)

#define PIN_TO_U8(pins,n)	(((pins) & (1 << (n))) ? 0xff : 0)

struct rnd_state {
	uint8_t a, b, c, x;
};

static uint8_t  zc_confidence, btn0_confidence, btn1_confidence;
static uint8_t  zc_state, btn0_state, btn1_state;
static uint16_t ticks;
static uint16_t zero_crossings;
static uint16_t btn0_presses;
static uint16_t btn1_presses;

// Hours high digit, hours low digit, minutes high digit, minutes low digit,
// seconds in binary
static uint8_t  time[5];

static uint8_t *const hrs_hi  = time[0];
static uint8_t *const hrs_lo  = time[1];
static uint8_t *const mins_hi = time[2];
static uint8_t *const mins_lo = time[3];
static uint8_t *const secs    = time[4];

struct rnd_state rnd;

static void advance_hour(void)
{
	if (++(*hrs_lo) >= 10) {
		*hrs_lo = 0;
		if (++(*hrs_hi) >= 6) {
			*hrs_hi = 0;
		}
	}
}

static void advance_minute(void)
{
	if (++(*mins_lo) >= 10) {
		*mins_lo = 0;
		if (++(*mins_hi) >= 6) {
			*mins_hi = 0;
			advance_hour();
		}
	}
}

static void advance_second(void)
{
	if (++(*secs) >= 60) {
		*secs = 0;
		advance_minute();
	}
}

static inline void rerandom(void)
{
	rndnum ^= rndnum << 13;
	rndnum ^= rndnum >> 17;
	rndnum ^= rndnum << 5;

	rnd.x++;
	rnd.a = (rnd.a ^ rnd.c ^ rnd.x);
	rnd.b = (rnd.b + rnd.a);
	rnd.c = (rnd.c + (rnd.b >> 1) ^ rnd.a);
}

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

static uint8_t update_digital_state(uint8_t   confidence,
                                    uint8_t   state,
                                    uint16_t *num_rising_edges)
{
	if (confidence > DIGITAL_LO2HI_THRES) {
		if (state == 0) {
			state = 1;
			(*num_rising_edges)++;
		}
	} else if (confidence < DIGITAL_HI2LO_THRES) {
		if (state == 1) {
			state = 0;
		}
	}
	return state;
}

static void read_digital_io(void)
{
	uint8_t pins;
	uint8_t zero_crossing;

	pins = PINE;
	if ((ticks & ZC_PRESCALER_MASK) == 0) {
		zero_crossing = PIN_TO_U8(pins, 2);
		zc_confidence = exponential_filter(zc_confidence,
		                                   zero_crossing,
		                                   ZC_ALPHA);

		zc_state = update_digital_state(zc_confidence,
		                                zc_state,
		                                &zero_crossings);
	}
	if ((ticks & BTN_PRESCALER_MASK) == 0) {
		uint8_t btn0, btn1;
		btn0 = PIN_TO_U8(pins, 0);
		btn1 = PIN_TO_U8(pins, 1);

		btn0_confidence = exponential_filter(btn0_confidence,
		                                     btn0,
		                                     BTN_ALPHA);
		btn1_confidence = exponential_filter(btn1_confidence,
		                                     btn1,
		                                     BTN_ALPHA);
		btn0_state = update_digital_state(btn0_confidence,
		                                  btn0_state,
		                                  &btn0_presses);
		btn1_state = update_digital_state(btn1_confidence,
		                                  btn1_state,
		                                  &btn1_presses);
	}
}

static void update_display(void)
{
	uint8_t which_num = ticks & 0x3;
	uint8_t brightness_mask = (brightness > rndnum) ? 0xff : 0;
	brightness 

ISR(TIMER0_OVF_vect)
{
	TCNT0 = TMR_RELOAD;
	update_display();
	read_digital_io();
	ticks++;
	if (ticks & 3 == 0)
		rerandom();
}

static void init_vars(void)
{
	rnd = { 1, 1, 1, 1 };
	ticks = 0;
	zc_state       = btn0_state      = btn1_state      = 0;
	zc_confidence  = btn0_confidence = btn1_confidence = 0;
	zero_crossings = btn0_presses    = btn1_presses    = 0;
	time[0] = time[1] = time[2] = time[3] = time[4] = 0;
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
	uint16_t next_adc_read  = ADC_INTERVAL;
	uint16_t next_second_zc = ZC_FREQ;

	cli();
	init_adc();
	init_vars();
	init_timer_irq();
	init_io();
	sei();

	for (;;) {
		if (TIME_AFTER(ticks, next_adc_read)) {
			// NOTE: Here you have to filter the reading etc
			if (read_adc())
				next_adc_read += ADC_INTERVAL;
		}
		if (TIME_AFTER(zero_crossings, next_second_zc)) {
			advance_second();
			next_second_zc += ZC_FREQ;
		}
		// TODO: handle buttons too
	}
}
