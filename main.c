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

#define BLINK_TOGGLE_FREQ	4
#define BLINK_ZC_COUNT		((ZC_FREQ) / (BLINK_TOGGLE_FREQ))

// For 4 kHz
#define BTN_PRESCALER_MASK	0xf
// 32 kHz
#define ZC_PRESCALER_MASK	0x1

// Which ticks (mod 16) buttons 0 and 1 are polled
#define BTN0_TICK		0x7
#define BTN1_TICK		0xf

// Note that these use different sampling frequencies
#define ZC_ALPHA		32
#define BTN_ALPHA		16

// Thresholds for deciding that a digital IO pin appears to have gone
// low to high, or high to low
#define DIGITAL_LO2HI_THRES	200
#define DIGITAL_HI2LO_THRES	50

#define ADC_INTERVAL		((IRQ_FREQ) / (ADC_FREQ))

// TODO: check which bit actually was the colon, this is to avoid ever
// blinking it. Also use it to shut down the pMOSFETs driving the high
// side, they're active low.
#define COLON_PFET_MASK		0xf8

// Stolen from the Linux kernel sources. These will probably not work for
// time scales more than half a second! Assumes 16-bit timestamps.
#define TIME_AFTER(a,b)		(!!(((b) - (a)) & 0x8000))
#define TIME_BEFORE(a,b)	TIME_AFTER(b, a)

#define PIN_TO_U8(pins,n)	(((pins) & (1 << (n))) ? 0xff : 0)

struct rnd_state {
	uint8_t a, b, c, x;
};

static const uint8_t *const num_los PROGMEM = {
	// TODO
};

static const uint8_t *const num_his PROGMEM = {
	// TODO
};

static uint8_t  blink_mask = 0;
static uint8_t  blinked_num = 0; // 0: hours, 1: mins, 2: no blink
static uint8_t  brightness = 0;
static uint8_t  zc_confidence = 0, btn0_confidence = 0, btn1_confidence = 0;
static uint8_t  zc_state = 0, btn0_state = 0, btn1_state = 0;
static uint16_t ticks = 0;
static uint16_t zero_crossings = 0;
static uint16_t btn0_presses = 0;
static uint16_t btn1_presses = 0;

static uint8_t  brightness_mask = 0;
static uint16_t brightness_disparity = 0;

// Hours high digit, hours low digit, minutes high digit, minutes low digit,
// seconds in binary
static uint8_t  time[5] = {0, 0, 0, 0, 0};

static uint8_t *const hrs_hi  = time + 0;
static uint8_t *const hrs_lo  = time + 1;
static uint8_t *const mins_hi = time + 2;
static uint8_t *const mins_lo = time + 3;
static uint8_t *const secs    = time + 4;

struct rnd_state rnd = {0, 0, 0, 0};

static void advance_hour(void)
{
	(*hrs_lo)++;
	if (*hrs_lo >= 10) {
		*hrs_lo = 0;
		(*hrs_hi)++;
	}
	if (*hrs_lo >= 4 && *hrs_hi >= 2) {
		*hrs_lo = 0;
		*hrs_hi = 0;
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
	rnd.x++;
	rnd.a = (rnd.a ^ rnd.c ^ rnd.x);
	rnd.b = (rnd.b + rnd.a);
	rnd.c = (rnd.c + (rnd.b >> 1) ^ rnd.a);
}

static uint8_t exponential_filter(uint8_t old, uint8_t new, uint8_t alpha)
{
	// No statics since this can be called from mainloop as well as from
	// the ISR
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
	static uint8_t pins;
	static uint8_t zero_crossing;
	static uint8_t btn0, btn1;

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
	if ((ticks & BTN_PRESCALER_MASK) == BTN0_TICK) {
		btn0 = PIN_TO_U8(pins, 0);
		btn0_confidence = exponential_filter(btn0_confidence,
		                                     btn0,
		                                     BTN_ALPHA);

		btn0_state = update_digital_state(btn0_confidence,
		                                  btn0_state,
		                                  &btn0_presses);
	}
	if ((ticks & BTN_PRESCALER_MASK) == BTN1_TICK) {
		btn1 = PIN_TO_U8(pins, 1);
		btn1_confidence = exponential_filter(btn1_confidence,
		                                     btn1,
		                                     BTN_ALPHA);

		btn1_state = update_digital_state(btn1_confidence,
		                                  btn1_state,
		                                  &btn1_presses);
	}
}

// Do things this way to achieve high-pass characteristic on the dither
// signal, ie. to force transitions reasonably often by keeping running
// disparity bounded, to eliminate any low-frequency fluctuation.
static void new_brightness_mask(void)
{
	// Wish there were saturating arithmetic opcodes, lol... Trying to
	// squeeze this in as few instructions as possible, that's why we're
	// handling sign bits by hand to determine overflows, who knows how
	// much the compiler actually knows what I want it to do
	static uint8_t  correction_lo = brightness_disparity >> 8;
	static uint8_t  correction_hi = (correction_lo & 0x80) ? 0xff : 0;
	static uint16_t correction    = ((uint16_t)correction_hi) << 8 |
	                                 (uint16_t)correction_lo;

	static uint16_t corrected     = (uint16_t)brightness - correction;
	static uint8_t  corrected_hi  = (uint8_t)(corrected >> 8);
	static uint8_t  corrected_lo  = (uint8_t)(corrected);
	if (corrected_hi & 0x80)
		corrected_lo = 0;
	else if (corrected_hi & 0x7f)
		corrected_lo = 255;

	rerandom();
	if (corrected_lo > rnd.c)
		brightness_mask = 0xff;
	else
		brightness_mask = 0;

	// A hacky way to update disparity, but it works, lol
	// disparity_update = (choice) ? (0xff - threshold) : (0 - threshold)
	// disparity += disparity_update
	// But the first part (0xff / 0) in the update var is the same as new
	// brightness mask, so use it there.
	brightness_disparity += brightness_mask;
	brightness_disparity -= brightness;
}

static void update_display(void)
{
	static uint8_t num_id, num, pfet_select;
	static uint8_t num_lo = 0, num_hi = COLON_PFET_MASK;

	// TODO: check how the port mappings actually went etc
	PORTB = num_lo;
	PORTD = num_hi;

	num_id = ticks & 0x3;
	num = time[num_id];

	// TODO: check how the pFETs are indexed
	pfet_select = 0x10 << num_id;

	num_lo  = pgm_read_byte(num_los + num);
	num_hi  = pgm_read_byte(num_his + num);
	num_lo &= brightness_mask;
	num_hi &= brightness_mask;

	if (blinked_num == (num_id >> 1)) {
		num_lo &= blink_mask;
		num_hi &= blink_mask;
	}
	num_hi |= COLON_PFET_MASK;
	num_hi ^= pfet_select;
}

static void init_timer_irq(void)
{
	TCCR0 = 0x01;	// Prescaler 1
	TCNT0 = TMR_RELOAD;
	TIMSK = 0x02;	// Timer0 Overflow Interrupt Enable
}

static void init_io(void)
{
	DDRB  = 0xff;
	DDRD  = 0xff;
	DDRC  = 0;
	DDRE  = 0;
	PORTC = 0;
	PORTE = 0;

	PORTB = 0;
	PORTD = 0;
}

static void init_adc(void)
{
	// Use AVCC, 8-bit ADC, single-ended Channel 0
	ADMUX  = 0x60;

	// Enable ADC, start conversion, use ADC prescaler 128
	ADCSRA = 0xc7;
}

static inline int read_adc_adjust_brightness(void)
{
	static uint8_t raw_brightness, new_brightness;

	// Still converting?
	if (ADCSRA & 0x40)
		return 0;
	
	raw_brightness = 255 - ADCL;
	// TODO: use some function here for brightness values

	new_brightness = exponential_filter(brightness,
	                                    curr_brightness,
	                                    BRIGHTNESS_ALPHA);
	if (new_brightness != brightness) {
		brightness = new_brightness;
		brightness_disparity = 0;
	}
	// Start reading the next value
	ADCSRA |= 0x40;
	return 1;
}

static inline void toggle_blink(void)
{
	blink_mask = ~blink_mask;
}

static inline void advance_blink(void)
{
	if (blinked_num++ >= 2)
		blinked_num = 0;
}

static inline void advance_blinked_num(void)
{
	if (blinked_num == 0) {
		advance_hour();
	} else if (blinked_num == 1) {
		seconds = 0;
		advance_minute();
	}
}

ISR(TIMER0_OVF_vect)
{
	TCNT0 = TMR_RELOAD;
	ticks++;

	update_display();
	read_digital_io();
	if ((ticks & 3) == 1)
		new_brightness_mask();
}

int main(void)
{
	static uint16_t next_adc_read  = ADC_INTERVAL;
	static uint16_t next_second_zc = ZC_FREQ;

	static uint16_t btn0_presses_recognized = 0;
	static uint16_t btn1_presses_recognized = 0;

	cli();
	init_adc();
	init_timer_irq();
	init_io();
	sei();

	for (;;) {
		if (TIME_AFTER(ticks, next_adc_read))
			if (read_adc_adjust_brightness())
				next_adc_read += ADC_INTERVAL;

		if (TIME_AFTER(zero_crossings, next_blink_zc)) {
			toggle_blink();
			next_blink_zc += BLINK_ZC_COUNT;
		}
		if (TIME_AFTER(zero_crossings, next_second_zc)) {
			advance_second();
			next_second_zc += ZC_FREQ;
		}
		if (TIME_AFTER(btn0_presses, btn0_presses_recognized)) {
			advance_blink();
			btn0_presses_recognized++;
		}
		if (TIME_AFTER(btn1_presses, btn1_presses_recognized)) {
			advance_blinked_num();
			btn1_presses_recognized++;
		}
	}
}
