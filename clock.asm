.include "m328PBdef.inc"

.equ	FREQ			= 8000000
.equ	TMR_FREQ		= ((FREQ) / 1)

; Take care that the longest ISR code path time will never exceed TMR_RELOAD..
; otherwise nothing spectacular will probably happen
.equ	IRQ_MAXFREQ		= 75000
.equ	TMR_RELOAD		= (256 - ((TMR_FREQ) / (IRQ_MAXFREQ)))

.equ	TICK_FREQ_APPROX	= 1024
.equ	MAX_ACTUAL_TICK_FREQ	= ((TICK_FREQ_APPROX) * 2)
.equ	ADC_FREQ		= 1024
.equ	ZC_FREQ			= 100

; This works differently from the C source, synchronize ADC to zero crossings
; so we only need to keep a 8-bit tick counter
.equ	ADC_INTERVAL		= ((TICK_FREQ_APPROX) / (ADC_FREQ))

; Has to be 2^n - 1 because it's a bitmask. A random number at most this large
; is added to TMR_RELOAD at the start of each IRQ, to spread out the IRQ
; frequency spikes in output signals.
.equ	TMR_RELOAD_RANGE	= 31

.equ	BLINK_TOGGLE_FREQ	= 3
.equ	BLINK_TICK_COUNT	= ((TICK_FREQ_APPROX) / (BLINK_TOGGLE_FREQ))

.equ	BRIGHTNESS_STEP		= 4

.equ	BTN_PRESCALER_MASK	= 0x0f	;  4 kHz
.equ	ZC_PRESCALER_MASK	= 0x01	; 32 kHz

; Polling ticks (mod 16) for buttons
.equ	BTN0_TICK		= 0x07
.equ	BTN1_TICK		= 0x0f

; Note that these use different sampling frequencies
.equ	ZC_ALPHA		= 32
.equ	BTN_ALPHA		= 16

; Thresholds for deciding that a digital IO pin appears to have gone
; low to high, or high to low
.equ	DIGITAL_LO2HI_THRES	= 200
.equ	DIGITAL_HI2LO_THRES	= 50

; All these in PORTD. ORing PORTD with PFETS_FORCE_OFF will turn off all the
; high side driver PFETs (they're active low). ANDing PORTD with PFETx_MASK
; will turn that PFET on.
.equ	PFET0_MASK		= (~0x04)
.equ	PFET1_MASK		= (~0x02)
.equ	PFET2_MASK		= (~0x08)
.equ	PFET3_MASK		= (~0x10)
.equ	COLON_MASK		= 0x01
.equ	PFETS_FORCE_OFF		= ~(PFET0_MASK & PFET1_MASK & PFET2_MASK & PFET3_MASK)

.def	brightness_mask		= r4
.def	rnd_c			= r5

; Keep status register here during ISR...
.def	sreg_cache		= r8

; And Z here
.def	z_cache_lo		= r2
.def	z_cache_hi		= r3

.def	ticks			= r9
.def	brightness_ticks	= r24

; User-writable registers. Also r0, r1 and Z are ok to use
.def	ur0			= r25
.def	ur1			= r10
.def	ur2			= r11
.def	ur3			= r12
.def	brightness_num		= r13
.def	brightness_denom	= r7
.def	blink_mask		= r14
.def	blinked_num		= r15

; Read-only for user, used by timer ISR
.def	num_lo			= r18
.def	num_hi			= r19

; Kernel scratchpad
.def	ksp0			= r20
.def	ksp1			= r21
.def	ksp2			= r22
.def	ksp3			= r23
.def	ksp4			= r16
.def	ksp5			= r17



; Params: ur0:ur1 timer to compare against target timestamp
;         @0      memory address of target timestamp variable
;         @1      escape label
;         @2      function to call if timer hass advanced past *(@0)
;         @3      event interval in timer ticks, low
;         @4      event interval in timer ticks, high
;
; Thrashes ur2 and ur3!
.macro	user_check_time_after
lds	ur3,		@0 + 1
lds	ur2,		@0 + 0

rcall	time_after
brpl	@1

	rcall	@2
	add	ur2,		@3
	adc	ur3,		@4
	sts	@0 + 1,		ur3
	sts	@0 + 0,		ur2
.endmacro

.macro	return_from_isr		; 11 cycles
	mov	ZH,		z_cache_hi
	mov	ZL,		z_cache_lo
	pop	r1
	pop	r0
	out	SREG,		sreg_cache
	reti
.endmacro

; Params:  ksp0:   old
;          ksp4:   new
;          ksp2:   alpha

; Returns: ksp1:   next confidence value

; Thrashes:ksp5, ksp2, r0, r1
.macro	exponential_filter_isr	; 13 cycles
	ser	ksp5
	sub	ksp5,		ksp2	; inv_alpha
	inc	ksp2			; alpha_p1

	mul	ksp0,		ksp5
	movw	ksp1:ksp0,	r1:r0
	mul	ksp4,		ksp2
	add	ksp0,		r0
	adc	ksp1,		r1
	sbrc	ksp0,		7	; Round, don't floor
		inc	ksp1
.endmacro

.DSEG

time:			.byte 5		; u8 * 5; has to be in first 63B
pfet_masks:		.byte 4		; u8 * 4; ditto
rnd_state:		.byte 3		; u8 * 3 - store rnd.c in register
zc_confidence:		.byte 1
btn0_confidence:	.byte 1
btn1_confidence:	.byte 1
zc_state:		.byte 1
btn0_state:		.byte 1
btn1_state:		.byte 1
zero_crossings:		.byte 2		; u16
btn0_presses:		.byte 2		; u16
btn1_presses:		.byte 2		; u16
ticks_1khz:		.byte 2		; u16

; User variables
next_adc_read:		.byte 2		; u16
next_blink_tick:	.byte 2		; u16
next_jiffies_tick:	.byte 2		; u16
next_second_tick:	.byte 2		; u16
btn0_presses_seen:	.byte 2		; u16
btn1_presses_seen:	.byte 2		; u16
zero_crossings_seen:	.byte 2		; u16
brightness:		.byte 1
brightness_setting:	.byte 1		; Only 4, 8, 12, 16, etc. Change
					; according to brightness, but with a
					; very heavy hysteresis.

brightness_shown:	.byte 1		; What brightness value we'll show

jiffies:		.byte 2		; u16 - grand total of 1kHz ticks, for
					; clock synchronization
last_1sec_jiffies:	.byte 2		; u16 - what it was at last 1sec mark
measured_local_f:	.byte 2		; u16 - local "1kHz" freq in Hz 

brite_error_accum:	.byte 2		; u16
adc_old_val:		.byte 1
adc_read_ticks:		.byte 1
brightness_num_cache:	.byte 1

.equ	KRNLPAGE	= high(SRAM_START)

.equ	rnd_a		= rnd_state + 0
.equ	rnd_b		= rnd_state + 1
.equ	rnd_x		= rnd_state + 2

.equ	hrs_hi		= time + 0
.equ	hrs_lo		= time + 1
.equ	mns_hi		= time + 2
.equ	mns_lo		= time + 3
.equ	secs		= time + 4



.CSEG

.org 0x00
cli
rjmp	start

.org TIMER0_OVFaddr
in	sreg_cache,		SREG

; Using a random number in [-64, -1], not [0, 63], to eat away spectral
; peaks
mov	ksp4,			rnd_c
ori	ksp4,			~TMR_RELOAD_RANGE
subi	ksp4,			(-TMR_RELOAD) & 0xff

out	TCNT0,			ksp4

push	r0
push	r1

mov	z_cache_lo,		ZL
mov	z_cache_hi,		ZH

update_display:

; Turn PFETs off for a couple cycles to discourage ghosting. They're slow to
; turn on<->off (like 2 cycles), the gate capacitance is like 600pF.
;
; The low side is driven by NPN transistors whose base capacitance is nothing
; compared to the high side PFETs' gates. Thus it's best to turn off the high
; side first and turn it back on last, so all the lines stop and start
; receiving current at the same time, despite being on different ports and
; thus having their low sides turned on on different clock cycles.
ldi	YL,			PFETS_FORCE_OFF
out	PORTD,			YL
clr	YL
out	PORTB,			YL

wdr
mov	YL,			ticks
inc	ticks

ldi	YH,			high(time)
andi	YL,			0x03
ldd	ZL,			Y + low(time)
ldd	ksp0,			Y + low(pfet_masks)

out	PORTB,			num_lo
out	PORTD,			num_hi

lsl	ZL
ldi	ZH,			high(nums << 1)
subi	ZL,			(-low(nums << 1)) & 0xff
sbci	ZH,			0xff

lpm	num_lo,			Z+
lpm	num_hi,			Z

lsr	YL
cp	YL,			blinked_num
brne	ud_noblink
	and	num_lo,			blink_mask
	and	num_hi,			blink_mask
	ori	num_hi,			COLON_MASK

ud_noblink:
and	num_lo,			brightness_mask
and	num_hi,			brightness_mask

ori	num_hi,			PFETS_FORCE_OFF	; All pfets off (high)...
and	num_hi,			ksp0		; save for next one

; TODO: 41 cycles + latency
read_digital_io:
mov	ksp4,			ticks
andi	ksp4,			ZC_PRESCALER_MASK
brne	digiio_no_zc_check

digiio_zc_check:
	; Translate ZC bit: 0 -> 0, 1 -> 255
	clr	ksp4
	sbic	PINE,		2
	ser	ksp4

	lds	ksp0,		zc_confidence
	ldi	ksp2,		ZC_ALPHA
	exponential_filter_isr
	sts	zc_confidence,	ksp1

	lds	ksp0,		zc_state
	ldi	YL,		low(zero_crossings)
	ldi	XL,		low(zc_state)
	rjmp	update_digital_state_isr_iret

; Assumes that BTN_PRESCALER_MASK > 3 and (BTN_PRESCALER_MASK & 3) == 3
digiio_no_zc_check:
mov	ksp4,		ticks
andi	ksp4,		BTN_PRESCALER_MASK
cpi	ksp4,		BTN0_TICK
breq	btn0_check
cpi	ksp4,		BTN1_TICK
breq	btn1_check

andi	ksp4,		3
cpi	ksp4,		1
breq	new_brightness_mask

; Has 1kHz timer advanced?
sbis	TIFR2,		0
	rjmp		exit_isr
sbi	TIFR2,		0

lds	ZL,		ticks_1khz + 0
lds	ZH,		ticks_1khz + 1
adiw	ZH:ZL,		1
sts	ticks_1khz + 0,	ZL
sts	ticks_1khz + 1,	ZH

rjmp	exit_isr

btn0_check:
	; Translate BTN0 bit: 1 -> 0, 0 -> 255
	clr	ksp4
	sbis	PINE,			0
	ser	ksp4

	lds	ksp0,			btn0_confidence
	ldi	ksp2,			BTN_ALPHA
	exponential_filter_isr
	sts	btn0_confidence,	ksp1

	lds	ksp0,			btn0_state
	ldi	YL,			low(btn0_presses)
	ldi	XL,			low(btn0_state)
	rjmp	update_digital_state_isr_iret

btn1_check:
	; Translate BTN1 bit: 1 -> 0, 0 -> 255
	clr	ksp4
	sbis	PINE,			1
	ser	ksp4

	lds	ksp0,			btn1_confidence
	; TODO 57
	ldi	ksp2,			BTN_ALPHA
	exponential_filter_isr
	sts	btn1_confidence,	ksp1

	lds	ksp0,			btn1_state
	ldi	YL,			low(btn1_presses)
	ldi	XL,			low(btn1_state)
	rjmp	update_digital_state_isr_iret

new_brightness_mask:
	dec	brightness_ticks
	brne	ticks_ok
		mov	brightness_ticks,	brightness_denom
		lds	brightness_num,		brightness_num_cache

	ticks_ok:
	clr	brightness_mask
	cp	brightness_num,		brightness_ticks
	brlo	rerandom
		dec	brightness_mask

	rerandom:
	lds	ksp0,		rnd_a
	lds	ksp1,		rnd_b
	lds	ksp2,		rnd_x

	inc	ksp2			; rnd.x++
	eor	ksp0,		rnd_c
	eor	ksp0,		ksp2	; rnd.a ^= rnd.c ^ rnd.x

	add	ksp1,		ksp0	; rnd.b += rnd.a

	mov	ksp4,		ksp1
	lsr	ksp4
	add	rnd_c,		ksp4
	eor	rnd_c,		ksp0	; rnd.c = rnd.c + (rnd.b >> 1) ^ rnd.a

	sts	rnd_a,		ksp0
	sts	rnd_b,		ksp1
	sts	rnd_x,		ksp2

exit_isr:
	return_from_isr

; Params:  ksp1: current confidence
;          ksp0: current state
;          Y:    ptr to rising edge counter
;          XL:   low(ptr) to where next state should be stored

; Returns: ksp0: next state

; Thrashes:ksp2, ksp3, ksp4, YL

; Side effects: Increments *(u8 *)Y upon L->H edge
update_digital_state_isr_iret:
	; TODO 77 thru btn1
	cpi	ksp1,		DIGITAL_LO2HI_THRES
	brsh	uds_signal_hi
	cpi	ksp1,		DIGITAL_HI2LO_THRES
	brlo	uds_signal_lo
	rjmp	uds_out
	uds_signal_hi:
		tst	ksp0
		brne	uds_out			; Was already high
			dec	ksp0
			ldd	ksp2,	Y + 0
			ldd	ksp3,	Y + 1
			inc	ksp2
			brne	uds_noinchi
				inc	ksp3
			uds_noinchi:
			std	Y + 0,	ksp2
			std	Y + 1,	ksp3

		uds_out:
		mov	YL,		XL
		st	Y,		ksp0
		return_from_isr

	uds_signal_lo:
		clr	ksp0
		rjmp	uds_out


start:
ldi	ksp4,	low (RAMEND)
out	SPL,	ksp4
ldi	ksp4,	high(RAMEND)
out	SPH,	ksp4

ldi	YH,	KRNLPAGE

; First of all, charge high side driver gates thru internal pull-up resistors.
; It'll something like 100us since the pull-ups could be as large as 60kohm,
; and C_gs something like 600pF. DDRx registers are zeroed at reset (ie. ports
; are inputs by default), we will configure port B as output later.
ldi	ksp4,	PFETS_FORCE_OFF
out	PORTD,	ksp4

zero_memory:
ldi	ZL,	low (SRAM_START)
ldi	ZH,	high(SRAM_START)
clr	ksp4
zeroloop:
	st	Z+,	ksp4
	cpi	ZH,	high(SRAM_START + SRAM_SIZE)
	brlo	zeroloop
	cpi	ZL,	low (SRAM_START + SRAM_SIZE)
	brlo	zeroloop

ldi	num_hi,		PFETS_FORCE_OFF

do_pfet_masks:
ldi	ksp4,		PFET0_MASK
sts	pfet_masks + 0,	ksp4
ldi	ksp4,		PFET1_MASK
sts	pfet_masks + 1,	ksp4
ldi	ksp4,		PFET2_MASK
sts	pfet_masks + 2,	ksp4
ldi	ksp4,		PFET3_MASK
sts	pfet_masks + 3,	ksp4

init_adc:
; Use AVCC, 8-bit ADC, single-ended Channel 0
ldi	ksp4,		0x60
sts	ADMUX,		ksp4

; Enable ADC, start conversion, use ADC prescaler 128
ldi	ksp4,		0xc7
sts	ADCSRA,		ksp4

init_io:
; Ports B and D as outputs, ports C and E as inputs, disable pullups except
; for unused pins (PC1-5, PE3).
ldi	ksp4,		0x3e
out	PORTC,		ksp4
ldi	ksp4,		0x0c ; TODO: pull up PE2 as well, was 0x08
out	PORTE,		ksp4

ser	ksp4
out	DDRB,		ksp4
out	DDRD,		ksp4

clr	ksp4
out	DDRC,		ksp4
out	DDRE,		ksp4

out	PORTB,		ksp4

; TODO: check if our power supply has shorted data lines

init_timer_irq:
; Prescaler 1, normal mode, etc
clr	ksp4
out	TCCR0A,		ksp4
ldi	ksp4,		1
out	TCCR0B,		ksp4

; Reload the timer...
ldi	ksp4,		TMR_RELOAD
out	TCNT0,		ksp4

; And enable overflow interrupt
ldi	ksp4,		1
sts	TIMSK0,		ksp4

init_1khz_timer:
; Prescaler 32 (will overflow at 1kHz), normal mode
clr	ksp4
sts	TCCR2A,		ksp4
ldi	ksp4,		3
sts	TCCR2B,		ksp4

; Reset it
clr	ksp4
sts	TCNT2,		ksp4

; Assumes that ADC_INTERVAL and ZC_FREQ fit in 8 bits and SRAM is already
; cleared
init_locals:
ldi	ksp4,			low (TICK_FREQ_APPROX)
sts	measured_local_f + 0,	ksp4
sts	next_second_tick + 0,	ksp4
ldi	ksp4,			high(TICK_FREQ_APPROX)
sts	measured_local_f + 1,	ksp4
sts	next_second_tick + 1,	ksp4

ldi	ksp4,			low (ADC_INTERVAL)
sts	next_adc_read + 0,	ksp4
ldi	ksp4,			high(ADC_INTERVAL)
sts	next_adc_read + 1,	ksp4

; Wait for the first second before actually calibrating
ldi	ksp4,			low (ZC_FREQ)
sts	zero_crossings_seen + 0,ksp4
ldi	ksp4,			high(ZC_FREQ)
sts	zero_crossings_seen + 1,ksp4

; Turn on watchdog, 16ms timer
wdr
lds	ksp4,			WDTCSR
ori	ksp4,			(1 << WDCE) | (1 << WDE)
sts	WDTCSR,			ksp4

ldi	ksp4,			(1 << WDE)
sts	WDTCSR,			ksp4

sei
mainloop:
	; NOTE: Always load high first and low then, to get magnitude right
	; if there is a race condition (wonder if these could be 8 bits tho)
	;
	; Motivation: If a race condition happens when going from eg.
	; 36ff -> 3700, we'll read 3600 for one loop cycle, and on the next
	; time we'll read 3700, so detecting the transition is only one loop
	; iteration late (all the variables should be updated much slower by
	; kernel than how they're being iterated over here). It is a much more
	; benign glitch than accidentally getting 37ff instead, because then
	; we would be 255 variable updates early.
	ml_check_adc_time:
	lds	ur1,		ticks_1khz + 1
	lds	ur0,		ticks_1khz + 0

	lds	ur3,		next_adc_read + 1
	lds	ur2,		next_adc_read + 0

	rcall	time_after
	brpl	ml_check_blink_time

		rcall	read_adc_adjust_brightness
		brcs	ml_check_blink_time
			; Increment ur2:ur3
			ldi	ZL,		low (ADC_INTERVAL)
			add	ur2,		ZL
			ldi	ZL,		high(ADC_INTERVAL)
			adc	ur3,		ZL
			sts	next_adc_read + 1,	ur3
			sts	next_adc_read + 0,	ur2

; Params: ur0:ur1 timer to compare against target timestamp
;         @0      memory address of target timestamp variable
;         @1      escape label
;         @2      function to call if timer hass advanced past *(@0)
;         @3      event interval in timer ticks
	ml_check_blink_time:
	ldi	ZL,		low (BLINK_TICK_COUNT)
	ldi	ZH,		high(BLINK_TICK_COUNT)
	user_check_time_after	next_blink_tick,	ml_check_jiffies_time, toggle_blink, ZL, ZH

	ml_check_jiffies_time:
	ldi	ZL,		1
	clr	ZH
	user_check_time_after	next_jiffies_tick,	ml_check_next_second_time, advance_jiffies, ZL, ZH

	ml_check_next_second_time:
	lds	ZL,		measured_local_f + 0
	lds	ZH,		measured_local_f + 1
	user_check_time_after	next_second_tick,	ml_check_btn0_presses,	advance_second, ZL, ZH

	ml_check_btn0_presses:
	ldi	ZL,		1
	clr	ZH
	lds	ur1,		btn0_presses + 1
	lds	ur0,		btn0_presses + 0
	user_check_time_after	btn0_presses_seen,	ml_check_btn1_presses,	advance_blink, ZL, ZH

	ml_check_btn1_presses:
	; ldi	ZL,		1
	; clr	ZH
	lds	ur1,		btn1_presses + 1
	lds	ur0,		btn1_presses + 0
	user_check_time_after	btn1_presses_seen,	ml_get_zcs,		advance_blinked_num, ZL, ZH

	ml_get_zcs:
	ldi	ZL,		100
	;clr	ZH
	lds	ur1,		zero_crossings + 1
	lds	ur0,		zero_crossings + 0
	user_check_time_after	zero_crossings_seen,	ml_continue,		advance_accurate_second, ZL, ZH

	ml_continue:
	rjmp	mainloop

; ur0:ur1: timestamp A
; ur2:ur3: timestamp B
; Sets negative flag if A is after B, clears if not
time_after:
	cp	ur2,		ur0
	cpc	ur3,		ur1
	ret

; Get down-rounded log2 of gcd(brightness, 256)
; Params: ZL:  brightness value 0 to 255
; Return: ur1: log2(gcd(brightness, 256))
gcd_256:
	push	ZL
	clr	ur1

	gcdloop:
		lsr	ZL
		brcs	gcd_256_out
		inc	ur1
		sbrs	ur1,		3	; Reached 8 yet?
		rjmp	gcdloop
gcd_256_out:
	pop	ZL
	ret

; Params: ur1: log2(gcd(n, 256))
;         ZL:  n
; Return: ZL:  n / gcd
;         ur3: 256 / gcd (256 => 0)
simplify_frac_256:
	push	ur1
	clr	ur3

	tst	ur1,			ur1
	breq	simplify_ready

	sec
	ror	ur3
	lsr	ZL

	simplify_loop:
		dec	ur1
		tst	ur1,			ur1
		breq	simplify_ready
		lsr	ZL
		lsr	ur3
		rjmp	simplify_loop

	simplify_ready:
	pop	ur1
	ret

read_adc_adjust_brightness:
	push	ur0
	push	ur1
	push	ur2
	push	ur3
	push	ZL
	push	ZH

	lds	ur0,			ADCSRA
	lds	ur1,			ADCH
	sbrc	ur0,			ADSC
		rjmp	adc_use_old_val

	; Start reading next value
	sts	adc_old_val,		ur1
	lds	ZL,			ADCSRA
	ori	ZL,			(1 << ADSC)
	sts	ADCSRA,			ZL
	rjmp	adc_continue

	adc_use_old_val:
		lds	ur1,			adc_old_val

	; ur0 is measured value
	adc_continue:
	ser	ur0
	sub	ur0,			ur1
	cpi	ur0,			0x40	; Prevent overflow
	brsh	full_brightness
		lsl	ur0
		lsl	ur0
		rjmp	nofull_brightness
	full_brightness:
		ser	ur0

	nofull_brightness:
	lds	ur2,			brightness
	clr	ur3

	; ur3:ur2 is (current - measured) clamped
	clr	ur3
	sub	ur2,			ur0
	sbc	ur3,			ur3

	lds	ZL,			brite_error_accum + 0
	lds	ZH,			brite_error_accum + 1
	add	ZL,			ur2
	adc	ZH,			ur3

	; Divide error accumulator by 1024, and correct reading if needed
	mov	ur2,			ZH
	asr	ur2
	asr	ur2

	lds	ur0,			brightness
	sub	ur0,			ur2
	sts	brightness,		ur0

	; If we correct, subtract corresponding value from accum
	mov	ur3,			ur2
	lsl	ur3
	lsl	ur3
	sub	ZH,			ur3

	sts	brite_error_accum + 0,	ZL
	sts	brite_error_accum + 1,	ZH

	; ur0: brightness, ur3: lower hysteresis value,
	; ZL: upper hysteresis value, ur2: brightness setting
	ldi	ZH,			BRIGHTNESS_STEP
	lds	ur2,			brightness_setting
	mov	ur3,			ur2
	mov	ZL,			ur3

	sub	ur3,			ZH
	brcc	no_underflow
		add	ur3,			ZH

	no_underflow:
	add	ZL,			ZH
	brcc	no_overflow
		sub	ZL,			ZH

	; Adjust down?
	no_overflow:
	cp	ur3,			ur0
	brlo	no_adjust_down
		; Don't go to zero though
		tst	ur3
			breq	setting_done
		mov	ur2,			ur3
		rjmp	setting_done

	; Adjust up?
	no_adjust_down:
	cp	ur0,			ZL
	brlo	setting_done
		mov	ur2,			ZL

	setting_done:
	sts	brightness_setting,	ur2

	; Slow down the transitions
	lds	ur0,			adc_read_ticks
	inc	ur0
	sts	adc_read_ticks,		ur0
	andi	ur0,			63
	brne	noadjust

	; ur0: Is 0 if we want to simplify the fraction (ie. if we're at the
	; target brightness currently)
	lds	ZH,			brightness_setting
	lds	ZL,			brightness_shown
	mov	ur0,			ZH
	eor	ur0,			ZL

	cp	ZL,			ZH
	brsh	dont_inc_shown
		inc	ZL
		rjmp	change_shown

	dont_inc_shown:
	cp	ZH,			ZL
	brsh	noadjust
		dec	ZL
		rjmp	change_shown

	change_shown:
	sts	brightness_shown,	ZL

	; Voila
	clr	ur3
	tst	ur0
	brne	nosimplify
		rcall	gcd_256
		rcall	simplify_frac_256

	nosimplify:
	sts	brightness_num_cache,	ZL
	mov	brightness_denom,	ur3

	noadjust:
	pop	ZH
	pop	ZL
	pop	ur3
	pop	ur2
	pop	ur1
	pop	ur0
	ret

advance_jiffies:
	push	ur0
	lds	ur0,		jiffies + 0
	inc	ur0
	sts	jiffies + 0,	ur0
	brcc	aj_no_ovf
		lds	ur0,		jiffies + 1
		inc	ur0
		sts	jiffies + 1,	ur1
	aj_no_ovf:
	pop	ur0
	ret

toggle_blink:
	com	blink_mask
	ret

advance_blink:
	push	ZL
	mov	ZL,		blinked_num
	inc	ZL
	cpi	ZL,		3
	brlo	blink_out
		clr	ZL
	blink_out:
	mov	blinked_num,	ZL
	pop	ZL
	ret

advance_blinked_num:
	push	ZL
	push	ZH
	push	ur0
	push	ur1

	; Reset blinking interval and blink mask
	; and blink mask
	ser	ZL
	mov	blink_mask,		ZL

	lds	ur1,			ticks_1khz + 1
	lds	ur0,			ticks_1khz + 0

	ldi	ZH,			high(BLINK_TICK_COUNT)
	ldi	ZL,			low (BLINK_TICK_COUNT)

	add	ur0,			ZL
	adc	ur1,			ZH

	sts	next_blink_tick + 1,	ur1
	sts	next_blink_tick + 0,	ur0

	mov	ZL,			blinked_num
	tst	ZL
	brne	abn_no_advance_hour
		rcall	advance_hour
		rjmp	abn_out

	abn_no_advance_hour:
	cpi	ZL,			1
	brne	abn_out
		clr	ZL
		sts	secs,		ZL
		rcall	advance_minute
	abn_out:
	pop	ur1
	pop	ur0
	pop	ZH
	pop	ZL
	ret

advance_second:
	push	ZL
	push	ZH
	lds	ZL,		secs
	inc	ZL
	cpi	ZL,		60
	brlo	as_no_minute
		clr	ZL
		rcall	advance_minute
	as_no_minute:
	sts	secs,		ZL

	pop	ZH
	pop	ZL
	ret

advance_minute:
	push	ZL
	lds	ZL,		mns_lo
	inc	ZL
	cpi	ZL,		10
	brsh	am_do_tens
		sts	mns_lo,		ZL
		rjmp	am_out

	am_do_tens:
		clr	ZL
		sts	mns_lo,		ZL
		lds	ZL,		mns_hi
		inc	ZL
		cpi	ZL,	6
		brsh	am_do_hour
			sts	mns_hi,		ZL
			rjmp	am_out
		am_do_hour:
			clr	ZL
			sts	mns_hi,		ZL
			rcall	advance_hour
	am_out:
	pop	ZL
	ret

advance_hour:
	push	ZL
	push	ZH

	lds	ZL,	hrs_lo
	lds	ZH,	hrs_hi
	inc	ZL
	cpi	ZL,	10
	brlo	ah_no_tens
		clr	ZL
		inc	ZH
	ah_no_tens:
		cpi	ZL,	4
		brlo	ah_out
			cpi	ZH,	2
			brlo	ah_out
				clr	ZL
				clr	ZH
	ah_out:
	sts	hrs_lo,	ZL
	sts	hrs_hi,	ZH
	pop	ZH
	pop	ZL
	ret

; The accurate clock has advanced one second. Jiffies should be up by
; measured_local_f
advance_accurate_second:
	push	ur0
	push	ur1
	push	ur2
	push	ur3

	lds	ur3,			jiffies + 1
	lds	ur2,			jiffies + 0
	lds	ur1,			last_1sec_jiffies + 1
	lds	ur0,			last_1sec_jiffies + 0

	sts	last_1sec_jiffies + 1,	ur3
	sts	last_1sec_jiffies + 0,	ur2

	; ur3:ur2 is the number of jiffies elapsed during previous second
	sub	ur2,			ur0
	sbc	ur3,			ur1

	; If we're back here after losing the clock signal, the difference
	; could be just anything. Assume we are actually increasing jiffies
	; at a frequency less than twice the intended one, and clamp
	; local timer adjustment accordingly.
	ldi	ur0,			high(MAX_ACTUAL_TICK_FREQ)
	cp	ur3,			ur0
	brlo	no_clamp_local_f
		mov	ur3,			ur0
		ldi	ur0,			low (MAX_ACTUAL_TICK_FREQ)
		cp	ur2,			ur0
		brlo	no_clamp_local_f
			mov	ur2,			ur0

	no_clamp_local_f:
	sts	measured_local_f + 1,	ur3
	sts	measured_local_f + 0,	ur2

	pop	ur3
	pop	ur2
	pop	ur1
	pop	ur0
	ret

; First byte is the PORTB part of the number, second one the PORTD part
nums:
.db 0xfe, 0xe1, 0x78, 0x01, 0xef, 0xa1, 0xfd, 0xa1, 0x39, 0xe1		; 0...4
.db 0xdd, 0xe1, 0xdf, 0xe1, 0xf9, 0x21, 0xff, 0xe1, 0xfd, 0xe1		; 5...9
