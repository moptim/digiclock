.include "m328PBdef.inc"

.equ	FREQ			= 8000000
.equ	TMR_FREQ		= ((FREQ) / 1)

; Take care that the longest ISR code path time will never exceed TMR_RELOAD..
; otherwise nothing spectacular will probably happen
.equ	IRQ_MAXFREQ		= 75000
.equ	TMR_RELOAD		= (256 - ((TMR_FREQ) / (IRQ_MAXFREQ)))

.equ	TICK_FREQ		= 1024
.equ	ADC_FREQ		= 1024
.equ	ZC_FREQ			= 100

; This works differently from the C source, synchronize ADC to zero crossings
; so we only need to keep a 8-bit tick counter
.equ	ADC_INTERVAL		= ((TICK_FREQ) / (ADC_FREQ))

; Has to be 2^n - 1 because it's a bitmask. A random number at most this large
; is added to TMR_RELOAD at the start of each IRQ, to spread out the IRQ
; frequency spikes in output signals.
.equ	TMR_RELOAD_RANGE	= 31

.equ	BLINK_TOGGLE_FREQ	= 3
.equ	BLINK_TICK_COUNT	= ((TICK_FREQ) / (BLINK_TOGGLE_FREQ))

.equ	BTN_PRESCALER_MASK	= 0x0f	;  4 kHz
.equ	ZC_PRESCALER_MASK	= 0x01	; 32 kHz

; Polling ticks (mod 16) for buttons
.equ	BTN0_TICK		= 0x07
.equ	BTN1_TICK		= 0x0f

; Note that these use different sampling frequencies
.equ	ZC_ALPHA		= 32
.equ	BTN_ALPHA		= 16

.equ	BRIGHTNESS_ALPHA	= 16

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
;         @3      event interval in timer ticks
;
; Thrashes ZL, ur2 and ur3!
.macro	user_check_time_after
lds	ur3,		@0 + 1
lds	ur2,		@0 + 0

rcall	time_after
brpl	@1

	rcall	@2
	ldi	ZL,		low (@3)
	add	ur2,		ZL
	ldi	ZL,		high(@3)
	adc	ur3,		ZL
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
.macro	exponential_filter_isr	; 11 cycles
	ser	ksp5
	sub	ksp5,		ksp2	; inv_alpha
	inc	ksp2			; alpha_p1

	mul	ksp0,		ksp5
	movw	ksp1:ksp0,	r1:r0
	mul	ksp4,		ksp2
	add	ksp0,		r0
	adc	ksp1,		r1
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
next_second_tick:	.byte 2		; u16
btn0_presses_seen:	.byte 2		; u16
btn1_presses_seen:	.byte 2		; u16
brightness:		.byte 1
seconds:		.byte 4		; u32 - grand total of seconds, for
					; clock synchronization
brite_error_accum:	.byte 2		; u16
adc_old_val:		.byte 1

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
	inc	brightness_ticks
	cp	brightness_ticks,	brightness_denom
	brlo	ticks_ok
		clr		brightness_ticks

	ticks_ok:
	clr	brightness_mask
	cp	brightness_ticks,	brightness_num
	brsh	rerandom
		dec		brightness_mask

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

ldi	ksp4,			low (ADC_INTERVAL)
sts	next_adc_read + 0,	ksp4
ldi	ksp4,			high(ADC_INTERVAL)
sts	next_adc_read + 1,	ksp4

ldi	ksp4,			low (ZC_FREQ)
sts	next_second_tick + 0,	ksp4
ldi	ksp4,			high(ZC_FREQ)
sts	next_second_tick + 1,	ksp4

; TODO
ldi	ksp4,			0
mov	blinked_num,		ksp4

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
	user_check_time_after	next_blink_tick,	ml_check_next_second_time, toggle_blink, BLINK_TICK_COUNT

	ml_check_next_second_time:
	user_check_time_after	next_second_tick,	ml_check_btn0_presses,	advance_second, TICK_FREQ

	ml_check_btn0_presses:
	lds	ur1,		btn0_presses + 1
	lds	ur0,		btn0_presses + 0
	user_check_time_after	btn0_presses_seen,	ml_check_btn1_presses,	advance_blink, 1

	ml_check_btn1_presses:
	lds	ur1,		btn1_presses + 1
	lds	ur0,		btn1_presses + 0
	user_check_time_after	btn1_presses_seen,	ml_continue,		advance_blinked_num, 1

	ml_continue:
	;lds	ur0,	ticks_1khz + 1
	;andi	ur0,	7
	;sts	mns_lo,	ur0
	rjmp	mainloop

; ur0:ur1: timestamp A
; ur2:ur3: timestamp B
; Sets negative flag if A is after B, clears if not
time_after:
	cp	ur2,		ur0
	cpc	ur3,		ur1
	ret

; If abs(error) < threshold, return 0. Otherwise return error
; Params: ur1:     threshold
;         ur3:ur2: error
;
; Returns thresholded error in ur3:ur2
ignore_small_errors:
	push	ur2

	; Abs. This is correct, because our range is [-255, 255], and its
	; absolute value will always fit in 8 bits.
	sbrc	ur3,		7
	neg	ur2
	cp	ur2,		ur1
	pop	ur2
	brsh	nozero
		clr	ur2
		clr	ur3
	nozero:
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
		lds	ur1,			adc_old_val
	sts	adc_old_val,		ur1

	; ur0 is measured value
	ser	ur0
	sub	ur0,			ur1

	lds	ur2,			brightness
	clr	ur3

	; ur1 is desirability of current brightness setting
	ldi	ZL,			low (desirabilities << 1)
	ldi	ZH,			high(desirabilities << 1)
	add	ZL,			ur2
	adc	ZH,			ur3
	lpm	ur1,			Z

	; ur3:ur2 is (current - measured) clamped
	sub	ur2,			ur0
	sbc	ur3,			ur3
	rcall	ignore_small_errors

	lds	ZL,			brite_error_accum + 0
	lds	ZH,			brite_error_accum + 1
	add	ZL,			ur2
	adc	ZH,			ur3

	; Divide error accumulator by 4096, and correct reading if needed
	mov	ur0,			ZH
	asr	ur0
	asr	ur0
	asr	ur0
	asr	ur0

	lds	ur2,			brightness
	sub	ur2,			ur0
	sts	brightness,		ur2

	; If we correct, subtract corresponding value from accum
	mov	ur1,			ur0
	lsl	ur1
	lsl	ur1
	lsl	ur1
	lsl	ur1
	sub	ZH,			ur1

	sts	brite_error_accum + 0,	ZL
	sts	brite_error_accum + 1,	ZH

	; Get actual brightness value from table
	clr	ur1
	lsl	ur2
	rol	ur1
	; Read corresponding entry from brightness table
	ldi	ZL,			low (brightness_table << 1)
	ldi	ZH,			high(brightness_table << 1)
	add	ZL,			ur2
	adc	ZH,			ur1
	lpm	brightness_num,		Z+
	lpm	brightness_denom,	Z

	; Start reading next value and yeet outta here
	adc_noupdate:
	lds	ZL,		ADCSRA
	ori	ZL,		(1 << ADSC)
	sts	ADCSRA,		ZL

	pop	ZH
	pop	ZL
	pop	ur3
	pop	ur2
	pop	ur1
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
	mov	ZL,	blinked_num
	tst	ZL
	brne	abn_no_advance_hour
		rcall	advance_hour
		rjmp	abn_out

	abn_no_advance_hour:
	cpi	ZL,	1
	brne	abn_out
		clr	ZL
		sts	secs,		ZL
		rcall	advance_minute
	abn_out:
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

	; Increment 32-bit seconds variable
	ldi	ZH,		high(seconds)
	ldi	ZL,		low (seconds)

	clr	ur1
	ld	ur0,		Z
	inc	ur0
	st	Z+,		ur0
	ld	ur0,		Z
	adc	ur0,		ur1
	st	Z+,		ur0
	ld	ur0,		Z
	adc	ur0,		ur1
	st	Z+,		ur0
	ld	ur0,		Z
	adc	ur0,		ur1
	st	Z+,		ur0

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

; First byte is the PORTB part of the number, second one the PORTD part
nums:
.db 0xfe, 0xe1, 0x78, 0x01, 0xef, 0xa1, 0xfd, 0xa1, 0x39, 0xe1		; 0...4
.db 0xdd, 0xe1, 0xdf, 0xe1, 0xf9, 0x21, 0xff, 0xe1, 0xfd, 0xe1		; 5...9

brightness_table:
.db 0x00, 0x01, 0x01, 0xff, 0x02, 0xff, 0x01, 0x55, 0x04, 0xff, 0x01, 0x33, 0x02, 0x55, 0x07, 0xff
.db 0x08, 0xff, 0x03, 0x55, 0x02, 0x33, 0x0b, 0xff, 0x04, 0x55, 0x0d, 0xff, 0x0e, 0xff, 0x01, 0x11
.db 0x10, 0xff, 0x01, 0x0f, 0x06, 0x55, 0x13, 0xff, 0x04, 0x33, 0x07, 0x55, 0x16, 0xff, 0x17, 0xff
.db 0x08, 0x55, 0x05, 0x33, 0x1a, 0xff, 0x09, 0x55, 0x1c, 0xff, 0x1d, 0xff, 0x02, 0x11, 0x1f, 0xff
.db 0x20, 0xff, 0x0b, 0x55, 0x02, 0x0f, 0x07, 0x33, 0x0c, 0x55, 0x25, 0xff, 0x26, 0xff, 0x0d, 0x55
.db 0x08, 0x33, 0x29, 0xff, 0x0e, 0x55, 0x2b, 0xff, 0x2c, 0xff, 0x03, 0x11, 0x2e, 0xff, 0x2f, 0xff
.db 0x10, 0x55, 0x31, 0xff, 0x0a, 0x33, 0x01, 0x05, 0x34, 0xff, 0x35, 0xff, 0x12, 0x55, 0x0b, 0x33
.db 0x38, 0xff, 0x13, 0x55, 0x3a, 0xff, 0x3b, 0xff, 0x04, 0x11, 0x3d, 0xff, 0x3e, 0xff, 0x15, 0x55
.db 0x40, 0xff, 0x0d, 0x33, 0x16, 0x55, 0x43, 0xff, 0x04, 0x0f, 0x17, 0x55, 0x0e, 0x33, 0x47, 0xff
.db 0x18, 0x55, 0x49, 0xff, 0x4a, 0xff, 0x05, 0x11, 0x4c, 0xff, 0x4d, 0xff, 0x1a, 0x55, 0x4f, 0xff
.db 0x10, 0x33, 0x1b, 0x55, 0x52, 0xff, 0x53, 0xff, 0x1c, 0x55, 0x01, 0x03, 0x56, 0xff, 0x1d, 0x55
.db 0x58, 0xff, 0x59, 0xff, 0x06, 0x11, 0x5b, 0xff, 0x5c, 0xff, 0x1f, 0x55, 0x5e, 0xff, 0x13, 0x33
.db 0x20, 0x55, 0x61, 0xff, 0x62, 0xff, 0x21, 0x55, 0x14, 0x33, 0x65, 0xff, 0x02, 0x05, 0x67, 0xff
.db 0x68, 0xff, 0x07, 0x11, 0x6a, 0xff, 0x6b, 0xff, 0x24, 0x55, 0x6d, 0xff, 0x16, 0x33, 0x25, 0x55
.db 0x70, 0xff, 0x71, 0xff, 0x26, 0x55, 0x17, 0x33, 0x74, 0xff, 0x27, 0x55, 0x76, 0xff, 0x07, 0x0f
.db 0x08, 0x11, 0x79, 0xff, 0x7a, 0xff, 0x29, 0x55, 0x7c, 0xff, 0x19, 0x33, 0x2a, 0x55, 0x7f, 0xff
.db 0x80, 0xff, 0x2b, 0x55, 0x1a, 0x33, 0x83, 0xff, 0x2c, 0x55, 0x85, 0xff, 0x86, 0xff, 0x09, 0x11
.db 0x08, 0x0f, 0x89, 0xff, 0x2e, 0x55, 0x8b, 0xff, 0x1c, 0x33, 0x2f, 0x55, 0x8e, 0xff, 0x8f, 0xff
.db 0x30, 0x55, 0x1d, 0x33, 0x92, 0xff, 0x31, 0x55, 0x94, 0xff, 0x95, 0xff, 0x0a, 0x11, 0x97, 0xff
.db 0x98, 0xff, 0x03, 0x05, 0x9a, 0xff, 0x1f, 0x33, 0x34, 0x55, 0x9d, 0xff, 0x9e, 0xff, 0x35, 0x55
.db 0x20, 0x33, 0xa1, 0xff, 0x36, 0x55, 0xa3, 0xff, 0xa4, 0xff, 0x0b, 0x11, 0xa6, 0xff, 0xa7, 0xff
.db 0x38, 0x55, 0xa9, 0xff, 0x02, 0x03, 0x39, 0x55, 0xac, 0xff, 0xad, 0xff, 0x3a, 0x55, 0x23, 0x33
.db 0xb0, 0xff, 0x3b, 0x55, 0xb2, 0xff, 0xb3, 0xff, 0x0c, 0x11, 0xb5, 0xff, 0xb6, 0xff, 0x3d, 0x55
.db 0xb8, 0xff, 0x25, 0x33, 0x3e, 0x55, 0x0b, 0x0f, 0xbc, 0xff, 0x3f, 0x55, 0x26, 0x33, 0xbf, 0xff
.db 0x40, 0x55, 0xc1, 0xff, 0xc2, 0xff, 0x0d, 0x11, 0xc4, 0xff, 0xc5, 0xff, 0x42, 0x55, 0xc7, 0xff
.db 0x28, 0x33, 0x43, 0x55, 0xca, 0xff, 0xcb, 0xff, 0x04, 0x05, 0x29, 0x33, 0xce, 0xff, 0x45, 0x55
.db 0xd0, 0xff, 0xd1, 0xff, 0x0e, 0x11, 0xd3, 0xff, 0xd4, 0xff, 0x47, 0x55, 0xd6, 0xff, 0x2b, 0x33
.db 0x48, 0x55, 0xd9, 0xff, 0xda, 0xff, 0x49, 0x55, 0x2c, 0x33, 0x0d, 0x0f, 0x4a, 0x55, 0xdf, 0xff
.db 0xe0, 0xff, 0x0f, 0x11, 0xe2, 0xff, 0xe3, 0xff, 0x4c, 0x55, 0xe5, 0xff, 0x2e, 0x33, 0x4d, 0x55
.db 0xe8, 0xff, 0xe9, 0xff, 0x4e, 0x55, 0x2f, 0x33, 0xec, 0xff, 0x4f, 0x55, 0x0e, 0x0f, 0xef, 0xff
.db 0x10, 0x11, 0xf1, 0xff, 0xf2, 0xff, 0x51, 0x55, 0xf4, 0xff, 0x31, 0x33, 0x52, 0x55, 0xf7, 0xff
.db 0xf8, 0xff, 0x53, 0x55, 0x32, 0x33, 0xfb, 0xff, 0x54, 0x55, 0xfd, 0xff, 0xfe, 0xff, 0x01, 0x01

; Some brightnesses are more desirable than others, because they cycle quicker
; (1/6 cycle PWMing has less low frequency components than 33/200 for example).
; Also hand remove the huge desirability from zero.
desirabilities:
.db 0x00, 0x00, 0x00, 0x08, 0x00, 0x08, 0x08, 0x00, 0x00, 0x08, 0x08, 0x00, 0x08, 0x00, 0x00, 0x10
.db 0x00, 0x10, 0x08, 0x00, 0x08, 0x08, 0x00, 0x00, 0x08, 0x08, 0x00, 0x08, 0x00, 0x00, 0x10, 0x00
.db 0x00, 0x08, 0x10, 0x08, 0x08, 0x00, 0x00, 0x08, 0x08, 0x00, 0x08, 0x00, 0x00, 0x10, 0x00, 0x00
.db 0x08, 0x00, 0x08, 0x18, 0x00, 0x00, 0x08, 0x08, 0x00, 0x08, 0x00, 0x00, 0x10, 0x00, 0x00, 0x08
.db 0x00, 0x08, 0x08, 0x00, 0x10, 0x08, 0x08, 0x00, 0x08, 0x00, 0x00, 0x10, 0x00, 0x00, 0x08, 0x00
.db 0x08, 0x08, 0x00, 0x00, 0x08, 0x18, 0x00, 0x08, 0x00, 0x00, 0x10, 0x00, 0x00, 0x08, 0x00, 0x08
.db 0x08, 0x00, 0x00, 0x08, 0x08, 0x00, 0x18, 0x00, 0x00, 0x10, 0x00, 0x00, 0x08, 0x00, 0x08, 0x08
.db 0x00, 0x00, 0x08, 0x08, 0x00, 0x08, 0x00, 0x10, 0x10, 0x00, 0x00, 0x08, 0x00, 0x08, 0x08, 0x00
.db 0x00, 0x08, 0x08, 0x00, 0x08, 0x00, 0x00, 0x10, 0x10, 0x00, 0x08, 0x00, 0x08, 0x08, 0x00, 0x00
.db 0x08, 0x08, 0x00, 0x08, 0x00, 0x00, 0x10, 0x00, 0x00, 0x18, 0x00, 0x08, 0x08, 0x00, 0x00, 0x08
.db 0x08, 0x00, 0x08, 0x00, 0x00, 0x10, 0x00, 0x00, 0x08, 0x00, 0x18, 0x08, 0x00, 0x00, 0x08, 0x08
.db 0x00, 0x08, 0x00, 0x00, 0x10, 0x00, 0x00, 0x08, 0x00, 0x08, 0x08, 0x10, 0x00, 0x08, 0x08, 0x00
.db 0x08, 0x00, 0x00, 0x10, 0x00, 0x00, 0x08, 0x00, 0x08, 0x08, 0x00, 0x00, 0x18, 0x08, 0x00, 0x08
.db 0x00, 0x00, 0x10, 0x00, 0x00, 0x08, 0x00, 0x08, 0x08, 0x00, 0x00, 0x08, 0x08, 0x10, 0x08, 0x00
.db 0x00, 0x10, 0x00, 0x00, 0x08, 0x00, 0x08, 0x08, 0x00, 0x00, 0x08, 0x08, 0x00, 0x08, 0x10, 0x00
.db 0x10, 0x00, 0x00, 0x08, 0x00, 0x08, 0x08, 0x00, 0x00, 0x08, 0x08, 0x00, 0x08, 0x00, 0x00, 0x20
