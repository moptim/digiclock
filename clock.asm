.include "m328PBdef.inc"

.equ	FREQ			= 8000000
.equ	TMR_FREQ		= ((FREQ) / 1)

; Take care that the longest ISR code path time will never exceed TMR_RELOAD..
; otherwise nothing spectacular will probably happen
.equ	IRQ_MAXFREQ		= 75000
.equ	TMR_RELOAD		= (256 - ((TMR_FREQ) / (IRQ_MAXFREQ)))

.equ	ZC_FREQ			= 100
.equ	ADC_FREQ		= ZC_FREQ

; This works differently from the C source, synchronize ADC to zero crossings
; so we only need to keep a 8-bit tick counter
.equ	ADC_INTERVAL		= ((ZC_FREQ) / (ADC_FREQ))

; Has to be 2^n - 1 because it's a bitmask. A random number at most this large
; is added to TMR_RELOAD at the start of each IRQ, to spread out the IRQ
; frequency spikes in output signals.
.equ	TMR_RELOAD_RANGE	= 63

.equ	BLINK_TOGGLE_FREQ	= 4
.equ	BLINK_ZC_COUNT		= ((ZC_FREQ) / (BLINK_TOGGLE_FREQ))

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
.def	brightness_disparity_lo = r5
.def	brightness_disparity_hi = r6

.def	rnd_c			= r7

; Keep status register here during ISR...
.def	sreg_cache		= r8

; And Z here
.def	z_cache_lo		= r2
.def	z_cache_hi		= r3

.def	ticks			= r9

; User-writable registers. Also r0, r1 and Z are ok to use
.def	ur0			= r25
.def	ur1			= r10
.def	ur2			= r11
.def	ur3			= r12
.def	brightness		= r13
.def	blink_mask		= r14
.def	blinked_num		= r15

; Read-only for user, used by timer ISR
.def	tmp_lo			= r16
.def	tmp_hi			= r17
.def	num_lo			= r18
.def	num_hi			= r19
.def	num_hi_old_close_hsd	= r24	; Previous number but with high side
					; drivers closed

; Kernel scratchpad
.def	ksp0			= r20
.def	ksp1			= r21
.def	ksp2			= r22
.def	ksp3			= r23



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

rcall		time_after
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
	wdr
	clr	XH	; TODO TODO TODO TODO
	reti
.endmacro

; Params:  ksp0:   old
;          tmp_lo: new
;          ksp2:   alpha

; Returns: ksp1:   next confidence value

; Thrashes:tmp_hi, ksp2, r0, r1
.macro	exponential_filter_isr	; 11 cycles
	ser	tmp_hi
	sub	tmp_hi,		ksp2	; inv_alpha
	inc	ksp2			; alpha_p1

	mul	ksp0,		tmp_hi
	movw	ksp1:ksp0,	r1:r0
	mul	tmp_lo,		ksp2
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

; User variables
next_adc_read:		.byte 2		; u16
next_blink_zc:		.byte 2		; u16
next_second_zc:		.byte 2		; u16
btn0_presses_seen:	.byte 2		; u16
btn1_presses_seen:	.byte 2		; u16
raw_brightness:		.byte 1

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

; TODO: speed check
tst	XH
breq	not_late
cli
sleep

not_late:
sei
ser	XH
; /TODO

; Using a random number in [-64, -1], not [0, 63], to eat away spectral
; peaks
mov	tmp_lo,			rnd_c
ori	tmp_lo,			~TMR_RELOAD_RANGE
subi	tmp_lo,			(-TMR_RELOAD) & 0xff

out	TCNT0,			tmp_lo

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
out	PORTD,			num_hi_old_close_hsd

mov	YL,			ticks
inc	ticks

out	PORTB,			num_lo
out	PORTD,			num_hi

ldi	YH,			high(time)
andi	YL,			0x03
ldd	ZL,			Y + low(time)
ldd	ksp0,			Y + low(pfet_masks)

lsl	ZL
ldi	ZH,			high(nums << 1)
subi	ZL,			(-low(nums << 1)) & 0xff
sbci	ZH,			0xff

ori	num_hi,			PFETS_FORCE_OFF
mov	num_hi_old_close_hsd,	num_hi

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
mov	tmp_lo,			ticks
andi	tmp_lo,			ZC_PRESCALER_MASK
brne	digiio_no_zc_check

digiio_zc_check:
	; Translate ZC bit: 0 -> 0, 1 -> 255
	clr	tmp_lo
	sbic	PINE,		2
	ser	tmp_lo

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
mov	tmp_lo,		ticks
andi	tmp_lo,		BTN_PRESCALER_MASK
cpi	tmp_lo,		BTN0_TICK
breq	btn0_check
cpi	tmp_lo,		BTN1_TICK
breq	btn1_check

andi	tmp_lo,		3
cpi	tmp_lo,		1
breq	new_brightness_mask
return_from_isr

btn0_check:
	; Translate BTN0 bit: 1 -> 0, 0 -> 255
	clr	tmp_lo
	sbis	PINE,			0
	ser	tmp_lo

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
	clr	tmp_lo
	sbis	PINE,			1
	ser	tmp_lo

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
	; Use tmp_lo:brightness_disparity_hi as 16-bit correction, note though
	; that "hi" is lo and "lo" is hi
	clr	tmp_lo
	sbrc	brightness_disparity_hi,	7
	ser	tmp_lo

	; And tmp_hi:ksp3 as 16-bit brightness, ksp3 will be corrected
	; brightness threshold
	clr	tmp_hi
	mov	ksp3,		brightness

	sub	ksp3,		brightness_disparity_hi
	sbc	tmp_hi,		tmp_lo

	; If bit 7 is set in tmp_hi, we went negative - use 0 as corrected
	; threshold. If bit 0 instead is set there, we use 255.
	sbrc	tmp_hi,		7
	clr	ksp3
	sbrc	tmp_hi,		0
	ser	ksp3

	nbm_rerandom:
	lds	ksp0,		rnd_a
	lds	ksp1,		rnd_b
	lds	ksp2,		rnd_x

	inc	ksp2			; rnd.x++
	eor	ksp0,		rnd_c
	eor	ksp0,		ksp2	; rnd.a ^= rnd.c ^ rnd.x

	add	ksp1,		ksp0	; rnd.b += rnd.a

	mov	tmp_lo,		ksp1
	lsr	tmp_lo
	add	rnd_c,		tmp_lo
	eor	rnd_c,		ksp0	; rnd.c = rnd.c + (rnd.b >> 1) ^ rnd.a

	sts	rnd_a,		ksp0
	sts	rnd_b,		ksp1
	sts	rnd_x,		ksp2

	clr	brightness_mask
	cp	rnd_c,		ksp3
	brcc	nbm_nozero_britemask
		dec brightness_mask

	nbm_nozero_britemask:
	clr	ksp2
	add	brightness_disparity_lo,	brightness_mask
	adc	brightness_disparity_hi,	ksp2

	sub	brightness_disparity_lo,	brightness
	sbc	brightness_disparity_lo,	ksp2

	; 90 cycles up to here?
	return_from_isr

; Params:  ksp1: current confidence
;          ksp0: current state
;          Y:    ptr to rising edge counter
;          XL:   low(ptr) to where next state should be stored

; Returns: ksp0: next state

; Thrashes:ksp2, ksp3, tmp_lo, YL

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
ldi	tmp_lo,	low (RAMEND)
out	SPL,	tmp_lo
ldi	tmp_lo,	high(RAMEND)
out	SPH,	tmp_lo

ldi	YH,	KRNLPAGE

; First of all, charge high side driver gates thru internal pull-up resistors.
; It'll something like 100us since the pull-ups could be as large as 60kohm,
; and C_gs something like 600pF. DDRx registers are zeroed at reset (ie. ports
; are inputs by default), we will configure port B as output later.
ldi	num_hi_old_close_hsd,	PFETS_FORCE_OFF
out	PORTD,			num_hi_old_close_hsd

zero_memory:
ldi	ZL,	low (SRAM_START)
ldi	ZH,	high(SRAM_START)
clr	tmp_lo
zeroloop:
	st	Z+,	tmp_lo
	cpi	ZH,	high(SRAM_START + SRAM_SIZE)
	brlo	zeroloop
	cpi	ZL,	low (SRAM_START + SRAM_SIZE)
	brlo	zeroloop

; TODO TODO TODO TEST TEST
ldi	tmp_lo,		1
sts	hrs_hi,		tmp_lo
ldi	tmp_lo,		3
sts	hrs_lo,		tmp_lo
ldi	tmp_lo,		5
sts	mns_hi,		tmp_lo
ldi	tmp_lo,		7
sts	mns_lo,		tmp_lo

ldi	num_hi,		PFETS_FORCE_OFF

do_pfet_masks:
ldi	tmp_lo,		PFET0_MASK
sts	pfet_masks + 0,	tmp_lo
ldi	tmp_lo,		PFET1_MASK
sts	pfet_masks + 1,	tmp_lo
ldi	tmp_lo,		PFET2_MASK
sts	pfet_masks + 2,	tmp_lo
ldi	tmp_lo,		PFET3_MASK
sts	pfet_masks + 3,	tmp_lo

init_adc:
; Use AVCC, 8-bit ADC, single-ended Channel 0
ldi	tmp_lo,		0x60
sts	ADMUX,		tmp_lo

; Enable ADC, start conversion, use ADC prescaler 128
ldi	tmp_lo,		0xc7
sts	ADCSRA,		tmp_lo

init_io:
; Ports B and D as outputs, ports C and E as inputs, disable pullups
ser	tmp_lo
out	DDRB,		tmp_lo
out	DDRD,		tmp_lo

clr	tmp_lo
out	DDRC,		tmp_lo
out	DDRE,		tmp_lo
out	PORTC,		tmp_lo
out	PORTE,		tmp_lo

out	PORTB,		tmp_lo
out	PORTD,		tmp_lo

; TODO: check if our power supply has shorted data lines

init_timer_irq:
; Prescaler 1, normal mode, etc
clr	tmp_lo
out	TCCR0A,		tmp_lo
ldi	tmp_lo,		1
out	TCCR0B,		tmp_lo

; Reload the timer...
ldi	tmp_lo,		TMR_RELOAD
out	TCNT0,		tmp_lo

; And enable overflow interrupt
ldi	tmp_lo,		1
sts	TIMSK0,		tmp_lo


; Assumes that ADC_INTERVAL and ZC_FREQ fit in 8 bits and SRAM is already
; cleared
init_locals:
ldi	tmp_lo,		ADC_INTERVAL
sts	next_adc_read,	tmp_lo
ldi	tmp_lo,		ZC_FREQ
sts	next_second_zc,	tmp_lo

ldi	tmp_lo,		2
mov	blinked_num,	tmp_lo

; TODO TODO TODO
ldi	tmp_lo,		230
mov	brightness,	tmp_lo

; Turn on watchdog, 16ms timer
wdr
lds	tmp_lo,		WDTCSR
ori	tmp_lo,		(1 << WDCE) | (1 << WDE)
sts	WDTCSR,		tmp_lo

ldi	tmp_lo,		(1 << WDE)
sts	WDTCSR,		tmp_lo


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
	lds	ur1,		zero_crossings + 1
	lds	ur0,		zero_crossings + 0

	lds	ur3,		next_adc_read + 1
	lds	ur2,		next_adc_read + 0

	rcall	time_after
	brpl	ml_check_blink_time

		rcall	read_adc_adjust_brightness
		brcs	ml_check_blink_time
			; Increment ur2:ur3
			ldi	ZL,		ADC_INTERVAL
			add	ur2,		ZL
			ldi	ZL,		0
			adc	ur3,		ZL
			sts	next_adc_read + 1,	ur3
			sts	next_adc_read + 0,	ur2

; Params: ur0:ur1 timer to compare against target timestamp
;         @0      memory address of target timestamp variable
;         @1      escape label
;         @2      function to call if timer hass advanced past *(@0)
;         @3      event interval in timer ticks
	ml_check_blink_time:
	user_check_time_after	next_blink_zc,		ml_check_next_second_time, toggle_blink, BLINK_ZC_COUNT

	ml_check_next_second_time:
	user_check_time_after	next_second_zc,		ml_check_btn0_presses,	advance_second, ZC_FREQ

	ml_check_btn0_presses:
	lds	ur3,		btn0_presses + 1
	lds	ur2,		btn0_presses + 0
	user_check_time_after	btn0_presses_seen,	ml_check_btn1_presses,	advance_blink, 1

	ml_check_btn1_presses:
	lds	ur3,		btn1_presses + 1
	lds	ur2,		btn1_presses + 0
	user_check_time_after	btn1_presses_seen,	ml_continue,		advance_blinked_num, 1

	ml_continue:
	rjmp mainloop

; ur0:ur1: timestamp A
; ur2:ur3: timestamp B
; Sets negative flag if A is after B, clears if not
time_after:
	push	ur2
	cp	ur3,		ur1
	sbc	ur2,		ur0
	pop	ur2
	ret

; Params:  ur0:    old
;          ur1:    new
;          ur2:    alpha
; Returns: ur3:    next confidence value
exponential_filter_user:
	push	ZL
	push	ur0
	push	ur1
	push	ur2

	ser	ZL
	sub	ZL,		ur2
	inc	ur2

	mul	ur0,		ZL
	mov	ur0,		r0
	mov	ur3,		r1

	mul	ur1,		ur2
	add	ur0,		r0
	adc	ur3,		r1

	pop	ur2
	pop	ur1
	pop	ur0
	pop	ZL
	ret

read_adc_adjust_brightness:
	push	ur0
	push	ur1
	push	ur2
	push	ur3
	push	ZL
	push	ZH

	lds	ur0,		ADCSRA
	sbrc	ur0,		ADSC		; Still converting?
		rjmp		adc_out_carry

	clr	ur0
	dec	ur0
	lds	ur1,		ADCH
	sub	ur0,		ur1

	lds	ur1,		raw_brightness
	ldi	ZL,		BRIGHTNESS_ALPHA
	mov	ur2,		ZL
	rcall	exponential_filter_user

	cp	ur3,		ur1
	breq	adc_noupdate
		sts	raw_brightness,	ur3

		; Get actual brightness value from table
		clr	ur1
		; Read corresponding entry from brightness table
		ldi	ZL,		low (brightness_table << 1)
		ldi	ZH,		high(brightness_table << 1)
		add	ZL,		ur3
		adc	ZH,		ur1
		lpm	brightness,	Z

	; Start reading next value
	adc_noupdate:
	lds	ZL,		ADCSRA
	ori	ZL,		(1 << ADSC)
	clc
	rjmp	adc_out

	adc_out_carry:
		sec

	adc_out:
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
	lds	ZL,		secs
	inc	ZL
	cpi	ZL,		60
	brlo	as_no_minute
		clr	ZL
		rcall	advance_minute
	as_no_minute:
	sts	secs,		ZL
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
.db 0xfe, 0xe0, 0x78, 0x00, 0xef, 0xa0, 0xfd, 0xa0, 0x39, 0xe0		; 0...4
.db 0xdd, 0xe0, 0xdf, 0xe0, 0xf9, 0x20, 0xff, 0xe0, 0xfd, 0xe0		; 5...9

brightness_table:
.db 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0
.db 0xf0, 0xef, 0xee, 0xed, 0xec, 0xeb, 0xea, 0xe9, 0xe8, 0xe7, 0xe6, 0xe5, 0xe4, 0xe3, 0xe2, 0xe1
.db 0xe0, 0xdf, 0xde, 0xdd, 0xdc, 0xdb, 0xda, 0xd9, 0xd8, 0xd7, 0xd6, 0xd5, 0xd4, 0xd3, 0xd2, 0xd1
.db 0xd0, 0xcf, 0xce, 0xcd, 0xcc, 0xcb, 0xca, 0xc9, 0xc8, 0xc7, 0xc6, 0xc5, 0xc4, 0xc3, 0xc2, 0xc1
.db 0xc0, 0xbf, 0xbe, 0xbd, 0xbc, 0xbb, 0xba, 0xb9, 0xb8, 0xb7, 0xb6, 0xb5, 0xb4, 0xb3, 0xb2, 0xb1
.db 0xb0, 0xaf, 0xae, 0xad, 0xac, 0xab, 0xaa, 0xa9, 0xa8, 0xa7, 0xa6, 0xa5, 0xa4, 0xa3, 0xa2, 0xa1
.db 0xa0, 0x9f, 0x9e, 0x9d, 0x9c, 0x9b, 0x9a, 0x99, 0x98, 0x97, 0x96, 0x95, 0x94, 0x93, 0x92, 0x91
.db 0x90, 0x8f, 0x8e, 0x8d, 0x8c, 0x8b, 0x8a, 0x89, 0x88, 0x87, 0x86, 0x85, 0x84, 0x83, 0x82, 0x81
.db 0x80, 0x7f, 0x7e, 0x7d, 0x7c, 0x7b, 0x7a, 0x79, 0x78, 0x77, 0x76, 0x75, 0x74, 0x73, 0x72, 0x71
.db 0x70, 0x6f, 0x6e, 0x6d, 0x6c, 0x6b, 0x6a, 0x69, 0x68, 0x67, 0x66, 0x65, 0x64, 0x63, 0x62, 0x61
.db 0x60, 0x5f, 0x5e, 0x5d, 0x5c, 0x5b, 0x5a, 0x59, 0x58, 0x57, 0x56, 0x55, 0x54, 0x53, 0x52, 0x51
.db 0x50, 0x4f, 0x4e, 0x4d, 0x4c, 0x4b, 0x4a, 0x49, 0x48, 0x47, 0x46, 0x45, 0x44, 0x43, 0x42, 0x41
.db 0x40, 0x3f, 0x3e, 0x3d, 0x3c, 0x3b, 0x3a, 0x39, 0x38, 0x37, 0x36, 0x35, 0x34, 0x33, 0x32, 0x31
.db 0x30, 0x2f, 0x2e, 0x2d, 0x2c, 0x2b, 0x2a, 0x29, 0x28, 0x27, 0x26, 0x25, 0x24, 0x23, 0x22, 0x21
.db 0x20, 0x1f, 0x1e, 0x1d, 0x1c, 0x1b, 0x1a, 0x19, 0x18, 0x17, 0x16, 0x15, 0x14, 0x13, 0x12, 0x11
.db 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10
