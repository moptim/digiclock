.include "m328PBdef.inc"

; TODO TODO: Align number data on a large enough boundary, so that they can
; never straddle a page boundary (they're 20 bytes, so at least 32B aligned)

.equ	FREQ			= 8000000
.equ	TMR_FREQ		= ((FREQ) / 1)
.equ	IRQ_MAXFREQ		= 64000
.equ	TMR_RELOAD		= (256 - ((TMR_FREQ) / (IRQ_MAXFREQ)))

.equ	ZC_FREQ			= 100
.equ	ADC_FREQ		= ZC_FREQ

; This works differently from the C source, synchronize ADC to zero crossings
; so we only need to keep a 8-bit tick counter
.equ	ADC_INTERVAL		= ((ZC_FREQ) / (ADC_FREQ))

; Has to be 2^n - 1 because it's a bitmask. A random number at most this large
; is added to TMR_RELOAD at the start of each IRQ, to spread out the IRQ
; frequency spikes in output signals.
.equ	TMR_RELOAD_RANGE	= 15

; TODO: were pfets in PD0...3? Was colon in PD4?
.equ	PFETS_FORCE_OFF		= 0x0f
.equ	COLON_MASK		= 0x10

.equ	PFET0_MASK		= (~0x01)
.equ	PFET1_MASK		= (~0x02)
.equ	PFET2_MASK		= (~0x04)
.equ	PFET3_MASK		= (~0x08)

; Kernel scratchpad
.def	ksp0			= r2
.def	ksp1			= r3
.def	ksp2			= r4
.def	ksp3			= r5
.def	ksp4			= r6

.def	rnd_c			= r7

; Keep status register here during ISR...
.def	sreg_cache		= r8

; And Z here. Do NOT use X for anything!
.def	z_cache_lo		= r26
.def	z_cache_hi		= r27

; User-writable registers. Also r0, r1 and Z are ok to use
.def	ur0			= r9
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
.def	brightness_mask		= r20
.def	brightness_disparity_lo = r21
.def	brightness_disparity_hi = r22
.def	tmp_3			= r23
.def	ticks			= r24


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
	sts	ur3,		@0 + 1
	sts	ur2,		@0 + 0
.endmacro


.DSEG
time:			.byte 5		; u8 * 5; has to be in first 63B
pfet_masks:		.byte 4		; u8 * 4; ditto, TODO: initialize (0xfe, 0xfd, 0xfb, 0xf7 or something)
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
jmp	start

.org TIMER0_OVFaddr
in	sreg_cache,		SREG

mov	z_cache_lo,		ZL
mov	z_cache_hi,		ZH

; TODO: either this...
ldi	tmp_lo,			TMR_RELOAD

; TODO: or this, it's only two cycles more and eats away TMR_FREQ spectral
; peaks
; Using a random number in [-16, -1] and not [0, 15]
;mov	tmp_lo,			rnd_c
;ori	tmp_lo,			~TMR_RELOAD_RANGE
;subi	tmp_lo,			-TMR_RELOAD

out	TCNT0,			tmp_lo

inc	ticks

push	r0
push	r1

update_display:
out	PORTB,			num_lo
out	PORTD,			num_hi

mov	YL,			ticks
andi	YL,			0x03
ldd	ZL,			Y + low(time)
ldd	ksp0,			Y + low(pfet_masks)

lsl	ZL
subi	ZL,			-low(nums)
ldi	ZH,			high(nums)
lpm	num_lo,			Z+
lpm	num_hi,			Z

lsr	YL
cp	YL,			blinked_num
brne	.noblink
	and	num_lo,			blink_mask
	and	num_hi,			blink_mask
	ori	num_hi,			COLON_MASK

.noblink:
and	num_lo,			brightness_mask
and	num_hi,			brightness_mask

ori	num_hi,			PFETS_FORCE_OFF	; All pfets off (high)...
and	num_hi,			ksp0		; save for next one

; TODO: 41 cycles + latency
read_digital_io:
mov	tmp_lo,			ticks
andi	tmp_lo,			ZC_PRESCALER_MASK
brne	.no_zc_check

.zc_check:
	; Translate ZC bit: 0 -> 0, 1 -> 255
	clr	tmp_lo
	sbic	PINE,		2
	ser	tmp_lo

	lds	ksp0,		zc_confidence
	rcall	exponential_filter_isr
	sts	zc_confidence,	ksp1

	lds	zc_state,	ksp0
	ldi	YL,		low(zero_crossings)
	rcall	update_digital_state_isr
	sts	zc_state,	ksp0
	rjmp	isr_return0

; Assumes that BTN_PRESCALER_MASK > 3 and (BTN_PRESCALER_MASK & 3) == 3
.no_zc_check:
mov	tmp_lo,		ticks
andi	tmp_lo,		BTN_PRESCALER_MASK
cpi	tmp_lo,		BTN0_TICK
breq	btn0_check
cpi	tmp_lo,		BTN1_TICK
breq	btn1_check

andi	tmp_lo,		3
cpi	tmp_lo,		1
breq	new_brightness_mask

isr_return0:
	mov	ZH,		z_cache_hi
	mov	ZL,		z_cache_lo
	pop	r1
	pop	r0
	out	SREG,		sreg_cache
	reti	; 128 cycles + IRQ entry latency (7 cycles? maybe 10) thru btn1

; NOTE: this is here just to fit well among other code, as no code flow can
; just end up here randomly
; Params:  ksp0:   old
;          tmp_lo: new
;          tmp_3:  alpha

; Returns: ksp1:   next confidence value

; Thrashes:tmp_hi, tmp_3, r0, r1, ksp1
exponential_filter_isr:
	ser	tmp_hi
	sub	tmp_hi,		tmp_3	; inv_alpha
	inc	tmp_3			; alpha_p1

	mul	ksp0,		tmp_hi
	movw	ksp1:ksp0,	r1:r0
	mul	tmp_lo,		tmp_3
	add	ksp0,		r0
	adc	ksp1,		r1
	; TODO 72 thru btn1
	ret

; Params:  ksp1: current confidence
;          ksp0: current state
;          Y:    ptr to rising edge counter

; Returns: ksp0: next state

; Thrashes:YL, ksp2, ksp3, tmp_lo

; Side effects: Increments *(u8 *)Y upon L->H edge
update_digital_state_isr:
	mov	tmp_lo,		ksp1
	cpi	tmp_lo,		DIGITAL_LO2HI_THRES
	brge	.signal_hi
	cpi	tmp_lo,		DIGITAL_HI2LO_THRES
	brlo	.signal_lo
	ret
	.signal_hi:
		tst	ksp0
		brne	.was_already_hi
			inc	ksp0
			ld	ksp2,	Y+
			ld	ksp3,	Y
			dec	YL
			inc	ksp2
			brne	.noinchi
				inc	ksp3
			.noinchi:
			st	Y+,	ksp2
			st	Y,	ksp3

		.was_already_hi:
		ret
	.signal_lo:
		tst	ksp0
		breq	.was_already_lo
			clr	ksp0
		.was_already_lo:
		ret

btn0_check:
	; Translate BTN0 bit: 0 -> 0, 1 -> 255
	clr	tmp_lo
	sbic	PINE,		0
	ser	tmp_lo

	lds	ksp0,		btn0_confidence
	rcall	exponential_filter_isr
	sts	btn0_confidence,	ksp1

	lds	btn0_state,	ksp0
	ldi	YL,		low(btn0_presses)
	rcall	update_digital_state_isr
	sts	btn0_state,	ksp0
	rjmp	isr_return0

btn1_check:
	; Translate BTN1 bit: 0 -> 0, 1 -> 255
	clr	tmp_lo
	sbic	PINE,		1
	ser	tmp_lo

	lds	ksp0,		btn1_confidence
	; TODO 58
	rcall	exponential_filter_isr
	sts	btn1_confidence,	ksp1

	lds	btn1_state,	ksp0
	ldi	YL,		low(btn1_presses)
	rcall	update_digital_state_isr
	; TODO 107
	sts	btn1_state,	ksp0
	rjmp	isr_return0

new_brightness_mask:
	; Use tmp_lo:brightness_disparity_hi as 16-bit correction, note though
	; that "hi" is lo and "lo" is hi
	clr	tmp_lo
	sbrc	brightness_disparity_hi,	7
	ser	tmp_lo

	; And tmp_hi:tmp_3 as 16-bit brightness, tmp_3 will be corrected
	; brightness threshold
	clr	tmp_hi
	mov	tmp_3,		brightness

	sub	tmp_3,		brightness_disparity_hi
	sbc	tmp_hi,		tmp_lo

	; If bit 7 is set in tmp_hi, we went negative - use 0 as corrected
	; threshold. If bit 0 instead is set there, we use 255.
	sbrc	tmp_hi,		7
	clr	tmp_3
	sbrc	tmp_hi,		0
	ser	tmp_3

	.rerandom:
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

	clr	brightness_mask
	cp	rnd_c,		tmp_3
	brcc	.nozero_britemask
		ser brightness_mask

	.nozero_britemask:
	clr	ksp2
	add	brightness_disparity_lo,	brightness_mask
	adc	brightness_disparity_hi,	ksp2

	sub	brightness_disparity_lo,	brightness
	sbc	brightness_disparity_lo,	ksp2

	rjmp	isr_return0


start:
ldi	SPL,	low (RAMEND)
ldi	SPH,	high(RAMEND)

ldi	YH,	KRNLPAGE

.zero_memory:
ldi	ZL,	low (SRAM_START)
ldi	ZH,	high(SRAM_START)
clr	tmp_lo
.zeroloop:
	st	Z+,	tmp_lo
	cpi	ZH,	high(SRAM_START + SRAM_SIZE)
	brlo	.zeroloop
	cpi	ZL,	low (SRAM_START + SRAM_SIZE)
	brlo	.zeroloop

.do_pfet_masks:
ldi	tmp_lo,		PFET0_MASK
sts	pfet_masks + 0,	tmp_lo
ldi	tmp_lo,		PFET1_MASK
sts	pfet_masks + 1,	tmp_lo
ldi	tmp_lo,		PFET2_MASK
sts	pfet_masks + 2,	tmp_lo
ldi	tmp_lo,		PFET3_MASK
sts	pfet_masks + 3,	tmp_lo

.init_adc:
; Use AVCC, 8-bit ADC, single-ended Channel 0
ldi	tmp_lo,		0x60
sts	ADMUX,		tmp_lo

; Enable ADC, start conversion, use ADC prescaler 128
ldi	tmp_lo,		0xc7
sts	ADCSRA,		tmp_lo

.init_io:
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

.init_timer_irq:
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


.init_locals:
; Assumes that ADC_INTERVAL and ZC_FREQ fit in 8 bits and SRAM is already
; cleared
ldi	tmp_lo,		ADC_INTERVAL
sts	next_adc_read,	tmp_lo
ldi	tmp_lo,		ZC_FREQ
sts	next_second_zc,	tmp_lo


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
	.check_adc_time:
	lds	ur1,		zero_crossings + 1
	lds	ur0,		zero_crossings + 0

	lds	ur3,		next_adc_read + 1
	lds	ur2,		next_adc_read + 0

	rcall	time_after
	brpl	.check_blink_time

		rcall	read_adc_adjust_brightness
		brcs	.check_blink_time
			; Increment ur2:ur3
			ldi	ZL,		ADC_INTERVAL
			add	ur2,		ZL
			ldi	ZL,		0
			adc	ur3,		ZL
			sts	ur3,		next_adc_read + 1
			sts	ur2,		next_adc_read + 0

; Params: ur0:ur1 timer to compare against target timestamp
;         @0      memory address of target timestamp variable
;         @1      escape label
;         @2      function to call if timer hass advanced past *(@0)
;         @3      event interval in timer ticks
	.check_blink_time:
	user_check_time_after	next_blink_zc,		.check_next_second_time, toggle_blink, BLINK_ZC_COUNT

	.check_next_second_time:
	user_check_time_after	next_second_zc,		.check_btn0_presses,	advance_second, ZC_FREQ

	.check_btn0_presses:
	lds	ur3,		btn0_presses + 1
	lds	ur2,		btn0_presses + 0
	user_check_time_after	btn0_presses_seen,	.check_btn1_presses,	advance_blink, 1

	.check_btn1_presses:
	lds	ur3,		btn1_presses + 1
	lds	ur2,		btn1_presses + 0
	user_check_time_after	btn1_presses_seen,	mainloop,		advance_blinked_num, 1

; ur0:ur1: timestamp A
; ur2:ur3: timestamp B
; Sets negative flag if A is after B, clears if not
time_after:
	push	ur2
	cp	ur3,		ur1
	sbc	ur2,		ur0
	pop	ur2
	ret

; Params:  ksp0:   old
;          tmp_lo: new
;          tmp_3:  alpha

; Returns: ksp1:   next confidence value

; Thrashes:tmp_hi, tmp_3, r0, r1, ksp1

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
		rjmp		.out_carry

	clr	ur0
	dec	ur0
	lds	ur1,		ADCL
	sub	ur0,		ur1

	lds	ur1,		raw_brightness
	ldi	ZL,		BRIGHTNESS_ALPHA
	mov	ur2,		ZL
	rcall	exponential_filter_user

	cp	ur3,		ur1
	breq	.noupdate
		sts	raw_brightness,	ur3

		; Get actual brightness value from table
		clr	ur1
		; Read corresponding entry from brightness table
		ldi	ZL,		low (brightness_table)
		ldi	ZH,		high(brightness_table)
		add	ZL,		ur3
		adc	ZH,		ur1
		lpm	brightness

	; Start reading next value
	.noupdate:
	lds	ZL,		ADCSRA
	ori	ZL,		(1 << ADSC)
	clc
	rjmp	.out

	.out_carry:
		sec

	.out:
	pop	ZH
	pop	ZL
	pop	ur3
	pop	ur2
	pop	ur1
	pop	ur0
	ret

toggle_blink:
	not	blink_mask
	ret

advance_blink:
	push	ZL
	mov	ZL,		blinked_num
	inc	ZL
	cpi	ZL,		3
	brlo	.out
		clr	ZL
	.out:
	mov	blinked_num,	ZL
	pop	ZL
	ret

advance_blinked_num:
	push	ZL
	mov	ZL,	blinked_num
	tst	ZL
	brne	.no_advance_hour
		rcall	advance_hour
		rjmp	.out

	.no_advance_hour:
	cpi	ZL,	1
	brne	.out
		clr	ZL
		sts	secs,		ZL
		rcall	advance_minute
	.out:
	pop	ZL
	ret

advance_second:
	push	ZL
	lds	ZL,		secs
	inc	ZL
	cpi	ZL,		60
	brlo	.no_minute
		clr	ZL
		rcall	advance_minute
	.no_minute:
	sts	secs,		ZL
	pop	ZL
	ret

advance_minute:
	push	ZL
	lds	ZL,		mns_lo
	inc	ZL
	cpi	ZL,		10
	brge	.do_tens
		sts	mns_lo,		ZL
		rjmp	.out

	.do_tens:
		clr	ZL
		sts	mns_lo,		ZL
		lds	ZL,		mns_hi
		inc	ZL
		cpi	ZL,	6
		brge	.do_hour
			sts	mns_hi,		ZL
			rjmp	.out
		.do_hour:
			clr	ZL
			sts	mns_hi,		ZL
			rcall	advance_hour
	.out:
	pop	ZL
	ret

advance_hour:
	push	ZL
	push	ZH

	lds	ZL,	hrs_lo
	lds	ZH,	hrs_hi
	inc	ZL
	cpi	ZL,	10
	brlo	.no_tens
		clr	ZL
		inc	ZH
	.no_tens:
		cpi	ZL,	4
		brlo	.out
			cpi	ZH,	2
			brlo	.out
				clr	ZL
				clr	ZH
	.out:
	sts	hrs_lo,	ZL
	sts	hrs_hi,	ZH
	pop	ZH
	pop	ZL
	ret
