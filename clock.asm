.include "m328PBdef.inc"

; Kernel scratchpad
.def	ksp0			= r2
.def	ksp1			= r3
.def	ksp2			= r4
.def	ksp3			= r5
.def	ksp4			= r6
.def	ksp5			= r7

; User-writable registers
.def	ur0			= r8
.def	ur1			= r9
.def	ur2			= r10
.def	ur3			= r11
.def	ur4			= r12
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
.def	ticks_lo		= r24
.def	ticks_hi		= r25

.DSEG
rnd_state:		.byte 4		; u8 * 4
zc_confidence:		.byte 1
btn0_confidence:	.byte 1
btn1_confidence:	.byte 1
zc_state:		.byte 1
btn0_state:		.byte 1
btn1_state:		.byte 1
zero_crossings:		.byte 2		; u16
btn0_presses:		.byte 2		; u16
btn1_presses:		.byte 2		; u16
time:			.byte 5		; u8 * 5

.equ	rnd_a		= rnd_state + 0
.equ	rnd_b		= rnd_state + 1
.equ	rnd_c		= rnd_state + 2
.equ	rnd_x		= rnd_state + 3

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
ldi	tmp_lo,			TMR_RELOAD
out	TCNT0,			tmp_lo

adiw	ticks_hi:ticks_lo,	1

push	r0
push	r1
push	zl
push	zh

update_display:
out	PORTB,			num_lo
out	PORTD,			num_hi

mov	tmp_lo,			ticks_lo
andi	tmp_lo,			0x03
ldi	XL,			low (time)
ldi	XH,			high(time)	; TODO: do we need this? do all irq vars fit in first 256B?
add	XL,			tmp_lo
ld	tmp_lo,			X
; TODO: tmp_lo has the number to show now, do something with it




read_digital_io:
in	tmp_lo,			PINE
mov	tmp_hi,			ticks_lo
andi	tmp_hi,			ZC_PRESCALER_MASK
brne	no_zc_check

zc_check:
	; Generate 0 or 0xff mask depending on ZC bit
	clr	tmp_lo
	sbrc	tmp_hi,		2
	ser	tmp_lo

	lds	ksp0,		zc_confidence
	rcall	exponential_filter_isr
	sts	zc_confidence,	ksp1

	lds	zc_state,	ksp0
	ldi	XL,		low(zero_crossings)
	rcall	update_digital_state_isr
	sts	zc_state,	ksp0
	rjmp	isr_return0

no_zc_check:
mov	tmp_hi,		ticks_lo
andi	tmp_hi,		BTN_PRESCALER_MASK
cpi	tmp_hi,		BTN0_TICK
breq	btn0_check
cpi	tmp_hi,		BTN1_TICK
breq	btn1_check

andi	tmp_hi,		3
cpi	tmp_hi,		1
breq	new_brightness_mask

isr_return0:
	pop	zh
	pop	zl
	pop	r1
	pop	r0
	reti

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
	ret

; Params:  ksp1: current confidence
;          ksp0: current state
;          X:    ptr to rising edge counter

; Returns: ksp0: next state

; Thrashes:XL, ksp2, ksp3
update_digital_state_isr:
	cpi	ksp1,		DIGITAL_LO2HI_THRES
	brge	.signal_hi
	cpi	ksp1,		DIGITAL_HI2LO_THRES
	brlo	.signal_lo
	ret
	.signal_hi:
		tst	ksp0
		brne	.was_already_hi
			inc	ksp0
			ld	ksp2,	X+
			ld	ksp3,	X
			dec	XL
			inc	ksp2
			brne	.noinchi
				inc	ksp3
			.noinchi:
			st	X+,	ksp2
			st	X,	ksp3

		.was_already_hi:
		ret
	.signal_lo:
		tst	ksp0
		breq	.was_already_lo
			clr	ksp0
		.was_already_lo:
		ret

btn0_check:
	; Generate 0 or 0xff mask depending on ZC bit
	clr	tmp_lo
	sbrc	tmp_hi,		2
	ser	tmp_lo

	lds	ksp0,		zc_confidence
	rcall	exponential_filter_isr
	sts	zc_confidence,	ksp1

	lds	zc_state,	ksp0
	ldi	XL,		low(zero_crossings)
	rcall	update_digital_state_isr
	sts	zc_state,	ksp0
	rjmp	isr_return0

isr_return1:
	pop	zh
	pop	zl
	pop	r1
	pop	r0
	reti

btn1_check:
	asdf_asdf
isr_return2:
	pop	zh
	pop	zl
	pop	r1
	pop	r0

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
	lds	ksp2,		rnd_c
	lds	ksp3,		rnd_x

	inc	ksp3			; rnd.x++
	eor	ksp0,		ksp2
	eor	ksp0,		ksp3	; rnd.a ^= rnd.c ^ rnd.x

	add	ksp1,		ksp0	; rnd.b += rnd.a

	mov	ksp4,		ksp1
	lsr	ksp4
	add	ksp2,		ksp4
	eor	ksp2,		ksp0	; rnd.c = rnd.c + (rnd.b >> 1) ^ rnd.a

	sts	rnd_a,		ksp0
	sts	rnd_b,		ksp1
	sts	rnd_c,		ksp2
	sts	rnd_x,		ksp3

	clr	brightness_mask
	cp	ksp2,		tmp_3
	brcc	.nozero_britemask
		ser brightness_mask
	.nozero_britemask:

	clr	ksp2
	add	brightness_disparity_lo,	brightness_mask
	adc	brightness_disparity_hi,	ksp2

	sub	brightness_disparity_lo,	brightness
	sbc	brightness_disparity_lo,	ksp2

isr_return3:
	pop	zh
	pop	zl
	pop	r1
	pop	r0




pop	zh
pop	zl
pop	r1
pop	r0
reti

start:
ldi SPL, low (RAMEND)
ldi SPH, high(RAMEND)
