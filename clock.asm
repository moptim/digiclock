.include "m328PBdef.inc"

; TODO TODO: Align number data on a large enough boundary, so that they can
; never straddle a page boundary (they're 20 bytes, so at least 32B aligned)

.equ	FREQ			= 8000000
.equ	TMR_FREQ		= ((FREQ) / 1)
.equ	IRQ_MAXFREQ		= 64000
.equ	ADC_FREQ		= 100
.equ	TMR_RELOAD		= (256 - ((TMR_FREQ) / (IRQ_MAXFREQ)))

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

; Keep status register here during ISR
.def	sreg_cache		= r8

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
.def	ticks_lo		= r24
.def	ticks_hi		= r25


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

# TODO: either this...
ldi	tmp_lo,			TMR_RELOAD

; TODO: or this, it's only two cycles more and eats away TMR_FREQ spectral
; peaks
# mov	tmp_lo,			rnd_c
# andi	tmp_lo,			TMR_RELOAD_RANGE
# subi	tmp_lo,			-TMR_RELOAD	; Lol, why no ADDI pseudo-op?

out	TCNT0,			tmp_lo
ldi	YH,			KRNLPAGE	; TODO: maybe move to init?

adiw	ticks_hi:ticks_lo,	1

push	r0
push	r1
push	zl
push	zh

update_display:
out	PORTB,			num_lo
out	PORTD,			num_hi

mov	YL,			ticks_lo
andi	YL,			0x03
ldd	ZL,			Y + low(time)
ldd	ksp0,			Y + low(pfet_masks)

lsl	ZL
addi	ZL,			low (nums)
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

read_digital_io:
mov	tmp_lo,			ticks_lo
andi	tmp_lo,			ZC_PRESCALER_MASK
brne	no_zc_check

zc_check:
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
no_zc_check:
mov	tmp_lo,		ticks_lo
andi	tmp_lo,		BTN_PRESCALER_MASK
cpi	tmp_lo,		BTN0_TICK
breq	btn0_check
cpi	tmp_lo,		BTN1_TICK
breq	btn1_check

andi	tmp_lo,		3
cpi	tmp_lo,		1
breq	new_brightness_mask

isr_return0:
	pop	zh
	pop	zl
	pop	r1
	pop	r0
	out	SREG,		sreg_cache
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
	rcall	exponential_filter_isr
	sts	btn1_confidence,	ksp1

	lds	btn1_state,	ksp0
	ldi	YL,		low(btn1_presses)
	rcall	update_digital_state_isr
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

	rjmp	isr_return0	; TODO: may fail if there's too many instructions in between
isr_return1:
	pop	zh
	pop	zl
	pop	r1
	pop	r0
	out	SREG,		sreg_cache
	reti

start:
ldi SPL, low (RAMEND)
ldi SPH, high(RAMEND)

.do_pfet_masks:
ldi	tmplo,		PFET0_MASK
sts	pfet_masks + 0,	tmplo
ldi	tmplo,		PFET1_MASK
sts	pfet_masks + 1,	tmplo
ldi	tmplo,		PFET2_MASK
sts	pfet_masks + 2,	tmplo
ldi	tmplo,		PFET3_MASK
sts	pfet_masks + 3,	tmplo
