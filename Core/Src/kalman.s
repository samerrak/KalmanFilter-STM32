/*
* kalman.s
*/
//.section .data

.syntax unified //as.pdf : p141
.align  16 //as.pdf :p71
.section .text, "x" //as.pdf :p96
//.rodata
.global kalman //as.pdf : p254



kalman:

	VSTMDB SP!, {s1-s6} // push temperory variables to the stack
	push {r1}
	VLDR.f32 s1, [r0] //load q into first available floating register s1
	VLDR.f32 s2, [r0, #4] //load r into floating register s2
	VLDR.f32 s3, [r0, #8] //load x into floating register s3
	VLDR.f32 s4, [r0, #12] //load p into floating register s4
	// we don't need to load k since it is changed but need to store it at [r0+16]


	VADD.f32 s4, s4, s1 // self.p = self.p + self.q
	VADD.f32 s1, s4, s2 // since q is not need we store (self.p + self.r here)
	VDIV.f32 s1, s4, s1 // since the interm value is not needed k is stored in s1


	VSUB.f32 s5, s0, s3 // measurement - self.x
	VMUL.f32 s5, s5, s1 // k is in s1 in line 13 so we do k(measurement - self.x)
	VADD.f32 s3, s3, s5 // self.x + k(measurement - self.x)


	VMOV.f32 s6, #0x3F800000 // move 1 into a temp variable
	VSUB.f32 s6, s6, s1 // (1 - k)
	VMUL.f32 s4, s4, s6 // (1-k) * p

	VMRS r1, FPSCR // load FPSCR Register

	AND r1, r1, 0xF // check for errors using mask

	CMP r1, #0 // compare it with 0

	BEQ no_err // if equal to 0 update state and return x

	MOV r1, #0

	VMSR FPSCR, r1 //reset FPSCR
	VMOV.f32 s0, #-1

	B restore // restore


no_err:

	VSTR.f32 s4, [r0, #12] //store p into where it was loaded from
	VSTR.f32 s1, [r0, #16] //store k into where it was loaded from
	VSTR.f32 s3, [r0, #8] //store x into where it was loaded from
	VMOV.f32 s0, s3 //move x into s0

restore:

	VLDMIA SP!, {s1-s6}
	pop {r1}

	BX LR
