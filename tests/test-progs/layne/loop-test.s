section		.data
        var 	DQ 	-5
        loopCnt	DQ 	20
section 	.text
        global 	_start
_start:
        mov 	rcx, 		0
loopStart:
        mov 	rdx, 		0
        add 	rdx, 		-5
	jmp 	target
	add	rdx,		10
target:
        mov	rbx,		rdx
	mov	rax,		rbx
	sub 	rdx,		5
        add 	rcx, 		1
        cmp 	rcx, 		loopCnt
        jne 	loopStart
exiting:
        mov 	ebx, 		0
        mov 	eax, 		1
        int 	80h
