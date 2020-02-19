	ORG 	0000H	    ;程序从此地址开始运行
	LJMP 	MAIN	    ;跳转到 MAIN 程序处

	ORG 	030H	    ;MAIN 从030H处开始
MAIN:
;*************Add your code below*************************************	
	MOV     P1,#0FFH
	MOV     A,P1
	CPL     A
	MOV     DPTR,#XSQR_TABLE
	MOVC    A,@A+DPTR
	MOV     DPTR,#TAB
	MOVC    A,@A+DPTR
	MOV     P0,A
	ACALL    DELAY
	  	
;*********************************************************************
	AJMP 	MAIN        ;跳转到主程序处



DELAY:	
	MOV 	R5,#02H	   ;将立即数传给寄存器R5
F3:	
	MOV 	R6,#0FFH
F2:	
	MOV 	R7,#0FFH
F1:	
	DJNZ 	R7,F1	       ;若为0程序向下执行，若不为0程序跳转到
	DJNZ 	R6,F2
	DJNZ 	R5,F3
	RET

;七段数码显管显示数字编码(对应0~F)
;TAB: DB 3Fh,06h,5Bh,4Fh,66h,6Dh,7Dh,07h,7Fh,6Fh,77h,7Ch,39h,5Eh,79h,71h 		;共阴极七段数码显管
TAB: DB 0C0h,0F9h,0A4h,0B0h,99h,92h,82h,0F8h,80h,90h,88h,83h,0C6h,0A1h,86h,8Eh 	;共阳极七段数码显管
XSQR_TABLE:
	 DB	0,1,4,9,16,25,36,49,64,81,100,121,144,169,196,225
	END
