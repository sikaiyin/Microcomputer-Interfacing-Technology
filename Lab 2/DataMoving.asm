	ORG 	0000H	    ;����Ӵ˵�ַ��ʼ����
	LJMP 	MAIN	    ;��ת�� MAIN ����

	ORG 	030H	    ;MAIN ��030H����ʼ
MAIN:
;*************Add your code below*************************************
	MOV     40H,#02
	MOV     R0,40H
	MOV     A,@R0
	MOV     DPTR,#TAB
	MOVC    A,@A+DPTR
	MOV     P0,A
	AJMP    DELAY
	
;*********************************************************************
	AJMP 	MAIN        ;��ת��������

DELAY:	
	MOV 	R5,#04H	   ;�������������Ĵ���R5
F3:	
	MOV 	R6,#0FFH
F2:	
	MOV 	R7,#0FFH
F1:	
	DJNZ 	R7,F1	       ;��Ϊ0��������ִ�У�����Ϊ0������ת��
	DJNZ 	R6,F2
	DJNZ 	R5,F3
	RET

;�߶������Թ���ʾ���ֱ���(��Ӧ0~F)
;TAB: DB 3Fh,06h,5Bh,4Fh,66h,6Dh,7Dh,07h,7Fh,6Fh,77h,7Ch,39h,5Eh,79h,71h 		;�������߶������Թ�
TAB: DB 0C0h,0F9h,0A4h,0B0h,99h,92h,82h,0F8h,80h,90h,88h,83h,0C6h,0A1h,86h,8Eh 	;�������߶������Թ�
	END
