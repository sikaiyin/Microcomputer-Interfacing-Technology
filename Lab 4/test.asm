    ORG 0000H
	LJMP MAIN
	ORG 000BH 			;��ʱ��0���ж�������ַ
	AJMP TIME0 			;��ת�������Ķ�ʱ������
	ORG 0050H
MAIN:
	MOV 30H,#00H 		;���������Ԥ��0,������ƵΪ1s
	MOV 31H,#00H 		;���������Ԥ��0,�Ƶ���������
	MOV TMOD,#00000001B ;��ʱ/������0�����ڷ�ʽ1
	MOV TH0,#3CH 
	MOV TL0,#0B0H	 	;Ԥ��15536����ʱ0.05s
	SETB EA 			;�����ж�����
	SETB ET0 			;����ʱ/������0����
	SETB TR0 			;��ʱ/������0��ʼ����
	MOV P2,#0FFH 		;��P2���е�
	  SETB  P3.0 
	  SETB  P3.1 		;��P3.0��P3.1��ӦP22�ĵ�
	  CPL  P2.1  
	  CPL  P2.4
	  CPL  P2.5
	  CPL  P3.0  		;���ݽ�ͨ���߼����õ�·Aͨ��
    SJMP $ 				;ԭ����ת�ȴ���ʱ�ж�
	
TIME0: 					;��ʱ��0���жϴ������
	PUSH ACC
	PUSH PSW 			;��PSW��ACC�����ջ����
	INC 30H             ;30H++
	MOV A,30H
    CJNE A,#20,T_RET	;�����Ƿ����룿û������ת�ټ�ʱ
	INC 31H				;�������룬��31H++
	MOV 30H,#00H		;31H++������30H�����¼���
	
	MOV A,31H
	CJNE A,#30,T_NEXT32 ;31H��Ԫ�е�ֵ����30S����? û������ת��T_NEXT32�ж���һ�����ּ�������ֵ
    MOV P2,#0FFH 		
	  SETB  P3.0		
	  SETB  P3.1		;�����е�
	  CPL  P2.3
	  CPL  P2.0
	  CPL  P2.6
	  CPL  P3.0			;�Ƶƽ׶�
    
T_NEXT32:
	  CJNE A,#35,T_NEXT33;31H��Ԫ�е�ֵ����35S����? û������ת��T_NEXT33�ж���һ�����ּ�������ֵ
	  MOV P2,#0FFH 		
	  SETB  P3.0
	  SETB  P3.1   		;�����е�
	  CPL  P2.2
	  CPL  P2.0
	  CPL  P2.7
	  CPL  P3.1			;���ݽ�ͨ���߼����õ�·Aͨ��
	 
T_NEXT33:
	  CJNE A,#65,T_NEXT34;31H��Ԫ�е�ֵ����65S����? û������ת��T_NEXT34�ж���һ�����ּ�������ֵ
	  MOV P2,#0FFH 		
	  SETB  P3.0
	  SETB  P3.1		;�����е�
	  CPL  P2.3
	  CPL  P2.0
	  CPL  P2.6
	  CPL  P3.0			;�Ƶƽ׶�

T_NEXT34:
	CJNE A,#70,T_RET	;31H��Ԫ�е�ֵ����70S����? û������ת��T_RET���ö�ʱ
    LJMP MAIN			;����70s,����MAIN����ʵ��ѭ��
T_RET:
	MOV TH0,#3CH
	MOV TL0,#0B0H 		;���ö�ʱ����
	POP PSW
	POP ACC
	RETI				;�жϷ���
	END 
