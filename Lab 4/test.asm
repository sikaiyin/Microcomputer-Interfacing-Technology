    ORG 0000H
	LJMP MAIN
	ORG 000BH 			;定时器0的中断向量地址
	AJMP TIME0 			;跳转到真正的定时器程序处
	ORG 0050H
MAIN:
	MOV 30H,#00H 		;软件计数器预清0,计数分频为1s
	MOV 31H,#00H 		;软件计数器预清0,计的是整秒数
	MOV TMOD,#00000001B ;定时/计数器0工作于方式1
	MOV TH0,#3CH 
	MOV TL0,#0B0H	 	;预置15536，计时0.05s
	SETB EA 			;开总中断允许
	SETB ET0 			;开定时/计数器0允许
	SETB TR0 			;定时/计数器0开始运行
	MOV P2,#0FFH 		;关P2所有灯
	  SETB  P3.0 
	  SETB  P3.1 		;关P3.0和P3.1对应P22的灯
	  CPL  P2.1  
	  CPL  P2.4
	  CPL  P2.5
	  CPL  P3.0  		;根据交通灯逻辑设置道路A通行
    SJMP $ 				;原地跳转等待定时中断
	
TIME0: 					;定时器0的中断处理程序
	PUSH ACC
	PUSH PSW 			;将PSW和ACC推入堆栈保护
	INC 30H             ;30H++
	MOV A,30H
    CJNE A,#20,T_RET	;计算是否到整秒？没到则跳转再计时
	INC 31H				;到了整秒，则31H++
	MOV 30H,#00H		;31H++后清零30H，重新计数
	
	MOV A,31H
	CJNE A,#30,T_NEXT32 ;31H单元中的值到了30S了吗? 没到，跳转到T_NEXT32判断下一个数字计数器的值
    MOV P2,#0FFH 		
	  SETB  P3.0		
	  SETB  P3.1		;关所有灯
	  CPL  P2.3
	  CPL  P2.0
	  CPL  P2.6
	  CPL  P3.0			;黄灯阶段
    
T_NEXT32:
	  CJNE A,#35,T_NEXT33;31H单元中的值到了35S了吗? 没到，跳转到T_NEXT33判断下一个数字计数器的值
	  MOV P2,#0FFH 		
	  SETB  P3.0
	  SETB  P3.1   		;关所有灯
	  CPL  P2.2
	  CPL  P2.0
	  CPL  P2.7
	  CPL  P3.1			;根据交通灯逻辑设置道路A通行
	 
T_NEXT33:
	  CJNE A,#65,T_NEXT34;31H单元中的值到了65S了吗? 没到，跳转到T_NEXT34判断下一个数字计数器的值
	  MOV P2,#0FFH 		
	  SETB  P3.0
	  SETB  P3.1		;关所有灯
	  CPL  P2.3
	  CPL  P2.0
	  CPL  P2.6
	  CPL  P3.0			;黄灯阶段

T_NEXT34:
	CJNE A,#70,T_RET	;31H单元中的值到了70S了吗? 没到，跳转到T_RET重置定时
    LJMP MAIN			;到了70s,跳回MAIN程序，实现循环
T_RET:
	MOV TH0,#3CH
	MOV TL0,#0B0H 		;重置定时常数
	POP PSW
	POP ACC
	RETI				;中断返回
	END 
