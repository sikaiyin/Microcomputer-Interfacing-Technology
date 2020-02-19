#line 1 "..\\HARDWARE\\MOTOR\\motor.c"
#line 1 "..\\HARDWARE\\MOTOR\\motor.h"
#line 1 "..\\SYSTEM\\sys\\sys.h"
#line 1 "..\\SYSTEM\\sys\\stm32f10x.h"







































 



 



 
    






  


 
  


 

#line 75 "..\\SYSTEM\\sys\\stm32f10x.h"


















 










 
   








            
#line 122 "..\\SYSTEM\\sys\\stm32f10x.h"





 






 
#line 143 "..\\SYSTEM\\sys\\stm32f10x.h"



 



 



 
#line 162 "..\\SYSTEM\\sys\\stm32f10x.h"




 
typedef enum IRQn
{
 
  NonMaskableInt_IRQn         = -14,     
  MemoryManagement_IRQn       = -12,     
  BusFault_IRQn               = -11,     
  UsageFault_IRQn             = -10,     
  SVCall_IRQn                 = -5,      
  DebugMonitor_IRQn           = -4,      
  PendSV_IRQn                 = -2,      
  SysTick_IRQn                = -1,      

 
  WWDG_IRQn                   = 0,       
  PVD_IRQn                    = 1,       
  TAMPER_IRQn                 = 2,       
  RTC_IRQn                    = 3,       
  FLASH_IRQn                  = 4,       
  RCC_IRQn                    = 5,       
  EXTI0_IRQn                  = 6,       
  EXTI1_IRQn                  = 7,       
  EXTI2_IRQn                  = 8,       
  EXTI3_IRQn                  = 9,       
  EXTI4_IRQn                  = 10,      
  DMA1_Channel1_IRQn          = 11,      
  DMA1_Channel2_IRQn          = 12,      
  DMA1_Channel3_IRQn          = 13,      
  DMA1_Channel4_IRQn          = 14,      
  DMA1_Channel5_IRQn          = 15,      
  DMA1_Channel6_IRQn          = 16,      
  DMA1_Channel7_IRQn          = 17,      

#line 221 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 242 "..\\SYSTEM\\sys\\stm32f10x.h"


  ADC1_2_IRQn                 = 18,      
  USB_HP_CAN1_TX_IRQn         = 19,      
  USB_LP_CAN1_RX0_IRQn        = 20,      
  CAN1_RX1_IRQn               = 21,      
  CAN1_SCE_IRQn               = 22,      
  EXTI9_5_IRQn                = 23,      
  TIM1_BRK_IRQn               = 24,      
  TIM1_UP_IRQn                = 25,      
  TIM1_TRG_COM_IRQn           = 26,      
  TIM1_CC_IRQn                = 27,      
  TIM2_IRQn                   = 28,      
  TIM3_IRQn                   = 29,      
  TIM4_IRQn                   = 30,      
  I2C1_EV_IRQn                = 31,      
  I2C1_ER_IRQn                = 32,      
  I2C2_EV_IRQn                = 33,      
  I2C2_ER_IRQn                = 34,      
  SPI1_IRQn                   = 35,      
  SPI2_IRQn                   = 36,      
  USART1_IRQn                 = 37,      
  USART2_IRQn                 = 38,      
  USART3_IRQn                 = 39,      
  EXTI15_10_IRQn              = 40,      
  RTCAlarm_IRQn               = 41,      
  USBWakeUp_IRQn              = 42         


#line 296 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 341 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 381 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 426 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 472 "..\\SYSTEM\\sys\\stm32f10x.h"
} IRQn_Type;



 

#line 1 "..\\SYSTEM\\sys\\core_cm3.h"
 




















 





































 

 
 
 
 
 
 
 
 








 











#line 1 "H:\\KEIL MDK4\\ARM\\RV31\\INC\\stdint.h"
 
 





 









#line 25 "H:\\KEIL MDK4\\ARM\\RV31\\INC\\stdint.h"







 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     
typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;

     
typedef   signed       __int64 intmax_t;
typedef unsigned       __int64 uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     


     


     


     

     


     


     


     

     



     



     


     
    
 



#line 196 "H:\\KEIL MDK4\\ARM\\RV31\\INC\\stdint.h"

     







     










     











#line 260 "H:\\KEIL MDK4\\ARM\\RV31\\INC\\stdint.h"



 


#line 91 "..\\SYSTEM\\sys\\core_cm3.h"

















 

#line 117 "..\\SYSTEM\\sys\\core_cm3.h"





 


 





 
typedef struct
{
  volatile uint32_t ISER[8];                       
       uint32_t RESERVED0[24];                                   
  volatile uint32_t ICER[8];                       
       uint32_t RSERVED1[24];                                    
  volatile uint32_t ISPR[8];                       
       uint32_t RESERVED2[24];                                   
  volatile uint32_t ICPR[8];                       
       uint32_t RESERVED3[24];                                   
  volatile uint32_t IABR[8];                       
       uint32_t RESERVED4[56];                                   
  volatile uint8_t  IP[240];                       
       uint32_t RESERVED5[644];                                  
  volatile  uint32_t STIR;                          
}  NVIC_Type;                                               
   





 
typedef struct
{
  volatile const  uint32_t CPUID;                         
  volatile uint32_t ICSR;                          
  volatile uint32_t VTOR;                          
  volatile uint32_t AIRCR;                         
  volatile uint32_t SCR;                           
  volatile uint32_t CCR;                           
  volatile uint8_t  SHP[12];                       
  volatile uint32_t SHCSR;                         
  volatile uint32_t CFSR;                          
  volatile uint32_t HFSR;                          
  volatile uint32_t DFSR;                          
  volatile uint32_t MMFAR;                         
  volatile uint32_t BFAR;                          
  volatile uint32_t AFSR;                          
  volatile const  uint32_t PFR[2];                        
  volatile const  uint32_t DFR;                           
  volatile const  uint32_t ADR;                           
  volatile const  uint32_t MMFR[4];                       
  volatile const  uint32_t ISAR[5];                       
} SCB_Type;                                                

 












 






























 






 





















 









 


















 
































                                     









 









 









 














   





 
typedef struct
{
  volatile uint32_t CTRL;                          
  volatile uint32_t LOAD;                          
  volatile uint32_t VAL;                           
  volatile const  uint32_t CALIB;                         
} SysTick_Type;

 












 



 



 








   





 
typedef struct
{
  volatile  union  
  {
    volatile  uint8_t    u8;                        
    volatile  uint16_t   u16;                       
    volatile  uint32_t   u32;                       
  }  PORT [32];                                
       uint32_t RESERVED0[864];                                 
  volatile uint32_t TER;                           
       uint32_t RESERVED1[15];                                  
  volatile uint32_t TPR;                           
       uint32_t RESERVED2[15];                                  
  volatile uint32_t TCR;                           
       uint32_t RESERVED3[29];                                  
  volatile uint32_t IWR;                           
  volatile uint32_t IRR;                           
  volatile uint32_t IMCR;                          
       uint32_t RESERVED4[43];                                  
  volatile uint32_t LAR;                           
  volatile uint32_t LSR;                           
       uint32_t RESERVED5[6];                                   
  volatile const  uint32_t PID4;                          
  volatile const  uint32_t PID5;                          
  volatile const  uint32_t PID6;                          
  volatile const  uint32_t PID7;                          
  volatile const  uint32_t PID0;                          
  volatile const  uint32_t PID1;                          
  volatile const  uint32_t PID2;                          
  volatile const  uint32_t PID3;                          
  volatile const  uint32_t CID0;                          
  volatile const  uint32_t CID1;                          
  volatile const  uint32_t CID2;                          
  volatile const  uint32_t CID3;                          
} ITM_Type;                                                

 



 
























 



 



 



 








   





 
typedef struct
{
       uint32_t RESERVED0;
  volatile const  uint32_t ICTR;                          



       uint32_t RESERVED1;

} InterruptType_Type;

 



 








   


#line 614 "..\\SYSTEM\\sys\\core_cm3.h"





 
typedef struct
{
  volatile uint32_t DHCSR;                         
  volatile  uint32_t DCRSR;                         
  volatile uint32_t DCRDR;                         
  volatile uint32_t DEMCR;                         
} CoreDebug_Type;

 




































 






 






































   


 
#line 721 "..\\SYSTEM\\sys\\core_cm3.h"

#line 728 "..\\SYSTEM\\sys\\core_cm3.h"






   




 





#line 758 "..\\SYSTEM\\sys\\core_cm3.h"


 


 




#line 783 "..\\SYSTEM\\sys\\core_cm3.h"


 
 
 
 








 
extern uint32_t __get_PSP(void);








 
extern void __set_PSP(uint32_t topOfProcStack);








 
extern uint32_t __get_MSP(void);








 
extern void __set_MSP(uint32_t topOfMainStack);








 
extern uint32_t __REV16(uint16_t value);








 
extern int32_t __REVSH(int16_t value);


#line 933 "..\\SYSTEM\\sys\\core_cm3.h"





 








 
static __inline uint32_t  __get_BASEPRI(void)
{
  register uint32_t __regBasePri         __asm("basepri");
  return(__regBasePri);
}







 
static __inline void __set_BASEPRI(uint32_t basePri)
{
  register uint32_t __regBasePri         __asm("basepri");
  __regBasePri = (basePri & 0xff);
}







 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}







 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}







 
static __inline uint32_t __get_FAULTMASK(void)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  return(__regFaultMask);
}







 
static __inline void __set_FAULTMASK(uint32_t faultMask)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  __regFaultMask = (faultMask & 1);
}







 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}







 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}





#line 1445 "..\\SYSTEM\\sys\\core_cm3.h"







 
 

 











 
static __inline void NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);                          
  
  reg_value  =  ((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR;                                                    
  reg_value &= ~((0xFFFFul << 16) | (7ul << 8));              
  reg_value  =  (reg_value                       |
                (0x5FA << 16) | 
                (PriorityGroupTmp << 8));                                      
  ((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR =  reg_value;
}








 
static __inline uint32_t NVIC_GetPriorityGrouping(void)
{
  return ((((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR & (7ul << 8)) >> 8);    
}








 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000) + 0x0100))->ISER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}








 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000) + 0x0100))->ICER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}









 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((((NVIC_Type *) ((0xE000E000) + 0x0100))->ISPR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));  
}








 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000) + 0x0100))->ISPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}








 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000) + 0x0100))->ICPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}









 
static __inline uint32_t NVIC_GetActive(IRQn_Type IRQn)
{
  return((uint32_t)((((NVIC_Type *) ((0xE000E000) + 0x0100))->IABR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));  
}












 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000) + 0x0D00))->SHP[((uint32_t)(IRQn) & 0xF)-4] = ((priority << (8 - 4)) & 0xff); }  
  else {
    ((NVIC_Type *) ((0xE000E000) + 0x0100))->IP[(uint32_t)(IRQn)] = ((priority << (8 - 4)) & 0xff);    }         
}















 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)(((SCB_Type *) ((0xE000E000) + 0x0D00))->SHP[((uint32_t)(IRQn) & 0xF)-4] >> (8 - 4)));  }  
  else {
    return((uint32_t)(((NVIC_Type *) ((0xE000E000) + 0x0100))->IP[(uint32_t)(IRQn)]           >> (8 - 4)));  }  
}
















 
static __inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);           
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 4) ? 4 : 7 - PriorityGroupTmp;
  SubPriorityBits     = ((PriorityGroupTmp + 4) < 7) ? 0 : PriorityGroupTmp - 7 + 4;
 
  return (
           ((PreemptPriority & ((1 << (PreemptPriorityBits)) - 1)) << SubPriorityBits) |
           ((SubPriority     & ((1 << (SubPriorityBits    )) - 1)))
         );
}
















 
static __inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* pPreemptPriority, uint32_t* pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);           
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 4) ? 4 : 7 - PriorityGroupTmp;
  SubPriorityBits     = ((PriorityGroupTmp + 4) < 7) ? 0 : PriorityGroupTmp - 7 + 4;
  
  *pPreemptPriority = (Priority >> SubPriorityBits) & ((1 << (PreemptPriorityBits)) - 1);
  *pSubPriority     = (Priority                   ) & ((1 << (SubPriorityBits    )) - 1);
}



 












 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{ 
  if (ticks > (0xFFFFFFul << 0))  return (1);             
                                                               
  ((SysTick_Type *) ((0xE000E000) + 0x0010))->LOAD  = (ticks & (0xFFFFFFul << 0)) - 1;       
  NVIC_SetPriority (SysTick_IRQn, (1<<4) - 1);   
  ((SysTick_Type *) ((0xE000E000) + 0x0010))->VAL   = 0;                                           
  ((SysTick_Type *) ((0xE000E000) + 0x0010))->CTRL  = (1ul << 2) | 
                   (1ul << 1)   | 
                   (1ul << 0);                     
  return (0);                                                   
}






 





 
static __inline void NVIC_SystemReset(void)
{
  ((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR  = ((0x5FA << 16)      | 
                 (((SCB_Type *) ((0xE000E000) + 0x0D00))->AIRCR & (7ul << 8)) | 
                 (1ul << 2));                    
  __dsb(0);                                                                    
  while(1);                                                     
}

   



 






 
 

extern volatile int ITM_RxBuffer;                     












 
static __inline uint32_t ITM_SendChar (uint32_t ch)
{
  if ((((CoreDebug_Type *) (0xE000EDF0))->DEMCR & (1ul << 24))  &&       
      (((ITM_Type *) (0xE0000000))->TCR & (1ul << 0))                  &&       
      (((ITM_Type *) (0xE0000000))->TER & (1ul << 0)        )                    )      
  {
    while (((ITM_Type *) (0xE0000000))->PORT[0].u32 == 0);
    ((ITM_Type *) (0xE0000000))->PORT[0].u8 = (uint8_t) ch;
  }  
  return (ch);
}










 
static __inline int ITM_ReceiveChar (void) {
  int ch = -1;                                

  if (ITM_RxBuffer != 0x5AA55AA5) {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = 0x5AA55AA5;        
  }
  
  return (ch); 
}









 
static __inline int ITM_CheckChar (void) {

  if (ITM_RxBuffer == 0x5AA55AA5) {
    return (0);                                  
  } else {
    return (1);                                  
  }
}

   






   



 
#line 479 "..\\SYSTEM\\sys\\stm32f10x.h"
#line 1 "..\\SYSTEM\\sys\\system_stm32f10x.h"



















 



 



   
  


 









 



 




 

extern uint32_t SystemCoreClock;           



 



 



 



 



 



 
  
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);


 









 
  


   
 
#line 480 "..\\SYSTEM\\sys\\stm32f10x.h"
#line 481 "..\\SYSTEM\\sys\\stm32f10x.h"



   

 
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;   
typedef const int16_t sc16;   
typedef const int8_t sc8;    

typedef volatile int32_t  vs32;
typedef volatile int16_t  vs16;
typedef volatile int8_t   vs8;

typedef volatile const int32_t vsc32;   
typedef volatile const int16_t vsc16;   
typedef volatile const int8_t vsc8;    

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;   
typedef const uint16_t uc16;   
typedef const uint8_t uc8;    

typedef volatile uint32_t  vu32;
typedef volatile uint16_t vu16;
typedef volatile uint8_t  vu8;

typedef volatile const uint32_t vuc32;   
typedef volatile const uint16_t vuc16;   
typedef volatile const uint8_t vuc8;    

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;


typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

 





 



    



 

typedef struct
{
  volatile uint32_t SR;
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t SMPR1;
  volatile uint32_t SMPR2;
  volatile uint32_t JOFR1;
  volatile uint32_t JOFR2;
  volatile uint32_t JOFR3;
  volatile uint32_t JOFR4;
  volatile uint32_t HTR;
  volatile uint32_t LTR;
  volatile uint32_t SQR1;
  volatile uint32_t SQR2;
  volatile uint32_t SQR3;
  volatile uint32_t JSQR;
  volatile uint32_t JDR1;
  volatile uint32_t JDR2;
  volatile uint32_t JDR3;
  volatile uint32_t JDR4;
  volatile uint32_t DR;
} ADC_TypeDef;



 

typedef struct
{
  uint32_t  RESERVED0;
  volatile uint16_t DR1;
  uint16_t  RESERVED1;
  volatile uint16_t DR2;
  uint16_t  RESERVED2;
  volatile uint16_t DR3;
  uint16_t  RESERVED3;
  volatile uint16_t DR4;
  uint16_t  RESERVED4;
  volatile uint16_t DR5;
  uint16_t  RESERVED5;
  volatile uint16_t DR6;
  uint16_t  RESERVED6;
  volatile uint16_t DR7;
  uint16_t  RESERVED7;
  volatile uint16_t DR8;
  uint16_t  RESERVED8;
  volatile uint16_t DR9;
  uint16_t  RESERVED9;
  volatile uint16_t DR10;
  uint16_t  RESERVED10; 
  volatile uint16_t RTCCR;
  uint16_t  RESERVED11;
  volatile uint16_t CR;
  uint16_t  RESERVED12;
  volatile uint16_t CSR;
  uint16_t  RESERVED13[5];
  volatile uint16_t DR11;
  uint16_t  RESERVED14;
  volatile uint16_t DR12;
  uint16_t  RESERVED15;
  volatile uint16_t DR13;
  uint16_t  RESERVED16;
  volatile uint16_t DR14;
  uint16_t  RESERVED17;
  volatile uint16_t DR15;
  uint16_t  RESERVED18;
  volatile uint16_t DR16;
  uint16_t  RESERVED19;
  volatile uint16_t DR17;
  uint16_t  RESERVED20;
  volatile uint16_t DR18;
  uint16_t  RESERVED21;
  volatile uint16_t DR19;
  uint16_t  RESERVED22;
  volatile uint16_t DR20;
  uint16_t  RESERVED23;
  volatile uint16_t DR21;
  uint16_t  RESERVED24;
  volatile uint16_t DR22;
  uint16_t  RESERVED25;
  volatile uint16_t DR23;
  uint16_t  RESERVED26;
  volatile uint16_t DR24;
  uint16_t  RESERVED27;
  volatile uint16_t DR25;
  uint16_t  RESERVED28;
  volatile uint16_t DR26;
  uint16_t  RESERVED29;
  volatile uint16_t DR27;
  uint16_t  RESERVED30;
  volatile uint16_t DR28;
  uint16_t  RESERVED31;
  volatile uint16_t DR29;
  uint16_t  RESERVED32;
  volatile uint16_t DR30;
  uint16_t  RESERVED33; 
  volatile uint16_t DR31;
  uint16_t  RESERVED34;
  volatile uint16_t DR32;
  uint16_t  RESERVED35;
  volatile uint16_t DR33;
  uint16_t  RESERVED36;
  volatile uint16_t DR34;
  uint16_t  RESERVED37;
  volatile uint16_t DR35;
  uint16_t  RESERVED38;
  volatile uint16_t DR36;
  uint16_t  RESERVED39;
  volatile uint16_t DR37;
  uint16_t  RESERVED40;
  volatile uint16_t DR38;
  uint16_t  RESERVED41;
  volatile uint16_t DR39;
  uint16_t  RESERVED42;
  volatile uint16_t DR40;
  uint16_t  RESERVED43;
  volatile uint16_t DR41;
  uint16_t  RESERVED44;
  volatile uint16_t DR42;
  uint16_t  RESERVED45;    
} BKP_TypeDef;
  


 

typedef struct
{
  volatile uint32_t TIR;
  volatile uint32_t TDTR;
  volatile uint32_t TDLR;
  volatile uint32_t TDHR;
} CAN_TxMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t RIR;
  volatile uint32_t RDTR;
  volatile uint32_t RDLR;
  volatile uint32_t RDHR;
} CAN_FIFOMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t FR1;
  volatile uint32_t FR2;
} CAN_FilterRegister_TypeDef;



 
  
typedef struct
{
  volatile uint32_t MCR;
  volatile uint32_t MSR;
  volatile uint32_t TSR;
  volatile uint32_t RF0R;
  volatile uint32_t RF1R;
  volatile uint32_t IER;
  volatile uint32_t ESR;
  volatile uint32_t BTR;
  uint32_t  RESERVED0[88];
  CAN_TxMailBox_TypeDef sTxMailBox[3];
  CAN_FIFOMailBox_TypeDef sFIFOMailBox[2];
  uint32_t  RESERVED1[12];
  volatile uint32_t FMR;
  volatile uint32_t FM1R;
  uint32_t  RESERVED2;
  volatile uint32_t FS1R;
  uint32_t  RESERVED3;
  volatile uint32_t FFA1R;
  uint32_t  RESERVED4;
  volatile uint32_t FA1R;
  uint32_t  RESERVED5[8];

  CAN_FilterRegister_TypeDef sFilterRegister[14];



} CAN_TypeDef;



 
typedef struct
{
  volatile uint32_t CFGR;
  volatile uint32_t OAR;
  volatile uint32_t PRES;
  volatile uint32_t ESR;
  volatile uint32_t CSR;
  volatile uint32_t TXD;
  volatile uint32_t RXD;  
} CEC_TypeDef;



 

typedef struct
{
  volatile uint32_t DR;
  volatile uint8_t  IDR;
  uint8_t   RESERVED0;
  uint16_t  RESERVED1;
  volatile uint32_t CR;
} CRC_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t SWTRIGR;
  volatile uint32_t DHR12R1;
  volatile uint32_t DHR12L1;
  volatile uint32_t DHR8R1;
  volatile uint32_t DHR12R2;
  volatile uint32_t DHR12L2;
  volatile uint32_t DHR8R2;
  volatile uint32_t DHR12RD;
  volatile uint32_t DHR12LD;
  volatile uint32_t DHR8RD;
  volatile uint32_t DOR1;
  volatile uint32_t DOR2;



} DAC_TypeDef;



 

typedef struct
{
  volatile uint32_t IDCODE;
  volatile uint32_t CR;	
}DBGMCU_TypeDef;



 

typedef struct
{
  volatile uint32_t CCR;
  volatile uint32_t CNDTR;
  volatile uint32_t CPAR;
  volatile uint32_t CMAR;
} DMA_Channel_TypeDef;

typedef struct
{
  volatile uint32_t ISR;
  volatile uint32_t IFCR;
} DMA_TypeDef;



 

typedef struct
{
  volatile uint32_t MACCR;
  volatile uint32_t MACFFR;
  volatile uint32_t MACHTHR;
  volatile uint32_t MACHTLR;
  volatile uint32_t MACMIIAR;
  volatile uint32_t MACMIIDR;
  volatile uint32_t MACFCR;
  volatile uint32_t MACVLANTR;              
       uint32_t RESERVED0[2];
  volatile uint32_t MACRWUFFR;              
  volatile uint32_t MACPMTCSR;
       uint32_t RESERVED1[2];
  volatile uint32_t MACSR;                  
  volatile uint32_t MACIMR;
  volatile uint32_t MACA0HR;
  volatile uint32_t MACA0LR;
  volatile uint32_t MACA1HR;
  volatile uint32_t MACA1LR;
  volatile uint32_t MACA2HR;
  volatile uint32_t MACA2LR;
  volatile uint32_t MACA3HR;
  volatile uint32_t MACA3LR;                
       uint32_t RESERVED2[40];
  volatile uint32_t MMCCR;                  
  volatile uint32_t MMCRIR;
  volatile uint32_t MMCTIR;
  volatile uint32_t MMCRIMR;
  volatile uint32_t MMCTIMR;                
       uint32_t RESERVED3[14];
  volatile uint32_t MMCTGFSCCR;             
  volatile uint32_t MMCTGFMSCCR;
       uint32_t RESERVED4[5];
  volatile uint32_t MMCTGFCR;
       uint32_t RESERVED5[10];
  volatile uint32_t MMCRFCECR;
  volatile uint32_t MMCRFAECR;
       uint32_t RESERVED6[10];
  volatile uint32_t MMCRGUFCR;
       uint32_t RESERVED7[334];
  volatile uint32_t PTPTSCR;
  volatile uint32_t PTPSSIR;
  volatile uint32_t PTPTSHR;
  volatile uint32_t PTPTSLR;
  volatile uint32_t PTPTSHUR;
  volatile uint32_t PTPTSLUR;
  volatile uint32_t PTPTSAR;
  volatile uint32_t PTPTTHR;
  volatile uint32_t PTPTTLR;
       uint32_t RESERVED8[567];
  volatile uint32_t DMABMR;
  volatile uint32_t DMATPDR;
  volatile uint32_t DMARPDR;
  volatile uint32_t DMARDLAR;
  volatile uint32_t DMATDLAR;
  volatile uint32_t DMASR;
  volatile uint32_t DMAOMR;
  volatile uint32_t DMAIER;
  volatile uint32_t DMAMFBOCR;
       uint32_t RESERVED9[9];
  volatile uint32_t DMACHTDR;
  volatile uint32_t DMACHRDR;
  volatile uint32_t DMACHTBAR;
  volatile uint32_t DMACHRBAR;
} ETH_TypeDef;



 

typedef struct
{
  volatile uint32_t IMR;
  volatile uint32_t EMR;
  volatile uint32_t RTSR;
  volatile uint32_t FTSR;
  volatile uint32_t SWIER;
  volatile uint32_t PR;
} EXTI_TypeDef;



 

typedef struct
{
  volatile uint32_t ACR;
  volatile uint32_t KEYR;
  volatile uint32_t OPTKEYR;
  volatile uint32_t SR;
  volatile uint32_t CR;
  volatile uint32_t AR;
  volatile uint32_t RESERVED;
  volatile uint32_t OBR;
  volatile uint32_t WRPR;
#line 920 "..\\SYSTEM\\sys\\stm32f10x.h"
} FLASH_TypeDef;



 
  
typedef struct
{
  volatile uint16_t RDP;
  volatile uint16_t USER;
  volatile uint16_t Data0;
  volatile uint16_t Data1;
  volatile uint16_t WRP0;
  volatile uint16_t WRP1;
  volatile uint16_t WRP2;
  volatile uint16_t WRP3;
} OB_TypeDef;



 

typedef struct
{
  volatile uint32_t BTCR[8];   
} FSMC_Bank1_TypeDef; 



 
  
typedef struct
{
  volatile uint32_t BWTR[7];
} FSMC_Bank1E_TypeDef;



 
  
typedef struct
{
  volatile uint32_t PCR2;
  volatile uint32_t SR2;
  volatile uint32_t PMEM2;
  volatile uint32_t PATT2;
  uint32_t  RESERVED0;   
  volatile uint32_t ECCR2; 
} FSMC_Bank2_TypeDef;  



 
  
typedef struct
{
  volatile uint32_t PCR3;
  volatile uint32_t SR3;
  volatile uint32_t PMEM3;
  volatile uint32_t PATT3;
  uint32_t  RESERVED0;   
  volatile uint32_t ECCR3; 
} FSMC_Bank3_TypeDef; 



 
  
typedef struct
{
  volatile uint32_t PCR4;
  volatile uint32_t SR4;
  volatile uint32_t PMEM4;
  volatile uint32_t PATT4;
  volatile uint32_t PIO4; 
} FSMC_Bank4_TypeDef; 



 

typedef struct
{
  volatile uint32_t CRL;
  volatile uint32_t CRH;
  volatile uint32_t IDR;
  volatile uint32_t ODR;
  volatile uint32_t BSRR;
  volatile uint32_t BRR;
  volatile uint32_t LCKR;
} GPIO_TypeDef;



 

typedef struct
{
  volatile uint32_t EVCR;
  volatile uint32_t MAPR;
  volatile uint32_t EXTICR[4];
  uint32_t RESERVED0;
  volatile uint32_t MAPR2;  
} AFIO_TypeDef;


 

typedef struct
{
  volatile uint16_t CR1;
  uint16_t  RESERVED0;
  volatile uint16_t CR2;
  uint16_t  RESERVED1;
  volatile uint16_t OAR1;
  uint16_t  RESERVED2;
  volatile uint16_t OAR2;
  uint16_t  RESERVED3;
  volatile uint16_t DR;
  uint16_t  RESERVED4;
  volatile uint16_t SR1;
  uint16_t  RESERVED5;
  volatile uint16_t SR2;
  uint16_t  RESERVED6;
  volatile uint16_t CCR;
  uint16_t  RESERVED7;
  volatile uint16_t TRISE;
  uint16_t  RESERVED8;
} I2C_TypeDef;



 

typedef struct
{
  volatile uint32_t KR;
  volatile uint32_t PR;
  volatile uint32_t RLR;
  volatile uint32_t SR;
} IWDG_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CSR;
} PWR_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CFGR;
  volatile uint32_t CIR;
  volatile uint32_t APB2RSTR;
  volatile uint32_t APB1RSTR;
  volatile uint32_t AHBENR;
  volatile uint32_t APB2ENR;
  volatile uint32_t APB1ENR;
  volatile uint32_t BDCR;
  volatile uint32_t CSR;










} RCC_TypeDef;



 

typedef struct
{
  volatile uint16_t CRH;
  uint16_t  RESERVED0;
  volatile uint16_t CRL;
  uint16_t  RESERVED1;
  volatile uint16_t PRLH;
  uint16_t  RESERVED2;
  volatile uint16_t PRLL;
  uint16_t  RESERVED3;
  volatile uint16_t DIVH;
  uint16_t  RESERVED4;
  volatile uint16_t DIVL;
  uint16_t  RESERVED5;
  volatile uint16_t CNTH;
  uint16_t  RESERVED6;
  volatile uint16_t CNTL;
  uint16_t  RESERVED7;
  volatile uint16_t ALRH;
  uint16_t  RESERVED8;
  volatile uint16_t ALRL;
  uint16_t  RESERVED9;
} RTC_TypeDef;



 

typedef struct
{
  volatile uint32_t POWER;
  volatile uint32_t CLKCR;
  volatile uint32_t ARG;
  volatile uint32_t CMD;
  volatile const uint32_t RESPCMD;
  volatile const uint32_t RESP1;
  volatile const uint32_t RESP2;
  volatile const uint32_t RESP3;
  volatile const uint32_t RESP4;
  volatile uint32_t DTIMER;
  volatile uint32_t DLEN;
  volatile uint32_t DCTRL;
  volatile const uint32_t DCOUNT;
  volatile const uint32_t STA;
  volatile uint32_t ICR;
  volatile uint32_t MASK;
  uint32_t  RESERVED0[2];
  volatile const uint32_t FIFOCNT;
  uint32_t  RESERVED1[13];
  volatile uint32_t FIFO;
} SDIO_TypeDef;



 

typedef struct
{
  volatile uint16_t CR1;
  uint16_t  RESERVED0;
  volatile uint16_t CR2;
  uint16_t  RESERVED1;
  volatile uint16_t SR;
  uint16_t  RESERVED2;
  volatile uint16_t DR;
  uint16_t  RESERVED3;
  volatile uint16_t CRCPR;
  uint16_t  RESERVED4;
  volatile uint16_t RXCRCR;
  uint16_t  RESERVED5;
  volatile uint16_t TXCRCR;
  uint16_t  RESERVED6;
  volatile uint16_t I2SCFGR;
  uint16_t  RESERVED7;
  volatile uint16_t I2SPR;
  uint16_t  RESERVED8;  
} SPI_TypeDef;



 

typedef struct
{
  volatile uint16_t CR1;
  uint16_t  RESERVED0;
  volatile uint16_t CR2;
  uint16_t  RESERVED1;
  volatile uint16_t SMCR;
  uint16_t  RESERVED2;
  volatile uint16_t DIER;
  uint16_t  RESERVED3;
  volatile uint16_t SR;
  uint16_t  RESERVED4;
  volatile uint16_t EGR;
  uint16_t  RESERVED5;
  volatile uint16_t CCMR1;
  uint16_t  RESERVED6;
  volatile uint16_t CCMR2;
  uint16_t  RESERVED7;
  volatile uint16_t CCER;
  uint16_t  RESERVED8;
  volatile uint16_t CNT;
  uint16_t  RESERVED9;
  volatile uint16_t PSC;
  uint16_t  RESERVED10;
  volatile uint16_t ARR;
  uint16_t  RESERVED11;
  volatile uint16_t RCR;
  uint16_t  RESERVED12;
  volatile uint16_t CCR1;
  uint16_t  RESERVED13;
  volatile uint16_t CCR2;
  uint16_t  RESERVED14;
  volatile uint16_t CCR3;
  uint16_t  RESERVED15;
  volatile uint16_t CCR4;
  uint16_t  RESERVED16;
  volatile uint16_t BDTR;
  uint16_t  RESERVED17;
  volatile uint16_t DCR;
  uint16_t  RESERVED18;
  volatile uint16_t DMAR;
  uint16_t  RESERVED19;
} TIM_TypeDef;



 
 
typedef struct
{
  volatile uint16_t SR;
  uint16_t  RESERVED0;
  volatile uint16_t DR;
  uint16_t  RESERVED1;
  volatile uint16_t BRR;
  uint16_t  RESERVED2;
  volatile uint16_t CR1;
  uint16_t  RESERVED3;
  volatile uint16_t CR2;
  uint16_t  RESERVED4;
  volatile uint16_t CR3;
  uint16_t  RESERVED5;
  volatile uint16_t GTPR;
  uint16_t  RESERVED6;
} USART_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CFR;
  volatile uint32_t SR;
} WWDG_TypeDef;



 
  


 











 




#line 1312 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 1335 "..\\SYSTEM\\sys\\stm32f10x.h"



#line 1354 "..\\SYSTEM\\sys\\stm32f10x.h"




















 
  


   

#line 1454 "..\\SYSTEM\\sys\\stm32f10x.h"



 



 
  
  

 
    
 
 
 

 
 
 
 
 

 



 



 


 
 
 
 
 

 











 
#line 1515 "..\\SYSTEM\\sys\\stm32f10x.h"




 





 
 
 
 
 

 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 





 



 






 
 
 
 
 

 
#line 1691 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 1698 "..\\SYSTEM\\sys\\stm32f10x.h"

 
 








 








 






#line 1734 "..\\SYSTEM\\sys\\stm32f10x.h"

 











 











 













 






#line 1850 "..\\SYSTEM\\sys\\stm32f10x.h"




#line 1870 "..\\SYSTEM\\sys\\stm32f10x.h"

 





#line 1883 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 1902 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 1911 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 1919 "..\\SYSTEM\\sys\\stm32f10x.h"



















#line 1944 "..\\SYSTEM\\sys\\stm32f10x.h"












 













#line 1976 "..\\SYSTEM\\sys\\stm32f10x.h"





#line 1990 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 1997 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 2007 "..\\SYSTEM\\sys\\stm32f10x.h"











 


















#line 2043 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 2051 "..\\SYSTEM\\sys\\stm32f10x.h"



















#line 2076 "..\\SYSTEM\\sys\\stm32f10x.h"












 













#line 2108 "..\\SYSTEM\\sys\\stm32f10x.h"





#line 2122 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 2129 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 2139 "..\\SYSTEM\\sys\\stm32f10x.h"











 








 








   
#line 2178 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 2273 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 2300 "..\\SYSTEM\\sys\\stm32f10x.h"
 
 
 
 
 
 

 




































































 




































































 
#line 2462 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 2480 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 2498 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 2515 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 2533 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 2552 "..\\SYSTEM\\sys\\stm32f10x.h"

 

 






 
#line 2579 "..\\SYSTEM\\sys\\stm32f10x.h"






 








 









 








 








 









 










 




#line 2654 "..\\SYSTEM\\sys\\stm32f10x.h"

 










#line 2685 "..\\SYSTEM\\sys\\stm32f10x.h"

 





 
#line 2700 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 2709 "..\\SYSTEM\\sys\\stm32f10x.h"

   
#line 2718 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 2727 "..\\SYSTEM\\sys\\stm32f10x.h"

 





 
#line 2742 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 2751 "..\\SYSTEM\\sys\\stm32f10x.h"

   
#line 2760 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 2769 "..\\SYSTEM\\sys\\stm32f10x.h"

 





 
#line 2784 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 2793 "..\\SYSTEM\\sys\\stm32f10x.h"

   
#line 2802 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 2811 "..\\SYSTEM\\sys\\stm32f10x.h"

 





 
#line 2826 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 2835 "..\\SYSTEM\\sys\\stm32f10x.h"

   
#line 2844 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 2853 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 2862 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 2871 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 2881 "..\\SYSTEM\\sys\\stm32f10x.h"

 
 
 
 
 

 





 


 


 




 
 
 
 
 

 
#line 2945 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 2980 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 3015 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 3050 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 3085 "..\\SYSTEM\\sys\\stm32f10x.h"

 





 





 





 





 





 





 





 





 






 
#line 3152 "..\\SYSTEM\\sys\\stm32f10x.h"

 



 









 
#line 3176 "..\\SYSTEM\\sys\\stm32f10x.h"




 




 
#line 3192 "..\\SYSTEM\\sys\\stm32f10x.h"

 





 
#line 3214 "..\\SYSTEM\\sys\\stm32f10x.h"

 
 





 
#line 3229 "..\\SYSTEM\\sys\\stm32f10x.h"
 
#line 3236 "..\\SYSTEM\\sys\\stm32f10x.h"

 




 






 


 


 


 
 
 
 
 

 
#line 3285 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 3307 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 3329 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 3351 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 3373 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 3395 "..\\SYSTEM\\sys\\stm32f10x.h"

 
 
 
 
 

 
#line 3431 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 3461 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 3471 "..\\SYSTEM\\sys\\stm32f10x.h"















 
#line 3495 "..\\SYSTEM\\sys\\stm32f10x.h"















 
#line 3519 "..\\SYSTEM\\sys\\stm32f10x.h"















 
#line 3543 "..\\SYSTEM\\sys\\stm32f10x.h"















 
#line 3567 "..\\SYSTEM\\sys\\stm32f10x.h"















 
#line 3591 "..\\SYSTEM\\sys\\stm32f10x.h"















 
#line 3615 "..\\SYSTEM\\sys\\stm32f10x.h"















 


 


 


 


 


 


 


 


 


 



 


 


 



 


 


 


 



 


 


 


 


 
 
 
 
 

 






 
#line 3716 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 3725 "..\\SYSTEM\\sys\\stm32f10x.h"















  
 
#line 3748 "..\\SYSTEM\\sys\\stm32f10x.h"


















 








































 


















































 


 


 


 


 


 


 
#line 3883 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 3890 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 3897 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 3904 "..\\SYSTEM\\sys\\stm32f10x.h"







 
#line 3918 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 3925 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 3932 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 3939 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 3946 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 3953 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 3961 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 3968 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 3975 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 3982 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 3989 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 3996 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 4004 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 4011 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 4018 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 4025 "..\\SYSTEM\\sys\\stm32f10x.h"





 


 


 


 


 



 
 
 
 
 

 









































 



 


 


 


 


 


 


 



 



 



 


 


 



 
 
 
 
 
 





 






 


 
#line 4167 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 4177 "..\\SYSTEM\\sys\\stm32f10x.h"

 


 


 
 
 
 
 

 
















 









#line 4225 "..\\SYSTEM\\sys\\stm32f10x.h"

 

























 
#line 4268 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 4282 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 4292 "..\\SYSTEM\\sys\\stm32f10x.h"

 




























 





















 




























 





















 
#line 4411 "..\\SYSTEM\\sys\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
#line 4446 "..\\SYSTEM\\sys\\stm32f10x.h"





#line 4457 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 4465 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 4472 "..\\SYSTEM\\sys\\stm32f10x.h"

 


 
 
 
 
 

 




 
#line 4494 "..\\SYSTEM\\sys\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
 
 
 
 

 


 





 


 



 
 
 
 
 

 
#line 4556 "..\\SYSTEM\\sys\\stm32f10x.h"



 
#line 4568 "..\\SYSTEM\\sys\\stm32f10x.h"







 


 
 
 
 
 

 











#line 4606 "..\\SYSTEM\\sys\\stm32f10x.h"

 











#line 4629 "..\\SYSTEM\\sys\\stm32f10x.h"

 











#line 4652 "..\\SYSTEM\\sys\\stm32f10x.h"

 











#line 4675 "..\\SYSTEM\\sys\\stm32f10x.h"

 








































 








































 








































 








































 


































 


































 


































 


































 



























 



























 



























 
#line 5072 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 5081 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 5090 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 5101 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 5111 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 5121 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 5131 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 5142 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 5152 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 5162 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 5172 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 5183 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 5193 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 5203 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 5213 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 5224 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 5234 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 5244 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 5254 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 5265 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 5275 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 5285 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 5295 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 5306 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 5316 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 5326 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 5336 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 5347 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 5357 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 5367 "..\\SYSTEM\\sys\\stm32f10x.h"

#line 5377 "..\\SYSTEM\\sys\\stm32f10x.h"

 


 


 
 
 
 
 

 




 












 


 






#line 5425 "..\\SYSTEM\\sys\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
















 


 
#line 5495 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 5510 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 5536 "..\\SYSTEM\\sys\\stm32f10x.h"

 


 


 
 
 
 
 

 
 























 























 























 























 























 























 























 























 
 
#line 5757 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 5769 "..\\SYSTEM\\sys\\stm32f10x.h"

 






 
#line 5786 "..\\SYSTEM\\sys\\stm32f10x.h"



     


 
 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 

 


#line 5930 "..\\SYSTEM\\sys\\stm32f10x.h"



 


#line 5942 "..\\SYSTEM\\sys\\stm32f10x.h"



 


#line 5954 "..\\SYSTEM\\sys\\stm32f10x.h"



 


#line 5966 "..\\SYSTEM\\sys\\stm32f10x.h"



 


#line 5978 "..\\SYSTEM\\sys\\stm32f10x.h"



 


#line 5990 "..\\SYSTEM\\sys\\stm32f10x.h"



 


#line 6002 "..\\SYSTEM\\sys\\stm32f10x.h"



 


#line 6014 "..\\SYSTEM\\sys\\stm32f10x.h"



 

 


#line 6028 "..\\SYSTEM\\sys\\stm32f10x.h"



 


#line 6040 "..\\SYSTEM\\sys\\stm32f10x.h"



 


#line 6052 "..\\SYSTEM\\sys\\stm32f10x.h"



 


#line 6064 "..\\SYSTEM\\sys\\stm32f10x.h"



 


#line 6076 "..\\SYSTEM\\sys\\stm32f10x.h"



 


#line 6088 "..\\SYSTEM\\sys\\stm32f10x.h"



 


#line 6100 "..\\SYSTEM\\sys\\stm32f10x.h"



 


#line 6112 "..\\SYSTEM\\sys\\stm32f10x.h"



 


#line 6124 "..\\SYSTEM\\sys\\stm32f10x.h"



 


#line 6136 "..\\SYSTEM\\sys\\stm32f10x.h"



 


#line 6148 "..\\SYSTEM\\sys\\stm32f10x.h"



 


#line 6160 "..\\SYSTEM\\sys\\stm32f10x.h"



 


#line 6172 "..\\SYSTEM\\sys\\stm32f10x.h"



 


#line 6184 "..\\SYSTEM\\sys\\stm32f10x.h"



 


#line 6196 "..\\SYSTEM\\sys\\stm32f10x.h"



 


#line 6208 "..\\SYSTEM\\sys\\stm32f10x.h"



 
 
 
 
 

 
 
#line 6228 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 6239 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 6257 "..\\SYSTEM\\sys\\stm32f10x.h"











 





 





 
#line 6295 "..\\SYSTEM\\sys\\stm32f10x.h"

 












 
#line 6316 "..\\SYSTEM\\sys\\stm32f10x.h"

 
 






 




 





 





 






 




 





 





 






   




 





 





 





 




 





 





 





 




 





 





 
 


 
#line 6456 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 6473 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 6490 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 6507 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 6541 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 6575 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 6609 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 6643 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 6677 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 6711 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 6745 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 6779 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 6813 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 6847 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 6881 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 6915 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 6949 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 6983 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 7017 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 7051 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 7085 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 7119 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 7153 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 7187 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 7221 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 7255 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 7289 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 7323 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 7357 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 7391 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 7425 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 7459 "..\\SYSTEM\\sys\\stm32f10x.h"

 
 
 
 
 

 









#line 7486 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 7494 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 7504 "..\\SYSTEM\\sys\\stm32f10x.h"

 


 


 


 


 





















 




 
 
 
 
 

 
#line 7565 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 7574 "..\\SYSTEM\\sys\\stm32f10x.h"







 



#line 7595 "..\\SYSTEM\\sys\\stm32f10x.h"



 



 


 
#line 7620 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 7630 "..\\SYSTEM\\sys\\stm32f10x.h"

 




 


 
 
 
 
 

 
#line 7656 "..\\SYSTEM\\sys\\stm32f10x.h"

 


 



 
#line 7680 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 7689 "..\\SYSTEM\\sys\\stm32f10x.h"







 
#line 7709 "..\\SYSTEM\\sys\\stm32f10x.h"

 
#line 7720 "..\\SYSTEM\\sys\\stm32f10x.h"



 
 
 
 
 

 


#line 7749 "..\\SYSTEM\\sys\\stm32f10x.h"

 









#line 7783 "..\\SYSTEM\\sys\\stm32f10x.h"

 
 
 
 
 

 









 


 


 





 
#line 7823 "..\\SYSTEM\\sys\\stm32f10x.h"

 


 









 


 

 



 



 



 



 



 



 



 



#line 8287 "..\\SYSTEM\\sys\\stm32f10x.h"



 

 

  







 

















 









 

  

 

 
#line 4 "..\\SYSTEM\\sys\\sys.h"




																	    
	 







#line 24 "..\\SYSTEM\\sys\\sys.h"

#line 32 "..\\SYSTEM\\sys\\sys.h"
 
























#line 64 "..\\SYSTEM\\sys\\sys.h"

#line 1 "..\\SYSTEM\\delay\\delay.h"
#line 1 "..\\SYSTEM\\sys\\sys.h"
#line 124 "..\\SYSTEM\\sys\\sys.h"











#line 4 "..\\SYSTEM\\delay\\delay.h"

void delay_init(u8 SYSCLK);
void delay_ms(u16 nms);
void delay_us(u32 nus);































#line 68 "..\\SYSTEM\\sys\\sys.h"
#line 1 "..\\HARDWARE\\LED\\led.h"
#line 4 "..\\HARDWARE\\LED\\led.h"


void LED_Init(void);  
void Led_Flash(u16 time);
#line 69 "..\\SYSTEM\\sys\\sys.h"
#line 1 "..\\HARDWARE\\KEY\\key.h"
#line 4 "..\\HARDWARE\\KEY\\key.h"

void KEY_Init(void);          
u8 click_N_Double (u8 time);  
u8 click(void);               
u8 Long_Press(void);
#line 70 "..\\SYSTEM\\sys\\sys.h"
#line 1 "..\\HARDWARE\\OLED\\oled.h"
#line 4 "..\\HARDWARE\\OLED\\oled.h"

















void OLED_WR_Byte(u8 dat,u8 cmd);	    
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);		   				   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode);
void OLED_ShowNumber(u8 x,u8 y,u32 num,u8 len,u8 size);
void OLED_ShowString(u8 x,u8 y,const u8 *p);	 

	 
#line 71 "..\\SYSTEM\\sys\\sys.h"
#line 1 "..\\SYSTEM\\usart\\usart.h"
#line 4 "..\\SYSTEM\\usart\\usart.h"
#line 1 "H:\\KEIL MDK4\\ARM\\RV31\\INC\\stdio.h"
 
 
 





 






 









#line 34 "H:\\KEIL MDK4\\ARM\\RV31\\INC\\stdio.h"


  
  typedef unsigned int size_t;    








 
 

 
  typedef struct __va_list __va_list;





   




 




typedef struct __fpos_t_struct {
    unsigned __int64 __pos;
    



 
    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
   


 


   

 

typedef struct __FILE FILE;
   






 

extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;

#line 125 "H:\\KEIL MDK4\\ARM\\RV31\\INC\\stdio.h"
    

    

    





     



   


 


   


 

   



 

   


 




   


 





    


 






extern __declspec(__nothrow) int remove(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int rename(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) FILE *tmpfile(void);
   




 
extern __declspec(__nothrow) char *tmpnam(char *  );
   











 

extern __declspec(__nothrow) int fclose(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) int fflush(FILE *  );
   







 
extern __declspec(__nothrow) FILE *fopen(const char * __restrict  ,
                           const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   








































 
extern __declspec(__nothrow) FILE *freopen(const char * __restrict  ,
                    const char * __restrict  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(2,3)));
   








 
extern __declspec(__nothrow) void setbuf(FILE * __restrict  ,
                    char * __restrict  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) int setvbuf(FILE * __restrict  ,
                   char * __restrict  ,
                   int  , size_t  ) __attribute__((__nonnull__(1)));
   















 
#pragma __printf_args
extern __declspec(__nothrow) int fprintf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   


















 
#pragma __printf_args
extern __declspec(__nothrow) int _fprintf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   




 
#pragma __printf_args
extern __declspec(__nothrow) int _printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






 
#pragma __printf_args
extern __declspec(__nothrow) int _sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   















 

#pragma __printf_args
extern __declspec(__nothrow) int _snprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int fscanf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






























 
#pragma __scanf_args
extern __declspec(__nothrow) int _fscanf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   






 
#pragma __scanf_args
extern __declspec(__nothrow) int _scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int sscanf(const char * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   








 
#pragma __scanf_args
extern __declspec(__nothrow) int _sscanf(const char * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

 
extern __declspec(__nothrow) int vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int _vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int _vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int _vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int _vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int vfprintf(FILE * __restrict  ,
                    const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int vsprintf(char * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 

extern __declspec(__nothrow) int vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   







 

extern __declspec(__nothrow) int _vsprintf(char * __restrict  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vfprintf(FILE * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vsnprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   



 
extern __declspec(__nothrow) int fgetc(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) char *fgets(char * __restrict  , int  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   










 
extern __declspec(__nothrow) int fputc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   







 
extern __declspec(__nothrow) int fputs(const char * __restrict  , FILE * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int getc(FILE *  ) __attribute__((__nonnull__(1)));
   







 




    extern __declspec(__nothrow) int (getchar)(void);

   





 
extern __declspec(__nothrow) char *gets(char *  ) __attribute__((__nonnull__(1)));
   









 
extern __declspec(__nothrow) int putc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   





 




    extern __declspec(__nothrow) int (putchar)(int  );

   



 
extern __declspec(__nothrow) int puts(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int ungetc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   






















 

extern __declspec(__nothrow) size_t fread(void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   











 

extern __declspec(__nothrow) size_t __fread_bytes_avail(void * __restrict  ,
                    size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   











 

extern __declspec(__nothrow) size_t fwrite(const void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   







 

extern __declspec(__nothrow) int fgetpos(FILE * __restrict  , fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) int fseek(FILE *  , long int  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) int fsetpos(FILE * __restrict  , const fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   










 
extern __declspec(__nothrow) long int ftell(FILE *  ) __attribute__((__nonnull__(1)));
   











 
extern __declspec(__nothrow) void rewind(FILE *  ) __attribute__((__nonnull__(1)));
   





 

extern __declspec(__nothrow) void clearerr(FILE *  ) __attribute__((__nonnull__(1)));
   




 

extern __declspec(__nothrow) int feof(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) int ferror(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) void perror(const char *  );
   









 

extern __declspec(__nothrow) int _fisatty(FILE *   ) __attribute__((__nonnull__(1)));
    
 

extern __declspec(__nothrow) void __use_no_semihosting_swi(void);
extern __declspec(__nothrow) void __use_no_semihosting(void);
    





 











#line 944 "H:\\KEIL MDK4\\ARM\\RV31\\INC\\stdio.h"



 
#line 5 "..\\SYSTEM\\usart\\usart.h"
void usart1_send(u8 data);
void uart_init(u32 pclk2,u32 bound);

















#line 72 "..\\SYSTEM\\sys\\sys.h"
#line 1 "..\\HARDWARE\\USART3\\usart3.h"
#line 4 "..\\HARDWARE\\USART3\\usart3.h"

extern u8 Usart3_Receive;
void uart3_init(u32 pclk2,u32 bound);
void USART3_IRQHandler(void);


#line 73 "..\\SYSTEM\\sys\\sys.h"
#line 1 "..\\HARDWARE\\ADC\\adc.h"
#line 4 "..\\HARDWARE\\ADC\\adc.h"

void Adc_Init(void);
u16 Get_Adc(u8 ch);
int Get_battery_volt(void);   
















#line 74 "..\\SYSTEM\\sys\\sys.h"
#line 1 "..\\HARDWARE\\TIMER\\timer.h"
#line 4 "..\\HARDWARE\\TIMER\\timer.h"
void TIM3_Cap_Init(u16 arr,u16 psc);
void Read_Distane(void);
void TIM3_IRQHandler(void);
#line 75 "..\\SYSTEM\\sys\\sys.h"
#line 1 "..\\HARDWARE\\MOTOR\\motor.h"
#line 76 "..\\SYSTEM\\sys\\sys.h"
#line 1 "..\\HARDWARE\\ENCODER\\encoder.h"
#line 4 "..\\HARDWARE\\ENCODER\\encoder.h"
  


 

void Encoder_Init_TIM2(void);
void Encoder_Init_TIM4(void);
int Read_Encoder(u8 TIMX);
void TIM4_IRQHandler(void);
void TIM2_IRQHandler(void);
#line 77 "..\\SYSTEM\\sys\\sys.h"
#line 1 "..\\HARDWARE\\IIC\\ioi2c.h"
#line 4 "..\\HARDWARE\\IIC\\ioi2c.h"
  


 






#line 21 "..\\HARDWARE\\IIC\\ioi2c.h"

#line 29 "..\\HARDWARE\\IIC\\ioi2c.h"





















void IIC_Init(void);                
int IIC_Start(void);				
void IIC_Stop(void);	  			
void IIC_Send_Byte(u8 txd);			
u8 IIC_Read_Byte(unsigned char ack);
int IIC_Wait_Ack(void); 				
void IIC_Ack(void);					
void IIC_NAck(void);				

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	 
unsigned char I2C_Readkey(unsigned char I2C_Addr);

unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr);
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data);
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data);
u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data);
u8 IICwriteBit(u8 dev,u8 reg,u8 bitNum,u8 data);
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data);

int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);




#line 78 "..\\SYSTEM\\sys\\sys.h"
#line 1 "..\\BALANCE\\MPU6050\\mpu6050.h"



#line 5 "..\\BALANCE\\MPU6050\\mpu6050.h"






#line 121 "..\\BALANCE\\MPU6050\\mpu6050.h"














#line 143 "..\\BALANCE\\MPU6050\\mpu6050.h"

#line 151 "..\\BALANCE\\MPU6050\\mpu6050.h"









#line 167 "..\\BALANCE\\MPU6050\\mpu6050.h"






#line 179 "..\\BALANCE\\MPU6050\\mpu6050.h"

#line 188 "..\\BALANCE\\MPU6050\\mpu6050.h"

#line 195 "..\\BALANCE\\MPU6050\\mpu6050.h"

#line 212 "..\\BALANCE\\MPU6050\\mpu6050.h"

#line 222 "..\\BALANCE\\MPU6050\\mpu6050.h"

#line 231 "..\\BALANCE\\MPU6050\\mpu6050.h"

#line 240 "..\\BALANCE\\MPU6050\\mpu6050.h"

#line 249 "..\\BALANCE\\MPU6050\\mpu6050.h"













#line 270 "..\\BALANCE\\MPU6050\\mpu6050.h"



#line 279 "..\\BALANCE\\MPU6050\\mpu6050.h"

#line 287 "..\\BALANCE\\MPU6050\\mpu6050.h"

#line 294 "..\\BALANCE\\MPU6050\\mpu6050.h"





#line 305 "..\\BALANCE\\MPU6050\\mpu6050.h"






#line 319 "..\\BALANCE\\MPU6050\\mpu6050.h"

#line 326 "..\\BALANCE\\MPU6050\\mpu6050.h"

#line 334 "..\\BALANCE\\MPU6050\\mpu6050.h"

#line 343 "..\\BALANCE\\MPU6050\\mpu6050.h"













extern	short gyro[3], accel[3];
extern int16_t Gx_offset,Gy_offset,Gz_offset;
extern float Acc1G_Values;
extern float Pitch,Roll; 

void MPU6050_initialize(void); 
uint8_t MPU6050_testConnection(void); 

void MPU6050_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
void MPU6050_getlastMotion6(int16_t* ax, int16_t* ay, 
		int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
uint8_t MPU6050_getDeviceID(void); 
void MPU6050_InitGyro_Offset(void);
void DMP_Init(void);
void Read_DMP(void);
int Read_Temperature(void);
#line 79 "..\\SYSTEM\\sys\\sys.h"
#line 1 "..\\BALANCE\\show\\show.h"
#line 4 "..\\BALANCE\\show\\show.h"
  


 
void oled_show(void);
void APP_Show(void);
void DataScope(void);
#line 80 "..\\SYSTEM\\sys\\sys.h"
#line 1 "..\\HARDWARE\\EXTI\\exti.h"
#line 4 "..\\HARDWARE\\EXTI\\exti.h"
  


 

void EXTI_Init(void);	


























#line 81 "..\\SYSTEM\\sys\\sys.h"
#line 1 "..\\HARDWARE\\DataScope_DP\\DataScope_DP.h"



 



 
 
extern unsigned char DataScope_OutPut_Buffer[42];	   


void DataScope_Get_Channel_Data(float Data,unsigned char Channel);    

unsigned char DataScope_Data_Generate(unsigned char Channel_Number);  
 
 




#line 82 "..\\SYSTEM\\sys\\sys.h"




extern u8 Way_Angle;                                      
extern int Encoder_Left,Encoder_Right;                     
extern int Moto1,Moto2;                                     
extern u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_sudu; 
extern u8 Flag_Stop,Flag_Show;                               
extern int Voltage,Voltage_Zheng,Voltage_Xiao;                
extern float Angle_Balance,Gyro_Balance,Gyro_Turn;           
extern float Show_Data_Mb;                                    
extern int Temperature;
extern u32 Distance;                                           
extern u8 Bi_zhang,delay_50,delay_flag;
extern float Acceleration_Z;

void Stm32_Clock_Init(u8 PLL);  
void Sys_Soft_Reset(void);      
void Sys_Standby(void);         
void MY_NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset);
void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group);
void MY_NVIC_Init(u8 NVIC_PreemptionPriority,u8 NVIC_SubPriority,u8 NVIC_Channel,u8 NVIC_Group);
void Ex_NVIC_Config(u8 GPIOx,u8 BITx,u8 TRIM);
void JTAG_Set(u8 mode);


void WFI_SET(void);		
void INTX_DISABLE(void);
void INTX_ENABLE(void);	
void MSR_MSP(u32 addr);	
#line 1 "..\\BALANCE\\DMP\\inv_mpu.h"













 




#line 25 "..\\BALANCE\\DMP\\inv_mpu.h"

struct int_param_s {

    void (*cb)(void);
    unsigned short pin;
    unsigned char lp_exit;
    unsigned char active_low;




 
};

#line 53 "..\\BALANCE\\DMP\\inv_mpu.h"

 
int mpu_init(void);
int mpu_init_slave(void);
int mpu_set_bypass(unsigned char bypass_on);

 
int mpu_lp_accel_mode(unsigned char rate);
int mpu_lp_motion_interrupt(unsigned short thresh, unsigned char time,
    unsigned char lpa_freq);
int mpu_set_int_level(unsigned char active_low);
int mpu_set_int_latched(unsigned char enable);

int mpu_set_dmp_state(unsigned char enable);
int mpu_get_dmp_state(unsigned char *enabled);

int mpu_get_lpf(unsigned short *lpf);
int mpu_set_lpf(unsigned short lpf);

int mpu_get_gyro_fsr(unsigned short *fsr);
int mpu_set_gyro_fsr(unsigned short fsr);

int mpu_get_accel_fsr(unsigned char *fsr);
int mpu_set_accel_fsr(unsigned char fsr);

int mpu_get_compass_fsr(unsigned short *fsr);

int mpu_get_gyro_sens(float *sens);
int mpu_get_accel_sens(unsigned short *sens);

int mpu_get_sample_rate(unsigned short *rate);
int mpu_set_sample_rate(unsigned short rate);
int mpu_get_compass_sample_rate(unsigned short *rate);
int mpu_set_compass_sample_rate(unsigned short rate);

int mpu_get_fifo_config(unsigned char *sensors);
int mpu_configure_fifo(unsigned char sensors);

int mpu_get_power_state(unsigned char *power_on);
int mpu_set_sensors(unsigned char sensors);

int mpu_set_accel_bias(const long *accel_bias);

 
int mpu_get_gyro_reg(short *data, unsigned long *timestamp);
int mpu_get_accel_reg(short *data, unsigned long *timestamp);
int mpu_get_compass_reg(short *data, unsigned long *timestamp);
int mpu_get_temperature(long *data, unsigned long *timestamp);

int mpu_get_int_status(short *status);
int mpu_read_fifo(short *gyro, short *accel, unsigned long *timestamp,
    unsigned char *sensors, unsigned char *more);
int mpu_read_fifo_stream(unsigned short length, unsigned char *data,
    unsigned char *more);
int mpu_reset_fifo(void);

int mpu_write_mem(unsigned short mem_addr, unsigned short length,
    unsigned char *data);
int mpu_read_mem(unsigned short mem_addr, unsigned short length,
    unsigned char *data);
int mpu_load_firmware(unsigned short length, const unsigned char *firmware,
    unsigned short start_addr, unsigned short sample_rate);

int mpu_reg_dump(void);
int mpu_read_reg(unsigned char reg, unsigned char *data);
int mpu_run_self_test(long *gyro, long *accel);
int mpu_register_tap_cb(void (*func)(unsigned char, unsigned char));
void myget_ms(unsigned long *time);


#line 114 "..\\SYSTEM\\sys\\sys.h"
#line 1 "..\\BALANCE\\DMP\\inv_mpu_dmp_motion_driver.h"










 








#line 26 "..\\BALANCE\\DMP\\inv_mpu_dmp_motion_driver.h"









#line 44 "..\\BALANCE\\DMP\\inv_mpu_dmp_motion_driver.h"



 
int dmp_load_motion_driver_firmware(void);
int dmp_set_fifo_rate(unsigned short rate);
int dmp_get_fifo_rate(unsigned short *rate);
int dmp_enable_feature(unsigned short mask);
int dmp_get_enabled_features(unsigned short *mask);
int dmp_set_interrupt_mode(unsigned char mode);
int dmp_set_orientation(unsigned short orient);
int dmp_set_gyro_bias(long *bias);
int dmp_set_accel_bias(long *bias);

 
int dmp_register_tap_cb(void (*func)(unsigned char, unsigned char));
int dmp_set_tap_thresh(unsigned char axis, unsigned short thresh);
int dmp_set_tap_axes(unsigned char axis);
int dmp_set_tap_count(unsigned char min_taps);
int dmp_set_tap_time(unsigned short time);
int dmp_set_tap_time_multi(unsigned short time);
int dmp_set_shake_reject_thresh(long sf, unsigned short thresh);
int dmp_set_shake_reject_time(unsigned short time);
int dmp_set_shake_reject_timeout(unsigned short time);

 
int dmp_register_android_orient_cb(void (*func)(unsigned char));

 
int dmp_enable_lp_quat(unsigned char enable);
int dmp_enable_6x_lp_quat(unsigned char enable);

 
int dmp_get_pedometer_step_count(unsigned long *count);
int dmp_set_pedometer_step_count(unsigned long count);
int dmp_get_pedometer_walk_time(unsigned long *time);
int dmp_set_pedometer_walk_time(unsigned long time);

 
int dmp_enable_gyro_cal(unsigned char enable);



 
int dmp_read_fifo(short *gyro, short *accel, long *quat,
    unsigned long *timestamp, short *sensors, unsigned char *more);



#line 115 "..\\SYSTEM\\sys\\sys.h"
#line 1 "..\\BALANCE\\DMP\\dmpKey.h"




 



#line 71 "..\\BALANCE\\DMP\\dmpKey.h"

 




#line 91 "..\\BALANCE\\DMP\\dmpKey.h"

#line 143 "..\\BALANCE\\DMP\\dmpKey.h"

#line 167 "..\\BALANCE\\DMP\\dmpKey.h"

 
#line 181 "..\\BALANCE\\DMP\\dmpKey.h"

 
#line 195 "..\\BALANCE\\DMP\\dmpKey.h"

#line 210 "..\\BALANCE\\DMP\\dmpKey.h"

 





 
#line 237 "..\\BALANCE\\DMP\\dmpKey.h"

 


 
#line 268 "..\\BALANCE\\DMP\\dmpKey.h"




 
#line 284 "..\\BALANCE\\DMP\\dmpKey.h"



typedef struct {
    unsigned short key;
    unsigned short addr;
} tKeyLabel;






#line 313 "..\\BALANCE\\DMP\\dmpKey.h"

#line 330 "..\\BALANCE\\DMP\\dmpKey.h"

#line 347 "..\\BALANCE\\DMP\\dmpKey.h"

#line 364 "..\\BALANCE\\DMP\\dmpKey.h"

#line 375 "..\\BALANCE\\DMP\\dmpKey.h"

#line 391 "..\\BALANCE\\DMP\\dmpKey.h"


#line 409 "..\\BALANCE\\DMP\\dmpKey.h"

#line 426 "..\\BALANCE\\DMP\\dmpKey.h"

#line 443 "..\\BALANCE\\DMP\\dmpKey.h"

#line 460 "..\\BALANCE\\DMP\\dmpKey.h"




#line 479 "..\\BALANCE\\DMP\\dmpKey.h"

#line 491 "..\\BALANCE\\DMP\\dmpKey.h"



#line 116 "..\\SYSTEM\\sys\\sys.h"
#line 1 "..\\BALANCE\\DMP\\dmpmap.h"




 








#line 117 "..\\SYSTEM\\sys\\sys.h"
#line 1 "H:\\KEIL MDK4\\ARM\\RV31\\INC\\string.h"
 
 
 
 




 








 











#line 37 "H:\\KEIL MDK4\\ARM\\RV31\\INC\\string.h"


  
  typedef unsigned int size_t;








extern __declspec(__nothrow) void *memcpy(void * __restrict  ,
                    const void * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) void *memmove(void *  ,
                    const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   







 
extern __declspec(__nothrow) char *strcpy(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncpy(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 

extern __declspec(__nothrow) char *strcat(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncat(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 






 

extern __declspec(__nothrow) int memcmp(const void *  , const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strcmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int strncmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcasecmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strncasecmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcoll(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   







 

extern __declspec(__nothrow) size_t strxfrm(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   













 


#line 184 "H:\\KEIL MDK4\\ARM\\RV31\\INC\\string.h"
extern __declspec(__nothrow) void *memchr(const void *  , int  , size_t  ) __attribute__((__nonnull__(1)));

   





 

#line 200 "H:\\KEIL MDK4\\ARM\\RV31\\INC\\string.h"
extern __declspec(__nothrow) char *strchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   




 

extern __declspec(__nothrow) size_t strcspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 

#line 223 "H:\\KEIL MDK4\\ARM\\RV31\\INC\\string.h"
extern __declspec(__nothrow) char *strpbrk(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   




 

#line 238 "H:\\KEIL MDK4\\ARM\\RV31\\INC\\string.h"
extern __declspec(__nothrow) char *strrchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   





 

extern __declspec(__nothrow) size_t strspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   



 

#line 261 "H:\\KEIL MDK4\\ARM\\RV31\\INC\\string.h"
extern __declspec(__nothrow) char *strstr(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   





 

extern __declspec(__nothrow) char *strtok(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) char *_strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

extern __declspec(__nothrow) char *strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

   

































 

extern __declspec(__nothrow) void *memset(void *  , int  , size_t  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) char *strerror(int  );
   





 
extern __declspec(__nothrow) size_t strlen(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) size_t strlcpy(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   
















 

extern __declspec(__nothrow) size_t strlcat(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






















 

extern __declspec(__nothrow) void _membitcpybl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpybb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
    














































 







#line 493 "H:\\KEIL MDK4\\ARM\\RV31\\INC\\string.h"



 
#line 118 "..\\SYSTEM\\sys\\sys.h"
#line 119 "..\\SYSTEM\\sys\\sys.h"
#line 120 "..\\SYSTEM\\sys\\sys.h"
#line 1 "H:\\KEIL MDK4\\ARM\\RV31\\INC\\stdlib.h"
 
 
 




 
 



 












  


 








#line 45 "H:\\KEIL MDK4\\ARM\\RV31\\INC\\stdlib.h"


  
  typedef unsigned int size_t;










    



    typedef unsigned short wchar_t;  
#line 74 "H:\\KEIL MDK4\\ARM\\RV31\\INC\\stdlib.h"

typedef struct div_t { int quot, rem; } div_t;
    
typedef struct ldiv_t { long int quot, rem; } ldiv_t;
    

typedef struct lldiv_t { __int64 quot, rem; } lldiv_t;
    


#line 95 "H:\\KEIL MDK4\\ARM\\RV31\\INC\\stdlib.h"
   



 

   




 
#line 114 "H:\\KEIL MDK4\\ARM\\RV31\\INC\\stdlib.h"
   


 
extern __declspec(__nothrow) int __aeabi_MB_CUR_MAX(void);

   




 

   




 




extern __declspec(__nothrow) double atof(const char *  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int atoi(const char *  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) long int atol(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) __int64 atoll(const char *  ) __attribute__((__nonnull__(1)));
   



 


extern __declspec(__nothrow) double strtod(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
   

















 

extern __declspec(__nothrow) float strtof(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) long double strtold(const char * __restrict  , char ** __restrict  ) __attribute__((__nonnull__(1)));
   

 

extern __declspec(__nothrow) long int strtol(const char * __restrict  ,
                        char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   



























 
extern __declspec(__nothrow) unsigned long int strtoul(const char * __restrict  ,
                                       char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   


























 

 
extern __declspec(__nothrow) __int64 strtoll(const char * __restrict  ,
                               char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) unsigned __int64 strtoull(const char * __restrict  ,
                                         char ** __restrict  , int  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) int rand(void);
   







 
extern __declspec(__nothrow) void srand(unsigned int  );
   






 

struct _rand_state { int __x[57]; };
extern __declspec(__nothrow) int _rand_r(struct _rand_state *);
extern __declspec(__nothrow) void _srand_r(struct _rand_state *, unsigned int);
struct _ANSI_rand_state { int __x[1]; };
extern __declspec(__nothrow) int _ANSI_rand_r(struct _ANSI_rand_state *);
extern __declspec(__nothrow) void _ANSI_srand_r(struct _ANSI_rand_state *, unsigned int);
   


 

extern __declspec(__nothrow) void *calloc(size_t  , size_t  );
   



 
extern __declspec(__nothrow) void free(void *  );
   





 
extern __declspec(__nothrow) void *malloc(size_t  );
   



 
extern __declspec(__nothrow) void *realloc(void *  , size_t  );
   













 

extern __declspec(__nothrow) int posix_memalign(void **  , size_t  , size_t  );
   









 

typedef int (*__heapprt)(void *, char const *, ...);
extern __declspec(__nothrow) void __heapstats(int (*  )(void *  ,
                                           char const *  , ...),
                        void *  ) __attribute__((__nonnull__(1)));
   










 
extern __declspec(__nothrow) int __heapvalid(int (*  )(void *  ,
                                           char const *  , ...),
                       void *  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) __declspec(__noreturn) void abort(void);
   







 

extern __declspec(__nothrow) int atexit(void (*  )(void)) __attribute__((__nonnull__(1)));
   




 
#line 414 "H:\\KEIL MDK4\\ARM\\RV31\\INC\\stdlib.h"


extern __declspec(__nothrow) __declspec(__noreturn) void exit(int  );
   












 

extern __declspec(__nothrow) __declspec(__noreturn) void _Exit(int  );
   







      

extern __declspec(__nothrow) char *getenv(const char *  ) __attribute__((__nonnull__(1)));
   









 

extern __declspec(__nothrow) int  system(const char *  );
   









 

extern  void *bsearch(const void *  , const void *  ,
              size_t  , size_t  ,
              int (*  )(const void *, const void *)) __attribute__((__nonnull__(1,2,5)));
   












 
#line 502 "H:\\KEIL MDK4\\ARM\\RV31\\INC\\stdlib.h"


extern  void qsort(void *  , size_t  , size_t  ,
           int (*  )(const void *, const void *)) __attribute__((__nonnull__(1,4)));
   









 

#line 531 "H:\\KEIL MDK4\\ARM\\RV31\\INC\\stdlib.h"

extern __declspec(__nothrow) __pure int abs(int  );
   



 

extern __declspec(__nothrow) __pure div_t div(int  , int  );
   









 
extern __declspec(__nothrow) __pure long int labs(long int  );
   



 




extern __declspec(__nothrow) __pure ldiv_t ldiv(long int  , long int  );
   











 







extern __declspec(__nothrow) __pure __int64 llabs(__int64  );
   



 




extern __declspec(__nothrow) __pure lldiv_t lldiv(__int64  , __int64  );
   











 
#line 612 "H:\\KEIL MDK4\\ARM\\RV31\\INC\\stdlib.h"



 
typedef struct __sdiv32by16 { int quot, rem; } __sdiv32by16;
typedef struct __udiv32by16 { unsigned int quot, rem; } __udiv32by16;
    
typedef struct __sdiv64by32 { int rem, quot; } __sdiv64by32;

__value_in_regs extern __declspec(__nothrow) __pure __sdiv32by16 __rt_sdiv32by16(
     int  ,
     short int  );
   

 
__value_in_regs extern __declspec(__nothrow) __pure __udiv32by16 __rt_udiv32by16(
     unsigned int  ,
     unsigned short  );
   

 
__value_in_regs extern __declspec(__nothrow) __pure __sdiv64by32 __rt_sdiv64by32(
     int  , unsigned int  ,
     int  );
   

 



 
extern __declspec(__nothrow) unsigned int __fp_status(unsigned int  , unsigned int  );
   







 























 
extern __declspec(__nothrow) int mblen(const char *  , size_t  );
   












 
extern __declspec(__nothrow) int mbtowc(wchar_t * __restrict  ,
                   const char * __restrict  , size_t  );
   















 
extern __declspec(__nothrow) int wctomb(char *  , wchar_t  );
   













 





 
extern __declspec(__nothrow) size_t mbstowcs(wchar_t * __restrict  ,
                      const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   














 
extern __declspec(__nothrow) size_t wcstombs(char * __restrict  ,
                      const wchar_t * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   














 

extern __declspec(__nothrow) void __use_realtime_heap(void);
extern __declspec(__nothrow) void __use_realtime_division(void);
extern __declspec(__nothrow) void __use_two_region_memory(void);
extern __declspec(__nothrow) void __use_no_heap(void);
extern __declspec(__nothrow) void __use_no_heap_region(void);

extern __declspec(__nothrow) char const *__C_library_version_string(void);
extern __declspec(__nothrow) int __C_library_version_number(void);











#line 866 "H:\\KEIL MDK4\\ARM\\RV31\\INC\\stdlib.h"


 
#line 121 "..\\SYSTEM\\sys\\sys.h"
#line 122 "..\\SYSTEM\\sys\\sys.h"
#line 1 "H:\\KEIL MDK4\\ARM\\RV31\\INC\\math.h"




 





 












 







 






#line 47 "H:\\KEIL MDK4\\ARM\\RV31\\INC\\math.h"

#line 61 "H:\\KEIL MDK4\\ARM\\RV31\\INC\\math.h"

   




 















 
#line 92 "H:\\KEIL MDK4\\ARM\\RV31\\INC\\math.h"











 





extern __softfp unsigned __ARM_dcmp4(double  , double  );
extern __softfp unsigned __ARM_fcmp4(float  , float  );
    




 

extern __declspec(__nothrow) __softfp int __ARM_fpclassifyf(float  );
extern __declspec(__nothrow) __softfp int __ARM_fpclassify(double  );
     
     

__inline __declspec(__nothrow) __softfp int __ARM_isfinitef(float __x)
{
    return (((*(unsigned *)&(__x)) >> 23) & 0xff) != 0xff;
}
__inline __declspec(__nothrow) __softfp int __ARM_isfinite(double __x)
{
    return (((*(1 + (unsigned *)&(__x))) >> 20) & 0x7ff) != 0x7ff;
}
     
     

__inline __declspec(__nothrow) __softfp int __ARM_isinff(float __x)
{
    return ((*(unsigned *)&(__x)) << 1) == 0xff000000;
}
__inline __declspec(__nothrow) __softfp int __ARM_isinf(double __x)
{
    return (((*(1 + (unsigned *)&(__x))) << 1) == 0xffe00000) && ((*(unsigned *)&(__x)) == 0);
}
     
     

__inline __declspec(__nothrow) __softfp int __ARM_islessgreaterf(float __x, float __y)
{
    unsigned __f = __ARM_fcmp4(__x, __y) >> 28;
    return (__f == 8) || (__f == 2);  
}
__inline __declspec(__nothrow) __softfp int __ARM_islessgreater(double __x, double __y)
{
    unsigned __f = __ARM_dcmp4(__x, __y) >> 28;
    return (__f == 8) || (__f == 2);  
}
    


 

__inline __declspec(__nothrow) __softfp int __ARM_isnanf(float __x)
{
    return (0x7f800000 - ((*(unsigned *)&(__x)) & 0x7fffffff)) >> 31;
}
__inline __declspec(__nothrow) __softfp int __ARM_isnan(double __x)
{
    unsigned __xf = (*(1 + (unsigned *)&(__x))) | (((*(unsigned *)&(__x)) == 0) ? 0 : 1);
    return (0x7ff00000 - (__xf & 0x7fffffff)) >> 31;
}
     
     

__inline __declspec(__nothrow) __softfp int __ARM_isnormalf(float __x)
{
    unsigned __xe = ((*(unsigned *)&(__x)) >> 23) & 0xff;
    return (__xe != 0xff) && (__xe != 0);
}
__inline __declspec(__nothrow) __softfp int __ARM_isnormal(double __x)
{
    unsigned __xe = ((*(1 + (unsigned *)&(__x))) >> 20) & 0x7ff;
    return (__xe != 0x7ff) && (__xe != 0);
}
     
     

__inline __declspec(__nothrow) __softfp int __ARM_signbitf(float __x)
{
    return (*(unsigned *)&(__x)) >> 31;
}
__inline __declspec(__nothrow) __softfp int __ARM_signbit(double __x)
{
    return (*(1 + (unsigned *)&(__x))) >> 31;
}
     
     








#line 210 "H:\\KEIL MDK4\\ARM\\RV31\\INC\\math.h"



   
  typedef float float_t;
  typedef double double_t;







extern const int math_errhandling;



extern __declspec(__nothrow) double acos(double  );
    
    
    
extern __declspec(__nothrow) double asin(double  );
    
    
    
    

extern __declspec(__nothrow) __pure double atan(double  );
    
    

extern __declspec(__nothrow) double atan2(double  , double  );
    
    
    
    

extern __declspec(__nothrow) double cos(double  );
    
    
    
    
extern __declspec(__nothrow) double sin(double  );
    
    
    
    

extern void __use_accurate_range_reduction(void);
    
    

extern __declspec(__nothrow) double tan(double  );
    
    
    
    

extern __declspec(__nothrow) double cosh(double  );
    
    
    
    
extern __declspec(__nothrow) double sinh(double  );
    
    
    
    
    

extern __declspec(__nothrow) __pure double tanh(double  );
    
    

extern __declspec(__nothrow) double exp(double  );
    
    
    
    
    

extern __declspec(__nothrow) double frexp(double  , int *  ) __attribute__((__nonnull__(2)));
    
    
    
    
    
    

extern __declspec(__nothrow) double ldexp(double  , int  );
    
    
    
    
extern __declspec(__nothrow) double log(double  );
    
    
    
    
    
extern __declspec(__nothrow) double log10(double  );
    
    
    
extern __declspec(__nothrow) double modf(double  , double *  ) __attribute__((__nonnull__(2)));
    
    
    
    

extern __declspec(__nothrow) double pow(double  , double  );
    
    
    
    
    
    
extern __declspec(__nothrow) double sqrt(double  );
    
    
    




    __inline double _sqrt(double __x) { return sqrt(__x); }




    __inline float _sqrtf(float __x) { return (float)sqrt(__x); }

    



 

extern __declspec(__nothrow) __pure double ceil(double  );
    
    
extern __declspec(__nothrow) __pure double fabs(double  );
    
    

extern __declspec(__nothrow) __pure double floor(double  );
    
    

extern __declspec(__nothrow) double fmod(double  , double  );
    
    
    
    
    

    









 



extern __declspec(__nothrow) double acosh(double  );
    

 
extern __declspec(__nothrow) double asinh(double  );
    

 
extern __declspec(__nothrow) double atanh(double  );
    

 
extern __declspec(__nothrow) double cbrt(double  );
    

 
__inline __declspec(__nothrow) __pure double copysign(double __x, double __y)
    

 
{
    (*(1 + (unsigned *)&(__x))) = ((*(1 + (unsigned *)&(__x))) & 0x7fffffff) | ((*(1 + (unsigned *)&(__y))) & 0x80000000);
    return __x;
}
__inline __declspec(__nothrow) __pure float copysignf(float __x, float __y)
    

 
{
    (*(unsigned *)&(__x)) = ((*(unsigned *)&(__x)) & 0x7fffffff) | ((*(unsigned *)&(__y)) & 0x80000000);
    return __x;
}
extern __declspec(__nothrow) double erf(double  );
    

 
extern __declspec(__nothrow) double erfc(double  );
    

 
extern __declspec(__nothrow) double expm1(double  );
    

 



    

 






#line 444 "H:\\KEIL MDK4\\ARM\\RV31\\INC\\math.h"


extern __declspec(__nothrow) double hypot(double  , double  );
    




 
extern __declspec(__nothrow) __pure int ilogb(double  );
    

 
extern __declspec(__nothrow) __pure int ilogbf(float  );
    

 
extern __declspec(__nothrow) __pure int ilogbl(long double  );
    

 







    

 





    



 





    



 





    

 





    



 





    



 





    



 





    

 





    

 





    


 

extern __declspec(__nothrow) double lgamma (double  );
    


 
extern __declspec(__nothrow) double log1p(double  );
    

 
extern __declspec(__nothrow) __pure double logb(double  );
    

 
extern __declspec(__nothrow) __pure float logbf(float  );
    

 
extern __declspec(__nothrow) __pure long double logbl(long double  );
    

 
extern __declspec(__nothrow) __pure double nextafter(double  , double  );
    


 
extern __declspec(__nothrow) __pure float nextafterf(float  , float  );
    


 
extern __declspec(__nothrow) __pure long double nextafterl(long double  , long double  );
    


 
extern __declspec(__nothrow) __pure double nexttoward(double  , long double  );
    


 
extern __declspec(__nothrow) __pure float nexttowardf(float  , long double  );
    


 
extern __declspec(__nothrow) __pure long double nexttowardl(long double  , long double  );
    


 
extern __declspec(__nothrow) __pure double remainder(double  , double  );
    

 
extern __declspec(__nothrow) __pure double rint(double  );
    

 
extern __declspec(__nothrow) __pure double scalbln(double  , long int  );
    

 
extern __declspec(__nothrow) __pure float scalblnf(float  , long int  );
    

 
extern __declspec(__nothrow) __pure long double scalblnl(long double  , long int  );
    

 
extern __declspec(__nothrow) __pure double scalbn(double  , int  );
    

 
extern __declspec(__nothrow) __pure float scalbnf(float  , int  );
    

 
extern __declspec(__nothrow) __pure long double scalbnl(long double  , int  );
    

 




    

 



 
extern __declspec(__nothrow) __pure float _fabsf(float);  
__inline __declspec(__nothrow) __pure float fabsf(float __f) { return _fabsf(__f); }
extern __declspec(__nothrow) float sinf(float  );
extern __declspec(__nothrow) float cosf(float  );
extern __declspec(__nothrow) float tanf(float  );
extern __declspec(__nothrow) float acosf(float  );
extern __declspec(__nothrow) float asinf(float  );
extern __declspec(__nothrow) float atanf(float  );
extern __declspec(__nothrow) float atan2f(float  , float  );
extern __declspec(__nothrow) float sinhf(float  );
extern __declspec(__nothrow) float coshf(float  );
extern __declspec(__nothrow) float tanhf(float  );
extern __declspec(__nothrow) float expf(float  );
extern __declspec(__nothrow) float logf(float  );
extern __declspec(__nothrow) float log10f(float  );
extern __declspec(__nothrow) float powf(float  , float  );
extern __declspec(__nothrow) float sqrtf(float  );
extern __declspec(__nothrow) float ldexpf(float  , int  );
extern __declspec(__nothrow) float frexpf(float  , int *  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) __pure float ceilf(float  );
extern __declspec(__nothrow) __pure float floorf(float  );
extern __declspec(__nothrow) float fmodf(float  , float  );
extern __declspec(__nothrow) float modff(float  , float *  ) __attribute__((__nonnull__(2)));

 
 













 
__declspec(__nothrow) long double acosl(long double );
__declspec(__nothrow) long double asinl(long double );
__declspec(__nothrow) long double atanl(long double );
__declspec(__nothrow) long double atan2l(long double , long double );
__declspec(__nothrow) long double ceill(long double );
__declspec(__nothrow) long double cosl(long double );
__declspec(__nothrow) long double coshl(long double );
__declspec(__nothrow) long double expl(long double );
__declspec(__nothrow) long double fabsl(long double );
__declspec(__nothrow) long double floorl(long double );
__declspec(__nothrow) long double fmodl(long double , long double );
__declspec(__nothrow) long double frexpl(long double , int* ) __attribute__((__nonnull__(2)));
__declspec(__nothrow) long double ldexpl(long double , int );
__declspec(__nothrow) long double logl(long double );
__declspec(__nothrow) long double log10l(long double );
__declspec(__nothrow) long double modfl(long double  , long double *  ) __attribute__((__nonnull__(2)));
__declspec(__nothrow) long double powl(long double , long double );
__declspec(__nothrow) long double sinl(long double );
__declspec(__nothrow) long double sinhl(long double );
__declspec(__nothrow) long double sqrtl(long double );
__declspec(__nothrow) long double tanl(long double );
__declspec(__nothrow) long double tanhl(long double );





 
extern __declspec(__nothrow) float acoshf(float  );
__declspec(__nothrow) long double acoshl(long double );
extern __declspec(__nothrow) float asinhf(float  );
__declspec(__nothrow) long double asinhl(long double );
extern __declspec(__nothrow) float atanhf(float  );
__declspec(__nothrow) long double atanhl(long double );
__declspec(__nothrow) long double copysignl(long double , long double );
extern __declspec(__nothrow) float cbrtf(float  );
__declspec(__nothrow) long double cbrtl(long double );
extern __declspec(__nothrow) float erff(float  );
__declspec(__nothrow) long double erfl(long double );
extern __declspec(__nothrow) float erfcf(float  );
__declspec(__nothrow) long double erfcl(long double );
extern __declspec(__nothrow) float expm1f(float  );
__declspec(__nothrow) long double expm1l(long double );
extern __declspec(__nothrow) float log1pf(float  );
__declspec(__nothrow) long double log1pl(long double );
extern __declspec(__nothrow) float hypotf(float  , float  );
__declspec(__nothrow) long double hypotl(long double , long double );
extern __declspec(__nothrow) float lgammaf(float  );
__declspec(__nothrow) long double lgammal(long double );
extern __declspec(__nothrow) float remainderf(float  , float  );
__declspec(__nothrow) long double remainderl(long double , long double );
extern __declspec(__nothrow) float rintf(float  );
__declspec(__nothrow) long double rintl(long double );



#line 824 "H:\\KEIL MDK4\\ARM\\RV31\\INC\\math.h"





#line 979 "H:\\KEIL MDK4\\ARM\\RV31\\INC\\math.h"











#line 1181 "H:\\KEIL MDK4\\ARM\\RV31\\INC\\math.h"



 
#line 123 "..\\SYSTEM\\sys\\sys.h"












#line 4 "..\\HARDWARE\\MOTOR\\motor.h"
  


 
#line 14 "..\\HARDWARE\\MOTOR\\motor.h"
void MiniBalance_PWM_Init(u16 arr,u16 psc);
void MiniBalance_Motor_Init(void);
#line 2 "..\\HARDWARE\\MOTOR\\motor.c"
  


 
void MiniBalance_Motor_Init(void)
{
	((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x1000))->APB2ENR|=1<<3;       
	((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x0C00))->CRH&=0X0000FFFF;   
	((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x0C00))->CRH|=0X22220000;   
}
void MiniBalance_PWM_Init(u16 arr,u16 psc)
{		 					 
	MiniBalance_Motor_Init();  
	((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x1000))->APB2ENR|=1<<11;       
	((RCC_TypeDef *) ((((uint32_t)0x40000000) + 0x20000) + 0x1000))->APB2ENR|=1<<2;        
	((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x0800))->CRH&=0XFFFF0FF0;    
	((GPIO_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x0800))->CRH|=0X0000B00B;    
	((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))->ARR=arr;             
	((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))->PSC=psc;             
	((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))->CCMR2|=6<<12;        
	((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))->CCMR1|=6<<4;         
	((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))->CCMR2|=1<<11;        
	((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))->CCMR1|=1<<3;         
	((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))->CCER|=1<<12;         
	((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))->CCER|=1<<0;          
	((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))->BDTR |= 1<<15;       
	((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))->CR1=0x8000;          
	((TIM_TypeDef *) ((((uint32_t)0x40000000) + 0x10000) + 0x2C00))->CR1|=0x01;          
} 

