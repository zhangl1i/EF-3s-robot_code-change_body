#include "sys.h"
#include <stdlib.h>
#include "string.h"
#include "VOFA.h"
#include "delay.h"
#include "uart.h" 
#include "pwm.h"
#include "timer.h"
#include "adc.h"
#include "can.h"
#include "gait.h"
#include "SV.h"
#include "matrix.h"
#include "24l01.h"


u8 tmp_buf[32];
extern bool isProbingMode;
extern bool isSecendTrial;
extern bool isSecendUp;
extern uint16_t pressureThreshold[4];
int legSuction[4];
int main(void){
        Cache_Enable();                 
    HAL_Init();				        	
    Stm32_Clock_Init(160,5,2,4);  	    //设置时钟,400Mhz 4分频
    delay_init(400);		
    // FDCAN1_Mode_Init(5,8,31,8,FDCAN_MODE_NORMAL);  
    HAL_Delay(100);	           
    
    SV_Init();
    //pid_value_init();
    // TIM16_PWM_Init(200-1,10000-1);//pump
    
    TIM1_PWM_Init(10000-1,30-1);	// 100M/30=3333k的计数频率, ARR自动重装载为10000, 那么PWM频率为3333k/10k=333HZ
    TIM3_PWM_Init(10000-1,30-1); 
    TIM4_PWM_Init(10000-1,30-1); 
    TIM12_PWM_Init(10000-1,30-1); 
    TIM15_PWM_Init(10000-1,30-1);
    
    MY_UART7_Init();

    CLASSMC_defMatrix(&transition);
    CLASSMC_initiation(&transition, 3);
    transition.times = -1;
    CLASSMC_defMatrix(&mc);
    CLASSMC_initiation(&mc, gaitModeBuffer);	
    targetCoMVelocityBuffer[0] = 0;		// X        0x02：trot_v=9	 0x03：amble_v=3 or 2
    targetCoMVelocityBuffer[1] = -2;	// y = 2
    targetCoMVelocityBuffer[2] = 0;//0 *3.1415/180.0;	// alpha (radian)
    CLASSMC_setCoMVel(&mc, targetCoMVelocityBuffer);	
    CLASSMC_inverseKinematics(&mc);
    CLASSMC_setJointPosition(&mc);	
    runFlag = 01;
    controlRunFlag=01;
    MX_ADC1_Init();
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);//vcc_led
    SV_ALL_N();
    //SV_ALL_P();//正压
    //SV_ALL_CLOSE();
    //LF_N_OPEN;RH_N_OPEN;
    HAL_Delay(2000);
    
    //TIM2_Init(10000-1,1000-1);	//	10hz
    TIM5_Init(10000-1,200-1);		// 	50hz
    for(int i=0;i<4;i++)
        sendAdhesionOnInf(i);
    while(1)
    {			
        
        if(TIM5_20msFlag >= 1)
        {
            Read_All_Ad();
					
					// ============== 发送air pressure to raspberry pi ==============
    // 定义一个缓冲区，50字节足够塞下4个整数了
    char tx_buf[64]; 
    
    // 格式化数据：加个包头 "$PRS" (Pressure)，中间用逗号分隔，最后换行
    // 这样树莓派那边好解析：split一下就行
    // adc[0]-LF, adc[1]-RF, adc[2]-LH, adc[3]-RH (根据你adc.c里的赋值顺序推断)
    sprintf(tx_buf, "$PRS,%d,%d,%d,%d\n", adc[0], adc[1], adc[2], adc[3]);
    
    // 通过 UART7 发送
    // 注意：你的 uart.c 里初始化的是 huart7
    // 超时时间设个 10ms 够了，别阻塞主循环太久
    HAL_UART_Transmit(&huart7, (uint8_t*)tx_buf, strlen(tx_buf), 10);
    // ============== end==============   
            TIM5_20msFlag = 0;
            if(runFlag == 1)
            {
                if(controlRunFlag==1)
                {
                        CLASSMC_setCoMVel(&mc, targetCoMVelocityBuffer);
                        CLASSMC_nextStep(&mc);
                        runOneStep(&mc);
                }
                else
                {
                    
                    for(int i=0;i<4;i++)
                    {
                        if(adc[i]<pressureThreshold[i])
                        mc.isSuction[i]++;
                        else 
                        mc.isSuction[i]=0;
                        if(mc.isSuction[i]>=3)
                            legSuction[i]=1;
                        else
                            legSuction[i]=0;
                    }
                    if((legSuction[0]+legSuction[1]+legSuction[2]+legSuction[3])==4)
                    {
                        controlRunFlag=1;
                        for(int i=0;i<4;i++)
                             sendAdhesionOnInf(i);
                    }
                }
            }

            
        
        }
    }// while(1)
    CLASSMC_freeMatrix(&mc);
    CLASSMC_freeMatrix(&transition);
    
}
