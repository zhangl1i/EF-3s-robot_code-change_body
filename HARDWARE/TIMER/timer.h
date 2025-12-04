#ifndef _TIMER_H
#define _TIMER_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
//ALIENTEK STM32H7������
//��ʱ����������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2017/8/12
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) �������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
extern TIM_HandleTypeDef TIM5_Handler;      //��ʱ��3PWM��� 
extern TIM_HandleTypeDef TIM2_Handler;      //��ʱ��3PWM��� 
extern TIM_HandleTypeDef TIM7_Handler;      //��ʱ��3PWM��� 
extern TIM_OC_InitTypeDef TIM2_CH2Handler;
//extern struct CreepMotionControl mc;
extern int TIM5_20msFlag, TIM2_100msFlag;

extern float T;
extern float s;
extern float delta_t;
extern float times;
extern u8 movement_flag;
extern u8 temp_movement_flag;
void TIM5_Init(u16 arr,u16 psc);    //��ʱ����ʼ��
void TIM2_Init(u16 arr,u16 psc);    //��ʱ����ʼ��
void TIM7_Init(u16 arr,u16 psc);    //��ʱ����ʼ��
#endif

