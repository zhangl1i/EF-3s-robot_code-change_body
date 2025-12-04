/*
 * VOFA.c
 *
 *  Created on: 2020年8月14日
 *      Author: WHY
 */
#include "VOFA.h"
#include "gait.h"

float my_data_buffer[20] = {0};//待发送浮点数据缓冲区
uint8_t Send_channels,ii;
extern int imitationSuction;
uint8_t float_to_uint8(float num, uint8_t bit)
{
	uint8_t* temp;

	temp = (uint8_t *) &num;
	return  *(temp + bit);
}
float uint8_to_float(uint8_t* p)
{
	float temp;
	*((uint8_t*)&temp) = *p;
	*((uint8_t*)&temp+1) = *(p+1);
	*((uint8_t*)&temp+2) = *(p+2);
	*((uint8_t*)&temp+3) = *(p+3);
	return temp;
}
//串口1中断后需要执行的函数
int8_t *recv_int8;
void example_uart_callback(uint8_t userData)
{
  static uint8_t state = 0,arg_num = 0,recv_char[4];
  float recv_float;

	//volt+接受状态机
	switch(state)
	{
		case 0:
			if(userData == 0x23)//帧头第一字节
				state = 1;
			else
				state = 0;
			break;
		case 1:
			if(userData == 0x02)//帧头第二字节
				state = 2;
			else
				state = 0;
			break;
		case 2:
			arg_num = userData;//数据类别
			state = 3;
			break;
		case 3:
			recv_char[0] = userData;//数据第一字节
			state = 4;
			break;
		case 4:
			recv_char[1] = userData;//数据第二字节
			state = 5;
			break;
		case 5:
			recv_char[2] = userData;//数据第三字节
			state = 6;
			break;
		case 6:
			recv_char[3] = userData;//数据第四字节
			state = 7;
			break;
		case 7:
			if(userData == 0xaa)//帧尾
			{
				recv_float = uint8_to_float(recv_char);//四字节转化为浮点数
				updateStatus = 1;
				statusNum = arg_num;
				switch(arg_num)
				{
					case 0:
						if(recv_float !=
							0)
							runFlag = 1;
						else
							runFlag = 0;
						break;
					case 1:		//	targetCoMVelocity		x
						targetCoMVelocityBuffer[0] = recv_float;
						if(targetCoMVelocityBuffer[0]<0.1&&targetCoMVelocityBuffer[0]>-0.1)
							targetCoMVelocityBuffer[0]=0;
//						if(recv_float>0.05)
//						targetCoMVelocityBuffer[0] = 2.0;
//						else if(recv_float<-0.05)targetCoMVelocityBuffer[0] = -2.0;
//						else targetCoMVelocityBuffer[0] = 0;
						break;
					case 2:		//	targetCoMVelocity		y
						targetCoMVelocityBuffer[1] = recv_float;
							if(targetCoMVelocityBuffer[1]<0.1&&targetCoMVelocityBuffer[1]>-0.1)
							targetCoMVelocityBuffer[1]=0;
						int sdadsa=0;
//						if(recv_float>0.05)
//						targetCoMVelocityBuffer[1] = 2.0;
//						else if(recv_float<-0.05)targetCoMVelocityBuffer[1] = -2.0;
//						else targetCoMVelocityBuffer[1] = 0;
						break;
					case 3:	//	targetCoMVelocity		alpha 
						targetCoMVelocityBuffer[2] = recv_float; //	radian
							if(targetCoMVelocityBuffer[2]<0.001&&targetCoMVelocityBuffer[2]>-0.001)
							targetCoMVelocityBuffer[2]=0;
						break;
					case 4:	//	timeGait
						timeGaitBuffer = recv_float;
						break;
					case 5:	//	the pressure height in the end of swing phase
						for(int i=0; i<4; i++)	//	LF RF LH RH
							pressHightBuffer[i]= (float)recv_char[i];
						break;
					case 6: //	init gaitMode with transitionStep
						gaitModeBuffer = recv_char[3];
						break;
					case 7: //	init gaitMode at once 
						gaitModeBuffer = recv_char[3];
						break;
					case 8: //	adjust parameters of ftsZeroSet
						static int status8Times;
						switch (status8Times)
						{
						case 0:
							for(int i=0; i<4; i++)
							{
								recv_int8 = &recv_char[i];
								ftsZeroSet[i*3] = (float) *recv_int8;
							}
							status8Times = 1;
							break;
						case 1:
							for(int i=0; i<4; i++)
							{
								recv_int8 = &recv_char[i];
								ftsZeroSet[i*3 +1] = (float) *recv_int8;
							}
							status8Times = 2;
							break;
						case 2:
							for(int i=0; i<4; i++)
							{
								recv_int8 = &recv_char[i];
								ftsZeroSet[i*3 +2] = (float) *recv_int8;
							}
							status8Times = 0;
							break;
						default:
							status8Times=0;
							break;
						}
						break;
						case 9: //	control air pump when stoping
							switch (recv_char[3])
							{
							case 0:
								SV_ALL_N();
								break;
							case 1:
								SV_ALL_P();
								break;
							case 2:
								SV_ALL_CLOSE();
								break;
							default:
								break;
							}
						break;
					case 0x0A:
						int chosendLeg=(int)recv_char[0];
						int selectedDirection=(int)recv_char[1];
						mc.ftsPos.element[chosendLeg][selectedDirection]+=(float)recv_char[2];
						mc.ftsPos.element[chosendLeg][selectedDirection]-=(float)recv_char[3];
						manualControl=1;
						case 0x10:
						legv_rate[0] = recv_float;
						break;
					case 0x11:
						legv_rate[1] = recv_float;
						break;
					case 0x12:
						legv_rate[2] = recv_float;
						break;
					case 0x13:
						legv_rate[3] = recv_float;
						break;
					break;
				}
			}
			state = 0; //状态复位，等待下一次接收
			break;
	}
}

//更新待发送浮点数据缓冲区
void store_my_data_buffer(void)
{
	my_data_buffer[0] = gaitModeBuffer;//mc.targetCoMVelocity.element[0][0];
	// my_data_buffer[1] = 666.6;
//	my_data_buffer[2] = realSpeed30;
//	my_data_buffer[3] = realSpeed40;
//	my_data_buffer[4] = pid2.kp;
//	my_data_buffer[5] = pid2.ki;
//	my_data_buffer[6] = XY.y;
//	my_data_buffer[7] = distance00;
//	my_data_buffer[8] = distance11;

	Send_channels=4;	// max 6
}

//volt+上位机发送函数
//buffer_addr  	  发送float数据地址
//channels     	  发送float数据通道数
//my_data_buffer 待发送浮点数据缓冲区
void vofa_send_lines(void)
{
	uint32_t j,k = 0;//i = 1,
	uint8_t buffer_to_send[100];

	ii=1;

	store_my_data_buffer();//更新待发送浮点数据缓冲区
	while(k < Send_channels)
	{
		for(j = 0;j < 4;j++)
		{
			buffer_to_send[ii] = float_to_uint8(* (my_data_buffer + k),j);
			ii++;
		}
		k++;
	}

    //四字节帧尾
	buffer_to_send[ii++] = 0x00;
	buffer_to_send[ii++] = 0x00;
	buffer_to_send[ii++] = 0x80;
	buffer_to_send[ii++] = 0x7f;
	
	NRF24L01_TX_Mode();
	//HAL_UART_Transmit(&huart7,buffer_to_send,i,0xffff);//sizeof(buffer_to_send)
	NRF24L01_TxPacket(buffer_to_send);
}

void sendMessageToQT(u8 *txbuf)
{
	u8 send_buf[32];
	for(int i=0;i<9;i++)
		send_buf[i]=*(txbuf+i);
	NRF24L01_TX_Mode();
	NRF24L01_TxPacket(send_buf);
	NRF24L01_RX_Mode();

}
void sendAdhesionOnInf(int legNum)
{
	if(legNum==0)	sendMessageToQT("1LFON");
	if(legNum==1)	sendMessageToQT("1RFON");
	if(legNum==2)	sendMessageToQT("1LHON");
	if(legNum==3)	sendMessageToQT("1RHON");
}