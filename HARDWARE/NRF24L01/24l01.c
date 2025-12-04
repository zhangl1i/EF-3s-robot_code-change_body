#include "24l01.h"
#include "spi.h"
#include "delay.h"
////////////////////////////////////////////////////////////
/*
NRF24l01 ͨ������
����������ݿ�����ͬ�����32���ֽڣ�
������յ�ַ��ͬ��5��8λ��ַ��
�������Ƶ����ͬ��0~125��
�������������ͬ��2M 1M 250K��
seekfree RF�ŵ�Ϊ100;TX_PLOAD_WIDTH,RX_PLOAD_WIDTH 31(��һλϵͳ����λ)
*/
////////////////////////////////////////////////////////////////////////////////////
const u8 TX_ADDRESS[TX_ADR_WIDTH]={0x51,0x08,0xFF,0xFF,0x02}; //���͵�ַ
const u8 RX_ADDRESS[RX_ADR_WIDTH]={0x51,0x08,0xFF,0xFF,0x02}; //���͵�ַ
//const u8 TX_ADDRESS[TX_ADR_WIDTH]={0x51,0x08,0xFF,0x01,0x01}; //���͵�ַ
//const u8 RX_ADDRESS[RX_ADR_WIDTH]={0x51,0x08,0xFF,0x01,0x01}; //���͵�ַ   //no1


//���NRF24L01�޸�SPI3����
void NRF24L01_SPI_Init(void)
{
    __HAL_SPI_DISABLE(&SPI3_Handler);               //�ȹر�SPI3
    SPI3_Handler.Init.CLKPolarity=SPI_POLARITY_LOW; //����ͬ��ʱ�ӵĿ���״̬Ϊ�͵�ƽ
    SPI3_Handler.Init.CLKPhase=SPI_PHASE_1EDGE;     //����ͬ��ʱ�ӵĵ�1�������أ��������½������ݱ�����
    HAL_SPI_Init(&SPI3_Handler);
    __HAL_SPI_ENABLE(&SPI3_Handler);                //ʹ��SPI3
}

//��ʼ��24L01��IO��
void NRF24L01_Init(void)
{
	GPIO_InitTypeDef GPIO_Initure;
	__HAL_RCC_GPIOA_CLK_ENABLE();			//����GPIOAʱ��
	
	/*	PCB_3.3		*/
	GPIO_Initure.Pin=GPIO_PIN_15|GPIO_PIN_9; //PA
	GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //�������
	GPIO_Initure.Pull=GPIO_PULLUP;          //����
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;     //����
	HAL_GPIO_Init(GPIOA,&GPIO_Initure);     //��ʼ��
	 
	GPIO_Initure.Pin=GPIO_PIN_10;           //PA10
	GPIO_Initure.Mode=GPIO_MODE_INPUT;      //����
	HAL_GPIO_Init(GPIOA,&GPIO_Initure);     //��ʼ��

	/*	PCB_3.4		*/
	__HAL_RCC_GPIOD_CLK_ENABLE();
	GPIO_Initure.Pin=GPIO_PIN_15; //PD
	GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //�������
	GPIO_Initure.Pull=GPIO_PULLUP;          //����
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;     //����
	HAL_GPIO_Init(GPIOD,&GPIO_Initure);     //��ʼ��
	
	SPI3_Init();    		                //��ʼ��SPI3  
	NRF24L01_SPI_Init();                    //���NRF���ص��޸�SPI������
	SPI3_SetSpeed(SPI_BAUDRATEPRESCALER_16);
	NRF24L01_CE(0); 			            //ʹ��24L01
	NRF24L01_CSN(1);			            //SPIƬѡȡ��
	
	NRF24L01_Write_Buf(NRF_WRITE_REG | TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);//дTX�ڵ��ַ 
	NRF24L01_Write_Buf(NRF_WRITE_REG | RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK	  

	NRF24L01_Write_Reg(NRF_WRITE_REG | EN_AA,0x01);     //ʹ��ͨ��0���Զ�Ӧ��   
	NRF24L01_Write_Reg(NRF_WRITE_REG | EN_RXADDR,0x01); //ʹ��ͨ��0�Ľ��յ�ַ  
	NRF24L01_Write_Reg(NRF_WRITE_REG | SETUP_RETR,0x1a);//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
	NRF24L01_Write_Reg(NRF_WRITE_REG | RF_CH,100);       //����RFͨ��Ϊ100
	NRF24L01_Write_Reg(NRF_WRITE_REG | RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ���
	NRF24L01_Write_Reg(NRF_WRITE_REG | RF_SETUP,0x0f);  //����TX�������,7db����,2Mbps,���������濪��   
	NRF24L01_Write_Reg(NRF_WRITE_REG | CONFIG, 0x0f);		//���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ 
	NRF24L01_Write_Reg(NRF_WRITE_REG | STATUS, 0xff);
	
	NRF24L01_CE(1); 	 		 	 
}
//���24L01�Ƿ����
//����ֵ:0���ɹ�;1��ʧ��	
u8 NRF24L01_Check(void)
{
	u8 buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	u8 i;

	//SPI3_SetSpeed(SPI_BAUDRATEPRESCALER_32); //spi�ٶ�Ϊ6.25Mhz��24L01�����SPIʱ��Ϊ10Mhz��   	 
	NRF24L01_Write_Buf(NRF_WRITE_REG | TX_ADDR,buf,5);//д��5���ֽڵĵ�ַ.	
	NRF24L01_Read_Buf(TX_ADDR,buf,5); //����д��ĵ�ַ  
	for(i=0;i<5;i++)if(buf[i]!=0XA5)break;	 							   
	if(i!=5)return 1;//���24L01����	
	return 0;		 //��⵽24L01
}	 	 
//SPIд�Ĵ���
//reg:ָ���Ĵ�����ַ
//value:д���ֵ
u8 NRF24L01_Write_Reg(u8 reg,u8 value)
{
	u8 status;	
   	NRF24L01_CSN(0);                //ʹ��SPI����
  	status =SPI3_ReadWriteByte(reg);//���ͼĴ����� 
  	SPI3_ReadWriteByte(value);      //д��Ĵ�����ֵ
  	NRF24L01_CSN(1);                //��ֹSPI����	   
  	return(status);       			//����״ֵ̬
}
//��ȡSPI�Ĵ���ֵ
//reg:Ҫ���ļĴ���
u8 NRF24L01_Read_Reg(u8 reg)
{
	u8 reg_val;	    
 	NRF24L01_CSN(0);            //ʹ��SPI����		
  	SPI3_ReadWriteByte(reg);    //���ͼĴ�����
  	reg_val=SPI3_ReadWriteByte(0XFF);//��ȡ�Ĵ�������
  	NRF24L01_CSN(1);            //��ֹSPI����		    
  	return(reg_val);            //����״ֵ̬
}	
//��ָ��λ�ö���ָ�����ȵ�����
//reg:�Ĵ���(λ��)
//*pBuf:����ָ��
//len:���ݳ���
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ 
u8 NRF24L01_Read_Buf(u8 reg,u8 *pBuf,u8 len)
{
	u8 status,u8_ctr;	       
	NRF24L01_CSN(0);            //ʹ��SPI����
	status=SPI3_ReadWriteByte(reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬   	   
 	for(u8_ctr=0;u8_ctr<len;u8_ctr++)
		pBuf[u8_ctr]=SPI3_ReadWriteByte(0XFF);//��������
	NRF24L01_CSN(1);            //�ر�SPI����
	return status;              //���ض�����״ֵ̬
}
//��ָ��λ��дָ�����ȵ�����
//reg:�Ĵ���(λ��)
//*pBuf:����ָ��
//len:���ݳ���
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ
u8 NRF24L01_Write_Buf(u8 reg, u8 *pBuf, u8 len)
{
	u8 status,u8_ctr;	    
 	NRF24L01_CSN(0);            //ʹ��SPI����
	status = SPI3_ReadWriteByte(reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
	for(u8_ctr=0; u8_ctr<len; u8_ctr++)
	{
		SPI3_ReadWriteByte(*pBuf); //д������		
		pBuf++;
	}
	NRF24L01_CSN(1);            //�ر�SPI����
	return status;              //���ض�����״ֵ̬
}				   
//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:�������״��
u8 NRF24L01_TxPacket(u8 *txbuf)
{
	u8 sta;
 	//SPI3_SetSpeed(SPI_BAUDRATEPRESCALER_32); //spi�ٶ�Ϊ6.25Mhz��24L01�����SPIʱ��Ϊ10Mhz��   
	NRF24L01_CE(0);
  	NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//д���ݵ�TX BUF  32���ֽ�
 	NRF24L01_CE(1);                         //��������	   
	while(NRF24L01_IRQ!=0);                 //�ȴ��������
	sta=NRF24L01_Read_Reg(STATUS);          //��ȡ״̬�Ĵ�����ֵ	   
	NRF24L01_Write_Reg(NRF_WRITE_REG | STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
	if(sta&MAX_TX)                          //�ﵽ����ط�����
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);  //���TX FIFO�Ĵ��� 
		return MAX_TX; 
	}
	if(sta&TX_OK)                           //�������
	{
		return TX_OK;
	}
	return 0xff;//����ԭ����ʧ��
}
//����NRF24L01����һ������
//rxbuf:�����������׵�ַ
//����ֵ:0��������ɣ��������������
u8 NRF24L01_RxPacket(u8 *rxbuf)
{
	u8 sta;		    							   
	//SPI3_SetSpeed(SPI_BAUDRATEPRESCALER_16); //spi�ٶ�Ϊ6.25Mhz��24L01�����SPIʱ��Ϊ10Mhz��   
	sta=NRF24L01_Read_Reg(STATUS);          //��ȡ״̬�Ĵ�����ֵ    	 
	NRF24L01_Write_Reg(NRF_WRITE_REG | STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
	if(sta&RX_OK)//���յ�����
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//��ȡ����
		NRF24L01_Write_Reg(FLUSH_RX,0xff);  //���RX FIFO�Ĵ��� 
		return 0; 
	}	   
	return 1;//û�յ��κ�����
}					    
//�ú�����ʼ��NRF24L01��RXģʽ
//����RX��ַ,дRX���ݿ���,ѡ��RFƵ��,�����ʺ�LNA HCURR
//��CE��ߺ�,������RXģʽ,�����Խ���������		   
void NRF24L01_RX_Mode(void)
{
	NRF24L01_CE(0);	  
	NRF24L01_Write_Reg(NRF_WRITE_REG | CONFIG, 0x0f);     //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ 
	NRF24L01_CE(1); //CEΪ��,�������ģʽ 
	delay_us(500);
}						 
//�ú�����ʼ��NRF24L01��TXģʽ
//����TX��ַ,дTX���ݿ���,����RX�Զ�Ӧ��ĵ�ַ,���TX��������,ѡ��RFƵ��,�����ʺ�LNA HCURR
//PWR_UP,CRCʹ��
//��CE��ߺ�,������RXģʽ,�����Խ���������		   
//CEΪ�ߴ���10us,����������.	 
void NRF24L01_TX_Mode(void)
{														 
	NRF24L01_CE(0);	     
	NRF24L01_Write_Reg(NRF_WRITE_REG | CONFIG,0x0e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
	NRF24L01_CE(1);//CEΪ��,10us����������
	delay_us(10);
}
