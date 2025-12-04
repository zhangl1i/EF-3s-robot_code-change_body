#include "spi.h"

//////////////////////////////////////////////////////////////////////////////////	 
//HALLIB add 
//stm32h7xx_hal_spi.c
//stm32h7xx_hal_spi_ex.c
////////////////////////////////////////////////////////////////////////////////// 	

SPI_HandleTypeDef SPI3_Handler;  //SPI3句柄

//以下是SPI模块的初始化代码，配置成主机模式 						  
//SPI口初始化
//这里针是对SPI3的初始化
void SPI3_Init(void)
{
    SPI3_Handler.Instance=SPI3;                      //SP3
    SPI3_Handler.Init.Mode=SPI_MODE_MASTER;          //设置SPI工作模式，设置为主模式
    SPI3_Handler.Init.Direction=SPI_DIRECTION_2LINES;//设置SPI单向或者双向的数据模式:SPI设置为双线模式
    SPI3_Handler.Init.DataSize=SPI_DATASIZE_8BIT;    //设置SPI的数据大小:SPI发送接收8位帧结构
    SPI3_Handler.Init.CLKPolarity=SPI_POLARITY_HIGH; //串行同步时钟的空闲状态为高电平
    SPI3_Handler.Init.CLKPhase=SPI_PHASE_2EDGE;      //串行同步时钟的第二个跳变沿（上升或下降）数据被采样
    SPI3_Handler.Init.NSS=SPI_NSS_SOFT;              //NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
    SPI3_Handler.Init.NSSPMode=SPI_NSS_PULSE_DISABLE;//NSS信号脉冲失能
    SPI3_Handler.Init.MasterKeepIOState=SPI_MASTER_KEEP_IO_STATE_ENABLE;  //SPI主模式IO状态保持使能
    SPI3_Handler.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_256;//定义波特率预分频的值:波特率预分频值为256
    SPI3_Handler.Init.FirstBit=SPI_FIRSTBIT_MSB;     //指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
    SPI3_Handler.Init.TIMode=SPI_TIMODE_DISABLE;     //关闭TI模式
    SPI3_Handler.Init.CRCCalculation=SPI_CRCCALCULATION_DISABLE;//关闭硬件CRC校验
    SPI3_Handler.Init.CRCPolynomial=7;               //CRC值计算的多项式
    HAL_SPI_Init(&SPI3_Handler);


//	HAL_NVIC_SetPriority(SPI3_IRQn, 3, 0);
//	HAL_NVIC_EnableIRQ(SPI3_IRQn);		
    __HAL_SPI_ENABLE(&SPI3_Handler);                 //使能SPI3
    SPI3_ReadWriteByte(0Xff);                        //启动传输
}

//SPI3底层驱动，时钟使能，引脚配置
//此函数会被HAL_SPI_Init()调用
//hspi:SPI句柄
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef GPIO_Initure;
    RCC_PeriphCLKInitTypeDef SPI3ClkInit;
	
    __HAL_RCC_GPIOC_CLK_ENABLE();                   //使能GPIOC时钟
    __HAL_RCC_SPI3_CLK_ENABLE();                    //使能SPI3时钟
    
	//设置SPI3的时钟源 
	SPI3ClkInit.PeriphClockSelection=RCC_PERIPHCLK_SPI3;	    //设置SPI3时钟源
	SPI3ClkInit.Spi123ClockSelection=RCC_SPI123CLKSOURCE_PLL;	//SPI3时钟源使用PLL1Q
	HAL_RCCEx_PeriphCLKConfig(&SPI3ClkInit);
	
    //PC10,11,12
    GPIO_Initure.Pin=GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;              //复用推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;                  //上拉
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;   //快速    
    GPIO_Initure.Alternate=GPIO_AF6_SPI3;           //复用为SPI3
    HAL_GPIO_Init(GPIOC,&GPIO_Initure);             //初始化
}

//SPI速度设置函数
//SPI速度=PLL1Q/分频系数
//@ref SPI_BaudRate_Prescaler:SPI_BAUDRATEPRESCALER_2~SPI_BAUDRATEPRESCALER_256
//PLL1Q时钟一般为200Mhz：
void SPI3_SetSpeed(u32 SPI_BaudRatePrescaler)
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));//判断有效性
    __HAL_SPI_DISABLE(&SPI3_Handler);            //关闭SPI
    SPI3_Handler.Instance->CFG1&=~(0X7<<28);     //位30-28清零，用来设置波特率
    SPI3_Handler.Instance->CFG1|=SPI_BaudRatePrescaler;//设置SPI速度
    __HAL_SPI_ENABLE(&SPI3_Handler);             //使能SPI
    
}

//SPI3 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
u8 SPI3_ReadWriteByte(u8 TxData)
{
    u8 Rxdata;
    HAL_SPI_TransmitReceive(&SPI3_Handler,&TxData,&Rxdata,1, 1000);       
 	return Rxdata;          		    //返回收到的数据		
}

///////////////IRQ
//u8 SPI3_IRQHandler(u8 TxData)
//{
//	u8 Rxdata;
//	HAL_SPI_IRQHandler(&SPI3_Handler);
//	//HAL_SPI_TransmitReceive_IT(&SPI3_Handler,&TxData,&Rxdata,1);
//	HAL_SPI_Receive_IT(&SPI3_Handler,&Rxdata,1);
//	return Rxdata;  
//}

//void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
//{
//    volatile uint8_t y = 5;
//}