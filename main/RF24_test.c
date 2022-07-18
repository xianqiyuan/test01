#include "RF24_test.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/portmacro.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "driver/gpio.h"//DENY
#include "driver/spi_master.h"


u8 TX_ADDRESS[TX_ADR_WIDTH]={0x34,0x43,0x10}; //发送地址
u8 RX_ADDRESS[RX_ADR_WIDTH]={0x34,0x43,0x10}; //发送地址

u8 Tx_Buf[4]={0};
u8 Rx_Buf[4];


void nRF_POR_Init()
{
	gpio_reset_pin(NRF_CE);
	gpio_reset_pin(NRF_CSN);
	gpio_reset_pin(NRF_SCK);
	gpio_reset_pin(NRF_MOSI);
	gpio_reset_pin(NRF_MISO);
	gpio_reset_pin(NRF_IRQ);
	
    gpio_set_direction(NRF_CE, GPIO_MODE_OUTPUT);
	gpio_set_direction(NRF_CSN, GPIO_MODE_OUTPUT);
	gpio_set_direction(NRF_SCK, GPIO_MODE_OUTPUT);
	gpio_set_direction(NRF_MOSI, GPIO_MODE_OUTPUT);
	gpio_set_direction(NRF_MISO, GPIO_MODE_INPUT);
	gpio_set_direction(NRF_IRQ, GPIO_MODE_INPUT);
	
	gpio_set_level(NRF_CE, 1);
	gpio_set_level(NRF_CSN, 1);
	gpio_set_level(NRF_SCK, 1);
	gpio_set_level(NRF_MOSI, 1);

}



/*************************************************************************
函数功能:SPI读写一个字节函数
入口参数:需要发送的1字节数据
返    回:无
备    注:无
*************************************************************************/
u8 SPI_RW(u8 data)
{
    u8 bit_ctr;
    for(bit_ctr=0;bit_ctr<8;bit_ctr++)// output 8-bit
    {
		gpio_set_level(NRF_MOSI,(data & 0x80)?1:0);	// output 'byte', MSB to MOSI       
	    data = (data << 1);           // shift next bit into MSB..
		gpio_set_level(NRF_SCK,1);		// Set SCK high..         
		data |= ((gpio_get_level(NRF_MISO))?1:0);	// capture current MISO bit
	    gpio_set_level(NRF_SCK,0); 						  // ..then set SCK low again        		  
    }
    return(data);           		  // return read byte       
}



/*************************************************************************
函数功能:写nRF24L01的寄存器
入口参数:寄存器地址和要写入得字节
返    回:无
备    注:无
*************************************************************************/
void SPI_RW_Reg(u8 reg, u8 value)
{
	gpio_set_level(NRF_CSN,0); 
    SPI_RW(reg); 
   	SPI_RW(value);             // ..and write value to it..
	gpio_set_level(NRF_CSN,1); 
}
/*************************************************************************
函数功能:读nRF24L01的寄存器
入口参数:需要读取的寄存器地址
返    回:读取到得寄存器值
备    注:无
*************************************************************************/
u8 SPI_Read(u8 reg)
{  
 	u8 reg_val;
   	gpio_set_level(NRF_CSN,0); //CSN IS LOW
   	SPI_RW(reg);            // Select register to read from..
   	reg_val=SPI_RW(0x00);    // ..then read registervalue
	gpio_set_level(NRF_CSN,1); 
	return(reg_val);        // return register value
}
/*************************************************************************
函数功能:读nRF24L01的数据缓冲区
入口参数:需要读取的缓冲区地址,目标数组指针,需要读取的字节数
返    回:无 (暂时还不知道返回的值有何用处)
备    注:无
*************************************************************************/
u8 SPI_Read_Buf(u8 reg, u8 *pBuf, u8 bytes)
{
 	u8 status,byte_ctr;
 	gpio_set_level(NRF_CSN,0); //CSN IS LOW
   	status = SPI_RW(reg);         // Select register to write to and read status byte
	for(byte_ctr=0;byte_ctr<bytes;byte_ctr++)
   		pBuf[byte_ctr] = SPI_RW(0);    // Perform SPI_RW to read byte from nRF24L01
	gpio_set_level(NRF_CSN,1); 
	return(status);
}
/*************************************************************************
函数功能:写nRF24L01的数据缓冲区
入口参数:需要写入的缓冲区地址,目标数组指针,需要写入的字节数
返    回:无 (暂时还不知道返回的值有何用处)
备    注:无
*************************************************************************/
u8 SPI_Write_Buf(u8 reg, u8 *pBuf, u8 bytes)
{
 	u8 status,byte_ctr;
 	gpio_set_level(NRF_CSN,0); //CSN IS LOW
 	status = SPI_RW(reg);    // Select register to write to and read status byte
	for(byte_ctr=0; byte_ctr<bytes; byte_ctr++) // then write all byte in buffer(*pBuf)
  		SPI_RW(*pBuf++);
	gpio_set_level(NRF_CSN,1); 
	return(status);
}



/*************************************************************************
函数功能:将nRF24L01置为接收模式
入口参数:无
返    回:无 
备    注:无
*************************************************************************/
void RX_Mode(void) 
{
	u8 Sta; 
 	gpio_set_level(NRF_CE,0);    
   SPI_Write_Buf((WRITE_REG + RX_ADDR_P0),RX_ADDRESS,RX_ADR_WIDTH);//写RX节点地址 
    
   SPI_RW_Reg(WRITE_REG + EN_AA, 0x00);      //关闭数据通道自动应答    
   SPI_RW_Reg(WRITE_REG+EN_RXADDR,0x01);//使能通道0的接收地址 
   SPI_RW_Reg(WRITE_REG+RF_CH,RF_CH_NUM);      //设置RF通信频率     
   SPI_RW_Reg(WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道 0的有效数据宽度       
   SPI_RW_Reg(WRITE_REG+RF_SETUP,0x0b);//设置TX发射参数,0db增益,2Mbps,低噪声增益开启    
   SPI_RW_Reg(WRITE_REG+CONFIG, 0x0b);//配置基本工作模式的参数;PWR_UP,1B CRC,接收模式  
   Sta=SPI_Read(RF_STATUS);  //读取状态寄存器的值       
   SPI_RW_Reg(WRITE_REG+RF_STATUS,Sta); //清除TX_DS或MAX_RT中断标志 
   SPI_RW_Reg(FLUSH_RX,0xff);//清除 RX FIFO寄存器
   gpio_set_level(NRF_CE,1);  ; //CE为高,进入接收模式  
}  


/*************************************************************************
函数功能:将nRF24L01置为发送模式
入口参数:无
返    回:无 
备    注:发送数据
*************************************************************************/
void TX_Mode(u8 *P) 
{                
	gpio_set_level(NRF_CE,0);      
   SPI_Write_Buf(WRITE_REG+TX_ADDR,P,TX_ADR_WIDTH);//写TX节点地址   
   SPI_Write_Buf(WRITE_REG+RX_ADDR_P0,P,RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK  
   SPI_RW_Reg(WRITE_REG+EN_AA,0x01);     //使能通道0的自动应答     
   SPI_RW_Reg(WRITE_REG+EN_RXADDR,0x01); //使能通道0的接收地址   
   SPI_RW_Reg(WRITE_REG+SETUP_RETR,0x1a);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次 
   SPI_RW_Reg(WRITE_REG+RF_CH,RF_CH_NUM);       //设置RF通道为40 
   SPI_RW_Reg(WRITE_REG+RF_SETUP,0x0b);  //设置TX发射参数,0db 增益,2Mbps,低噪声增益开启    
   SPI_RW_Reg(WRITE_REG+CONFIG,0x0e);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断 
	gpio_set_level(NRF_CE,1);//CE为高,10us后启动发送 
} 




u8 NRF24L01_TxPacket(u8 *txbuf) 
{ 
 	u8 Sta;    
 	gpio_set_level(NRF_CE,0); 
   	SPI_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//写数据到TX BUF  32个字节 
  	gpio_set_level(NRF_CE,1);//启动发送    
	 while( gpio_get_level(NRF_IRQ));  //等待发送完成 	   
    Sta = SPI_Read(RF_STATUS);  
 	SPI_RW_Reg(WRITE_REG+RF_STATUS,Sta); //清除TX_DS或MAX_RT中断标志 
 	if(Sta&MAX_TX)//达到最大重发次数 
	{ 
		SPI_RW_Reg(FLUSH_TX,0xff);//清除 TX FIFO寄存器  
		return MAX_TX;  
	} 
	if(Sta&TX_OK)//发送完成 
	{ 
		return TX_OK; 
	} 
	return 0xff;//其他原因发送失败 
} 

u8 NRF24L01_RxPacket(u8 *rxbuf) 
{  
	u8 Sta;
	u16 wait_count=0; 
	while( gpio_get_level(NRF_IRQ))
	{
	 wait_count++;
	 //delay(1);
	 vTaskDelay(10 / portTICK_PERIOD_MS);
	 //if(wait_count>=150)
	 if(wait_count>=50)
	 	break;	
	}  //等待接收完成
		   
    wait_count = 0; 
	Sta=SPI_Read(RF_STATUS);  //读取状态寄存器的值       
	SPI_RW_Reg(WRITE_REG+RF_STATUS,Sta); //清除TX_DS或MAX_RT中断标志 
	if(Sta&RX_OK)//接收到数据 
	{ 
		SPI_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//读取数据 
		SPI_RW_Reg(FLUSH_RX,0xff);//清除 RX FIFO寄存器  
		return RX_OK;  
	}     
	return 1;//没收到任何数据 
}  




/*************************************************************************
函数功能:nRF24L01配置的初始化函数
入口参数:无
返    回:无 
备    注:没有用到得配置使用内部默认值
*************************************************************************/
void init_24l01()
{
  SPI_RW_Reg(WRITE_REG + EN_AA, 0x00);      //关闭数据通道自动应答
  SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x0f);  //接受数据通道0允许
  SPI_RW_Reg(WRITE_REG + SETUP_AW, 0x01);  //地址位宽：0x01:3B; default: 0x03:5B
  SPI_RW_Reg(WRITE_REG + SETUP_RETR, 0x00);//关闭自动重发
  SPI_RW_Reg(WRITE_REG + RF_CH, RF_CH_NUM);        // 选择射频通道(40)
  SPI_RW_Reg(WRITE_REG + RF_SETUP, 0X0b);   //射频寄存器,数据传输率(2Mbps),发射功率(0dBm),低噪声放大器增益
  SPI_RW_Reg(WRITE_REG + RX_PW_P1, TX_PLOAD_WIDTH); //接收数据通道1的有效数据宽度(1~32字节)
  SPI_RW_Reg(WRITE_REG + RX_PW_P2, TX_PLOAD_WIDTH); //接收数据通道2s的有效数据宽度(1~32字节)
  SPI_RW_Reg(WRITE_REG + RX_PW_P3, TX_PLOAD_WIDTH); //接收数据通道2s的有效数据宽度(1~32字节)
  SPI_RW_Reg(WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH); //接收数据通道0的有效数据宽度(1~32字节)
}  



void NRF_Init(void)
{	
	nRF_POR_Init(); 
	init_24l01();
} 


/********************************************/
/* 函数功能：检测24L01是否存在              */
/* 返回值；  0  存在                        */
/*           1  不存在                      */
/********************************************/	
u8 NRF24L01_Check(void)
{
    u8 check_in_buf[5]={0x11,0x22,0x33,0x44,0x55};
    u8 check_out_buf[5]={0x00};
		
    //tx_addr是发送地址，发送check_in_buf 数组数据到tx_addr寄存器地址里，NRF_WRITE_REG是写寄存器地址
    SPI_Write_Buf(WRITE_REG+TX_ADDR, check_in_buf, 5); 
    SPI_Read_Buf(READ_REG+TX_ADDR, check_out_buf, 5);//然后再读出此寄存器中数组数据
    
    if( (check_out_buf[0] == 0x11)&&\
        (check_out_buf[1] == 0x22)&&\
        (check_out_buf[2] == 0x33)&&\
        (check_out_buf[3] == 0x44)&&\
        (check_out_buf[4] == 0x55))return 0;//如果返回0说明24L01是正常的模块，实现检测功能
    else return 1;
}
