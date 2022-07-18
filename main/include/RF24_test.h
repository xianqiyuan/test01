#ifndef __RF24_test_H__
#define __RF24_test_H__

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

#define u8 unsigned char
#define u16 unsigned short

#define RF_CH_NUM 90   //信道频点：2400+RF_CH_NUM

//NRF24L01 IO定义
#define NRF_CE		13
#define NRF_CSN		32
#define NRF_SCK		33
#define NRF_MOSI	14
#define NRF_MISO	35
#define NRF_IRQ		34


#define digitalRead(x)          gpio_get_level(x)
#define digitalWrite(x,y)    	gpio_set_level(x, y)


//软件spi
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1)| (1ULL<<GPIO_OUTPUT_IO_2)| (1ULL<<PIN_NUM_CS)| (1ULL<<PIN_NUM_RST) |(1ULL<<PIN_NUM_CLK)|(1ULL<<PIN_NUM_MOSI))
#define GPIO_INPUT_IO_0     0  //KEY RELOAD

//软件spi
#define GPIO_INPUT_PIN_SEL  ((1ULL<<PIN_NUM_NIRQ) |(1ULL<<PIN_NUM_MISO))




#define nSEL_H() 						NSEL_PIN(1)  
#define nSEL_L() 						NSEL_PIN(0)
#define SDN_H()							SDN_PIN(1)
#define SDN_L()							SDN_PIN(0)
#define SCK_H()							SCK_PIN(1)
#define SCK_L()							SCK_PIN(0)
#define SDO_H()							SDO_PIN(1)
#define SDO_L()							SDO_PIN(0)


//********************************************************************************************************************// 
// SPI(nRF24L01) commands 
#define READ_REG        0x00  // Define read command to register 
#define WRITE_REG       0x20  // Define write command to register 
#define RD_RX_PLOAD     0x61  // Define RX payload register address 
#define WR_TX_PLOAD     0xA0  // Define TX payload register address 
#define FLUSH_TX        0xE1  // Define flush TX register command 
#define FLUSH_RX        0xE2  // Define flush RX register command 
#define REUSE_TX_PL     0xE3  // Define reuse TX payload register command 
//#define NOP             0xFF  // Define No Operation, might be used to read status register 

//********************************************************************************************************************// 
// SPI(nRF24L01) registers(addresses) 
#define CONFIG          0x00  // 'Config' register address 
#define EN_AA           0x01  // 'Enable Auto Acknowledgment' register address 
#define EN_RXADDR       0x02  // 'Enabled RX addresses' register address 
#define SETUP_AW        0x03  // 'Setup address width' register address 
#define SETUP_RETR      0x04  // 'Setup Auto. Retrans' register address 
#define RF_CH           0x05  // 'RF channel' register address 
#define RF_SETUP        0x06  // 'RF setup' register address 
#define RF_STATUS          0x07  // 'Status' register address 

#define MAX_TX   0x10  //达到最大发送次数中断 
#define TX_OK    0x20  //TX发送完成中断 
#define RX_OK    0x40  //接收到数据中断 
#define TimeOut  0x80	//超时


#define OBSERVE_TX      0x08  // 'Observe TX' register address 
#define CD              0x09  // 'Carrier Detect' register address 
#define RX_ADDR_P0      0x0A  // 'RX address pipe0' register address 
#define RX_ADDR_P1      0x0B  // 'RX address pipe1' register address 
#define RX_ADDR_P2      0x0C  // 'RX address pipe2' register address 
#define RX_ADDR_P3      0x0D  // 'RX address pipe3' register address 
#define RX_ADDR_P4      0x0E  // 'RX address pipe4' register address 
#define RX_ADDR_P5      0x0F  // 'RX address pipe5' register address 
#define TX_ADDR         0x10  // 'TX address' register address 
#define RX_PW_P0        0x11  // 'RX payload width, pipe0' register address 
#define RX_PW_P1        0x12  // 'RX payload width, pipe1' register address 
#define RX_PW_P2        0x13  // 'RX payload width, pipe2' register address 
#define RX_PW_P3        0x14  // 'RX payload width, pipe3' register address 
#define RX_PW_P4        0x15  // 'RX payload width, pipe4' register address 
#define RX_PW_P5        0x16  // 'RX payload width, pipe5' register address 
#define FIFO_STATUS     0x17  // 'FIFO Status Register' register address 

#define TX_ADR_WIDTH    5   //5字节的地址宽度 
#define RX_ADR_WIDTH    5   //5字节的地址宽度 
#define TX_PLOAD_WIDTH  4  //4字节的用户数据宽度 
#define RX_PLOAD_WIDTH  4  //4字节的用户数据宽度 




void NRF_Init(void);
u8 SPI_RW(u8 dat);
void nRF_POR_Init(void);
void SPI_RW_Reg(u8 reg, u8 value);
u8 SPI_Read(u8 reg);
u8 SPI_Read_Buf(u8 reg, u8 *pBuf, u8 bytes);
u8 SPI_Write_Buf(u8 reg, u8 *pBuf, u8 bytes);
void RX_Mode(void);
void TX_Mode(u8 *P);
void init_24l01(void);
u8 NRF24L01_RxPacket(u8 *rxbuf); 
u8 NRF24L01_TxPacket(u8 *txbuf);
u8 NRF24L01_Check(void);
extern u8 TX_ADDRESS[TX_ADR_WIDTH];
extern u8 RX_ADDRESS[TX_ADR_WIDTH];
//extern volatile u8 flag;
//extern u8 State;
extern u8 Tx_Buf[];
extern u8 Rx_Buf[];	


#endif
