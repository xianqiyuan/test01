/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "RF24_test.h"


#define BLINK_GPIO 12

static uint8_t s_led_state = 0;

static void configure_led(void)
{
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

static void blink_led(void)
{
    gpio_set_level(BLINK_GPIO, s_led_state);
}

void led_blink_task(void)
{
	while (1) 
	{
        blink_led();
        s_led_state = !s_led_state;
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
	
void si24r1_rx_task(void)
{
	float pressure = 0.0;
	float voltage = 0.0;
	uint8_t i = 0;
	u16 pkg_id=0;
	u16 rx_cnt=0;	//历史不重复收包总数
	u16 cnt1 = 0xffff;
	u16 loss_cnt = 0;
	float loss_rate = 0.0;
	u8 no_rx_cnt = 0;
	while (1) 
	{
		if((NRF24L01_RxPacket(Rx_Buf))==RX_OK)
		{   
			#if 0
			printf("RX OK!\r\n");
			printf("RX Data(4 bytes):\r\n");
			for(i=0;i<4;i++)
			{
				printf("%d ", Rx_Buf[i]);
				printf(" ");
			}
			printf("\r\n\r\nDevice ID = %d\r\n",Rx_Buf[0]);
			printf("Temperature = %d C\r\n",Rx_Buf[1]);
			voltage = ((float)((Rx_Buf[2]>>4)&0x0f)*100+1800)/1000.0;
			printf("Voltage = %.1f V\r\n",voltage);


			//pkg_id = ((u16)Rx_Buf[0]<<4)+(Rx_Buf[2]&0x0f);
			pkg_id = (Rx_Buf[2]&0x0f);
			printf("Packet ID = %d\r\n", pkg_id);
			pressure = Rx_Buf[3]*3.13+101.35;  //换算为KPa
			printf("Press = %.2f KPa\r\n\r\n\r\n",pressure);
			#endif
			pkg_id = (((u16)Rx_Buf[0]<<4)+(Rx_Buf[2]&0x0f));

			if(pkg_id < cnt1)	//小于上次，说明传感器掉电或复位过，或主机复位过，重头开始算
			{
				loss_cnt = 0;
				cnt1 = pkg_id;
				rx_cnt = 1;
				printf("\r\nrestart\r\n");
			}
			else
			{
				if(pkg_id != cnt1) //与上次不重复
				{
					if(pkg_id != (cnt1+1))	//丢包
					{
						loss_cnt = loss_cnt + pkg_id - cnt1 -1;	
					}
					cnt1 = pkg_id;
					rx_cnt ++;		//收包次数	
				}
				
			}
			loss_rate = ((float)loss_cnt / (rx_cnt+loss_cnt))*100;
			printf("pkg_ID = %d, rx_cnt = %d, loss_rate = %.2f%%\r\n", pkg_id, rx_cnt-1, loss_rate);
			no_rx_cnt = 0;
		}

		else
		{
			if(no_rx_cnt++==5)
			{
				printf("Lisening\r\n");
				no_rx_cnt = 0;	
			}
		}
		vTaskDelay(2 / portTICK_RATE_MS);//10ms 循环流程
	}
	vTaskDelete(NULL);
	
}

void app_main(void)
{
	configure_led();
	NRF_Init();
	if(NRF24L01_Check()==0)
	{
		printf("\r\nSi24R1 check OK!\r\n");
	}
	else
	{
		printf("\r\nSi24R1 check Fail!\r\n");
	}
	RX_Mode();
	printf("\r\nSi24R1 is in RX mode!\r\n");


	xTaskCreate(led_blink_task, "led_blink",4096, NULL, 4, NULL);//led
	xTaskCreate(si24r1_rx_task, "si24r1_rx",4096, NULL, 4, NULL);//轮训方式读取 SI24RI

}
