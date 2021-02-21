/**
 ****************************************************************************************
 *
 * @file app.c
 *
 * @brief Application entry point
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"             // SW configuration

#include <string.h>
//#include "rwapp_config.h"
#include "app_task.h"                // Application task Definition
#include "app.h"                     // Application Definition
#include "gap.h"                     // GAP Definition
#include "gapm_task.h"               // GAP Manager Task API
#include "gapc_task.h"               // GAP Controller Task API

#include "co_bt.h"                   // Common BT Definition
#include "co_math.h"                 // Common Maths Definition
#include "ke_timer.h"

#include "app_fff0.h"              // fff0 Module Definition


#include "app_dis.h"                 // Device Information Service Application Definitions
#include "app_batt.h"                // Battery Application Definitions
#include "app_oads.h"                 // Application oads Definition
#if (NVDS_SUPPORT)
#include "nvds.h"           // NVDS API Definitions
#else
#include "flash.h"
#endif
#include "rf.h"
#include "uart.h"
#include "adc.h"
#include "gpio.h"
#include "wdt.h"
#include "rtc.h"
#include "prf_utils.h"
#include "uart2.h"       // uart definition
#include "icu.h"
#include "utc_clock.h"

#include "gap.h"
#include "gattc_task.h"
#include "attm.h"
#include "atts.h"
#include "co_utils.h"
#include "fff0s.h"
#include "fff0s_task.h"
#include "uart.h"


#include "bass.h"
#include "bass_task.h"

/*
 * DEFINES
 ****************************************************************************************
 */
#define ADV_NAME_ADDR 0x1E000

void on_ffe0_ffe2_received(uint8_t* buf,uint8_t len);
void set_fff0_fff1_nft(uint8_t* buf,uint8_t len);
void chu_li_one_cmd();
void xs_uart_received_cb(uint8_t *buf,uint8_t len);
void create_packet(char *buf);
void send_ble_state_to_mcu();
void xs_uart2_send_data(uint8_t *buf, uint8_t len);
void xs_uart_send_data(uint8_t *buf, uint8_t len);
void send_ble_mac_to_mcu();
void send_ble_name_cfg_to_mcu();
void send_ble_data_recieved_to_mcu();
void store_adv_name();
void read_store_adv_name();
void send_ble_manu_data_cfg_to_mcu();

uint8_t g_uart_rev_buff[50];
uint32_t g_time_counter=0;
uint8_t g_ble_state=0;
uint8_t g_mac[6]={0};
uint8_t g_ble_adv_name[35]={0};
//ble state
#define BLE_STATE_PIN 0x07

struct ADV_NAME
{
	uint8_t inited;
	uint8_t adv_name[35];
	uint8_t rsp_name[37];
	uint8_t manu_data[37];
	uint8_t adv_name_len;
	uint8_t rsp_name_len;
	uint8_t manu_data_len;
}g_s_name;
//ble state
#define BLE_WAKE_UP_PIN 0x34

void xs_user_task()
{
	static int t=0,inited=0;

	if(inited==0)
	{
		inited=1;
	}

	if(t++>9)
	{
		t=0;		
	}
	g_time_counter++;

}
void init_app_work()
{
	UART_PRINTF("init_app_work\n");

	gpio_config(BLE_STATE_PIN, OUTPUT, PULL_NONE);
	gpio_set(BLE_STATE_PIN, 1);

	icu_set_sleep_mode(1);
}
// s=0 已连接   s=1 未连接
void set_ble_state(uint8_t s)
{
	g_ble_state=s;
	gpio_set(BLE_STATE_PIN, g_ble_state);
	g_time_counter=0;
}
//uart----------------------------------------------------------------------------------

 void xs_uart2_send_data(uint8_t *buf, uint8_t len)
 {
 	UART_PRINTF("uart2_tx:");
 	for(uint8_t i=0; i<len; i++)
	{
		UART_PRINTF("%x ", buf[i]);
	}
	UART_PRINTF("\r\n");
 	uart2_write(buf,len,NULL,NULL);
 }

 void xs_uart2_received_isr(uint8_t *buf, uint8_t len)
 {
 	UART_PRINTF("uart2_rx:");
 	for(uint8_t i=0; i<len; i++)
	{
		UART_PRINTF("%02x ", buf[i]);
	}
	UART_PRINTF("\r\n");
	if(buf[0]=='0')
	{
		
	}
 }
 //uart2_write("123456",6,0,0);
 void xs_uart_send_data(uint8_t *buf, uint8_t len)
 {
 	UART_PRINTF("uart_tx:");
 	for(uint8_t i=0; i<len; i++)
	{
		UART_PRINTF("%02X ", buf[i]);
	}
	UART_PRINTF("\r\n");
 	uart_write(buf,len,NULL,NULL);
 	
 }
 
 void xs_uart_received_isr(uint8_t *buf, uint8_t len)
 {
 	UART_PRINTF("uart_rx:");
 	for(uint8_t i=0; i<len; i++)
	{
		UART_PRINTF("%02X ", buf[i]);
	}
	UART_PRINTF("\r\n");
	xs_uart_received_cb(buf,len);
}


void on_fff0_fff2_received(uint8_t* buf,uint8_t len)
{
	UART_PRINTF("Fon_fff0_fff2_received = 0x ");

	for(uint8_t i = 0; i < len; i++)
	{
		UART_PRINTF("%02x ",buf[i]);
	}
	UART_PRINTF("\r\n");
	xs_uart_send_data(buf,len);


}
void set_fff0_fff1_nft(uint8_t* buf,uint8_t len)
{
	extern void app_fff1_send_lvl(uint8_t* buf, uint8_t len);
	for(uint8_t i = 0; i < len; i++)
	{
		UART_PRINTF("%02x ",buf[i]);
	}
	UART_PRINTF("\r\n");
	app_fff1_send_lvl(buf,len);
	UART_PRINTF("set_fff0_fff1_ntf \r\n");
}

void xs_uart_received_cb(uint8_t *buf,uint8_t len)
{
	static int receive_counter=0;
	static uint32_t last_tick=0;
	set_fff0_fff1_nft(buf,len);	

	if(receive_counter>0 ||(buf[0]==0x10 && buf[1]==0x00 && buf[2]==0x00 && buf[3]==0xC5) )
	{
		memcpy(g_uart_rev_buff+receive_counter,buf,len);
		receive_counter+=len;
		if(g_uart_rev_buff[4]<=receive_counter+1)
		{
			chu_li_one_cmd();
			receive_counter=0;

		}
	}
	else
	{
		receive_counter=0;
	}
	
}
void chu_li_one_cmd()
{
	UART_PRINTF("chu_li_one_cmd \r\n");
	if(g_uart_rev_buff[5]==0xaa)
	{

		if(g_uart_rev_buff[6]==0x87)
		{
			send_ble_mac_to_mcu();
		}
		else if(g_uart_rev_buff[6]==0x81)
		{
			if(g_uart_rev_buff[4]<40)
			{
				send_ble_name_cfg_to_mcu();
				memset(g_ble_adv_name, 0, 35);
				memcpy(g_ble_adv_name,g_uart_rev_buff+7,g_uart_rev_buff[4]-8);
				UART_PRINTF("g_ble_adv_name:%s \n",g_ble_adv_name);
				g_s_name.adv_name_len=strlen(g_ble_adv_name);
				g_s_name.rsp_name_len=g_s_name.adv_name_len+2;
				g_s_name.rsp_name[0]=g_s_name.adv_name_len+1;
				g_s_name.rsp_name[1]=0x08;
				memcpy(g_s_name.adv_name,g_ble_adv_name,g_s_name.adv_name_len);
				memcpy(g_s_name.rsp_name+2,g_ble_adv_name,g_s_name.adv_name_len);

				store_adv_name();				
			}
		}
		else if(g_uart_rev_buff[6]==0x82)
		{
			if(g_uart_rev_buff[4]<40)
			{
				uint8_t manufactery_data[35];
				send_ble_manu_data_cfg_to_mcu();
				g_s_name.manu_data_len=g_uart_rev_buff[4]-8+2;
				manufactery_data[0]=g_s_name.manu_data_len-1;;
				manufactery_data[1]=0xff;

				memcpy(manufactery_data+2,g_uart_rev_buff+7,g_uart_rev_buff[4]-8);

				memcpy(g_s_name.manu_data,manufactery_data,g_s_name.manu_data_len);

				store_adv_name();
			}
			
		}
		
	}

}

void create_packet(char *buf)
{
	uint32_t check_sum=0;
	buf[0]=0x10;
	buf[1]=0x00;
	buf[2]=0x00;
	buf[3]=0xc5;
	for(int i=4;i<buf[4]-1;i++)
	{
		check_sum+=buf[i];
	}
	buf[buf[4]-1]=check_sum & 0xff;
}
void send_ble_state_to_mcu()
{
	uint8_t buf[9]={0};
	buf[4]=9;
	buf[5]=0xaa;
	buf[6]=0x00;
	buf[7]=g_ble_state==1?2:0;
	create_packet(buf);
	xs_uart_send_data(buf,buf[4]);
}
void send_ble_mac_to_mcu()
{
	uint8_t buf[14]={0};
	buf[4]=14;
	buf[5]=0xaa;
	buf[6]=0x07;
	buf[7]=g_mac[5];
	buf[8]=g_mac[4];
	buf[9]=g_mac[3];
	buf[10]=g_mac[2];
	buf[11]=g_mac[1];
	buf[12]=g_mac[0];

	create_packet(buf);
	xs_uart_send_data(buf,buf[4]);
}
void send_ble_name_cfg_to_mcu()
{
	uint8_t buf[9]={0};
	buf[4]=9;
	buf[5]=0xaa;
	buf[6]=0x01;
	buf[7]=1;

	create_packet(buf);
	xs_uart_send_data(buf,buf[4]);
}
void send_ble_manu_data_cfg_to_mcu()
{
	uint8_t buf[9]={0};
	buf[4]=9;
	buf[5]=0xaa;
	buf[6]=0x02;
	buf[7]=1;

	create_packet(buf);
	xs_uart_send_data(buf,buf[4]);
}
void send_ble_data_recieved_to_mcu()
{
	uint8_t buf[9]={0};
	buf[4]=9;
	buf[5]=0xaa;
	buf[6]=0x04;
	buf[7]=1;

	create_packet(buf);
	xs_uart_send_data(buf,buf[4]);
}

void init_sys_mac(uint8_t *mac)
{
	memcpy(g_mac,mac,6); 
	UART_PRINTF("mac: 0x");

	for(uint8_t i = 0; i < 6; i++)
	{
		UART_PRINTF("%02x ",g_mac[i]);
	}
	read_store_adv_name();
}
uint8_t* get_manufacturer_data()
{
	UART_PRINTF("get_manufacturer_data\r\n");
	for(uint8_t i = 0; i < g_s_name.manu_data_len; i++)
	{
		UART_PRINTF("%02x ",g_s_name.manu_data[i]);
	}
	UART_PRINTF("\r\n");
	return g_s_name.manu_data;
}
uint8_t get_manufacturer_data_len()
{
	return g_s_name.manu_data_len;
}
uint8_t *get_adv_name()
{
	return g_s_name.adv_name;
}
uint8_t *get_rsp_name()
{
	return g_s_name.rsp_name;
}
uint8_t get_adv_name_len()
{
	return g_s_name.adv_name_len;
}
uint8_t get_rsp_name_len()
{
	return g_s_name.rsp_name_len;
}
void store_adv_name()
{
	flash_erase_sector(FLASH_SPACE_TYPE_NVR, ADV_NAME_ADDR);
	flash_write(FLASH_SPACE_TYPE_NVR,ADV_NAME_ADDR,sizeof(g_s_name), (uint8_t*)&g_s_name);
}

void read_store_adv_name()
{
	flash_read(FLASH_SPACE_TYPE_NVR, ADV_NAME_ADDR, sizeof(g_s_name), (uint8_t*)&g_s_name);
	if(g_s_name.inited!=0x68)
	{
		uint8_t temp=0;
		uint8_t manufactery_data[13]={
		12,0xff,0xd0,0x06,
		0x01,0,0,0,0,0,0,0xc0,0xa6
		};

		g_s_name.inited=0x68;
		memset(g_ble_adv_name, 0, 35);
		memcpy(g_ble_adv_name,"Etekcity Fitness Scale",22);	

		g_s_name.adv_name_len=strlen(g_ble_adv_name);
		g_s_name.rsp_name_len=g_s_name.adv_name_len+2;
		g_s_name.rsp_name[0]=g_s_name.adv_name_len+1;
		g_s_name.rsp_name[1]=0x08;

		memcpy(g_s_name.adv_name,g_ble_adv_name,g_s_name.adv_name_len);
		memcpy(g_s_name.rsp_name+2,g_ble_adv_name,g_s_name.adv_name_len);



		manufactery_data[5]=g_mac[0];
		manufactery_data[6]=g_mac[1];
		manufactery_data[7]=g_mac[2];
		manufactery_data[8]=g_mac[3];
		manufactery_data[9]=g_mac[4];
		manufactery_data[10]=g_mac[5];

		memcpy(g_s_name.manu_data,manufactery_data,13);
		g_s_name.manu_data_len=13;

		UART_PRINTF("manufacturer_data\r\n");
		for(uint8_t i = 0; i < g_s_name.manu_data_len; i++)
		{
			UART_PRINTF("%02x ",g_s_name.manu_data[i]);
		}

		store_adv_name();
	}

	UART_PRINTF("g_ble_adv_name:%s \n\r",g_ble_adv_name);

}

