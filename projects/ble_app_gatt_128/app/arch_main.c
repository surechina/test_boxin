/**
 ****************************************************************************************
 *
 * @file arch_main.c
 *
 * @brief Main loop of the application.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ******** ********************************************************************************
 */

 
/*
 * INCLUDES
 ****************************************************************************************
 */
 
#include "rwip_config.h" // RW SW configuration

#include "arch.h"      // architectural platform definitions
#include <stdlib.h>    // standard lib functions
#include <stddef.h>    // standard definitions
#include <stdint.h>    // standard integer definition
#include <stdbool.h>   // boolean definition
#include "boot.h"      // boot definition
#include "rwip.h"      // RW SW initialization
#include "syscntl.h"   // System control initialization
#include "emi.h"       // EMI initialization
#include "intc.h"      // Interrupt initialization
#include "timer.h"     // TIMER initialization
#include "icu.h"
#include "flash.h"
#include "uart.h"      	// UART initialization
#include "flash.h"     // Flash initialization
//#include "led.h"       // Led initialization
#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
#include "rf.h"        // RF initialization
#endif // BLE_EMB_PRESENT || BT_EMB_PRESENT

#if (BLE_APP_PRESENT)
#include "app.h"       // application functions
#endif // BLE_APP_PRESENT

#include "nvds.h"         // NVDS definitions

#include "reg_assert_mgr.h"
#include "BK3435_reg.h"
#include "RomCallFlash.h"
#include "gpio.h"
#include "pwm.h"
#include "audio.h"
#include "app_task.h"
#include "ir.h"
#include "oads.h"
#include "wdt.h"
#include "user_config.h"
#include "app_fcc0.h"


/**
 ****************************************************************************************
 * @addtogroup DRIVERS
 * @{
 *
 *
 * ****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

// Creation of uart external interface api
struct rwip_eif_api uart_api;

/*
 * LOCAL FUNCTION DECLARATIONS
 ****************************************************************************************
 */
uint16_t ms_lock_flash_buffer[MS_LOCK_SERVICE_DATA_LEN] = {0};
uint8_t upgrade_in_progress;

static void Stack_Integrity_Check(void);

extern void code_sanity_check(void);

extern void rw_fcc_enter(void);

#if (UART_DRIVER)
void uart_rx_handler(uint8_t *buf, uint8_t len);
#endif

#if ((UART_PRINTF_EN) &&(UART_DRIVER))
void assert_err(const char *condition, const char * file, int line)
{
	//uart_printf("%s,condition %s,file %s,line = %d\r\n",__func__,condition,file,line);
  
}

void assert_param(int param0, int param1, const char * file, int line)
{
	//uart_printf("%s,param0 = %d,param1 = %d,file = %s,line = %d\r\n",__func__,param0,param1,file,line);
  
}

void assert_warn(int param0, int param1, const char * file, int line)
{
	// uart_printf("%s,param0 = %d,param1 = %d,file = %s,line = %d\r\n",__func__,param0,param1,file,line);
 
}

void dump_data(uint8_t* data, uint16_t length)
{
	//uart_printf("%s,data = %d,length = %d,file = %s,line = %d\r\n",__func__,data,length);
 
}
#else
void assert_err(const char *condition, const char * file, int line)
{
  
}

void assert_param(int param0, int param1, const char * file, int line)
{
  
}

void assert_warn(int param0, int param1, const char * file, int line)
{
 
}

void dump_data(uint8_t* data, uint16_t length)
{
 
}
#endif //UART_PRINTF_EN

#if 1
/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

//用来加密的原始数据，需要在烧录代码的时候提供
//0102210630355cff0078b69d5538dd22
uint8_t encrypt_key_array[16] = 
{
	0x01, 0x02, 0x21, 0x06,
	0x30, 0x35, 0x5c, 0xff,
	0x00, 0x78, 0xb6, 0x9d,
	0x55, 0x38, 0xdd, 0x22
};
#endif

/*
* EXPORTED FUNCTION DEFINITIONS
****************************************************************************************
*/

/*-----------------------------------------------------------------------------*
 *	NAME
 *		appUartProcessReadIndEvent
 *
 *	DESCRIPTION
 *		This function is called when a UART_READ_IND message is received.
 *
 *	PARAMETERS
 *		void *p_rx_buffer  Pointer to the receive buffer (uint8 if 'unpacked'
 *						   or uint16 if 'packed' depending on the chosen UART
 *						   data mode - this application uses 'unpacked')
 *
 *		uint16 length	   Number of bytes ('unpacked') or words ('packed')
 *						   received
 *
 *	RETURNS
 *		The number of bytes ('unpacked') or words ('packed') that have been
 *		processed.
 *----------------------------------------------------------------------------*/
 
//ms_uart_struct ms_uart_buf;
uint8_t ms_uart_send_data[64];
extern void lld_util_get_bd_address(struct bd_addr *bd_addr);
extern void app_fcc1_send_lvl(uint8_t* buf, uint8_t len);
extern uint16_t cur_baud_rate;

/*extern void ms_reset_uart_para() 
{
	 memset(&ms_uart_buf, 0, sizeof(ms_uart_struct));
} */	
static void appUartProcessReadIndEvent(uint8_t   *p_rx_buffer,
									   uint16_t  length)
{

	//nt8_t flag_err = 0;

	uint16_t vlen = 0;
	uint16_t cur_rate_temp = 0;
	uint8_t tmp_head[3] = {0};
	//char *cmd_pos = NULL;
	//char *ask_pos = NULL;
	//char cmd[10] = {0};
	memcpy(tmp_head, p_rx_buffer, 3);
	
	if (length >= 40)
		return;
	//nt8_t data[1]={0};
	if(memcmp(tmp_head, "AT+", 3) == 0)
	{
		/*ask_pos = strstr((char *)p_rx_buffer, "?");
		if(ask_pos && (length <= 64))
			memcpy(cmd,p_rx_buffer+3,(uint8_t *)ask_pos-((uint8_t *)p_rx_buffer+3)+1);//need addr? not addr,so +1
		cmd_pos = strstr((char *)p_rx_buffer, "=");
		if(cmd_pos && (length <= 64))
			{memcpy(cmd,p_rx_buffer+3,(uint8_t *)cmd_pos-((uint8_t *)p_rx_buffer+3));
			uart_printf("cmd = %s\nlen=%d\n",cmd,(uint8_t *)cmd_pos-((uint8_t *)p_rx_buffer+3));}
		if((ask_pos== NULL) && (cmd_pos == NULL) && (length <= 64))//such as AT+BROADCAST, AT+FACTORY...
		{
			char ch_r[1] = '\r';
			cmd_pos = strstr((char *)p_rx_buffer, ch_r);
			memcpy(cmd,p_rx_buffer+3,(uint8_t *)cmd_pos-((uint8_t *)p_rx_buffer+3));
		}
		if(cmd[0] == 0)
		{
			uart_printf("rx return \n");
			return;
		}*/
		
		//uart_send(p_rx_buffer,length);
		//uart_printf("buff=%s,len=%d\n",p_rx_buffer,length);
		if(memcmp((char *)p_rx_buffer+3, "DATA=", 5) == 0)
		{
			//if(p_rx_buffer[8] <= 20 && p_rx_buffer[8] > 0 )//&& 
			//	length >= (p_rx_buffer[8] + 11))
			{
				/*uart_printf("rec_data = ");
				for(uint8_t j=0; j<p_rx_buffer[8]; j++)
					{
						uart_printf("0x%x ", p_rx_buffer[9+j]);
					}
					uart_printf("\r\nlen=%d\r\n",p_rx_buffer[8]);
					*/
				app_fcc1_send_lvl(p_rx_buffer+9, p_rx_buffer[8]);
			}
			/*if (nvds_put(NVDS_TAG_APP_SPECIFIC_FIRST, 5, p_rx_buffer+9) != NVDS_OK)
			{
				ASSERT_INFO(0, NVDS_TAG_LOC_IRK, 0);
			}*/

		}else if(memcmp((char *)p_rx_buffer+3, "RESET", 5) == 0)
		{
			memset(ms_uart_send_data, 0, 64);
			memcpy(ms_uart_send_data, "OK\r\n", 4);
			uart_send(ms_uart_send_data,4);

			//MsHandleReSet();
			
		}else if(memcmp((char *)p_rx_buffer+3, "FACTORYSET", 10) == 0)
		{
			
			memset(ms_uart_send_data, 0, 64);
			memcpy(ms_uart_send_data, "OK\r\n", 4);
			uart_send(ms_uart_send_data,4);

			//reset all parameters
		#if 0//def MS_LOCK_FLASH_ENABLE
			memset(ms_lock_flash_buffer, 0, MS_LOCK_SERVICE_DATA_LEN);
			Nvm_KeyWrite(MS_LOCK_SERVICE_ID, ms_lock_flash_buffer, MS_LOCK_SERVICE_DATA_LEN, 0);
		#endif
			//MsHandleFactory();

		}else if(memcmp((char *)p_rx_buffer+3, "VERSION?", 8) == 0)
		{
			// send version numbeer
			memset(ms_uart_send_data, 0, 64);
			vlen = strlen(DEVICE_VERSION);
			memcpy(ms_uart_send_data, "+VERSION=", 9);
			memcpy(ms_uart_send_data+9, DEVICE_VERSION, vlen);
			ms_uart_send_data[9+vlen] = '\r';
			ms_uart_send_data[9+vlen+1] = '\n';
			if(upgrade_in_progress == 1)
			{
				upgrade_in_progress = 0;
			#if ENABLE_UNUSED_SECTION_WRITE
				flash_write(FLASH_APP_OTA_PROGRESS_FLAG, SEC_UNUSED_ALLOC_FADDR, 1, &upgrade_in_progress, NULL);
			#endif
			uart_printf("##sure [AT+VERSION?] flash_write upgrade_in_progress	= %d\n",upgrade_in_progress);
			}

			
			uart_send(ms_uart_send_data,9+vlen+1+1);
			//UQ_ForceQueueBytes(ms_uart_send_data, 9+vlen+1+1);
			//AppUartSendPendingData();			
		}else if(memcmp((char *)p_rx_buffer+3, "ADDR?", 5) == 0)
		{
			// send the mac address
			
			struct bd_addr bd_addr;
			lld_util_get_bd_address(&bd_addr);
			memset(ms_uart_send_data, 0, 64);
			memcpy(ms_uart_send_data, "+ADDR=", 6);
			memcpy(ms_uart_send_data+6, bd_addr.addr, 6);
			ms_uart_send_data[12] = '\r';
			ms_uart_send_data[13] = '\n';

 		    uart_send(ms_uart_send_data,6+6+2);
			//UQ_ForceQueueBytes(ms_uart_send_data, 14);
			//AppUartSendPendingData();

		}else if(memcmp((char *)p_rx_buffer+3, "BROADCAST", 9) == 0)
		{
			// do broadcast
			memset(ms_uart_send_data, 0, 64);
			memcpy(ms_uart_send_data, "OK\r\n", 4);
			// Go to the ready state
			ke_state_set(TASK_APP, APPM_READY);
						
			appm_start_advertising();

			uart_send(ms_uart_send_data,4);
			//UQ_ForceQueueBytes(ms_uart_send_data, 4);
			//AppUartSendPendingData();

			//rebroadcast now
			//MsHandleReAdv();

		}else if(memcmp((char *)p_rx_buffer+3, "UART?", 5) == 0)
		{
			vlen = 0;
			memset(ms_uart_send_data, 0, 64);
			memcpy(ms_uart_send_data, "+UART=", 6);

			switch(cur_baud_rate)

			{
				case UART_RATE_2K4:
					
					memcpy(ms_uart_send_data+6, "2400", 4);
					vlen = 10;
					break;

				case UART_RATE_9K6:
					
					memcpy(ms_uart_send_data+6, "9600", 4);
					vlen = 10;
					break;

				case UART_RATE_19K2:
					
					memcpy(ms_uart_send_data+6, "19200", 5);
					vlen = 11;
					break;

				case UART_RATE_38K4:
					
					memcpy(ms_uart_send_data+6, "38400", 5);
					vlen = 11;
					break;

				case UART_RATE_57K6:
					
					memcpy(ms_uart_send_data+6, "57600", 5);
					vlen = 11;
					break;

				case UART_RATE_115K2:
					
					memcpy(ms_uart_send_data+6, "115200", 6);
					vlen = 12;
					break;

				default:

					break;

			}

			if(vlen > 0)
			{
				memcpy(ms_uart_send_data+vlen, ",0,0\r\n", 6);
				vlen = vlen + 6;
				uart_send(ms_uart_send_data,vlen);
				//UQ_ForceQueueBytes(ms_uart_send_data, vlen);
				//AppUartSendPendingData();
			}

		}
		else if(memcmp((char *)p_rx_buffer+3, "UART=", 5) == 0)
		{
			memset(ms_uart_send_data, 0, 64);
			memcpy(ms_uart_send_data, "OK\r\n", 4);
			//UQ_ForceQueueBytes(ms_uart_send_data, 4);
			//AppUartSendPendingData();

			if(memcmp((char *)p_rx_buffer+8, "115200", 6) == 0)
			{
				cur_rate_temp = UART_RATE_115K2;
			}else if(memcmp((char *)p_rx_buffer+8, "57600", 5) == 0)
			{
				cur_rate_temp = UART_RATE_57K6;
			}else if(memcmp((char *)p_rx_buffer+8, "38400", 5) == 0)
			{
				cur_rate_temp = UART_RATE_38K4;
			}else if(memcmp((char *)p_rx_buffer+8, "19200", 5) == 0)
			{
				cur_rate_temp = UART_RATE_19K2;
			}else if(memcmp((char *)p_rx_buffer+8, "9600", 4) == 0)
			{
				cur_rate_temp = UART_RATE_9K6;
			}else
			{
				cur_rate_temp = cur_baud_rate;
			}
			
			if(cur_baud_rate != cur_rate_temp)
			{	
				cur_baud_rate = cur_rate_temp;
				uart_init(cur_baud_rate);
				//TimerCreate(1 * SECOND, TRUE, handleUartParaUpdate);
			}
		}
		else if(memcmp((char *)p_rx_buffer+3, "OTA=", 4) == 0)
		{
			memset(ms_uart_send_data, 0, 64);
			memcpy(ms_uart_send_data, "OK\r\n", 4);
			//UQ_ForceQueueBytes(ms_uart_send_data, 4);
			//AppUartSendPendingData();
			uart_send(ms_uart_send_data,4);


			//set the ota enable

			if(memcmp((char *)p_rx_buffer+3+4, "ON", 2) == 0)
			{
				ms_lock_flash_buffer[0] = 1;

			}else if(memcmp((char *)p_rx_buffer+3+4, "OFF", 3) == 0)
			{
				ms_lock_flash_buffer[0] = 0;
			}	
		}

		memset(&ms_uart_send_data, 0, 64);

	}

}


void platform_reset(uint32_t error)
{
    //void (*pReset)(void);
	
	UART_PRINTF("error = %x\r\n", error);

    // Disable interrupts
    GLOBAL_INT_STOP();

    #if UART_PRINTF_EN
    // Wait UART transfer finished
    uart_finish_transfers();
    #endif //UART_PRINTF_EN


    if(error == RESET_AND_LOAD_FW || error == RESET_TO_ROM)
    {
        // Not yet supported
    }
    else
    {
        //Restart FW
        //pReset = (void * )(0x0);
        //pReset();
		wdt_enable(10);
		while(1);
    }
}



void bdaddr_env_init(void)
{    
	struct bd_addr co_bdaddr;    
	flash_read_data(&co_bdaddr.addr[0],0x400e3,6);    
	if(co_bdaddr.addr[0]!=0xff ||co_bdaddr.addr[1]!=0xff||
	   co_bdaddr.addr[2]!=0xff||co_bdaddr.addr[3]!=0xff||        
	   co_bdaddr.addr[4]!=0xff||co_bdaddr.addr[5]!=0xff )    
	{        
		memcpy(&co_default_bdaddr,&co_bdaddr,6);    
	}
}


void rw_dut_enter(void)
{
    /*
     ***************************************************************************
     * Main loop
     ***************************************************************************
     */
    while(1)
    {
        // schedule all pending events
	    rwip_schedule();
    }
}

static void app_gpio_int_cb(void)
{
	//UART_PRINTF("Callback\r\n");
	//if(icu_get_sleep_mode()== 0)
	//if(gpio_get_input(GPIOA_3) == 0)
	if(REG_APB5_GPIO_WUATOD_TYPE == 0x0008)//MantisID:SU2018071653
	{
		icu_set_sleep_mode(1); //iedl
		//restart adv
		ke_msg_send_basic(APP_ENABLE_ADV_REQ,TASK_APP,TASK_APP);
		uart_printf("##sure interrupt!!! upgrade_in_progress = %d\n",upgrade_in_progress);
	}
	#if 0//!ENABLE_BUTTON_CHECK
	else
	{
		uart_printf("##sure upgrade_in_progress = %d\n",upgrade_in_progress);
		if(!upgrade_in_progress)
		{
		icu_set_sleep_mode(0); //low voltage
		
		ke_msg_send_basic(APP_STOP_ADV_REQ,TASK_APP,TASK_APP);
		}
	}
	#endif
	
	UART_PRINTF("power_mode = %d\r\n", icu_get_sleep_mode());
}
	
/*void config_all_pin_as_input(void)
{
	int8_t i,j=0;
	for(j=0; j<4; j++)
	{
			for(i=0; i<8; i++)
				gpio_config(GPIOA_0+GPIOB_0*j+i, INPUT, PULL_HIGH);
	}
}
*/
void test_key_config(void)
{
	//config_all_pin_as_input();

	gpio_config(GPIOA_3, INPUT, PULL_HIGH);
	REG_AHB0_ICU_DEEP_SLEEP0 = 0x00000008; 
	REG_APB5_GPIO_WUATOD_TYPE= 0x00000008;
	REG_APB5_GPIO_WUATOD_STAT = 0x00000008;
	Delay_ms(2);
	REG_APB5_GPIO_WUATOD_ENABLE = 0x00000008; 
}

#if ENABLE_UNUSED_SECTION_WRITE
void oad_progress_init(void)
{
	flash_read(FLASH_APP_OTA_PROGRESS_FLAG, SEC_UNUSED_ALLOC_FADDR, 1, &upgrade_in_progress, NULL);
	uart_printf("##sure oad_progress_init upgrade_in_progress = %d\n",upgrade_in_progress);
	if(upgrade_in_progress == 0xff)//the ist time,0xffff
		upgrade_in_progress = 0;
}
#endif
extern void stop_advertise(void);

void rw_app_enter(void)
{
#if SYSTEM_SLEEP
	uint8_t sleep_type = 0;
#endif
	//upgrade_in_progress = 0;

    /*
     ***************************************************************************
     * Main loop
     ***************************************************************************
     */
     //gpio wake up pin config
     //gpio_config(GPIOA_3, INPUT, PULL_HIGH);
	//uint8_t buf_sleep[9]="sleep=0\r\n";
	//uint8_t buf_wake[6]="wake\r\n";
    while(1)
    {
        //schedule all pending events
    	rwip_schedule();

    	// Checks for sleep have to be done with interrupt disabled
    	GLOBAL_INT_DISABLE();

    	oad_updating_user_section_pro();

    	if(wdt_disable_flag==1)
    	{
    		wdt_disable();
    	}
		#if 0//ENABLE_BUTTON_CHECK
		 uint8_t wakeup_pin = gpio_get_input(GPIOA_3);
		if(wakeup_pin && (icu_get_sleep_mode() == 1))
		{
			icu_set_sleep_mode(0); //low voltage
			while(gpio_get_input(GPIOA_3) == 0);
			
			delay_ms(50);
			
			//config test key int
			test_key_config();
			
			//stop advertising
			appm_stop_advertising();
		} 
		#endif

#if 0//SYSTEM_SLEEP
    	// Check if the processor clock can be gated
    	if(wakeup_pin) //高电平休眠，低电平唤醒
    	{
    		//stop_advertise() 30s起一次
    		appm_stop_advertising();
		//uart_send(buf_sleep,8);
	    	sleep_type = rwip_sleep();
		
			/*buf_sleep[6] = '0'+ sleep_type;
			uart_send(buf_sleep,9);//1003*/
	    	if((sleep_type & RW_MCU_DEEP_SLEEP) == RW_MCU_DEEP_SLEEP)
	    	{
	    		// 1:idel  0:reduce voltage
	    		if(icu_get_sleep_mode())
	    		{
	    			cpu_idle_sleep();
	    		}
	    		else
	    		{
	    			cpu_reduce_voltage_sleep();
	    		}
	    	}
	    	else if((sleep_type & RW_MCU_IDLE_SLEEP) == RW_MCU_IDLE_SLEEP)
	    	{
	    		cpu_idle_sleep();
	    	}
    	}
#endif  
		//if(REG_APB5_GPIO_WUATOD_TYPE == 0x00000008)//不能完全看是否下降沿，升级完成重启后都是下降沿出发，就会进入休眠
#if 0//SYSTEM_SLEEP	
			// Check if the processor clock can be gated
			sleep_type = rwip_sleep();	
			if((sleep_type & RW_MCU_DEEP_SLEEP) == RW_MCU_DEEP_SLEEP)
			{	
				if((gpio_get_input(GPIOA_3) == 1) && (upgrade_in_progress == 0))
				{
					uart_printf("##sure (gpio_get_input(GPIOA_3) == 1)\n");
					stop_advertise();
					// 1:idel  0:reduce voltage
					if(icu_get_sleep_mode()) //在没有icu_set_sleep_mode(0);之前，无法进入降压休眠
					{
						cpu_idle_sleep();
					}
					else
					{
						//config_all_pin_as_input();
						//gpio_sleep();
						cpu_reduce_voltage_sleep();
					}
					/*uart_printf("##sure APP_STOP_ADV_REQ!!!!!!!!!!!!!upgrade_in_progress = %d\n",upgrade_in_progress);
					ke_msg_send_basic(APP_STOP_ADV_REQ,TASK_APP,TASK_APP);*/
				}
			}
				/*else if(((sleep_type & RW_MCU_IDLE_SLEEP) == RW_MCU_IDLE_SLEEP) &&  (upgrade_in_progress == 2))//Mantis ID:SU2018071666
			{
				cpu_idle_sleep();
			}*/
#endif

		//uart_send(buf_wake,6);

    	Stack_Integrity_Check();
    	GLOBAL_INT_RESTORE();
    }
}

void config_dut_pin(void)
{
	gpio_config(GPIOB_0, INPUT, PULL_LOW);
	gpio_config(GPIOB_1, INPUT, PULL_LOW);
	gpio_config(GPIOB_2, INPUT, PULL_LOW);
	gpio_config(GPIOB_3, INPUT, PULL_LOW);
	Delay_ms(2);
}

uint8_t get_dut_mode(void)
{
	if(!gpio_get_input(GPIOB_0) && !gpio_get_input(GPIOB_1))
		if(gpio_get_input(GPIOB_2) && gpio_get_input(GPIOB_3))
			return 0x1;
	
	gpio_config(GPIOB_0, INPUT, PULL_HIGH);
	gpio_config(GPIOB_1, INPUT, PULL_HIGH);
	gpio_config(GPIOB_2, INPUT, PULL_HIGH);
	gpio_config(GPIOB_3, INPUT, PULL_HIGH);

	return 0;
}
void sys_mode_init(void)
{
	if(get_dut_mode())
		system_mode |= RW_DUT_MODE;
	else
    system_mode |= RW_NO_MODE;
}

void app_nvds_data_init(void)
{	
	memset(ms_lock_flash_buffer, 0, MS_LOCK_SERVICE_DATA_LEN);
}
/**
 *******************************************************************************
 * @brief RW main function.
 *
 * This function is called right after the booting process has completed.
 *
 * @return status   exit status
 *******************************************************************************
 */

extern struct rom_env_tag rom_env;

void rwip_eif_api_init(void);
void rw_main(void)
{
    /*
     ***************************************************************************
     * Platform initialization
     ***************************************************************************
     */

    // Initialize random process
    srand(1);

	config_dut_pin();
    // Iniialize sys run mode
    sys_mode_init();
	
	//get System sleep flag
	system_sleep_init();
	
    // Initialize the exchange memory interface
    emi_init();
	
    // Initialize timer module
    timer_init();
		
	rwip_eif_api_init();
	
    // Initialize the Interrupt Controller
    intc_init();
    // Initialize UART component
	
#if (UART_DRIVER)
    uart_init(UART_RATE_115K2);
	uart_cb_register(uart_rx_handler);
#endif 

    flash_advance_init();
    bdaddr_env_init();
	
#if  0//bk encrypt interface test
	code_sanity_check();
#endif

	 // Initialize NVDS module
    struct nvds_env_tag env;
	env.flash_read = &flash_read;
	env.flash_write = &flash_write;
	env.flash_erase = &flash_erase;
	nvds_init(env);
	
    rom_env_init(&rom_env);

    /*
      ***************************************************************************
      * RW SW stack initialization
      ***************************************************************************
      */
    // Initialize RW SW stack
    rwip_init(0);

    icu_init();
	
    //mcu_clk_config();
    flash_init();

	//init nvds_buf
	app_nvds_data_init();
	
	//ms_reset_uart_para();//add by sure init uart param
	
	test_key_config();
	gpio_cb_register(app_gpio_int_cb);

	REG_AHB0_ICU_INT_ENABLE |= (0x01 << 15); //BLE INT
	REG_AHB0_ICU_IRQ_ENABLE = 0x03;

    // finally start interrupt handling
    GLOBAL_INT_START();

	//UART_PRINTF("start 3\r\n");

	//user_timer_init();
#if ENABLE_UNUSED_SECTION_WRITE
	//init upgrade_in_progress
	oad_progress_init();
#endif

	
    /*
     ***************************************************************************
     * Choose whitch enter to use
     ***************************************************************************
     */
    if((system_mode & RW_DUT_MODE) == RW_DUT_MODE) //dut mode
    {
        rw_dut_enter();
    }
    else if((system_mode & RW_FCC_MODE) == RW_FCC_MODE) //fcc mode
    {
        rw_fcc_enter();
    }
    else //normal mode
    {
        rw_app_enter();
    }
}


#if (UART_DRIVER)
static void uart_rx_handler(uint8_t *buf, uint8_t len)
{
	/*for(uint8_t i=0; i<len; i++)
	{
		uart_printf("0x%x ", buf[i]);
	}
	uart_printf("\r\nlen=%d\r\n",len);
	*/
	
	//app_fcc1_send_lvl(buf, len);
	appUartProcessReadIndEvent(buf, len);
}
#endif

void rwip_eif_api_init(void)
{
	uart_api.read = &uart_read;
	uart_api.write = &uart_write;
	uart_api.flow_on = &uart_flow_on;
	uart_api.flow_off = &uart_flow_off;
}

const struct rwip_eif_api* rwip_eif_get(uint8_t type)
{
    const struct rwip_eif_api* ret = NULL;
    switch(type)
    {
        case RWIP_EIF_AHI:
        {
            ret = &uart_api;
        }
        break;
        #if (BLE_EMB_PRESENT) || (BT_EMB_PRESENT)
        case RWIP_EIF_HCIC:
        {
            ret = &uart_api;
        }
        break;
        #elif !(BLE_EMB_PRESENT) || !(BT_EMB_PRESENT)
        case RWIP_EIF_HCIH:
        {
            ret = &uart_api;
        }
        break;
        #endif 
        default:
        {
            ASSERT_INFO(0, type, 0);
        }
        break;
    }
    return ret;
}

static void Stack_Integrity_Check(void)
{
	if ((REG_PL_RD(STACK_BASE_UNUSED)!= BOOT_PATTERN_UNUSED))
	{
		while(1)
		{
			uart_putchar("Stack_Integrity_Check STACK_BASE_UNUSED fail!\r\n");
		}
	}
	
	if ((REG_PL_RD(STACK_BASE_SVC)!= BOOT_PATTERN_SVC))
	{
		while(1)
		{
			uart_putchar("Stack_Integrity_Check STACK_BASE_SVC fail!\r\n");
		}
	}
	
	if ((REG_PL_RD(STACK_BASE_FIQ)!= BOOT_PATTERN_FIQ))
	{
		while(1)
		{
			uart_putchar("Stack_Integrity_Check STACK_BASE_FIQ fail!\r\n");
		}
	}
	
	if ((REG_PL_RD(STACK_BASE_IRQ)!= BOOT_PATTERN_IRQ))
	{
		while(1)
		{
			uart_putchar("Stack_Integrity_Check STACK_BASE_IRQ fail!\r\n");
		}
	}
	
}


void rom_env_init(struct rom_env_tag *api)
{
	memset(&rom_env,0,sizeof(struct rom_env_tag));
	rom_env.prf_get_id_from_task = prf_get_id_from_task;
	rom_env.prf_get_task_from_id = prf_get_task_from_id;
	rom_env.prf_init = prf_init;	
	rom_env.prf_create = prf_create;
	rom_env.prf_cleanup = prf_cleanup;
	rom_env.prf_add_profile = prf_add_profile;
	rom_env.rwble_hl_reset = rwble_hl_reset;
	rom_env.rwip_reset = rwip_reset;
#if SYSTEM_SLEEP		
	rom_env.rwip_prevent_sleep_set = rwip_prevent_sleep_set;
    rom_env.rwip_prevent_sleep_clear = rwip_prevent_sleep_clear;
	rom_env.rwip_sleep_lpcycles_2_us = rwip_sleep_lpcycles_2_us;
	rom_env.rwip_us_2_lpcycles = rwip_us_2_lpcycles;
	rom_env.rwip_wakeup_delay_set = rwip_wakeup_delay_set;
#endif	
	rom_env.platform_reset = platform_reset;
	rom_env.assert_err = assert_err;
	rom_env.assert_param = assert_param;
	rom_env.Read_Uart_Buf = Read_Uart_Buf;
	rom_env.uart_clear_rxfifo = uart_clear_rxfifo;
	
}

/// @} DRIVERS
