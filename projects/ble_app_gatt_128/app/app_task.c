/**
 ****************************************************************************************
 *
 * @file appm_task.c
 *
 * @brief RW APP Task implementation
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"          // SW configuration

#if (BLE_APP_PRESENT)
#include <string.h>
#include "app_task.h"              // Application Manager Task API
#include "app.h"                      // Application Manager Definition
#include "gapc_task.h"            // GAP Controller Task API
#include "gapm_task.h"          // GAP Manager Task API
#include "gattc_task.h"
#include "arch.h"                    // Platform Definitions

#include "ke_timer.h"             // Kernel timer
#include "app_fcc0.h"              // fff0 Module Definition
#include "fcc0s_task.h"
#include "app_dis.h"              // Device Information Module Definition
#include "diss_task.h"
#include "app_batt.h"             // Battery Module Definition
#include "bass_task.h"
#include "app_oads.h"             
#include "oads_task.h"              
#include "gpio.h"
#include "audio.h"
#include "uart.h"
#include "BK3435_reg.h"
#include "icu.h"
#include "reg_ble_em_cs.h"
#include "lld.h"
#include "wdt.h"

#include "app.h"
#include "flash.h"
extern uint8_t upgrade_in_progress;
extern void Delay_ms(int num); //sync from svn revision 18

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */


static uint8_t appm_get_handler(const struct ke_state_handler *handler_list,
                                ke_msg_id_t msgid,
                                void *param,
                                ke_task_id_t src_id)
{
    // Counter
    uint8_t counter;

    // Get the message handler function by parsing the message table
    for (counter = handler_list->msg_cnt; 0 < counter; counter--)
    {
			
        struct ke_msg_handler handler = (*(handler_list->msg_table + counter - 1));
			
        if ((handler.id == msgid) ||
            (handler.id == KE_MSG_DEFAULT_HANDLER))
        {
            // If handler is NULL, message should not have been received in this state
            ASSERT_ERR(handler.func);

            return (uint8_t)(handler.func(msgid, param, TASK_APP, src_id));
        }
    }

    // If we are here no handler has been found, drop the message
    return (KE_MSG_CONSUMED);
}

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles ready indication from the GAP. - Reset the stack
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_device_ready_ind_handler(ke_msg_id_t const msgid,
                                         void const *param,
                                         ke_task_id_t const dest_id,
                                         ke_task_id_t const src_id)
{
    // Application has not been initialized
    ASSERT_ERR(ke_state_get(dest_id) == APPM_INIT);

    // Reset the stack
    struct gapm_reset_cmd* cmd = KE_MSG_ALLOC(GAPM_RESET_CMD,
                                              TASK_GAPM, TASK_APP,
                                              gapm_reset_cmd);

    cmd->operation = GAPM_RESET;

    ke_msg_send(cmd);

    return (KE_MSG_CONSUMED);
}

static uint8_t adver_state = 1;

void app_set_state(uint8_t v)
{
 	adver_state = v;
}

uint8_t app_get_state(void)
{	
 	return adver_state;
}

/**
 ****************************************************************************************
 * @brief Handles GAP manager command complete events.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_cmp_evt_handler(ke_msg_id_t const msgid,
                                struct gapm_cmp_evt const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
	UART_PRINTF("param->operation = 0x%x, param->status = 0x%x \r\n", param->operation, param->status);
    switch(param->operation)
    {
        // Reset completed
        case (GAPM_RESET):
        {
            if(param->status == GAP_ERR_NO_ERROR)
            {
                // Set Device configuration
                struct gapm_set_dev_config_cmd* cmd = KE_MSG_ALLOC(GAPM_SET_DEV_CONFIG_CMD,
	                                                                   TASK_GAPM, TASK_APP,
                                                                   gapm_set_dev_config_cmd);
                // Set the operation
                cmd->operation = GAPM_SET_DEV_CONFIG;
                // Set the device role - Peripheral
                cmd->role      = GAP_ROLE_PERIPHERAL;
                // Set Data length parameters
                cmd->sugg_max_tx_octets = BLE_MIN_OCTETS;
                cmd->sugg_max_tx_time   = BLE_MIN_TIME;
								
		 		cmd->max_mtu = 131;//BLE_MIN_OCTETS;
                //Do not support secure connections
                cmd->pairing_mode = GAPM_PAIRING_LEGACY;
                
                // load IRK
                memcpy(cmd->irk.key, app_env.loc_irk, KEY_LEN);

                // Send message
                ke_msg_send(cmd);
            }
            else
            {
                ASSERT_ERR(0);
            }
        }
        break;
        case (GAPM_PROFILE_TASK_ADD):
        {
            // Add the next requested service
            if (!appm_add_svc())
            {
                // Go to the ready state
                ke_state_set(TASK_APP, APPM_READY);
							
				appm_start_advertising();
            }
        }
        break;
        // Device Configuration updated
        case (GAPM_SET_DEV_CONFIG):
        {
            ASSERT_INFO(param->status == GAP_ERR_NO_ERROR, param->operation, param->status);

            // Go to the create db state
            ke_state_set(TASK_APP, APPM_CREATE_DB);
            // Add the first required service in the database
            // and wait for the PROFILE_ADDED_IND
            appm_add_svc();
        }
        break;	

        case (GAPM_ADV_NON_CONN):
        case (GAPM_ADV_UNDIRECT):
        case (GAPM_ADV_DIRECT):
		case (GAPM_UPDATE_ADVERTISE_DATA):
        case (GAPM_ADV_DIRECT_LDC):
		{
			if (param->status == GAP_ERR_TIMEOUT)
			{
                ke_state_set(TASK_APP, APPM_READY);
				
				//device not bonded, start general adv
				appm_start_advertising();
            }
			if(param->status == GAP_ERR_CANCELED)
			{
				ke_state_set(TASK_APP, APPM_READY);

				app_set_state(0);
			}
		}
        break;

        default:
        {
            // Drop the message
        }
        break;
    }

    return (KE_MSG_CONSUMED);
}

static int gapc_get_dev_info_req_ind_handler(ke_msg_id_t const msgid,
        struct gapc_get_dev_info_req_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    switch(param->req)
    {
        case GAPC_DEV_NAME:
        {
            struct gapc_get_dev_info_cfm * cfm = KE_MSG_ALLOC_DYN(GAPC_GET_DEV_INFO_CFM,
                                                    src_id, dest_id,
                                                    gapc_get_dev_info_cfm, APP_DEVICE_NAME_MAX_LEN);
            cfm->req = param->req;
            cfm->info.name.length = appm_get_dev_name(cfm->info.name.value);

            // Send message
            ke_msg_send(cfm);
        } break;

        case GAPC_DEV_APPEARANCE:
        {
            // Allocate message
            struct gapc_get_dev_info_cfm *cfm = KE_MSG_ALLOC(GAPC_GET_DEV_INFO_CFM,
                                                    src_id, dest_id,
                                                    gapc_get_dev_info_cfm);
            cfm->req = param->req;
            
            // No appearance
            cfm->info.appearance = 0;

            // Send message
            ke_msg_send(cfm);
        } break;

        case GAPC_DEV_SLV_PREF_PARAMS:
        {
            // Allocate message
            struct gapc_get_dev_info_cfm *cfm = KE_MSG_ALLOC(GAPC_GET_DEV_INFO_CFM,
                    								src_id, dest_id,
                                                    gapc_get_dev_info_cfm);
            cfm->req = param->req;
            // Slave preferred Connection interval Min
            cfm->info.slv_params.con_intv_min = 8;
            // Slave preferred Connection interval Max
            cfm->info.slv_params.con_intv_max = 10;
            // Slave preferred Connection latency
            cfm->info.slv_params.slave_latency = 180;
            // Slave preferred Link supervision timeout
            cfm->info.slv_params.conn_timeout  = 600;  // 6s (600*10ms)

            // Send message
            ke_msg_send(cfm);
        } break;

        default: /* Do Nothing */
			break;
    }


    return (KE_MSG_CONSUMED);
}
/**
 ****************************************************************************************
 * @brief Handles GAPC_SET_DEV_INFO_REQ_IND message.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_set_dev_info_req_ind_handler(ke_msg_id_t const msgid,
        struct gapc_set_dev_info_req_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
	// Set Device configuration
	struct gapc_set_dev_info_cfm* cfm = KE_MSG_ALLOC(GAPC_SET_DEV_INFO_CFM, src_id, dest_id,
                                                 gapc_set_dev_info_cfm);
	// Reject to change parameters
	cfm->status = GAP_ERR_REJECTED;
	cfm->req = param->req;
	// Send message
	ke_msg_send(cfm);

	return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles connection complete event from the GAP. Enable all required profiles
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_connection_req_ind_handler(ke_msg_id_t const msgid,
                                           struct gapc_connection_req_ind const *param,
                                           ke_task_id_t const dest_id,
                                           ke_task_id_t const src_id)
{	
	UART_PRINTF("%s\r\n", __func__);
	uint8_t buf_link[]="AT+LINK\r\n";
    app_env.conidx = KE_IDX_GET(src_id);
    // Check if the received Connection Handle was valid
    if (app_env.conidx != GAP_INVALID_CONIDX)
    {
        // Retrieve the connection info from the parameters
        app_env.conhdl = param->conhdl;

        // Send connection confirmation
        struct gapc_connection_cfm *cfm = KE_MSG_ALLOC(GAPC_CONNECTION_CFM,
                KE_BUILD_ID(TASK_GAPC, app_env.conidx), TASK_APP,
                gapc_connection_cfm);

        cfm->auth = GAP_AUTH_REQ_NO_MITM_NO_BOND;
        // Send the message
        ke_msg_send(cfm);

		
		uart_send(buf_link,9);
		
        /*--------------------------------------------------------------
         * ENABLE REQUIRED PROFILES
         *--------------------------------------------------------------*/
         
        // Enable Battery Service
        app_batt_enable_prf(app_env.conhdl);
		
        // We are now in connected State
        ke_state_set(dest_id, APPM_CONNECTED);
		
		#if UPDATE_CONNENCT_PARAM
		ke_timer_set(APP_PARAM_UPDATE_REQ_IND,TASK_APP,100); 
		#endif	
	        
    }
    else
    {
        // No connection has been establish, restart advertising
		appm_start_advertising();
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles GAP controller command complete events.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_cmp_evt_handler(ke_msg_id_t const msgid,
                                struct gapc_cmp_evt const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
	UART_PRINTF("gapc_cmp_evt_handler operation = %x\r\n",param->operation);
	switch(param->operation)
	{
    	case (GAPC_UPDATE_PARAMS):  //0x09
    	{
			if (param->status != GAP_ERR_NO_ERROR)
        	{
            	UART_PRINTF("gapc update params fail !\r\n");
			}
			else
			{
				UART_PRINTF("gapc update params ok !\r\n");
			}
			
    	} break;

		case (GAPC_SECURITY_REQ): //0x0c
		{
			if (param->status != GAP_ERR_NO_ERROR)
	        {
	            UART_PRINTF("gapc security req fail !\r\n");
	        }
	        else
	        {
	            UART_PRINTF("gapc security req ok !\r\n");
	        }
		}break;
		case (GAPC_BOND): // 0xa
    	{
	        if (param->status != GAP_ERR_NO_ERROR)
	        {
	            UART_PRINTF("gapc bond fail !\r\n");
	        }
	        else
	        {
	            UART_PRINTF("gapc bond ok !\r\n");
	        }
    	}break;
		
		case (GAPC_ENCRYPT): // 0xb
		{
			if (param->status != GAP_ERR_NO_ERROR)
			{
				UART_PRINTF("gapc encrypt start fail !\r\n");
			}
			else
			{
				UART_PRINTF("gapc encrypt start ok !\r\n");
			}
		}
		break;
		

    	default:
    	  break;
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles disconnection complete event from the GAP.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
 									  
static int gapc_disconnect_ind_handler(ke_msg_id_t const msgid,
                                      struct gapc_disconnect_ind const *param,
                                      ke_task_id_t const dest_id,
                                      ke_task_id_t const src_id)
{
	UART_PRINTF("disconnect link reason = 0x%x\r\n",param->reason);
	uint8_t buf_dis[]="AT+DISC\r\n";
#if UPDATE_CONNENCT_PARAM
	if(ke_timer_active(APP_PARAM_UPDATE_REQ_IND, TASK_APP))
	{
		ke_timer_clear(APP_PARAM_UPDATE_REQ_IND, TASK_APP);
	}
#endif

	// Go to the ready state
	ke_state_set(TASK_APP, APPM_READY);

	wdt_disable_flag = 1;

	// Restart Advertising
	if(gpio_get_input(GPIOA_3) == 0)//Mantis ID:SU2018071655
		appm_start_advertising(); //Mantis ID:SU2018071654
	//uart_printf("##sure appm_start_advertising\r\n");

	
	uart_send(buf_dis,9);
    return (KE_MSG_CONSUMED);
}


/**
 ****************************************************************************************
 * @brief Handles profile add indication from the GAP.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapm_profile_added_ind_handler(ke_msg_id_t const msgid,
                                          struct gapm_profile_added_ind *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
    // Current State
    uint8_t state = ke_state_get(dest_id);

    if (state == APPM_CREATE_DB)
    {
        switch (param->prf_task_id)
        {
            default: 
			break;
        }
    }
    else
    {
        ASSERT_INFO(0, state, src_id);
    }

    return KE_MSG_CONSUMED;
}


/*******************************************************************************
* Function: app_period_timer_handler
* Description: app period timer process
* Input: msgid -Id of the message received.
*		param -Pointer to the parameters of the message.
*		dest_id -ID of the receiving task instance (TASK_GAP).
*		ID of the sending task instance.
* Return: If the message was consumed or not.
* Others: void
*******************************************************************************/
static int app_oad_in_progress_timer_handle(ke_msg_id_t const msgid,
										struct gapm_profile_added_ind *param,
										ke_task_id_t const dest_id,
										ke_task_id_t const src_id)
{
	  upgrade_in_progress = 0;
#if ENABLE_UNUSED_SECTION_WRITE
	  flash_write(FLASH_APP_OTA_PROGRESS_FLAG, SEC_UNUSED_ALLOC_FADDR, 1, &upgrade_in_progress, NULL);
#endif
	uart_printf("##sure [oad 5 minute timeout] flash_write upgrade_in_progress  = %d\n",upgrade_in_progress);
	if(ke_timer_active(APP_OAD_IN_PROGRESS_TIMER, TASK_APP))
		ke_timer_clear(APP_OAD_IN_PROGRESS_TIMER, TASK_APP);
	return (KE_MSG_CONSUMED);
}


/*******************************************************************************
 * Function: app_period_timer_handler
 * Description: app period timer process
 * Input: msgid -Id of the message received.
 *		  param -Pointer to the parameters of the message.
 *		  dest_id -ID of the receiving task instance (TASK_GAP).
 *		  ID of the sending task instance.
 * Return: If the message was consumed or not.
 * Others: void
*******************************************************************************/
static int app_period_timer_handler(ke_msg_id_t const msgid,
                                          struct gapm_profile_added_ind *param,
                                          ke_task_id_t const dest_id,
                                          ke_task_id_t const src_id)
{
   	UART_PRINTF("%s\r\n", __func__);
	upgrade_in_progress = 0;
#if ENABLE_UNUSED_SECTION_WRITE
	flash_write(FLASH_APP_OTA_PROGRESS_FLAG, SEC_UNUSED_ALLOC_FADDR, 1, &upgrade_in_progress, NULL);
#endif
	uart_printf("##sure [oad reboot 1 minute timeout] flash_write upgrade_in_progress  = %d\n",upgrade_in_progress);
	if(ke_timer_active(APP_PERIOD_TIMER, TASK_APP))
		ke_timer_clear(APP_PERIOD_TIMER, TASK_APP);
    return KE_MSG_CONSUMED;
}

extern void test_key_config(void);

/*static void delay_ms(int num) 
{
  int x, y;
  for(y = 0; y < num; y ++ )
  {
	  for(x = 0; x < 3260; x++);
  }
	  
}*/

/**
****************************************************************************************
* @brief
*
* @param[in] msgid	  Id of the message received.
* @param[in] param	  Pointer to the parameters of the message.
* @param[in] dest_id   ID of the receiving task instance (TASK_GAP).
* @param[in] src_id	  ID of the sending task instance.
*
* @return If the message was consumed or not.
****************************************************************************************
*/
void stop_advertise(void)
{	
	icu_set_sleep_mode(0);

	//while(gpio_get_input(GPIOA_3) == 0);


	//config test key int
	test_key_config();

	if(ke_state_get(TASK_APP) == APPM_CONNECTED)//Mantis ID:SU2018071654
			appm_disconnect();
	//stop advertising
	appm_stop_advertising();
	uart_printf("##sure appm_stop_advertising\r\n");
	//cpu_reduce_voltage_sleep();
}

static int app_stop_adv_req_handler(ke_msg_id_t const msgid,
								 void const *param,
								 ke_task_id_t const dest_id,
								 ke_task_id_t const src_id)
{   
	icu_set_sleep_mode(0);

	//while(gpio_get_input(GPIOA_3) == 0);

	Delay_ms(50);

	//config test key int
	test_key_config();

	if(ke_state_get(TASK_APP) == APPM_CONNECTED)//Mantis ID:SU2018071654
			appm_disconnect();
	//stop advertising
	appm_stop_advertising();
	uart_printf("##sure appm_stop_advertising\r\n");
	//cpu_reduce_voltage_sleep();
	return (KE_MSG_CONSUMED);
}


/**
****************************************************************************************
* @brief	 app_adv_enable_handler
*
* @param[in] msgid	 Id of the message received.
* @param[in] param	 Pointer to the parameters of the message.
* @param[in] dest_id	 ID of the receiving task instance (TASK_GAP).
* @param[in] src_id	 ID of the sending task instance.
*
* @return If the message was consumed or not.
****************************************************************************************
*/
static int app_enable_adv_handler(ke_msg_id_t const msgid,
							 void const *param,
							 ke_task_id_t const dest_id,
							 ke_task_id_t const src_id)  
{ 
	 icu_set_sleep_mode(1); 
	 
	// while(gpio_get_input(GPIOA_3) == 0);
	// gpio_wakeup();

	 Delay_ms(50);
	 

	//config test key int
	#if 1//!ENABLE_BUTTON_CHECK
	 //test_key_config();
	 gpio_config(GPIOA_3, INPUT, PULL_HIGH);
	 REG_AHB0_ICU_DEEP_SLEEP0 = 0x00000000; 
	 REG_APB5_GPIO_WUATOD_TYPE= 0x00000000;
	 REG_APB5_GPIO_WUATOD_STAT = 0x00000000;
	 Delay_ms(2);
	 REG_APB5_GPIO_WUATOD_ENABLE = 0x00000000; 
	#else
	 gpio_config(GPIOA_3, INPUT, PULL_HIGH);
	 REG_AHB0_ICU_DEEP_SLEEP0 = 0x00000008; 
	 REG_APB5_GPIO_WUATOD_TYPE= 0x00000000;
	 REG_APB5_GPIO_WUATOD_STAT = 0x00000008;
	 Delay_ms(2);
	 REG_APB5_GPIO_WUATOD_ENABLE = 0x00000008; 
	#endif
	 if(!app_get_state())
	 {
		 //start advertising
		 appm_start_advertising();
	 }
	 
	 uart_printf("##sure appm_start_advertising\r\n");
	 return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles reception of all messages sent from the lower layers to the application
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int appm_msg_handler(ke_msg_id_t const msgid,
                            void *param,
                            ke_task_id_t const dest_id,
                            ke_task_id_t const src_id)
{
    // Retrieve identifier of the task from received message
    ke_task_id_t src_task_id = MSG_T(msgid);
    // Message policy
    uint8_t msg_pol          = KE_MSG_CONSUMED;


    switch (src_task_id)
    {
        case (TASK_ID_GAPC):
        {
            // else drop the message
        } break;

        case (TASK_ID_GATTC):
        {
            // Service Changed - Drop
        } break;

        case (TASK_ID_FCC0S):
        {
            // Call the Health Thermometer Module
            msg_pol = appm_get_handler(&app_fcc0_table_handler, msgid, param, src_id);
        } break;
				
        case (TASK_ID_DISS):
        {
            // Call the Device Information Module
            msg_pol = appm_get_handler(&app_dis_table_handler, msgid, param, src_id);
        } break;

        case (TASK_ID_BASS):
        {
            // Call the Battery Module
            msg_pol = appm_get_handler(&app_batt_table_handler, msgid, param, src_id);
        } break;
		
        case (TASK_ID_OADS):
        {
            // Call the Health Thermometer Module
            msg_pol = appm_get_handler(&app_oads_table_handler, msgid, param, src_id);
        } break;

        default:
        {
        } break;
    }

    return (msg_pol);
}


/*******************************************************************************
 * Function: gapc_update_conn_param_req_ind_handler
 * Description: Update request command processing from slaver connection parameters
 * Input: msgid   -Id of the message received.
 *		  param   -Pointer to the parameters of the message.
 *		  dest_id -ID of the receiving task instance
 *		  src_id  -ID of the sending task instance.
 * Return: If the message was consumed or not.
 * Others: void
*******************************************************************************/
static int gapc_update_conn_param_req_ind_handler (ke_msg_id_t const msgid, 
									const struct gapc_param_update_req_ind  *param,
                 					ke_task_id_t const dest_id, ke_task_id_t const src_id)
{

	UART_PRINTF("slave send param_update_req\r\n");
	struct gapc_conn_param  up_param;
	
	up_param.intv_min   = BLE_UAPDATA_MIN_INTVALUE;
	up_param.intv_max   = BLE_UAPDATA_MAX_INTVALUE; 
	up_param.latency    = BLE_UAPDATA_LATENCY;  
	up_param.time_out   = BLE_UAPDATA_TIMEOUT; 
	
	appm_update_param(&up_param);
	
	return KE_MSG_CONSUMED;
}

 
/*******************************************************************************
 * Function: gapc_le_pkt_size_ind_handler
 * Description: GAPC_LE_PKT_SIZE_IND
 * Input: msgid   -Id of the message received.
 *		  param   -Pointer to the parameters of the message.
 *		  dest_id -ID of the receiving task instance
 *		  src_id  -ID of the sending task instance.
 * Return: If the message was consumed or not.
 * Others: void
*******************************************************************************/
static int gapc_le_pkt_size_ind_handler (ke_msg_id_t const msgid, 
									const struct gapc_le_pkt_size_ind  *param,
                 					ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
   	UART_PRINTF("%s \r\n", __func__);
	UART_PRINTF("1max_rx_octets = %d\r\n",param->max_rx_octets);
	UART_PRINTF("1max_rx_time = %d\r\n",param->max_rx_time);
	UART_PRINTF("1max_tx_octets = %d\r\n",param->max_tx_octets);
	UART_PRINTF("1max_tx_time = %d\r\n",param->max_tx_time);
	
	return KE_MSG_CONSUMED;
}

/**
 ****************************************************************************************
 * @brief  GAPC_PARAM_UPDATED_IND
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_param_updated_ind_handler (ke_msg_id_t const msgid, 
									const struct gapc_param_updated_ind  *param,
                 					ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    UART_PRINTF("%s \r\n", __func__);
	UART_PRINTF("con_interval = %d\r\n",param->con_interval);
	UART_PRINTF("con_latency = %d\r\n",param->con_latency);
	UART_PRINTF("sup_to = %d\r\n",param->sup_to);
	
	return KE_MSG_CONSUMED;
}


/**
 ****************************************************************************************
 * @brief  GATTC_MTU_CHANGED_IND
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gattc_mtu_changed_ind_handler(ke_msg_id_t const msgid,
                                     struct gattc_mtu_changed_ind const *ind,
                                     ke_task_id_t const dest_id,
                                     ke_task_id_t const src_id)
{
	UART_PRINTF("%s \r\n",__func__);
	UART_PRINTF("ind->mtu = %d,seq = %d\r\n",ind->mtu,ind->seq_num);
	
 	return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief   GAPC_PARAM_UPDATE_REQ_IND
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
static int gapc_param_update_req_ind_handler(ke_msg_id_t const msgid,
                                struct gapc_param_update_req_ind const *param,
                                ke_task_id_t const dest_id,
                                ke_task_id_t const src_id)
{
	UART_PRINTF("%s \r\n", __func__);
	// Prepare the GAPC_PARAM_UPDATE_CFM message
    struct gapc_param_update_cfm *cfm = KE_MSG_ALLOC(GAPC_PARAM_UPDATE_CFM,
                                             src_id, dest_id,
                                             gapc_param_update_cfm);
	 
	cfm->ce_len_max = 0xffff;
	cfm->ce_len_min = 0xffff;
	cfm->accept = true; 

	// Send message
    ke_msg_send(cfm);
	 
	return (KE_MSG_CONSUMED);
}

/*
 * GLOBAL VARIABLES DEFINITION
 ****************************************************************************************
 */

/* Default State handlers definition. */
const struct ke_msg_handler appm_default_state[] =
{
    // Note: first message is latest message checked by kernel so default is put on top.
    {KE_MSG_DEFAULT_HANDLER,    	(ke_msg_func_t)appm_msg_handler},
    {GAPM_DEVICE_READY_IND,     	(ke_msg_func_t)gapm_device_ready_ind_handler},
    {GAPM_CMP_EVT,             		(ke_msg_func_t)gapm_cmp_evt_handler},
    {GAPC_GET_DEV_INFO_REQ_IND, 	(ke_msg_func_t)gapc_get_dev_info_req_ind_handler},
    {GAPC_SET_DEV_INFO_REQ_IND, 	(ke_msg_func_t)gapc_set_dev_info_req_ind_handler},
    {GAPC_CONNECTION_REQ_IND,   	(ke_msg_func_t)gapc_connection_req_ind_handler},
    {GAPC_CMP_EVT,             		(ke_msg_func_t)gapc_cmp_evt_handler},
    {GAPC_DISCONNECT_IND,       	(ke_msg_func_t)gapc_disconnect_ind_handler},
    {GAPM_PROFILE_ADDED_IND,    	(ke_msg_func_t)gapm_profile_added_ind_handler},
    {GAPC_LE_PKT_SIZE_IND,			(ke_msg_func_t)gapc_le_pkt_size_ind_handler},
    {GAPC_PARAM_UPDATED_IND,		(ke_msg_func_t)gapc_param_updated_ind_handler},
    {GATTC_MTU_CHANGED_IND,			(ke_msg_func_t)gattc_mtu_changed_ind_handler},	
    {GAPC_PARAM_UPDATE_REQ_IND, 	(ke_msg_func_t)gapc_param_update_req_ind_handler},
    {APP_PARAM_UPDATE_REQ_IND, 		(ke_msg_func_t)gapc_update_conn_param_req_ind_handler},
    {APP_PERIOD_TIMER,				(ke_msg_func_t)app_period_timer_handler},
	{APP_STOP_ADV_REQ,				(ke_msg_func_t)app_stop_adv_req_handler},
	{APP_ENABLE_ADV_REQ,			(ke_msg_func_t)app_enable_adv_handler},
	{APP_OAD_IN_PROGRESS_TIMER,		(ke_msg_func_t)app_oad_in_progress_timer_handle},
};

/* Specifies the message handlers that are common to all states. */
const struct ke_state_handler appm_default_handler = KE_STATE_HANDLER(appm_default_state);

/* Defines the place holder for the states of all the task instances. */
ke_state_t appm_state[APP_IDX_MAX];

#endif //(BLE_APP_PRESENT)

/// @} APPTASK
