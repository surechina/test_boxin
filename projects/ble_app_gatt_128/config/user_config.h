/**
 *******************************************************************************
 *
 * @file user_config.h
 *
 * @brief Application configuration definition
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *******************************************************************************
 */
 
#ifndef _USER_CONFIG_H_
#define _USER_CONFIG_H_


 
 /******************************************************************************
  *############################################################################*
  * 							SYSTEM MACRO CTRL                              *
  *############################################################################*
  *****************************************************************************/

//�����Ҫʹ��GPIO���е��ԣ���Ҫ�������
#define GPIO_DBG_MSG					0
//UARTʹ�ܿ��ƺ�
#define UART_PRINTF_EN					1
//����Ӳ�����Կ���
#define DEBUG_HW						0




 
/*******************************************************************************
 *#############################################################################*
 *								APPLICATION MACRO CTRL                         *
 *#############################################################################*
 *******************************************************************************/
 
//���Ӳ������¿���
#define UPDATE_CONNENCT_PARAM  			1

//��С���Ӽ��
#define BLE_UAPDATA_MIN_INTVALUE		20
//������Ӽ�� 
#define BLE_UAPDATA_MAX_INTVALUE		40
//����Latency
#define BLE_UAPDATA_LATENCY				0
//���ӳ�ʱ
#define BLE_UAPDATA_TIMEOUT				600

#define UUID_IN_ADV_SCAN_DATA			0
//�豸����
#define APP_DFLT_DEVICE_NAME           ("midea_lock_xxxx")

#if UUID_IN_ADV_SCAN_DATA
 //�㲥��UUID����
#define APP_FFC0_ADV_DATA_UUID        "\x11\x06\xA9\x00\x00\xFB\x0E\x00\x23\x9B\xE1\x11\x02\xD1\x00\x1C\x00\x00"
#define APP_FFC0_ADV_DATA_UUID_LEN    (18)

//ɨ����Ӧ������
//#define APP_SCNRSP_DATA        "\x0c\x08\x42\x4B\x33\x34\x33\x35\x2D\x47\x41\x54\x54" //BK3435-GATT"
#define APP_SCNRSP_MANUAL_FAC_DATA        "\x0b\xff\x4d\x73\x00\x00\x30\x30\x30\x30\x30\x30" //BK3435-GATT"
#define APP_SCNRSP_MANUAL_FAC_DATA_LEN    (12)

#define APP_SCNRSP_TX_POWER        "\x02\x0A\x00" 
#define APP_SCNRSP_TX_POWER_LEN    (3)

#else
 //�㲥��UUID����
#define APP_FFC0_ADV_MANUAL_FAC_DATA      "\x0b\xff\x4d\x73\x03\x00\x00\x00\x00\x00\x00\x00" //BK3435-GATT"
#define APP_FFC0_ADV_MANUAL_FAC_DATA_LEN      (12)

//ɨ����Ӧ������
//#define APP_SCNRSP_DATA        "\x0c\x08\x42\x4B\x33\x34\x33\x35\x2D\x47\x41\x54\x54" //BK3435-GATT"
#define APP_SCNRSP_UUID        "\x11\x07\xA9\x00\x00\xFB\x0E\x00\x23\x9B\xE1\x11\x02\xD1\x00\x1C\x00\x00"
#define APP_SCNRSP_UUID_LEN     (18)

#define APP_SCNRSP_TX_POWER        "\x02\x0A\x00" 
#define APP_SCNRSP_TX_POWER_LEN    (3)

#endif



//�㲥��������
/// Advertising channel map - 37, 38, 39
#define APP_ADV_CHMAP           (0x07)
/// Advertising minimum interval - 100ms (160*0.625ms)
#define APP_ADV_INT_MIN         (80)
/// Advertising maximum interval - 100ms (160*0.625ms)
#define APP_ADV_INT_MAX         (80)
/// Fast advertising interval
#define APP_ADV_FAST_INT        (32)


#define ENABLE_BUTTON_CHECK		0


#define	ENABLE_UNUSED_SECTION_WRITE	1
/*******************************************************************************
 *#############################################################################*
 *								DRIVER MACRO CTRL                              *
 *#############################################################################*
 ******************************************************************************/

//DRIVER CONFIG
#define UART_DRIVER						1
#define GPIO_DRIVER						1
#define AUDIO_DRIVER					0
#define RTC_DRIVER						0
#define ADC_DRIVER						0
#define I2C_DRIVER						0
#define PWM_DRIVER						0

#define DEVICE_VERSION				"m01.01.0001"








#endif /* _USER_CONFIG_H_ */