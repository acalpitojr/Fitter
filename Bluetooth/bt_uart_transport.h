/*****************************************************************************
*   This software is copyright protected. All rights reserved
*   Toshiba Electronics Europe GmbH
*   European LSI Design and Engineering Center (ELDEC)
******************************************************************************
*   Description : Bluetooth(SPP) driver
******************************************************************************
*   PROJECT     : TC35661 "Bluetooth SPP"
*   MODULE      : Bluetooth SPP driver
******************************************************************************
*   VERSION     : 1.00
*   DATE        : 1.12.2012
*****************************************************************************/

#ifndef _H_BT_UART_TRANSPORT_
#define _H_BT_UART_TRANSPORT_


/**********************************************
*                Include files                *
**********************************************/
#include "stdint.h"


/**********************************************
*             Constants & Macros              *
**********************************************/
/* Definitions of TCU_MODE & HCI_MODE */
#define TCU_MODE	0
#define HCI_MODE	1

/*Packet Format in TCU Complete mode*/
typedef struct t_tcu_packet_format_{
	union {
	  struct {
	  	uint8_t Packet_LengthByte[3];      
	  }sPacket_Length;
	  uint32_t Packet_LengthLong;
	}uPacket_Length;
	//uint8_t Packet_Length[3];								 
	uint8_t Service_ID;
	uint8_t OpCode;
	union {
	  struct {
	  	uint8_t LengthByte[2];      
	  }sLength;
	  uint16_t Length;
	}uLength;
	//uint16_t Length;
	uint8_t *Parameter;
}t_tcu_packet_format;
extern t_tcu_packet_format tcu_packet_format;

/*TCU Service ID*/
typedef enum
{
  TCU_BT_HCI =          0x04,
  TCU_BT_MNG =          0xe1,
  TCU_BTLE_MNG =        0xd1,
  TCU_BT_SPP =          0xe5,
  TCU_BT_VEN =          0xEF,  
  TCU_BT_LE_MNG =       0xD1,
  TCU_BT_LE_CLI =       0xD2,
  TCU_BT_LE_SRV =       0xD3,
  TCU_BT_CNTRL =        0xFF//message from the application layer to abort driver process
} eTCU_SERVICE_ID_;

/* TCU Callback event structure*/
typedef struct t_tcu_event_{
  eTCU_SERVICE_ID_ Service_ID;
  uint8_t eventType;
  uint8_t status;   
  uint32_t * ptEvent;//pointer to event structure
}t_tcu_event;

/*Decleration of TCU Callback event tcu_event*/
volatile extern t_tcu_event tcu_event;
void bt_uart_manager(uint8_t dataRX_);  /*this needs to be in the systems uart rx isr*/
void send_tx (uint8_t * strADDR, uint32_t len);
#endif                          /* _H_BT_UART_TRANSPORT_ */

/****************************************************************************/


/***********************  BEGIN of DISCLAIMER   *****************************

- TOSHIBA is continually working to improve the quality and reliability of
  its products.
  Nevertheless, semiconductor devices in general can malfunction or fail
  due to their inherent electrical sensitivity and vulnerability to physical
  stress. It is the responsibility of the buyer, when utilizing TOSHIBA
  products, to comply with the standards of safety in making a safe design
  for the entire system, and to avoid situations in which a malfunction or
  failure of such TOSHIBA products could cause loss of human life,
  bodily injury or damage to property.

  In developing your designs, please ensure that TOSHIBA products are used
  within specified operating ranges as set forth in the most recent TOSHIBA
  products specifications. Also, please keep in mind the precautions and
  conditions set forth in the “Handling Guide for Semiconductor Devices, or
  TOSHIBA Semiconductor Reliability Handbook etc..

- The Toshiba products listed in this document are intended for usage in
  general electronics applications (computer, personal equipment,
  office equipment, measuring equipment,industrial robotics, domestic
  appliances, etc.). These Toshiba products are neither intended nor
  warranted for usage in equipment that requires extraordinarily high quality
  and/or reliability or a malfunction or failure of which may cause loss
  of human life or bodily injury (“Unintended Usage).
  Unintended Usage include atomic energy control instruments, airplane or
  spaceship instruments, transportation instruments, traffic signal
  instruments, combustion control instruments, medical instruments,
  all types of safety devices, etc..
  Unintended Usage of Toshiba products listed in this document shall be made
  at the customer’s own risk.

- Toshiba assumes no liability for any damage or losses (including but not
  limited to, loss of business profit,business interruption, loss of business
  information,and other pecuniary losses) occurring from the use of,
  or inability to use, this product.

- The products described in this document are subject to the foreign exchange
  and foreign trade laws.

- The products described in this document contain components made in the
  United States and subject to export control of the U.S. authorities.
  Diversion contrary to the U.S. law is prohibited.

- The information contained herein is presented only as a guide for the
  applications of our products.
  No responsibility is assumed by TOSHIBA CORPORATION for any infringements
  of intellectual property or other rights of the third parties which may
  result from its use.
  No license is granted by implication or otherwise under any intellectual
  property or other rights of TOSHIBA CORPORATION or others.

- The information contained herein is subject to change without notice.

****************************  END of DISCLAIMER ****************************/
/*eof*/
