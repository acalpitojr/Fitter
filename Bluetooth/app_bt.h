/*****************************************************************************
*   This software is copyright protected. All rights reserved
*   Toshiba Electronics Europe GmbH
*   European LSI Design and Engineering Center (ELDEC)
******************************************************************************
*   Description : Bluetooth(SPP) API
******************************************************************************
*   PROJECT     : TC35661 "Bluetooth SPP"
*   MODULE      : Bluetooth SPP API
******************************************************************************
*   VERSION     : 1.00
*   DATE        : 1.12.2012
*****************************************************************************/

#ifndef __APP_BT_H
#define __APP_BT_H

/**********************************************
*                Include files                *
**********************************************/
#include "stdint.h"
#ifndef FreeRTOS
  #define FreeRTOS
#endif
#ifdef FreeRTOS
  //#include "FreeRTOS.h"
  //#include "queue.h"
  //#include "task.h"
#endif

#include "bt_common.h"

/**********************************************
*             Constants & Macros              *
**********************************************/
/*    Configuration parameters    --------------------------------------*/
#define SPP_CONFIG_DEVICE_NAME     "TOSHIBA SPP"
//#define SPP_CONFIG_PEER_BD_ADDR    {0x77, 0x84, 0x00, 0x39, 0x00, 0x00}
#define SPP_CONFIG_LOCAL_BD_ADDR    {0x55, 0x55, 0x55, 0x55, 0x33, 0x55}
#define PIN_CODE                   {0x31, 0x32, 0x33, 0x34}
#define PIN_LENGHT                 4
#define SPP_IO_CAPABILITY          DISPLAY_ONLY//DISPLAY_YES_NO
#define SPP_OOB_AUTHENTICATION     OOB_AUTHENTICATION_DATA_NOT_PRESENT
#define SPP_MITM_PROTECTION        MITM_Protection_Not_Required_NoBonding//MITM_Protection_Required_Dedicated_BondingIO_Capabilities//
#define MAX_NUMBER_OF_SEARCHED_DEVICES 10

/*Maximum Packet length in Complete mode*/
#define UART_PACKET_LENGHT 256

/*Maximum length of SP received data*/
#define SPP_DATA_RECEIVE_LENGHT UART_PACKET_LENGHT


/*  Paired information strred in the external EEPROM*/
#define PAIRED_INFORMATON_STORED_DISABLE   0x00 //do not use paired information stred in the EEPROM
#define PAIRED_INFORMATON_STORED_ENABLE    0x01 //use paired information stred in the EEPROM
#define PAIRED_INFORMATON_STORED_YES_NO PAIRED_INFORMATON_STORED_DISABLE



#define BOARD_TOPASM369_BT
#if defined(BOARD_TOPASM369_BT)
#if defined(BOARD_TOPASM369)
  ERROR !!
    Define BOARD_TOPASM369_BT or BOARD_TOPASM369
#endif
#endif


#if defined(BOARD_TOPASM369)
  #define RTS_low //GPIO_WriteDataBit(GPIO_PE, GPIO_BIT_3, 0); 
  #define RTS_high //GPIO_WriteDataBit(GPIO_PE, GPIO_BIT_3, 1); 
  //#define CTS_BT_interface GPIO_ReadDataBit(GPIO_PE, GPIO_BIT_4)//define CTS pin

  #define BT_SPP_UART 1 //use SIO1 for BT SPP

  #if BT_SPP_UART == 0
   #define UART_BT_SSP   UART0
   #define UART_BT_SSP_TXINT   INTTX0_IRQn
   #define UART_BT_SSP_RXINT   INTRX0_IRQn
   #define INT_SPP_RX_IRQHandler  INTRX0_IRQHandler
   #define INT_SPP_TX_IRQHandler  INTTX0_IRQHandler
  #elif BT_SPP_UART == 1
   #define UART_BT_SSP   UART1
   #define UART_BT_SSP_TXINT   INTTX1_IRQn
   #define UART_BT_SSP_RXINT   INTRX1_IRQn
   #define INT_SPP_RX_IRQHandler  INTRX1_IRQHandler
   #define INT_SPP_TX_IRQHandler  INTTX1_IRQHandler
  #endif
  #define UART_BT_GetRxData UART_GetRxData
  #define UART_BT_GetErrState UART_GetErrState
#elif defined(BOARD_TOPASM369_BT)
  #define RTS_low //GPIO_WriteDataBit(GPIO_PE, GPIO_BIT_3, 0); 
  #define RTS_high //GPIO_WriteDataBit(GPIO_PE, GPIO_BIT_3, 1); 
  //#define CTS_BT_interface GPIO_ReadDataBit(GPIO_PF, GPIO_BIT_0)//define CTS pin as PF0n

  #define UART_BT_SSP   FUART4
  #define UART_BT_SSP_TXINT   INTTX1_IRQn
  #define UART_BT_SSP_RXINT   INTRX1_IRQn
  #define INT_SPP_RX_IRQHandler  INTRX1_IRQHandler
  #define INT_SPP_TX_IRQHandler  INTTX1_IRQHandler
  #define UART_BT_GetRxData FUART_GetRxData
  #define UART_BT_GetErrState FUART_GetErrStatus
#endif

typedef union tag_time_u{
  uint8_t realTIME[24];  
  struct {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t sec;
    uint8_t dummy;
    uint8_t dayOfWeek;    
  }stTime;
} time_u;

extern uint8_t fHR_1HZ_tick;
      
void app_BT_SPP_Acceptor_DEMO(void);
void app_BT_SPP_Initiator_DEMO(void);
void app_BT_BLE_peripherial_DEMO(void);
#endif //__APP_BT_H

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

