/*****************************************************************************
*   This software is copyright protected. All rights reserved
*   Toshiba Electronics Europe GmbH
*   European LSI Design and Engineering Center (ELDEC)
******************************************************************************
*   Description : Bluetooth(SPP) DEMO
******************************************************************************
*   PROJECT     : TC35661 "Bluetooth SPP"
*   MODULE      : Bluetooth SPP Driver
******************************************************************************
*   VERSION     : 1.01
*   DATE        : 2.05.2012
*****************************************************************************/

/*****************************************************************************
DEMO description:
In this sample TC35661 is passive device. It accepts connection ininitiated by
PartyB device.

TC35661 starts connection procedure if PartyB supports SPP.
If connection succeeds the demo sample transmits string "! HELLO !" 
and sends back all received data.
*****************************************************************************/

/**********************************************
*                Include files                *
**********************************************/
#include "bt_common.h"
#include "bt_spp_api.h"

/**********************************************
*         Variables and forward declarations  *
**********************************************/
uint8_t * pPartyB_BD_ADDR;
static uint32_t err;//error code
extern uint8_t DATA_FROM_BLUETOOTH_UART;
/**********************************************
*             Constants & Macros              *
**********************************************/
#define tskIDLE_PRIORITY			( ( unsigned portBASE_TYPE ) 0U )
#define tskLED1_PRIORITY                        ( ( unsigned portBASE_TYPE ) 1U )
#define tskLED2_PRIORITY                        ( ( unsigned portBASE_TYPE ) 2U )
#define mainQUEUE_RECEIVE_TASK_PRIORITY         ( ( unsigned portBASE_TYPE ) 3U )


/**********************************************
*  Static variables and forward declarations  *
**********************************************/
/* The rate at which data is sent to the queue, specified in milliseconds. */
#define mainQUEUE_SEND_FREQUENCY_MS			( 10 / portTICK_RATE_MS )

/**************************************
**************************************/
//void vApplicationStackOverflowHook (xTaskHandle *pxTask, signed portCHAR *pcTaskName)
//{
	/* This function will get called if a task overflows its stack. */
  //(void)pxTask;
  //(void)pcTaskName;
 // while(1);
//}

/**************************************
**************************************/
void error(void){
  while(1);
}
/**************************************
**************************************/

/********************************************************************************
 Function		app_BT_SPP_DEMO(void* pvParameters)
 Argumets		:pvParameters
 Returns		:non
 Explanation	:
*********************************************************************************/


void app_BT_SPP_Acceptor_DEMO(void)
{  
  
  //portBASE_TYPE queueRETURN;
  uint32_t loop;
  
  
  


  /*Switch Chiron to TCU Compete mode*/
  err = BT_hci_init((uint8_t *)(&BD_ADDR), (uint8_t *)&Device_Name);
  //err = BT_hci_init__((uint8_t *)(&BD_ADDR), (uint8_t *)&Device_Name);
//  if(err != API_SUCCESS){
  //  while(1){};//endless loop
    //}


  /*Set Inquiry Scan, Page Scan*/
  err = BT_spp_scan(Scan_Mode_InquiryScan_ON_PageScan_ON);
  if(err != API_SUCCESS){
    while(1){};//endless loop
    }
  
  /*Receive connection request from remote device*/
  /*Address of remote device requesting connection will be passed using pPartyB_BD_ADDR*/
  err = BT_spp_rcv_connect_req(&pPartyB_BD_ADDR, SPP_WAIT_RESPONSE_TIME_60s);
  if(err != API_SUCCESS){
    while(1){};//endless loop
    }

  /*Accept connection to PartyB device*/
  err = BT_spp_connection_accept_req(pPartyB_BD_ADDR);
  if(err != API_SUCCESS){
    while(1){};//endless loop
   }  
    
  /*Send string "! HELLO !"*/
  DATA_FROM_BLUETOOTH_UART = 0;
  if (tcu_event.eventType == TCU_SPP_CONNECT_EVENT){
    BT_spp_send("! HELLO !", 9);
  }
  
  /*run demo application*/
  while(1){        
      while(DATA_FROM_BLUETOOTH_UART == 0)  /*we wait for data from uart to process*/
         {
         }

  if(TCU_BT_SPP == tcu_event.Service_ID){
    if(TCU_NO_EVENT != tcu_event.eventType){
      /*Process tcu_event*/
      switch (tcu_event.eventType){
        
        /*SPP RECEIVED EVENT*/ 
        case TCU_SPP_DATA_RECEIVE_EVENT:
          BT_spp_send((uint8_t*)&cSPP_DATA_RECEIVE_buff[0], tcu_spp_data_receive_event.Length_of_Data);           
          break;
          
        /*SPP DISCONNECT EVENT*/          
        case TCU_SPP_DISCONNECT_EVENT:
          /*write code for TCU_SPP_DISCONNECT_EVENT*/
          break;
          
        /*SPP CONNECT EVENT*/ 
        case TCU_SPP_CONNECT_EVENT:
          /*write code for TCU_SPP_CONNECT_EVENT*/
          break;
          
        /*SPP SEND EVENT*/           
        case TCU_SPP_DATA_SEND_EVENT:
          /*write code for TCU_SPP_DATA_SEND_EVENT*/          
          break;
          
        default:
        break;      
      }            
    }
	
    tcu_event.eventType = TCU_NO_EVENT;//clear event
    }
    
  }   
   
 
}    
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
