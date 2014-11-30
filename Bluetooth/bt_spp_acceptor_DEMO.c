

/**********************************************
*                Include files                *
**********************************************/
#include "bt_common.h"
#include "bt_spp_api.h"
#include <string.h>

/**********************************************
*         Variables and forward declarations  *
**********************************************/
uint8_t * pPartyB_BD_ADDR;
static uint32_t err;//error code
extern volatile uint8_t DATA_FROM_BLUETOOTH_UART;







/********************************************************************************
 Function		app_BT_SPP_DEMO(void* pvParameters)
 Argumets		:pvParameters
 Returns		:non
 Explanation	:
*********************************************************************************/

struct BlueToothStatusSTRUCT{uint8_t u8PairStatus;
						 uint8_t u8SerialPortStatus;
}stBlueToothStatus;


void app_BT_SPP_Acceptor_DEMO(void)
{  
  
  //portBASE_TYPE queueRETURN;
  uint32_t loop;
  
  
  


  /*Switch Chiron to TCU Compete mode*/
  err = BT_hci_init((uint8_t *)(&BD_ADDR), (uint8_t *)&Device_Name);
 


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
		DATA_FROM_BLUETOOTH_UART = 0; 

  if(TCU_BT_SPP == tcu_event.Service_ID){
    if(TCU_NO_EVENT != tcu_event.eventType){
      /*Process tcu_event*/
      switch (tcu_event.eventType){
        
        /*SPP RECEIVED EVENT*/ 
        case TCU_SPP_DATA_RECEIVE_EVENT:
         // BT_spp_send((uint8_t*)&cSPP_DATA_RECEIVE_buff[0], tcu_spp_data_receive_event.Length_of_Data);           
		  BT_spp_send("GOT IT", strlen("GOT IT"));
          break;
          
        /*SPP DISCONNECT EVENT*/          
        case TCU_SPP_DISCONNECT_EVENT:
          /*write code for TCU_SPP_DISCONNECT_EVENT*/
		  stBlueToothStatus.u8SerialPortStatus = 0;
		  BT_spp_connection_accept_req(pPartyB_BD_ADDR);  /*waiting for serial port to reconnect*/
		  
          break;
          
        /*SPP CONNECT EVENT*/ 
        case TCU_SPP_CONNECT_EVENT:
          /*write code for TCU_SPP_CONNECT_EVENT*/
		  stBlueToothStatus.u8SerialPortStatus = 1;
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
