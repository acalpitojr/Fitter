

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








struct BlueToothStatusSTRUCT{uint8_t u8PairStatus;
						 uint8_t u8SerialPortStatus;
}stBlueToothStatus;



/********************************************************************************
 Function		BT_serial_port_profile_task(void)
 Argumets		:none
 Returns		:none
 Explanation	:initializes bluetooth module as a serial port, and manages serial
                 port functions.
*********************************************************************************/
void BT_serial_port_profile_task(void)
{  
  enum{
	     INIT_BLUETOOTH_MODULE,
		 WAIT_FOR_PAIR_AND_SERIAL_CONNECT,
		 ACCEPT_SERIAL_PORT_CONNECTION,
		 SERIAL_PORT_CONNECTED,
		 DONE,
		 
	}state = INIT_BLUETOOTH_MODULE;
	
 while(state!=DONE)
 {
  switch(state)
  { /*-------------------------------------------------------------------------------------------------*/
	case INIT_BLUETOOTH_MODULE
	/*-------------------------------------------------------------------------------------------------*/
		 /*Switch Chiron to TCU Compete mode*/
		 err = BT_hci_init((uint8_t *)(&BD_ADDR), (uint8_t *)&Device_Name);
		 /*Set Inquiry Scan, Page Scan*/
		 err = BT_spp_scan(Scan_Mode_InquiryScan_ON_PageScan_ON); //if(err != API_SUCCESS)
	break;
	/*-------------------------------------------------------------------------------------------------*/
	case WAIT_FOR_PAIR_AND_SERIAL_CONNECT:
	/*-------------------------------------------------------------------------------------------------*/
		/*Receive connection request from remote device*/
		err = BT_spp_rcv_connect_req(&pPartyB_BD_ADDR, SPP_WAIT_RESPONSE_TIME_60s);  /*Address of remote device requesting connection will be passed using pPartyB_BD_ADDR*/
		/*does not come here until connection from master serial port requested*/
		state = ACCEPT_SERIAL_PORT_CONNECTION;
	break;
	/*-------------------------------------------------------------------------------------------------*/
	case ACCEPT_SERIAL_PORT_CONNECTION:
	/*-------------------------------------------------------------------------------------------------*/
		err = BT_spp_connection_accept_req(pPartyB_BD_ADDR);  /*Accept connection to PartyB device*/
		if (tcu_event.eventType == TCU_SPP_CONNECT_EVENT)
		{
			DATA_FROM_BLUETOOTH_UART = 0;
			BT_spp_send("connected", strlen("connected") );
			state = SERIAL_PORT_CONNECTED;
		}
		else
		{
			state = WAIT_FOR_PAIR_AND_SERIAL_CONNECT;
		}
		
	break;
	/*-------------------------------------------------------------------------------------------------*/
	case SERIAL_PORT_CONNECTED:
	/*-------------------------------------------------------------------------------------------------*/
		while(DATA_FROM_BLUETOOTH_UART == 0)  /*we wait for data from uart to process*/
		{}
		DATA_FROM_BLUETOOTH_UART = 0;

		if( ( tcu_event.Service_ID == TCU_BT_SPP ) && ( tcu_event.eventType != TCU_NO_EVENT ) )
		{
			/*Process tcu_event*/
			switch (tcu_event.eventType)
			{
				/*-------------------------------------------------------------------------------------------------*/
				case TCU_SPP_DATA_RECEIVE_EVENT:
				/*-------------------------------------------------------------------------------------------------*/
					// BT_spp_send((uint8_t*)&cSPP_DATA_RECEIVE_buff[0], tcu_spp_data_receive_event.Length_of_Data);
					BT_spp_send("GOT IT", strlen("GOT IT"));
				break;
				/*-------------------------------------------------------------------------------------------------*/
				case TCU_SPP_DISCONNECT_EVENT:
				/*-------------------------------------------------------------------------------------------------*/
					stBlueToothStatus.u8SerialPortStatus = 0;
					state = ACCEPT_SERIAL_PORT_CONNECTION;
				break;
				/*-------------------------------------------------------------------------------------------------*/
				case TCU_SPP_CONNECT_EVENT:
				/*-------------------------------------------------------------------------------------------------*/
					stBlueToothStatus.u8SerialPortStatus = 1;
				break;
				/*-------------------------------------------------------------------------------------------------*/
				case TCU_SPP_DATA_SEND_EVENT:
				/*-------------------------------------------------------------------------------------------------*/
				
				break;
				/*-------------------------------------------------------------------------------------------------*/
				default:
				/*-------------------------------------------------------------------------------------------------*/
				break;
			}/*switch*/
			
		}/*  if( ( tcu_event.Service_ID == TCU_BT_SPP ) && ( tcu_event.eventType != TCU_NO_EVENT ) )   */
		tcu_event.eventType = TCU_NO_EVENT;//clear event
	break;
	/*-------------------------------------------------------------------------------------------------*/
	default:
	/*-------------------------------------------------------------------------------------------------*/
		state = INIT_BLUETOOTH_MODULE;
	break;  
  }/* switch*/
  
 }/*while!DONE*/
 
}    
