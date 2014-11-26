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
*   VERSION     : 1.01
*   DATE        : 1.12.2012
*   Update      : 18.10.2013   LE_RF Parameter improvement
*****************************************************************************/



/**********************************************
*                Include files                *
**********************************************/
#include <stdlib.h>
#include <string.h>
#include "bt_common.h"
#include "ble_api.h"
#include "usart.h"




/**********************************************
*         Variables and forward declarations  *
**********************************************/
uint8_t au8SendBuff[UART_PACKET_LENGHT];

volatile uint8_t DATA_FROM_BLUETOOTH_UART = 0;  /*THIS IS A FLAG THAT WILL BE USED TO LET THE APPLICATION KNOW THAT THE BLUETOOTH HAS SENT DATA THOUGH THE UART*/



volatile t_tcu_event tcu_event;//Callback TCU Event
static uint16_t gReceivePacketCountINT;
/*--------------------------------------------------------------------------
--------------------------------------------------------------------------*/
void processPaketHCI(uint8_t * pt){

//  pt_hci_event_packet = (t_hci_event_packet*)(pt);
  uint32_t * pt_buff_32;
  
  if(rxUARTbuff[0] == HCI_Event_Packet)//check if HCI Event
  {
    switch (rxUARTbuff[1]){
      case 0xFF:
        if((rxUARTbuff[3]==0x08) &(rxUARTbuff[5])==0x99){
            hci_event.event = HCI_EVENT_SET_MODE;
            hci_event.status = rxUARTbuff[6];            
        } else if( (rxUARTbuff[3]==0x08) & (rxUARTbuff[5]==0xa1) & (rxUARTbuff[10]==0x0d) ){
            hci_event.event = HCI_GETTING_FIRMWARE_VERSION_EVENT;
            hci_event.status = rxUARTbuff[11];            
        } else if( (rxUARTbuff[3]==0x08) & (rxUARTbuff[5]==0xa0) & (rxUARTbuff[10]==0x68) ){ 
            hci_event.event = HCI_LE_RF_IMPROVE11;
            hci_event.status = rxUARTbuff[11];            
        }//HCI_LOC_DBUS_WRITE event
        else if( (rxUARTbuff[3]==0x03) & (rxUARTbuff[5]==0xC2) ){
            hci_event.event = HCI_LE_RF_IMPROVE11;
            hci_event.status = rxUARTbuff[6]; 
        }else if( (rxUARTbuff[3]==0x08) & (rxUARTbuff[5]==0xa0) & (rxUARTbuff[10]==0x55))
        {//respond to hci_Patch12_RF_LE_param
            hci_event.event = HCI_LE_RF_IMPROVE12;
            hci_event.status = rxUARTbuff[11];                        
        }
        else if( (rxUARTbuff[3]==0x08) & (rxUARTbuff[5]==0xa0) & (rxUARTbuff[10]==0x56))
        {//respond to hci_Patch13_RF_LE_param
            hci_event.event = HCI_LE_RF_IMPROVE13;
            hci_event.status = rxUARTbuff[11];                        
        }                 
        else if( (rxUARTbuff[3]==0x08) & (rxUARTbuff[5]==0xa0) & (rxUARTbuff[10]==0x57))
        {//respond to hci_Patch13_RF_LE_param
            hci_event.event = HCI_LE_RF_IMPROVE14;
            hci_event.status = rxUARTbuff[11];                        
        }
        else if( (rxUARTbuff[3]==0x08) & (rxUARTbuff[5]==0xa0) & (rxUARTbuff[10]==0x01))
        {//respond to hci_Patch13_RF_LE_param
            hci_event.event = HCI_LE_RF_IMPROVE15;
            hci_event.status = rxUARTbuff[11];                        
        }            
        break;

      case 0x0E:
        pt_buff_32 = (uint32_t *)&rxUARTbuff[2];
        if( *pt_buff_32 == 0x00000404){
            hci_event.event = HCI_EVENT_HARD_RESET;
            hci_event.status = rxUARTbuff[6];
        } else  if( *pt_buff_32 == 0x0c030404){
            hci_event.event = HCI_EVENT_RESET;
            hci_event.status = rxUARTbuff[6];
        } 
        else if( (rxUARTbuff[4]==0x13) & (rxUARTbuff[5]==0x10)){
            hci_event.event = HCI_WRITE_BD_ADDR;
            hci_event.status = rxUARTbuff[6];
        }      
        break;

      case 0x0F://HCI COMMAND STATUS
            hci_event.event = HCI_COMMAND_STATUS;
            hci_event.status = rxUARTbuff[3];            
        break;                 
        
     /*Error Event*/
      case 0x10:
            hci_event.event = HCI_EVENT_ERROR;
            hci_event.status = rxUARTbuff[3];    
        break;    

      default:
        hci_event.event = HCI_NO_EVENT;      
        break;              
      }                     
  }
    
  if(hci_event.event != HCI_NO_EVENT){
    tcu_event.Service_ID = TCU_BT_MNG;    

    /*THIS IS A FLAG LETTING THE BLUETOOTH PROCESS KNOW THAT THERE IS SOMETHING TO LOOK AT*/
    DATA_FROM_BLUETOOTH_UART = 1;
    //SendEvent( btQueueEVENT, (void *)&pt_hci_event, 0 );
  }
}

/*--------------------------------------------------------------------------
--------------------------------------------------------------------------*/
void processPaketTCU(uint8_t * buff){
	uint8_t loop;
  tcu_event.eventType = 0;	//reset tcu_event
        //BT Management Interface
  if(tcu_packet_format.Service_ID == 0xe1){
  switch( tcu_packet_format.OpCode ){
          /*TCU_MNG_INIT_RESP*/
          case TCU_MNG_INIT_RESP://0x81
            tcu_mng_init_resp.Status=*buff++;
            tcu_mng_init_resp.BD_ADDR[0]=*buff++;
            tcu_mng_init_resp.BD_ADDR[1]=*buff++;
            tcu_mng_init_resp.BD_ADDR[2]=*buff++;
            tcu_mng_init_resp.BD_ADDR[3]=*buff++;
            tcu_mng_init_resp.BD_ADDR[4]=*buff++;
            tcu_mng_init_resp.BD_ADDR[5]=*buff++;
            
            tcu_event.Service_ID = TCU_BT_MNG;
            tcu_event.eventType = TCU_MNG_INIT_RESP;
            tcu_event.status = tcu_mng_init_resp.Status;
            tcu_event.ptEvent = (uint32_t *)&tcu_mng_init_resp;                                                        
                  break;
          /*TCU_ACCEPT*/
          case TCU_ACCEPT://0xf1
            tcu_mng_tcu_accept.Status=*buff++;
            tcu_mng_tcu_accept.Command_ServiceID= *buff++;
            tcu_mng_tcu_accept.Command_OpCode= *buff++;
            
            tcu_event.Service_ID = TCU_BT_MNG;
            tcu_event.eventType = TCU_ACCEPT;
            tcu_event.status = tcu_mng_tcu_accept.Status;
            tcu_event.ptEvent = (uint32_t *)&tcu_mng_tcu_accept;                                                                                               
                  break;
          /*TCU_NOT_ACCEPT*/
          case TCU_MNG_TCU_NOT_ACCEPT://0xf2
            tcu_mng_tcu_non_accept.Command_ServiceID= *buff++;
            tcu_mng_tcu_non_accept.Command_OpCode= *buff++;
            
            tcu_event.Service_ID = TCU_BT_MNG;
            tcu_event.eventType = TCU_MNG_TCU_NOT_ACCEPT;
            tcu_event.ptEvent = (uint32_t *)&tcu_mng_tcu_non_accept;                                 
                 
                  break;                                
          /*TCU_MNG_SET_SCAN_RESP*/
          case TCU_MNG_SET_SCAN_RESP://0x8c
            tcu_mng_set_scan_resp.Status=*buff;                                
            
            tcu_event.Service_ID = TCU_BT_MNG;
            tcu_event.eventType = TCU_MNG_SET_SCAN_RESP;
            tcu_event.status = tcu_mng_set_scan_resp.Status;
            tcu_event.ptEvent = (uint32_t *)&tcu_mng_set_scan_resp;                                 
            break;
          /*TCU_MNG_SSP_INFO_EVENT*/
          case TCU_MNG_SSP_INFO_EVENT://0x7d
            tcu_mng_ssp_info_event.OpCode=(eTCU_MNG_SSP_INFO_EVENT_OpCode_)*buff++;
            tcu_mng_ssp_info_event.Parameter_Length=*buff++;                               
            for (loop=0;loop<tcu_mng_ssp_info_event.Parameter_Length;loop++){
                    tcu_mng_ssp_info_event.Parameter[loop]=*buff++;
            }				
            tcu_event.Service_ID = TCU_BT_MNG;
            tcu_event.eventType = TCU_MNG_SSP_INFO_EVENT;
            tcu_event.ptEvent = (uint32_t *)&tcu_mng_ssp_info_event;
            break;
            
          /*TCU_MNG_DISCOVER_REMOTE_DEVICE_RESULT_EVENT*/
          case TCU_MNG_DISCOVER_REMOTE_DEVICE_RESULT_EVENT://0x44
            //tcu_mng_discover_remote_device_result_event.ParameterLength=*buff++;
            //tcu_mng_discover_remote_device_result_event.ParameterLength += (*buff++)<<8;
            tcu_mng_discover_remote_device_result_event.BD_ADDR[0]=*buff++;
            tcu_mng_discover_remote_device_result_event.BD_ADDR[1]=*buff++;
            tcu_mng_discover_remote_device_result_event.BD_ADDR[2]=*buff++;
            tcu_mng_discover_remote_device_result_event.BD_ADDR[3]=*buff++;
            tcu_mng_discover_remote_device_result_event.BD_ADDR[4]=*buff++;
            tcu_mng_discover_remote_device_result_event.BD_ADDR[5]=*buff++;
            tcu_mng_discover_remote_device_result_event.Class_of_Device[0]= *buff++;
            tcu_mng_discover_remote_device_result_event.Class_of_Device[1]= *buff++;
            tcu_mng_discover_remote_device_result_event.Class_of_Device[2]= *buff++;
            tcu_mng_discover_remote_device_result_event.Length_of_Device_Name =*buff++;
            for (loop=0;loop<(tcu_mng_discover_remote_device_result_event.Length_of_Device_Name);loop++){
                    tcu_mng_discover_remote_device_result_event.Device_Name[loop] =*buff++;				
            }
            tcu_event.Service_ID = TCU_BT_MNG;
            tcu_event.eventType = TCU_MNG_DISCOVER_REMOTE_DEVICE_RESULT_EVENT;
            tcu_event.ptEvent = (uint32_t *)&tcu_mng_discover_remote_device_result_event; 									   
            break;
  
          /*TCU_MNG_DISCOVER_REMOTE_SERVICE_EVENT*/
          case TCU_MNG_DISCOVER_REMOTE_SERVICE_EVENT://0x45
            //tcu_mng_discover_remote_device_result_event.ParameterLength=*buff++;
            //tcu_mng_discover_remote_device_result_event.ParameterLength += (*buff++)<<8;
            tcu_mng_discover_remote_service_event.status = *buff++;
            tcu_mng_discover_remote_service_event.BD_ADDR[0]=*buff++;
            tcu_mng_discover_remote_service_event.BD_ADDR[1]=*buff++;
            tcu_mng_discover_remote_service_event.BD_ADDR[2]=*buff++;
            tcu_mng_discover_remote_service_event.BD_ADDR[3]=*buff++;
            tcu_mng_discover_remote_service_event.BD_ADDR[4]=*buff++;
            tcu_mng_discover_remote_service_event.BD_ADDR[5]=*buff++;
            
            tcu_mng_discover_remote_service_event.Service_Count= *buff++;
            if(tcu_mng_discover_remote_service_event.Service_Count != 0){
              tcu_mng_discover_remote_service_event.Service_Type= *buff++;
              
              tcu_mng_discover_remote_service_event.SPP_Ex_info_Profile=*buff++;
              tcu_mng_discover_remote_service_event.SPP_Ext_Info_Length=*buff++;
                                          
              tcu_mng_discover_remote_service_event.Number_of_ServerChannel= *buff++; 
            for (loop=0;loop<(tcu_mng_discover_remote_service_event.SPP_Ext_Info_Length-1);loop++){
              tcu_mng_discover_remote_service_event.SPP_Server_Channel[loop]= *buff++;			
            }
  
            }
            tcu_event.status = tcu_mng_discover_remote_service_event.status;
            tcu_event.Service_ID = TCU_BT_MNG;
            tcu_event.eventType = TCU_MNG_DISCOVER_REMOTE_SERVICE_EVENT;
            tcu_event.ptEvent = (uint32_t *)&tcu_mng_discover_remote_service_event; 									   
            break;
            
          /*TCU_MNG_DISCOVER_REMOTE_DEVICE_COMPLETE_EVENT*/
          case TCU_MNG_DISCOVER_REMOTE_DEVICE_COMPLETE_EVENT://0x43            
            tcu_event.Service_ID = TCU_BT_MNG;
            tcu_event.eventType = TCU_MNG_DISCOVER_REMOTE_DEVICE_COMPLETE_EVENT;                                
            break;
                  
          /*TCU_MNG_PIN_REQUEST_EVENT*/
          case TCU_MNG_PIN_REQUEST_EVENT://0x48
            //tcu_mng_pin_request_event.ParameterLength=*buff++;
            //tcu_mng_pin_request_event.ParameterLength += (*buff++)<<8;
              tcu_mng_pin_request_event.BD_ADDR[0]=*buff++;
              tcu_mng_pin_request_event.BD_ADDR[1]=*buff++;
              tcu_mng_pin_request_event.BD_ADDR[2]=*buff++;
              tcu_mng_pin_request_event.BD_ADDR[3]=*buff++;
              tcu_mng_pin_request_event.BD_ADDR[4]=*buff++;
              tcu_mng_pin_request_event.BD_ADDR[5]=*buff++;
              tcu_mng_pin_request_event.Length_of_Device_Name=*buff++;
              if(tcu_mng_pin_request_event.Length_of_Device_Name<248)
                for (loop=0;loop<(tcu_mng_pin_request_event.Length_of_Device_Name);loop++){
                        tcu_mng_pin_request_event.Device_Name[loop] =*buff++;				
                }
              tcu_event.Service_ID = TCU_BT_MNG;
              tcu_event.eventType = TCU_MNG_PIN_REQUEST_EVENT;
              tcu_event.ptEvent = (uint32_t *)&tcu_mng_pin_request_event;                            
              break;
  
          /*TCU_MNG_CONNECTION_REQUEST_EVENT*/
          case TCU_MNG_CONNECTION_REQUEST_EVENT://0x55
            //tcu_mng_connection_request_event.ParameterLength=*buff++;
            //tcu_mng_connection_request_event.ParameterLength += (*buff++)<<8;
              tcu_mng_connection_request_event.BD_ADDR[0]=*buff++;
              tcu_mng_connection_request_event.BD_ADDR[1]=*buff++;
              tcu_mng_connection_request_event.BD_ADDR[2]=*buff++;
              tcu_mng_connection_request_event.BD_ADDR[3]=*buff++;
              tcu_mng_connection_request_event.BD_ADDR[4]=*buff++;
              tcu_mng_connection_request_event.BD_ADDR[5]=*buff++;
              tcu_mng_connection_request_event.Class_of_Device[0]=*buff++;
              tcu_mng_connection_request_event.Class_of_Device[1]=*buff++;
              tcu_mng_connection_request_event.Class_of_Device[2]=*buff++;                                
              
              tcu_event.Service_ID = TCU_BT_MNG;
              tcu_event.eventType = TCU_MNG_CONNECTION_REQUEST_EVENT;
              tcu_event.ptEvent = (uint32_t *)&tcu_mng_connection_request_event;                                
              break; 
                  
          /*TCU_MNG_CONNECTION_STATUS_EVENT*/	
          case TCU_MNG_CONNECTION_STATUS_EVENT://0x47
              tcu_mng_connection_status_event.Status=*buff++;
              tcu_mng_connection_status_event.BD_ADDR[0]=*buff++;
              tcu_mng_connection_status_event.BD_ADDR[1]=*buff++;
              tcu_mng_connection_status_event.BD_ADDR[2]=*buff++;
              tcu_mng_connection_status_event.BD_ADDR[3]=*buff++;
              tcu_mng_connection_status_event.BD_ADDR[4]=*buff++;
              tcu_mng_connection_status_event.BD_ADDR[5]=*buff++;
              tcu_mng_connection_status_event.Connection_Status =*buff++;
              for (loop=0;loop<16;loop++){
                      tcu_mng_connection_status_event.Link_Key[loop] =*buff++;				
              }
              tcu_mng_connection_status_event.Link_Key_Type =*buff;
  
              tcu_event.Service_ID = TCU_BT_MNG;
              tcu_event.eventType = TCU_MNG_CONNECTION_STATUS_EVENT;
              tcu_event.status = tcu_mng_connection_status_event.Status;
              tcu_event.ptEvent = (uint32_t *)&tcu_mng_connection_status_event;
              break;
  
           /*TCU_MNG_CHANGE_LOCAL_DEVICE_PARAM_RESP*/
          case TCU_MNG_CHANGE_LOCAL_DEVICE_PARAM_RESP://0x91                  
            tcu_event.Service_ID = TCU_BT_MNG;
            tcu_event.eventType = TCU_MNG_CHANGE_LOCAL_DEVICE_PARAM_RESP;
            tcu_event.status = *buff;                                
            break;
                  
           /*TCU_MNG_CONNECTION_ACCEPT_RESP*/
          case TCU_MNG_CONNECTION_ACCEPT_RESP://0x93
            tcu_mng_connection_accept_resp.Status=*buff;
                                            
            tcu_event.Service_ID = TCU_BT_MNG;
            tcu_event.eventType = TCU_MNG_CONNECTION_ACCEPT_RESP;
            tcu_event.status = tcu_mng_connection_accept_resp.Status;
            tcu_event.ptEvent = (uint32_t *)&tcu_mng_connection_accept_resp; 
            break;                                
                                                  
          /*TCU_MNG_PIN_WRITE_RESP*/
          case TCU_MNG_PIN_WRITE_RESP://0x89
            //tcu_mng_pin_write_resp.ParameterLength=*buff++;
            //tcu_mng_pin_write_resp.ParameterLength += (*buff++)<<8;
            tcu_mng_pin_write_resp.status=*buff++;;
            tcu_mng_pin_write_resp.BD_ADDR[0]=*buff++;
            tcu_mng_pin_write_resp.BD_ADDR[1]=*buff++;
            tcu_mng_pin_write_resp.BD_ADDR[2]=*buff++;
            tcu_mng_pin_write_resp.BD_ADDR[3]=*buff++;
            tcu_mng_pin_write_resp.BD_ADDR[4]=*buff++;
            tcu_mng_pin_write_resp.BD_ADDR[5]=*buff++;
            
            tcu_event.Service_ID = TCU_BT_MNG;
            tcu_event.eventType = TCU_MNG_PIN_WRITE_RESP;                                
            break;
          /*TCU_MNG_STANDARD_HCI_SET_RESP*/
          case TCU_MNG_STANDARD_HCI_SET_RESP://0xbd
            //tcu_mng_pin_write_resp.ParameterLength=*buff++;
            //tcu_mng_pin_write_resp.ParameterLength += (*buff++)<<8;
            tcu_mng_standard_hci_set_resp.status = *buff;
            tcu_event.Service_ID = TCU_BT_MNG;
            tcu_event.eventType = TCU_MNG_STANDARD_HCI_SET_RESP; 
            break;
  
          /*TCU_MNG_REMOTE_DEVICE_NAME_AUTO_NOTIFY_EVENT*/
          case TCU_MNG_REMOTE_DEVICE_NAME_AUTO_NOTIFY_EVENT:
  
            tcu_event.Service_ID = TCU_BT_MNG;
            tcu_event.eventType = TCU_MNG_REMOTE_DEVICE_NAME_AUTO_NOTIFY_EVENT;                             
           break;
          default:
                                    
            break;			
  }
	}	
	//BT SPP			
	if(tcu_packet_format.Service_ID == 0xe5){
          switch( tcu_packet_format.OpCode ){
          
            /*TCU_SPP_SETUP_RESP*/
            case TCU_SPP_SETUP_RESP://0x81
              tcu_spp_setup_resp.status=(TCU_SPP_SETUP_RESPOND_STATUS_)*buff;

              tcu_event.Service_ID = TCU_BT_SPP;
              tcu_event.eventType = TCU_SPP_SETUP_RESP;
              tcu_event.status = tcu_spp_setup_resp.status;
              tcu_event.ptEvent = (uint32_t *)&tcu_spp_setup_resp;              
            break;
            
            /*TCU_SPP_CONNECT_EVENT*/
            case TCU_SPP_CONNECT_EVENT://0x43
              //tcu_spp_connect_event.ParameterLength=*buff++;
              //tcu_spp_connect_event.ParameterLength += (*buff++)<<8;
              tcu_spp_connect_event.status=(SPP_EVENT_STATUS)*buff++;
              tcu_spp_connect_event.BD_ADDR[0]=*buff++;
              tcu_spp_connect_event.BD_ADDR[1]=*buff++;
              tcu_spp_connect_event.BD_ADDR[2]=*buff++;
              tcu_spp_connect_event.BD_ADDR[3]=*buff++;
              tcu_spp_connect_event.BD_ADDR[4]=*buff++;
              tcu_spp_connect_event.BD_ADDR[5]=*buff++;
              tcu_spp_connect_event.Negotiated_Frame_Size[0]=*buff++;
              tcu_spp_connect_event.Negotiated_Frame_Size[1]=*buff++;
              tcu_spp_connect_event.Length_of_Device_Name=*buff++;
              
              if(tcu_spp_connect_event.Length_of_Device_Name<248)
              for (loop=0;loop<(tcu_spp_connect_event.Length_of_Device_Name);loop++){
                tcu_spp_connect_event.Device_Name[loop] =*buff++;				
              }

              tcu_event.Service_ID = TCU_BT_SPP;
              tcu_event.eventType = TCU_SPP_CONNECT_EVENT;
              tcu_event.ptEvent = (uint32_t *)&tcu_spp_connect_event;
             
              break;
            
            /*TCU_SPP_DATA_RECEIVE_EVENT*/
            case TCU_SPP_DATA_RECEIVE_EVENT://0x48
              //tcu_spp_data_receive_event.ParameterLength=*buff++;
              //tcu_spp_data_receive_event.ParameterLength += (*buff++)<<8;
              
              tcu_spp_data_receive_event.Length_of_Data=*buff++;
              tcu_spp_data_receive_event.Length_of_Data += (*buff++)<<8;
              
              tcu_spp_data_receive_event.Rxdata = (uint8_t *)(&cSPP_DATA_RECEIVE_buff[0]);
              
              if(tcu_spp_connect_event.Length_of_Device_Name<248)
              for (loop=0;loop<(uint16_t)(tcu_spp_data_receive_event.Length_of_Data);loop++){
                cSPP_DATA_RECEIVE_buff[loop] =*buff++;				
              }
              
              tcu_event.Service_ID = TCU_BT_SPP;
              tcu_event.eventType = TCU_SPP_DATA_RECEIVE_EVENT;
              tcu_event.ptEvent = (uint32_t *)&tcu_spp_data_receive_event;
              
              break;
            
          /*TCU_SPP_DATA_SEND_EVENT*/
          case TCU_SPP_DATA_SEND_EVENT://0xf1                               
            tcu_event.Service_ID = TCU_BT_SPP;
            tcu_event.eventType = TCU_SPP_DATA_SEND_EVENT;
                      
            break;

          /*TCU_SPP_DISCONNECT_EVENT*/
          case TCU_SPP_DISCONNECT_EVENT://0x44
            tcu_spp_disconnect_event.status = *buff++;
            tcu_spp_disconnect_event.BD_ADDR[0]=*buff++;
            tcu_spp_disconnect_event.BD_ADDR[1]=*buff++;
            tcu_spp_disconnect_event.BD_ADDR[2]=*buff++;
            tcu_spp_disconnect_event.BD_ADDR[3]=*buff++;
            tcu_spp_disconnect_event.BD_ADDR[4]=*buff++;
            tcu_spp_disconnect_event.BD_ADDR[5]=*buff++;
            tcu_spp_disconnect_event.Reason = *buff;
            
            tcu_event.Service_ID = TCU_BT_SPP;
            tcu_event.eventType = TCU_SPP_DISCONNECT_EVENT;
            tcu_event.ptEvent = (uint32_t *)&tcu_spp_disconnect_event;
            
            break;

          /*TCU_SPP_SHUTDOWN_RESP*/
          case TCU_SPP_SHUTDOWN_RESP://0x82
            tcu_spp_shutdown_resp.status = (TCU_SPP_SETUP_RESPOND_STATUS_)*buff;
            
            tcu_event.Service_ID = TCU_BT_SPP;
            tcu_event.eventType = TCU_SPP_SHUTDOWN_RESP;
            tcu_event.ptEvent = (uint32_t *)&tcu_spp_shutdown_resp;
            
            break;
              
          /*TCU_SPP_UUID_ASSIGN_RESP*/
          case TCU_SPP_UUID_ASSIGN_RESP:
            tcu_spp_uuid_assign_resp.status = (TCU_SPP_SETUP_RESPOND_STATUS_)*buff;
            
            tcu_event.Service_ID = TCU_BT_SPP;
            tcu_event.eventType = TCU_SPP_UUID_ASSIGN_RESP;
            tcu_event.ptEvent = (uint32_t *)&tcu_spp_uuid_assign_resp;
            break;

          /*TCU_SPP_LINE_NOTIFY_EVENT*/
          case TCU_SPP_LINE_NOTIFY_EVENT:
            tcu_spp_line_notify_event.Line_Status = *buff;
            
            tcu_event.Service_ID = TCU_BT_SPP;
            tcu_event.eventType = TCU_SPP_LINE_NOTIFY_EVENT;
            tcu_event.ptEvent = (uint32_t *)&tcu_spp_line_notify_event;            
           break;
           
          default:                        
          
            break;
            }																		
	}
        else if(tcu_packet_format.Service_ID == 0xD1 || 
                tcu_packet_format.Service_ID == 0xD2 ||
                tcu_packet_format.Service_ID == 0xD3)  
        {                   
          vSmartProcessPacket((buff-7), &tcu_event);
        }       
        if(tcu_event.eventType != 0)
        {
           /*THIS IS A FLAG LETTING THE BLUETOOTH PROCESS KNOW THAT THERE IS SOMETHING TO LOOK AT*/
            DATA_FROM_BLUETOOTH_UART = 1;
            //SendEvent( btQueueEVENT, (void *)&tcu_event, 0 );
        }       
}

/*------------------------------------------------------------------------------
-------------------------------------------------------------------------------*/
void bt_uart_manager(uint8_t dataRX_)   /*we are going to just place this code into our UART1 RX ISR and pass the data to this function there*/
{
  //uint8_t dataRX_;
  
  //uint32_t errorUART_SSP;  	
  //FUART_Err errorUART_SSP;
  
  if(gReceivePacketCountINT < UART_PACKET_LENGHT){
  //dataRX_ = UART_BT_GetRxData(UART_BT_SSP);//read received data

//  #if defined(BOARD_TOPASM369)

 // #elif defined(BOARD_TOPASM369_BT)
   // errorUART_SSP = UART_BT_GetErrState(UART_BT_SSP);
   // if(errorUART_SSP.All !=0)
   // {
      //write code for case of UART ERROR
  //  }
 // #endif
  
  rxUARTbuff[gReceivePacketCountINT]=dataRX_;  
  if (TCU_MODE == BT_Mode){//TCU_MODE
  
    switch(gReceivePacketCountINT++){
      case 0:tcu_packet_format.uPacket_Length.sPacket_Length.Packet_LengthByte[0]=(uint8_t)dataRX_; break;
      case 1:tcu_packet_format.uPacket_Length.sPacket_Length.Packet_LengthByte[1]=(uint8_t)dataRX_; break;
      case 2:tcu_packet_format.uPacket_Length.sPacket_Length.Packet_LengthByte[2]=(uint8_t)dataRX_; break;              
      case 3:tcu_packet_format.Service_ID=dataRX_; break; 
      case 4:tcu_packet_format.OpCode=dataRX_;  break;
      case 5:tcu_packet_format.uLength.sLength.LengthByte[0]=dataRX_; break;
      case 6:tcu_packet_format.uLength.sLength.LengthByte[1]=dataRX_;                               
        if((tcu_packet_format.uLength.Length) != 0L)
        {
        }
        else
        {
            gReceivePacketCountINT=0;
            RTS_high;
            processPaketTCU((uint8_t *)&rxUARTbuff[(tcu_packet_format.OpCode+1)]);
        }                                                
      break;						
      default:
        if(gReceivePacketCountINT < (tcu_packet_format.uLength.Length)+7){
        }else if(gReceivePacketCountINT == (tcu_packet_format.uLength.Length)+7){//last byte
        gReceivePacketCountINT=0;                      
        processPaketTCU((uint8_t *)&rxUARTbuff[7]);                                       
      }																												
      break;
    }//switch
  
  }else{//HCI_MODE                               
    switch(gReceivePacketCountINT++){
      case 0: 
        /*if(dataRX_ == HCI_Event_Packet){               
        eHCI_PACKET_INDICATOR = HCI_EVENT_PACKET;
        eHCI_EVENT=HCI_EVENT_UNKNOWN;
        }*/
        break;              
      case 1:
        break;
      case 2:
        hci_event.ParameterLength = dataRX_;
        break;
      default:
        if(gReceivePacketCountINT < (hci_event.ParameterLength)+3){
        }else if(gReceivePacketCountINT == (hci_event.ParameterLength)+3){//last byte                  
        gReceivePacketCountINT=0;
        processPaketHCI((uint8_t *)&rxUARTbuff[0]);
        }
        break;              
      }//switch                                                          
    }
  }
}



/**
 * \fn      send_tx
 * \brief   Function to start UART transmission
 * \param   strADDR: pointer to data to be sent
 * \param   len: lenght of data to be sent
 * \return  void
 */
void send_tx (uint8_t * strADDR, uint32_t len)
{         
   /*need to call our uart send data function here*/
   extern struct usart_module      usart_module_bt ;
   usart_write_buffer_wait(&usart_module_bt,strADDR, len);

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
