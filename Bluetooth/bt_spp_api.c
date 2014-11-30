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

/**********************************************
*                Include files                *
**********************************************/
//#include "main.h"
#include "bt_spp_api.h"
#include "systick_counter.h"

#define errQUEUE_EMPTY     (0u)    /*from old projdef used in freetos*/
/**********************************************
*         Variables and forward declarations  *
**********************************************/
t_tcu_event ul_BT_SPP_rcvValue;//used as respond/evnet value 

/*Device Name*/
const uint8_t Device_Name[] = SPP_CONFIG_DEVICE_NAME;

/*Local Device BD_ADDR*/
const uint8_t BD_ADDR[6] = SPP_CONFIG_LOCAL_BD_ADDR;//{0x77, 0x84, 0x00, 0x39, 0x00, 0x00};

/*PIN code */
const uint8_t PIN[PIN_LENGHT] = PIN_CODE; 

t_bt_remote_device bt_remote_device[MAX_NUMBER_OF_SEARCHED_DEVICES];
t_bt_service_information bt_service_information;


extern volatile uint8_t DATA_FROM_BLUETOOTH_UART;


extern void configure_uart_bt(void);
/* Functions */
/***********************************************************************
 * @brief This API initializes Complete mode on Chiron
 * @param BD_ADDR
 *   Local Device BD_ADDR
 * @param Dev_Name
 *   Local Device Name
 * @return API_RESULT:
 *   API_SUCCESS:  Complete Mode on Chiron initialized successfully
 *   Value different from API_SUCCESS: see error code list bt_error.h
 */

#if 0
API_RESULT BT_hci_init__(unsigned char *BD_ADDR_, unsigned char *Dev_Name)
{
  unsigned long ret = API_SUCCESS;    
  int32_t queueRETURN;
  
  /*Initiate hci_event */
  pt_hci_event = &hci_event;
  hci_event.event = HCI_NO_EVENT;
  
  /*Start from HCI mode. Chiron after reset is in the HCI mode.*/
  BT_Mode = HCI_MODE;

//  hci_RESET();//send reset sequence
  /*Process sequence*/  
    while(BT_Mode == HCI_MODE){ 
    //ul_BT_SPP_rcvValue = 0;
    hci_event.event = HCI_NO_EVENT;
    
    queueRETURN = ReceiveEvent( btQueueEVENT, &ul_BT_SPP_rcvValue, HCI_CHANGE_TO_COMPLETE_MODE_TIMEOUT );
    if(queueRETURN != pdPASS){
      queueRETURN = errQUEUE_EMPTY;
      ret = HCI_ERR_TIMEOUT;
      break;
    }else{       
      //while(CTS_BT_interface == 1){};//wait for CTS low, Chiron ready
      
      switch (hci_event.event) {      
        case HCI_EVENT_HARD_RESET:
            hci_RESET();break;
      
        case HCI_EVENT_RESET:
            hci_Get_Version();break;


        case HCI_GETTING_FIRMWARE_VERSION_EVENT:
          //HCI_SET_MODE();break;
          //hci_Write_BD_ADDR((uint8_t *)(&BD_ADDR_)); break;
          hci_Write_BD_ADDR(BD_ADDR_); break;
          //hci_M2_SET_DeepSleep();break;
            
            
            
        case HCI_EVENT_ERROR: 
            //write code for HCI_EVENT_ERROR
            hci_RESET(); break;      
      
        case HCI_M2_DEEP_SLEEP_SET:
          hci_SET_MODE();break;
          //hci_Write_BD_ADDR((uint8_t *)(&BD_ADDR)); 
          break;                
          
        case HCI_WRITE_BD_ADDR:
          //hci_SET_MODE();
          hci_M2_SET_DeepSleep();
          break;
                    
        case HCI_EVENT_SET_MODE:
            BT_Mode = TCU_MODE;//switch to TCU            
            /*Wait 30ms*/
            queueRETURN = ReceiveEvent( btQueueEVENT, &ul_BT_SPP_rcvValue, HCI_WAIT_RESPONSE_TIME_30ms );
  
            tcu_mng_init_req.Supported_Profiles = SUPPORTED_PROFILE_SPP;
            tcu_mng_init_req.Device_Name_ = Dev_Name;
            tcu_mng_init_req.Paired_Information_Stored = PAIRED_INFORMATON_STORED_YES_NO;
            tcu_MNG_INIT_REQ(&tcu_mng_init_req); 
            break;   
        default:break;  
      }
    }
  }//while
  
  if(ret == API_SUCCESS){
    //wait for tcu_MNG_INIT_RESP
    queueRETURN = ReceiveEvent( btQueueEVENT, &ul_BT_SPP_rcvValue, HCI_CHANGE_TO_COMPLETE_MODE_TIMEOUT );
    if(queueRETURN != pdPASS){
      queueRETURN = errQUEUE_EMPTY;
      ret = HCI_ERR_TIMEOUT;
    }else{ 
      if(tcu_event.Service_ID == TCU_BT_MNG){
        //TCU_BT_MNG_INTERFACE
        switch (tcu_event.eventType)
        {
          //TCU_MNG_INIT_RESP  
          case TCU_MNG_INIT_RESP:
            tcu_event.eventType = TCU_NO_EVENT; //reset
            ret = errorCode(tcu_event.status);
          break;
          
          default: 
          tcu_event.eventType = TCU_NO_EVENT;           
        }  
      }
    }
  }
  
  return ret;
}

#endif
/***********************************************************************
 * @brief This API initializes Complete mode on Chiron
 * @param BD_ADDR
 *   Local Device BD_ADDR
 * @param Dev_Name
 *   Local Device Name
 * @return API_RESULT:
 *   API_SUCCESS:  Complete Mode on Chiron initialized successfully
 *   Value different from API_SUCCESS: see error code list bt_error.h
 */
API_RESULT BT_hci_init(unsigned char *BD_ADDR_, unsigned char *Dev_Name)
{
  unsigned long ret = API_SUCCESS;    
  int32_t queueRETURN;
uint16_t loop;
uint8_t commandRecord=0;
uint32_t timeout =0xffffffff;
  /*Initiate hci_event */
  pt_hci_event = (t_hci_event *)&hci_event;
  hci_event.event = HCI_NO_EVENT;
  
  /*Start from HCI mode. Chiron after reset is in the HCI mode.*/
  BT_Mode = HCI_MODE;

  /*TURN ON BLUETOOTH HERE AFTER BASIC INITIALIZATIONS*/
    DATA_FROM_BLUETOOTH_UART = 0;  /*this will change in the uart rx interrupt*/



         hci_event.event = HCI_NO_EVENT;
    configure_uart_bt();  /*inits the uart, and reset line for the bluetooth.  Once the bluetooth turns on, it will start sending hard reset packet*/



  /*Process sequence*/  
    while(BT_Mode == HCI_MODE)
    {

    





    //timeout = 0xffffffff;

   // while( (DATA_FROM_BLUETOOTH_UART == 0 ) &&(timeout>0) )
   // {
    //    timeout--;
  //  }
    while( (DATA_FROM_BLUETOOTH_UART == 0 ))
	{}
		
    if(DATA_FROM_BLUETOOTH_UART == 0)
    {

      ret = HCI_ERR_TIMEOUT;
      break;
    }
    else  /*bluettooth send some packet through the uart and set hci_event.event to some value*/
    {
        DATA_FROM_BLUETOOTH_UART = 0;
      //while(CTS_BT_interface == 1){};//wait for CTS low, Chiron ready
      
      switch (hci_event.event) {      
        case HCI_EVENT_HARD_RESET:
            hci_RESET();break;

        case HCI_COMMAND_STATUS:
          //check HCI_COMMAND_STATUS
          if(hci_event.status != 0)
            hci_RESET();
        break;
        case HCI_EVENT_RESET:
            hci_Get_Version();break;

        case HCI_EVENT_ERROR: 
            //write code for HCI_EVENT_ERROR
            hci_RESET(); break;  
            
        case HCI_GETTING_FIRMWARE_VERSION_EVENT:
          //HCI_SET_MODE();break;
          //hci_Write_BD_ADDR((uint8_t *)(&BD_ADDR_)); break;
          hci_Write_BD_ADDR(BD_ADDR_); break;
          //hci_M2_SET_DeepSleep();break;
                                             
        case HCI_WRITE_BD_ADDR:
          //hci_SET_MODE();
          hci_M2_SET_DeepSleep();
          //hci_Patch11_RF_LE_param();
          break;
          
        //case HCI_M2_DEEP_SLEEP_SET:
        case HCI_LE_RF_IMPROVE11:
          #ifdef FIRMWARE_501 
            if(commandRecord<60)
              hci_Patch11_RF_LE_param(commandRecord++);
            else
            {
              hci_Patch12_RF_LE_param();
            }                                                              
            //hci_SET_MODE();
          #else          
            hci_SET_MODE();
          #endif
          break;                
                  
      case  HCI_LE_RF_IMPROVE12:
        hci_Patch13_RF_LE_param();
        break;
      case  HCI_LE_RF_IMPROVE13:
        hci_Patch14_RF_LE_param();
        break;                
      case  HCI_LE_RF_IMPROVE14:
        hci_Patch15_RF_LE_param();
        break;           

      case  HCI_LE_RF_IMPROVE15:        
            hci_SET_MODE();
            break;
        
        case HCI_EVENT_SET_MODE:
            BT_Mode = TCU_MODE;//switch to TCU            

            /*Wait 30ms*/
           // queueRETURN = ReceiveEvent( btQueueEVENT, &ul_BT_SPP_rcvValue, HCI_WAIT_RESPONSE_TIME_30ms );
            delay_cycles_ms(30);


            tcu_mng_init_req.Supported_Profiles = SUPPORTED_PROFILE_SPP;
            tcu_mng_init_req.Device_Name_ = Dev_Name;
            tcu_mng_init_req.Paired_Information_Stored = PAIRED_INFORMATON_STORED_YES_NO;
            tcu_MNG_INIT_REQ(&tcu_mng_init_req); 
            break;   
        default:          
        break;  
      }
    }
  }//while
  
  if(ret == API_SUCCESS){
    //wait for tcu_MNG_INIT_RESP  waitinf for max of  10,000 ms
    //queueRETURN = ReceiveEvent( btQueueEVENT, &ul_BT_SPP_rcvValue, HCI_CHANGE_TO_COMPLETE_MODE_TIMEOUT );
      DATA_FROM_BLUETOOTH_UART = 0;
      timeout = 0xffffffff;
      while( (DATA_FROM_BLUETOOTH_UART == 0 ) &&(timeout>0) )
      {
             timeout--;  /*CHANGE THIS TO BE A 10,000 ms TIMEOUT INSTEAD*/
       }



    // if(queueRETURN != pdPASS)
    if(DATA_FROM_BLUETOOTH_UART == 0)
    {
      queueRETURN = errQUEUE_EMPTY;
      ret = HCI_ERR_TIMEOUT;
    }
    else  /*we got data from uart to process*/
    {
        DATA_FROM_BLUETOOTH_UART = 0;
      if(tcu_event.Service_ID == TCU_BT_MNG){
        //TCU_BT_MNG_INTERFACE
        switch (tcu_event.eventType)
        {
          //TCU_MNG_INIT_RESP  
          case TCU_MNG_INIT_RESP:
            tcu_event.eventType = TCU_NO_EVENT; //reset
            ret = errorCode(tcu_event.status);
          break;
          
          default: 
          tcu_event.eventType = TCU_NO_EVENT;           
        }  
      }
    }
  }
  
  return ret;
}

/***********************************************************************
 * @brief This API starts remote device search function.
 * @param max_Number_of_Reports_
    This parameter defines number of searched device.
    Input parameter 
 * @param remote_dev
    Structure contains information about found remote devices.
    Output parameter.
 * @param foundDevices
     Number of found remote devices.
    Output parameter.
 * @return API_RESULT:
 *   API_SUCCESS: API finished successfully.
 *   Value different from API_SUCCESS: see error code list bt_error.h
 */
API_RESULT BT_spp_discover_remote_device( uint8_t max_Number_of_Reports_, t_bt_remote_device * remote_dev, uint8_t * foundDevices){
  int32_t queueRETURN;
  uint32_t process = 0;
  uint32_t ret = API_SUCCESS;
  uint8_t curr_remote_dev=0;//current discovedred remote device
  uint32_t temp, loop;
  uint32_t timeout = 0xffffffff;

  DATA_FROM_BLUETOOTH_UART = 0;  /*clear the flag before we send a command.  Maybe put this in the tx_data to bt uart function?*/
  /*HERE WE WAIT FOR A MESSAGE TO COME THROUGH THE UART*/
     tcu_event.eventType = 0;//reset tcu_event
  tcu_MNG_DISCOVER_REMOTE_DEVICE_REQ(max_Number_of_Reports_);
  
  
  while(process == 0){




  //  queueRETURN = ReceiveEvent( btQueueEVENT, &ul_BT_SPP_rcvValue, (portMAX_DELAY) );//200ms portMAX_DELAY

    timeout = 0xffffffff;
   while( (DATA_FROM_BLUETOOTH_UART == 0) && (timeout>0) )
   {
       timeout--;  /*CHANGE THIS TO BE A 200ms TIMEOUT*/
   }


   //if(queueRETURN != pdPASS)
    if(DATA_FROM_BLUETOOTH_UART == 0)
    {
      ret = BT_ERR_TIMEOUT;
    }
    else  /*we got data from the bluetooth uart*/
    {
        DATA_FROM_BLUETOOTH_UART = 0;
      if(tcu_event.Service_ID == TCU_BT_MNG){
/*TCU_BT_MNG_INTERFACE*/        
        switch (tcu_event.eventType)
        {
          /*TCU_ACCEPT*/
          case TCU_ACCEPT:
            ret = errorCode(tcu_event.status);              
            tcu_event.eventType = TCU_NO_EVENT;//reset 
          break;
          
          /*TCU_MNG_DISCOVER_REMOTE_DEVICE_RESULT_EVENT*/  
          case TCU_MNG_DISCOVER_REMOTE_DEVICE_RESULT_EVENT:
            remote_dev[curr_remote_dev].currDevNo = curr_remote_dev+1;
            remote_dev[curr_remote_dev].BD_ADDR[0] = tcu_mng_discover_remote_device_result_event.BD_ADDR[0];
            remote_dev[curr_remote_dev].BD_ADDR[1] = tcu_mng_discover_remote_device_result_event.BD_ADDR[1];
            remote_dev[curr_remote_dev].BD_ADDR[2] = tcu_mng_discover_remote_device_result_event.BD_ADDR[2];
            remote_dev[curr_remote_dev].BD_ADDR[3] = tcu_mng_discover_remote_device_result_event.BD_ADDR[3];
            remote_dev[curr_remote_dev].BD_ADDR[4] = tcu_mng_discover_remote_device_result_event.BD_ADDR[4];
            remote_dev[curr_remote_dev].BD_ADDR[5] = tcu_mng_discover_remote_device_result_event.BD_ADDR[5];            
                        
            temp = tcu_mng_discover_remote_device_result_event.Class_of_Device[2]<<16;
            temp |= tcu_mng_discover_remote_device_result_event.Class_of_Device[1]<<8; 
            temp |= tcu_mng_discover_remote_device_result_event.Class_of_Device[0];            
            
            remote_dev[curr_remote_dev].Class_of_Device = temp;
              
            remote_dev[curr_remote_dev].Length_of_Device_Name = tcu_mng_discover_remote_device_result_event.Length_of_Device_Name;
            for (loop=0;loop<(tcu_mng_discover_remote_device_result_event.Length_of_Device_Name);loop++){
                    remote_dev[curr_remote_dev].Device_Name[loop]=tcu_mng_discover_remote_device_result_event.Device_Name[loop];
            }
            
            curr_remote_dev++;
            *foundDevices = curr_remote_dev;
          break;          

          /*TCU_MNG_DISCOVER_REMOTE_DEVICE_COMPLETE_EVENT*/
          case TCU_MNG_DISCOVER_REMOTE_DEVICE_COMPLETE_EVENT:
            //end discover remot device
            process=1;            
          break; 
          
          default: 
            tcu_event.eventType = TCU_NO_EVENT;             
          break;          
        }
      }
                  
    if(ret != BT_NO_ERR)
      process=1;                                    
   }
  }
  return ret;   
}  
/***********************************************************************
 * @brief This API discovers services of remote device.
 * @param param1
    Pointer to remote device of which the API discovers supported services. 
    Input parameter
* @param param2
   Pointer to service information structure. 
   This structure contains information related to discovered services of remote device.
   Output parameter
 * @return API_RESULT:
 *   API_SUCCESS: API finished successfully.
 *   Value different from API_SUCCESS: see error code list bt_error.h
 */
API_RESULT BT_spp_discover_remote_service(t_bt_remote_device * param1, t_bt_service_information * param2){
  int32_t queueRETURN;
  uint32_t process = 0;
  uint32_t ret = API_SUCCESS;
  uint32_t loop;
  uint32_t timeout = 0xffffffff;
    
  tcu_mng_discover_remote_service_req.Security_Mode = 1;
  tcu_mng_discover_remote_service_req.BD_ADDR[0]=param1->BD_ADDR[0];
  tcu_mng_discover_remote_service_req.BD_ADDR[1]=param1->BD_ADDR[1];
  tcu_mng_discover_remote_service_req.BD_ADDR[2]=param1->BD_ADDR[2];
  tcu_mng_discover_remote_service_req.BD_ADDR[3]=param1->BD_ADDR[3];
  tcu_mng_discover_remote_service_req.BD_ADDR[4]=param1->BD_ADDR[4];  
  tcu_mng_discover_remote_service_req.BD_ADDR[5]=param1->BD_ADDR[5];
  tcu_mng_discover_remote_service_req.Use_of_Link_Key = 0;
  

  tcu_event.eventType = 0;//reset tcu_event
  DATA_FROM_BLUETOOTH_UART = 0;

  tcu_MNG_DISCOVER_REMOTE_SERVICE_REQ(&tcu_mng_discover_remote_service_req);

  while(process == 0){



   // queueRETURN = ReceiveEvent( btQueueEVENT, &ul_BT_SPP_rcvValue, (portMAX_DELAY) );//200ms portMAX_DELAY

        timeout = 0xffffffff;
       while( (DATA_FROM_BLUETOOTH_UART == 0) && (timeout>0) )
       {
           timeout--;  /*CHANGE THIS TO BE A 200ms TIMEOUT*/
       }

    if(DATA_FROM_BLUETOOTH_UART == 0)
    {
      ret = BT_ERR_TIMEOUT;
    }
    else  /*data from bluetooth from the uart was received*/
    {
        DATA_FROM_BLUETOOTH_UART = 0;
      if(tcu_event.Service_ID == TCU_BT_MNG){
/*TCU_BT_MNG_INTERFACE*/        
        switch (tcu_event.eventType){
          /*TCU_ACCEPT*/
          case TCU_ACCEPT:
            ret = errorCode(tcu_event.status);              
            tcu_event.eventType = TCU_NO_EVENT;//reset 
          break;
          
          /*TCU_MNG_DISCOVER_REMOTE_SERVICE_EVENT*/  
          case TCU_MNG_DISCOVER_REMOTE_SERVICE_EVENT:
            ret = errorCode(tcu_event.status);
            if(ret == BT_NO_ERR){                
              param2->Service_Count = tcu_mng_discover_remote_service_event.Service_Count;
              
              if(param2->Service_Count != 0){
                param2->Service_Type = tcu_mng_discover_remote_service_event.Service_Type;
                param2->SPP_Ex_info_Profile = tcu_mng_discover_remote_service_event.SPP_Ex_info_Profile;
                param2->Number_of_ServerChannel = tcu_mng_discover_remote_service_event.Number_of_ServerChannel;
                
                for (loop=0;loop<(tcu_mng_discover_remote_service_event.SPP_Ext_Info_Length-1);loop++){
                  param2->SPP_Server_Channel[loop]= tcu_mng_discover_remote_service_event.SPP_Server_Channel[loop];			
                }               
             }                          
          }
          process=1;            
          break;          
          
          default: 
            tcu_event.eventType = TCU_NO_EVENT;             
          break;          
        }
      }
                  
    if(ret != BT_NO_ERR)
      process=1;                                    
   }
  } 
  return ret;   
}
/***********************************************************************
 * @brief This API establishes service level connection between SPP Dev A and
 *        SPP Dev B
 * @param param1
   Remote device to be connected to.
   Inpuu parameter

 * @param param1
    Structure containing information related to SPP server channles.
    Inpuu parameter

* @return API_RESULT:
 *   API_SUCCESS: API finished successfully.
 *   Value different from API_SUCCESS: see error code list bt_error.h
 */
API_RESULT BT_spp_connect_remote_device(t_bt_remote_device * param1, t_bt_service_information * param2){
  int32_t queueRETURN;
  uint32_t process = 0;
  uint32_t ret = API_SUCCESS;
  uint32_t * pPasskey;//used for MITM_Protection_Required_Dedicated_BondingIO_Capabilities 
  uint32_t timeout = 0xffffffff;

  DATA_FROM_BLUETOOTH_UART = 0;
  tcu_event.eventType = 0;//reset tcu_event

  tcu_SPP_SETUP_REQ();  
  
  while(process == 0){


    //queueRETURN = ReceiveEvent( btQueueEVENT, &ul_BT_SPP_rcvValue, (portMAX_DELAY) );//200ms portMAX_DELAY

    timeout = 0xffffffff;
   while( (DATA_FROM_BLUETOOTH_UART == 0) && (timeout>0) )
   {
       timeout--;  /*CHANGE THIS TO BE A 200ms TIMEOUT*/
   }

    if(DATA_FROM_BLUETOOTH_UART == 0)
    {
      ret = BT_ERR_TIMEOUT;
    }
    else
    {
        DATA_FROM_BLUETOOTH_UART = 0;
      if(tcu_event.Service_ID == TCU_BT_MNG){
/*TCU_BT_MNG_INTERFACE*/        
        switch (tcu_event.eventType)
        {
          /*TCU_MNG_CONNECTION_REQUEST_EVENT*/    
          case TCU_MNG_CONNECTION_REQUEST_EVENT:                                                    
              tcu_event.eventType = TCU_NO_EVENT;
                tcu_mng_connection_accept_req.Response_Type=0x00;//accept
                tcu_mng_connection_accept_req.Use_of_Link_Key=0x00;//
                tcu_mng_connection_accept_req.BD_ADDR[0]=tcu_mng_connection_request_event.BD_ADDR[0];
                tcu_mng_connection_accept_req.BD_ADDR[1]=tcu_mng_connection_request_event.BD_ADDR[1];
                tcu_mng_connection_accept_req.BD_ADDR[2]=tcu_mng_connection_request_event.BD_ADDR[2];
                tcu_mng_connection_accept_req.BD_ADDR[3]=tcu_mng_connection_request_event.BD_ADDR[3];
                tcu_mng_connection_accept_req.BD_ADDR[4]=tcu_mng_connection_request_event.BD_ADDR[4];
                tcu_mng_connection_accept_req.BD_ADDR[5]=tcu_mng_connection_request_event.BD_ADDR[5];
    
                tcu_MNG_CONNECTION_ACCEPT_REQ(&tcu_mng_connection_accept_req);//wait TCU_MNG_CONNECTION_ACCEPT_RESP           
                break;
          /*TCU_MNG_SSP_INFO_EVENT*/    
          case TCU_MNG_SSP_INFO_EVENT:
              tcu_event.eventType = TCU_NO_EVENT;
              if (tcu_mng_ssp_info_event.OpCode == IO_User_Confirmation_Request_Event){//0x33 User_Confirmation_Request 
                //password stored in tcu_mng_ssp_info_event.Parameter7 to 10
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[0]=tcu_mng_ssp_info_event.Parameter[0];
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[1]=tcu_mng_ssp_info_event.Parameter[1];
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[2]=tcu_mng_ssp_info_event.Parameter[2];
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[3]=tcu_mng_ssp_info_event.Parameter[3];
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[4]=tcu_mng_ssp_info_event.Parameter[4];
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[5]=tcu_mng_ssp_info_event.Parameter[5];                            
                tcu_mng_spp_set_req.OpCode = HCI_User_Confirmation_Request_Reply; //0x042C
    
                tcu_MNG_SSP_SET_REQ(&tcu_mng_spp_set_req);
              }else if(tcu_mng_ssp_info_event.OpCode == IO_Capability_Request_Event){//0x31 IO_Capability_Request_Event            
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[0]=tcu_mng_ssp_info_event.Parameter[0];
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[1]=tcu_mng_ssp_info_event.Parameter[1];
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[2]=tcu_mng_ssp_info_event.Parameter[2];
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[3]=tcu_mng_ssp_info_event.Parameter[3];
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[4]=tcu_mng_ssp_info_event.Parameter[4];
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[5]=tcu_mng_ssp_info_event.Parameter[5];
                tcu_mng_spp_set_req.OpCode = HCI_IO_Capability_Request_Reply;     //0x042B
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.IO_Capability = SPP_IO_CAPABILITY;
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.OOB_Data_Present = SPP_OOB_AUTHENTICATION;
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.Authentication_Requirement = SPP_MITM_PROTECTION;//0x00 lwi
    
                tcu_MNG_SSP_SET_REQ(&tcu_mng_spp_set_req);
              }else if(tcu_mng_ssp_info_event.OpCode == IO_Capability_Response_Event){//0x32 IO_Capability_Response_Event
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[0]=tcu_mng_ssp_info_event.Parameter[0];
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[1]=tcu_mng_ssp_info_event.Parameter[1];
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[2]=tcu_mng_ssp_info_event.Parameter[2];
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[3]=tcu_mng_ssp_info_event.Parameter[3];
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[4]=tcu_mng_ssp_info_event.Parameter[4];
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[5]=tcu_mng_ssp_info_event.Parameter[5];
                tcu_mng_spp_set_req.OpCode = HCI_IO_User_Passkey_Request_Reply;     //0x042E                                                
              }else if(tcu_mng_ssp_info_event.OpCode == User_Passkey_Notification){//0x3b(User_Passkey_Notification)
                pPasskey = (uint32_t *)&tcu_mng_ssp_info_event.Parameter[6];//party B must use this passkey on "User_Passkey_Request_Replay"
              }
              break;
              
            /*TCU_MNG_INIT_RESP*/  
            case TCU_MNG_INIT_RESP:
              tcu_event.eventType = TCU_NO_EVENT; //reset
              ret = errorCode(tcu_event.status);
              if( ret == BT_NO_ERR)
                 tcu_SPP_SETUP_REQ();
              break;
              
            /*TCU_MNG_SET_SCAN_RESP*/  
            case TCU_MNG_SET_SCAN_RESP:
              tcu_event.eventType = TCU_NO_EVENT;//reset
              ret = errorCode(tcu_event.status);                            
              break;
            
            /*TCU_ACCEPT*/
            case TCU_ACCEPT:
              ret = errorCode(tcu_event.status);              
              tcu_event.eventType = TCU_NO_EVENT;//reset 
              break;
            
            /*TCU_MNG_CONNECTION_ACCEPT_RESP*/
            case TCU_MNG_CONNECTION_ACCEPT_RESP:
              tcu_event.eventType = TCU_NO_EVENT;//reset
              ret = errorCode(tcu_event.status);
              if( ret == BT_NO_ERR)
                 tcu_MNG_SET_SCAN_REQ(Scan_Mode_InquiryScan_ON_PageScan_ON);              
              break;

            /*TCU_MNG_CONNECTION_STATUS_EVENT*/            
            case TCU_MNG_CONNECTION_STATUS_EVENT:
                tcu_event.eventType = TCU_NO_EVENT;//reset
                ret = errorCode(tcu_event.status);                               
              break;
            
            /*TTCU_MNG_REMOTE_DEVICE_NAME_AUTO_NOTIFY_EVENT*/            
            case TCU_MNG_REMOTE_DEVICE_NAME_AUTO_NOTIFY_EVENT:
              tcu_event.eventType = TCU_NO_EVENT;//reset
              break;
            
            /*TCU_MNG_STANDARD_HCI_SET_RESP*/
            case TCU_MNG_STANDARD_HCI_SET_RESP:
              ret = errorCode(tcu_event.status);
              tcu_event.eventType = TCU_NO_EVENT;              
              break;

            /*TCU_MNG_PIN_REQUEST_EVENT*/
            case TCU_MNG_PIN_REQUEST_EVENT:
              tcu_mng_pin_write_req.BD_ADDR[0]=tcu_mng_pin_request_event.BD_ADDR[0];
              tcu_mng_pin_write_req.BD_ADDR[1]=tcu_mng_pin_request_event.BD_ADDR[1];
              tcu_mng_pin_write_req.BD_ADDR[2]=tcu_mng_pin_request_event.BD_ADDR[2];
              tcu_mng_pin_write_req.BD_ADDR[3]=tcu_mng_pin_request_event.BD_ADDR[3];
              tcu_mng_pin_write_req.BD_ADDR[4]=tcu_mng_pin_request_event.BD_ADDR[4];
              tcu_mng_pin_write_req.BD_ADDR[5]=tcu_mng_pin_request_event.BD_ADDR[5];
              tcu_mng_pin_write_req.Length_of_PIN_code = PIN_LENGHT;
              tcu_mng_pin_write_req.PIN_code[0] = PIN[0];
              tcu_mng_pin_write_req.PIN_code[1] = PIN[1];
              tcu_mng_pin_write_req.PIN_code[2] = PIN[2];
              tcu_mng_pin_write_req.PIN_code[3] = PIN[3];
               
              tcu_MNG_PIN_WRITE_REQ(&tcu_mng_pin_write_req);
                        
              break;

            /*TCU_MNG_PIN_WRITE_RESP*/
            case TCU_MNG_PIN_WRITE_RESP:
              ret = errorCode(tcu_event.status);
              tcu_event.eventType = TCU_NO_EVENT;                
            break;
            default: 
                tcu_event.eventType = TCU_NO_EVENT;             
              break;
              }      
    }else if(tcu_event.Service_ID == TCU_BT_SPP){
/*TCU_BT_SPP*/
      switch (tcu_event.eventType)
      {
        /*TCU_SPP_SETUP_RESP*/  
        case TCU_SPP_SETUP_RESP:
          tcu_event.eventType = TCU_NO_EVENT;//reset
          ret = errorCode(tcu_event.status);
          if( (ret == BT_NO_ERR) | (ret == SPP_ERR_SETUP_SPP)){
            ret = BT_NO_ERR;
            //TCU_MNG_SET_SCAN_REQ(Scan_Mode_InquiryScan_ON_PageScan_ON);

            tcu_spp_connect_req.BD_ADDR[0]=param1->BD_ADDR[0];
            tcu_spp_connect_req.BD_ADDR[1]=param1->BD_ADDR[1];
            tcu_spp_connect_req.BD_ADDR[2]=param1->BD_ADDR[2];
            tcu_spp_connect_req.BD_ADDR[3]=param1->BD_ADDR[3];
            tcu_spp_connect_req.BD_ADDR[4]=param1->BD_ADDR[4];
            tcu_spp_connect_req.BD_ADDR[5]=param1->BD_ADDR[5];
            tcu_spp_connect_req.BaudRate=0x07;
            tcu_spp_connect_req.uDataFormat.DataFormatByte=0x16;
            tcu_spp_connect_req.uFlowControl.FlowControlByte = 0x06;
            tcu_spp_connect_req.XonChar= 0x01;
            tcu_spp_connect_req.XoffChar= 0x01;
            tcu_spp_connect_req.uParmMask.ParmMaskbyte=0x00;
            tcu_spp_connect_req.Server_Channel_Validity = 0x01;
            
            tcu_spp_connect_req.Sever_Channel = param2->SPP_Server_Channel[param2->Number_of_ServerChannel-1];
            
            tcu_spp_connect_req.Use_of_Link_Key = 0;
            
            tcu_SPP_CONNECT_REQ(&tcu_spp_connect_req);
          }
          break;
          
        /*TCU_SPP_CONNECT_EVENT*/  
        case TCU_SPP_CONNECT_EVENT:
          //tcu_event.eventType = TCU_NO_EVENT;//reset
          ret = errorCode(tcu_event.status);
          
          process = 1;//exit routine due to established connection
          //BT_spp_send((uint8_t*)&Device_Name, 1);
          break;
      }                
     }                  
    }
    if(ret != BT_NO_ERR)
      process=1;                                    
   }    
  return ret;  
}
/***********************************************************************
 * @brief This API sends the data to remote SPP device
 * @param txBuff
 *   Data to be sent over SPP channel
 * @param data_len
 *   Length of data to be sent over SPP channel
 * @return API_RESULT:
 *   API_SUCCESS: API finished successfully.
 *   Value different from API_SUCCESS: see error code list bt_error.h
 */
API_RESULT BT_spp_send(uint8_t *txBuff, uint16_t data_len){
  uint32_t ret = API_SUCCESS;
  if(data_len < UART_PACKET_LENGHT){
    tcu_data_transfer_req.Length_of_Data = data_len;
    tcu_data_transfer_req.Txdata = txBuff;
    tcu_SPP_DATA_TRANSFER_REQ(&tcu_data_transfer_req); 
  }else
    ret = SPP_ERR_DATA_EXCEEDS_BUFFER_SIZE;
  return ret;
}
/***********************************************************************
 * @brief This API sends request to shutdown the SPP function
 * @param none
 * @return none
 */
void BT_spp_shutdown(void){
  tcu_SPP_SHUTDOWN_REQ();
}

/***********************************************************************
 * @brief This API sends request to disconnect SPP connection.
 * @param none
 * @return none
 */
void BT_spp_disconnect(void){
  tcu_SPP_DISCONNECT_REQ();
}

/***********************************************************************
 * @brief This API starts inquiry Scan, Page Scan.
 * @param Inquiry Scan, Page Scan Setting
      0x00: Inquiry Scan OFF Page Scan OFF
      0x01: Inquiry Scan ON Page Scan OFF
      0x02: Inquiry Scan OFF Page Scan ON
      0x03: Inquiry Scan ON Page Scan ON
 * @return none
 */
API_RESULT BT_spp_scan(uint8_t parm){
  unsigned long ret = API_SUCCESS;   
  uint32_t process=0;
  int32_t queueRETURN;
  uint32_t timeout = 0xffffffff;

  DATA_FROM_BLUETOOTH_UART = 0;
  tcu_event.eventType = 0;//reset tcu_event

  tcu_SPP_SETUP_REQ();//setup SPP device

  while(process == 0){


   // queueRETURN = ReceiveEvent( btQueueEVENT, &ul_BT_SPP_rcvValue, (SPP_WAIT_RESPONSE_TIME_100ms) );//200ms portMAX_DELAY

    timeout = 0xffffffff;
   while( (DATA_FROM_BLUETOOTH_UART == 0) && (timeout>0) )
   {
       timeout--;  /*CHANGE THIS TO BE A 200ms TIMEOUT*/
   }


    if(DATA_FROM_BLUETOOTH_UART == 0)
    {
      ret = BT_ERR_TIMEOUT;
      process=1;      
    }
    else
    {
        DATA_FROM_BLUETOOTH_UART = 0;
      if(tcu_event.Service_ID == TCU_BT_SPP)
      {
        switch (tcu_event.eventType)
        {
            case TCU_SPP_SETUP_RESP:
                    ret = errorCode(tcu_event.status);
                    if(ret == BT_NO_ERR)
                    {
                        tcu_MNG_SET_SCAN_REQ(parm);
                    }
                    else
                    {
                        process=1;
                    }
             break;
          
            default:
            break;
        }        
      }
      else
          if(tcu_event.Service_ID == TCU_BT_MNG)
          {
              if (tcu_event.eventType == TCU_MNG_SET_SCAN_RESP)
              {
                  ret = errorCode(tcu_event.status);
                  process=1;
                  tcu_event.eventType = TCU_NO_EVENT;//reset tcu_event
              }
          }
    }
  }
  return ret;
}
/***********************************************************************
* @brief This API is used for connection request from PartyB device.
* @param pADDR 
* @return none
*/
API_RESULT BT_spp_rcv_connect_req(void*  pADDR, uint32_t time_interval){  
  unsigned long ret = API_SUCCESS;   
  uint32_t process=0;
  int32_t queueRETURN;
  uint32_t timeout = 0xffffffff;
  

  tcu_event.eventType = 0;//reset tcu_event
  DATA_FROM_BLUETOOTH_UART = 0;

  while(process == 0){



    

    timeout = 0xffffffff;
   while( (DATA_FROM_BLUETOOTH_UART == 0) && (timeout>0) )
   {
       timeout--;  /*CHANGE THIS TO BE A  TIMEOUT specified from input paramter "time_interval"  which is in milliseconds*/
   }


    if(DATA_FROM_BLUETOOTH_UART == 0)
    {
      ret = BT_ERR_TIMEOUT;
      process = 1;//      
    }
    else
    {
      
      if(tcu_event.Service_ID == TCU_BT_MNG){
      switch (tcu_event.eventType)
      {
      case TCU_MNG_CONNECTION_REQUEST_EVENT:
/*
        tcu_mng_connection_accept_req.BD_ADDR[0]=tcu_mng_connection_request_event.BD_ADDR[0];
        tcu_mng_connection_accept_req.BD_ADDR[1]=tcu_mng_connection_request_event.BD_ADDR[1];
        tcu_mng_connection_accept_req.BD_ADDR[2]=tcu_mng_connection_request_event.BD_ADDR[2];
        tcu_mng_connection_accept_req.BD_ADDR[3]=tcu_mng_connection_request_event.BD_ADDR[3];
        tcu_mng_connection_accept_req.BD_ADDR[4]=tcu_mng_connection_request_event.BD_ADDR[4];
        tcu_mng_connection_accept_req.BD_ADDR[5]=tcu_mng_connection_request_event.BD_ADDR[5];        
        tcu_mng_connection_accept_req.Use_of_Link_Key=0x00;//no link key
        tcu_MNG_CONNECTION_ACCEPT_REQ(&tcu_mng_connection_accept_req);
*/        
        *(uint32_t*)pADDR = (uint32_t)&tcu_mng_connection_request_event.BD_ADDR[0];
        process = 1;//
        break;
        
      case TCU_MNG_CONNECTION_ACCEPT_RESP:
        ret = errorCode(tcu_event.status);
        process = 1;//
        break;
      default:break;
      }        
      }
    }
  }
  return ret;
}
/***********************************************************************
* @brief This API starts connection accept procedure. PartyB is initiator, 
          TC35661 is acceptor.
* @param OUT:   BD_ADDR contains BT addres of device requesting connection.
* @param IN:    Use_of_Link_Key_ used to set Inquiry Scan, Page Scan.
* @return none
*/
API_RESULT BT_spp_connection_accept_req(uint8_t *BD_ADDR){
  unsigned long ret = API_SUCCESS;   
  uint32_t process=0;
  int32_t queueRETURN;
  uint32_t * pPasskey;//used for MITM_Protection_Required_Dedicated_BondingIO_Capabilities  
  uint32_t timeout = 0xffffffff;

  tcu_mng_connection_accept_req.BD_ADDR[0]=(uint8_t)*BD_ADDR++;
  tcu_mng_connection_accept_req.BD_ADDR[1]=(uint8_t)*BD_ADDR++;
  tcu_mng_connection_accept_req.BD_ADDR[2]=(uint8_t)*BD_ADDR++;
  tcu_mng_connection_accept_req.BD_ADDR[3]=(uint8_t)*BD_ADDR++;
  tcu_mng_connection_accept_req.BD_ADDR[4]=(uint8_t)*BD_ADDR++;
  tcu_mng_connection_accept_req.BD_ADDR[5]=(uint8_t)*BD_ADDR++;
  tcu_mng_connection_accept_req.Use_of_Link_Key=0;//no link key
  tcu_mng_connection_accept_req.Response_Type=0;
  


  tcu_event.eventType = 0;//reset tcu_event
  DATA_FROM_BLUETOOTH_UART = 0;
  tcu_MNG_CONNECTION_ACCEPT_REQ(&tcu_mng_connection_accept_req);
  
  while(process == 0){


    //queueRETURN = ReceiveEvent( btQueueEVENT, &ul_BT_SPP_rcvValue, (SPP_WAIT_RESPONSE_TIME_100ms) );

    timeout = 0xffffffff;
   while( (DATA_FROM_BLUETOOTH_UART == 0) && (timeout>0) )
   {
       timeout--;  /*CHANGE THIS TO BE A  TIMEOUT of 100ms*/
   }

    if(DATA_FROM_BLUETOOTH_UART == 0)
    {
      ret = BT_ERR_TIMEOUT;
    }
    else
    {
        DATA_FROM_BLUETOOTH_UART = 0;
      if(tcu_event.Service_ID == TCU_BT_MNG){
        switch (tcu_event.eventType){
          case TCU_MNG_CONNECTION_ACCEPT_RESP:
            //ret = errorCode(tcu_event.status);
            //if ( BT_NO_ERR != ret)
            //  process = 1;//
          break;
          case TCU_MNG_CONNECTION_STATUS_EVENT:
            ret = errorCode(tcu_event.status);
              if ( BT_NO_ERR != ret)
                process = 1;//he is exiting in this case
            break;                    
          case TCU_MNG_SSP_INFO_EVENT:
              if (tcu_mng_ssp_info_event.OpCode == IO_User_Confirmation_Request_Event){//0x33 User_Confirmation_Request 
                //password stored in tcu_mng_ssp_info_event.Parameter7 to 10
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[0]=tcu_mng_ssp_info_event.Parameter[0];
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[1]=tcu_mng_ssp_info_event.Parameter[1];
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[2]=tcu_mng_ssp_info_event.Parameter[2];
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[3]=tcu_mng_ssp_info_event.Parameter[3];
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[4]=tcu_mng_ssp_info_event.Parameter[4];
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[5]=tcu_mng_ssp_info_event.Parameter[5];                            
                tcu_mng_spp_set_req.OpCode = HCI_User_Confirmation_Request_Reply; //0x042C
    
                tcu_MNG_SSP_SET_REQ(&tcu_mng_spp_set_req);
              }else if(tcu_mng_ssp_info_event.OpCode == IO_Capability_Request_Event){//0x31 IO_Capability_Request_Event            
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[0]=tcu_mng_ssp_info_event.Parameter[0];
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[1]=tcu_mng_ssp_info_event.Parameter[1];
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[2]=tcu_mng_ssp_info_event.Parameter[2];
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[3]=tcu_mng_ssp_info_event.Parameter[3];
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[4]=tcu_mng_ssp_info_event.Parameter[4];
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[5]=tcu_mng_ssp_info_event.Parameter[5];
                tcu_mng_spp_set_req.OpCode = HCI_IO_Capability_Request_Reply;     //0x042B
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.IO_Capability = SPP_IO_CAPABILITY;
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.OOB_Data_Present = SPP_OOB_AUTHENTICATION;
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.Authentication_Requirement = SPP_MITM_PROTECTION;//0x00 lwi
    
                tcu_MNG_SSP_SET_REQ(&tcu_mng_spp_set_req);
              }else if(tcu_mng_ssp_info_event.OpCode == IO_Capability_Response_Event){//0x32 IO_Capability_Response_Event
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[0]=tcu_mng_ssp_info_event.Parameter[0];
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[1]=tcu_mng_ssp_info_event.Parameter[1];
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[2]=tcu_mng_ssp_info_event.Parameter[2];
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[3]=tcu_mng_ssp_info_event.Parameter[3];
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[4]=tcu_mng_ssp_info_event.Parameter[4];
                tcu_mng_spp_set_req.param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[5]=tcu_mng_ssp_info_event.Parameter[5];
                tcu_mng_spp_set_req.OpCode = HCI_IO_User_Passkey_Request_Reply;     //0x042E

            //    tcu_MNG_SSP_SET_REQ(&tcu_mng_spp_set_req);

              }else if(tcu_mng_ssp_info_event.OpCode == User_Passkey_Notification){//0x3b(User_Passkey_Notification)
                pPasskey = (uint32_t *)&tcu_mng_ssp_info_event.Parameter[6];//party B must use this passkey on "User_Passkey_Request_Replay"
              }else if(tcu_mng_ssp_info_event.OpCode == Simple_Pairing_Complete_Event){//0x36(Simple_Pairing_Complete_Event)
              }              
          break;

          /*TCU_MNG_CONNECTION_REQUEST_EVENT*/    
          case TCU_MNG_CONNECTION_REQUEST_EVENT:                                                    
              tcu_event.eventType = TCU_NO_EVENT;
              tcu_mng_connection_accept_req.Response_Type=0x00;//accept
              tcu_mng_connection_accept_req.Use_of_Link_Key=0x00;//
              tcu_mng_connection_accept_req.BD_ADDR[0]=tcu_mng_connection_request_event.BD_ADDR[0];
              tcu_mng_connection_accept_req.BD_ADDR[1]=tcu_mng_connection_request_event.BD_ADDR[1];
              tcu_mng_connection_accept_req.BD_ADDR[2]=tcu_mng_connection_request_event.BD_ADDR[2];
              tcu_mng_connection_accept_req.BD_ADDR[3]=tcu_mng_connection_request_event.BD_ADDR[3];
              tcu_mng_connection_accept_req.BD_ADDR[4]=tcu_mng_connection_request_event.BD_ADDR[4];
              tcu_mng_connection_accept_req.BD_ADDR[5]=tcu_mng_connection_request_event.BD_ADDR[5];
  
              tcu_MNG_CONNECTION_ACCEPT_REQ(&tcu_mng_connection_accept_req);//wait TCU_MNG_CONNECTION_ACCEPT_RESP           
          break;
          /*TCU_MNG_STANDARD_HCI_SET_RESP*/
          case TCU_MNG_STANDARD_HCI_SET_RESP:
            ret = errorCode(tcu_event.status);
            if ( BT_NO_ERR != ret)
              process = 1;//               
            break;          
        case TCU_MNG_REMOTE_DEVICE_NAME_AUTO_NOTIFY_EVENT:
          break;
                             
        default:break;
        }
        
      }else if(tcu_event.Service_ID == TCU_BT_SPP ){
        
        switch (tcu_event.eventType){
          case TCU_SPP_CONNECT_EVENT:
            ret = errorCode(tcu_event.status);
            process = 1;//connection procedure finished !!!!               
            break;

        default:break;            
        }
      }
    }
  }
  return ret;
}
/***********************************************************************
 * @brief This function initializes queue for BT application
 * @param none 
 * @return API_RESULT:
 *   API_SUCCESS: Queue initialized successfully
 *   BT_ERR_QUEUE_COULD_NOT_BE_CREATE: Queue counld not be initialized successfully
 */
API_RESULT BT_os_init(void){
  unsigned long ret = API_SUCCESS;

  /* Create the BT_Queue */  
  /*BT_Queue is used to pass events from the SPP driver layer to BT_SPP_API layer*/
  //btQueueEVENT = CreateQueue( BT_QUEUE_LENGTH, sizeof( unsigned long ) );
 // btQueueEVENT = CreateQueue( BT_QUEUE_LENGTH, 8 );
  
//  if(btQueueEVENT == NULL)
  //  ret = BT_ERR_QUEUE_COULD_NOT_BE_CREATED;
  return ret;
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


