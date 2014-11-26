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

/**********************************************
*                Include files                *
**********************************************/
#include <string.h>
#include "bt_mng_drv.h"


/**********************************************
*         Variables and forward declarations  *
**********************************************/
t_tcu_mng_init_req tcu_mng_init_req;
t_tcu_mng_pin_write_req tcu_mng_pin_write_req;
t_tcu_mng_change_local_device_param_req tcu_mng_change_local_device_param_req;

t_tcu_packet_format tcu_packet_format;
t_tcu_mng_init_resp tcu_mng_init_resp;
t_tcu_mng_set_scan_resp tcu_mng_set_scan_resp;
t_tcu_mng_pin_write_resp tcu_mng_pin_write_resp;
t_tcu_mng_standard_hci_set_resp tcu_mng_standard_hci_set_resp;
t_tcu_mng_connection_accept_req tcu_mng_connection_accept_req;
t_tcu_mng_connection_accept_resp tcu_mng_connection_accept_resp;

t_tcu_mng_ssp_info_event_ tcu_mng_ssp_info_event;
t_tcu_mng_discover_remote_device_result_event tcu_mng_discover_remote_device_result_event;
t_tcu_mng_discover_remote_service_event tcu_mng_discover_remote_service_event;
t_tcu_mng_connection_status_event tcu_mng_connection_status_event;
t_tcu_mng_pin_request_event tcu_mng_pin_request_event;
t_tcu_mng_connection_request_event tcu_mng_connection_request_event;

t_tcu_mng_discover_remote_service_req tcu_mng_discover_remote_service_req;
t_tcu_mng_spp_set_req tcu_mng_spp_set_req;
t_tcu_mng_tcu_accept tcu_mng_tcu_accept;
t_tcu_mng_tcu_non_accept tcu_mng_tcu_non_accept;



 /**
 * \fn      tcu_MNG_INIT_REQ
 * \brief   Ccommand initializes TC35661 firmware
 * \param   Device_Name
 * \return  void
 */
void tcu_MNG_INIT_REQ(t_tcu_mng_init_req  * tcu_mng_init_req_){
   uint32_t count;
RTS_high;     
   au8SendBuff[3]=MNG_ServiceID;
   au8SendBuff[4]=TCU_MNG_INIT_REQ;

   au8SendBuff[7]=tcu_mng_init_req_->Supported_Profiles;//Supported_Profiles
   au8SendBuff[8]=tcu_mng_init_req_->Paired_Information_Stored;           

   /*calculate Length_of_Device_Name*/
   for(count=0;(*tcu_mng_init_req_->Device_Name_) != '\0';count++){
     au8SendBuff[count+10]=(uint8_t)(*tcu_mng_init_req_->Device_Name_++);
   }   
   au8SendBuff[9]=count;
      
   count=count+10;
   
   au8SendBuff[0]=(0x0000ff & count);
   au8SendBuff[1]=(0x00ff00 & count);
   au8SendBuff[2]=(0xff0000 & count);
   
   /*calculate ParameterLength*/
   count -=7;//
   au8SendBuff[5]=(0x00ff & count);//ParameterLength
   au8SendBuff[6]=(0xff00 & count);//ParameterLength
   
   send_tx((uint8_t *)&au8SendBuff[0], count+7); 
 }
  /**
 * \fn      tcu_MNG_SET_SCAN_REQ
 * \brief   Function to set Inquiry Scan, Page Scan.
 * \param   void
 * \return  void
 */
 void tcu_MNG_SET_SCAN_REQ(uint8_t Scan_Mode){//08 00 00 e1 0c 01 00 03
RTS_high;   
   au8SendBuff[0]=0x08;
   au8SendBuff[1]=0x00;
   au8SendBuff[2]=0x00;   
   au8SendBuff[3]=MNG_ServiceID;
   au8SendBuff[4]=TCU_MNG_SET_SCAN_REQ;
   au8SendBuff[5]=0x01;//ParameterLength 
   au8SendBuff[6]=0x00;//ParameterLength
   au8SendBuff[7]=Scan_Mode;//Supported_Profiles

   send_tx((uint8_t *)&au8SendBuff[0], au8SendBuff[0]); 
 }

  /**
 * \fn      tcu_MNG_CONNECTION_ACCEPT_REQ
 * \brief   This command is used to accept or 
 * \brief   reject for Bluetooth connection request from remote device.
 * \param   void
 * \return  void
 */
 void tcu_MNG_CONNECTION_ACCEPT_REQ(t_tcu_mng_connection_accept_req * param){//08 00 00 e1 0c 01 00 03
   uint32_t i;
RTS_high;   
   au8SendBuff[1]=0x00;
   au8SendBuff[2]=0x00;   
   au8SendBuff[3]=MNG_ServiceID;
   au8SendBuff[4]=TCU_MNG_CONNECTION_ACCEPT_REQ;

   au8SendBuff[7]=param->Response_Type;
   au8SendBuff[8]=(uint8_t)(param->BD_ADDR[0]);
   au8SendBuff[9]=(uint8_t)(param->BD_ADDR[1]);
   au8SendBuff[10]=(uint8_t)(param->BD_ADDR[2]);
   au8SendBuff[11]=(uint8_t)(param->BD_ADDR[3]);
   au8SendBuff[12]=(uint8_t)(param->BD_ADDR[4]);
   au8SendBuff[13]=(uint8_t)(param->BD_ADDR[5]);
   au8SendBuff[14]=(uint8_t)(param->Use_of_Link_Key);
   
   if((uint8_t)(param->Use_of_Link_Key)==1){
     for(i=0;i<16;i++){
      au8SendBuff[15+i]=(uint8_t)(param->Link_Key[i]);            
     }
     au8SendBuff[0]=31;//15+16;
   }else{
     au8SendBuff[0]=15;
   }
   au8SendBuff[5]=au8SendBuff[0]-7;//ParameterLength 
   au8SendBuff[6]=0x00;//ParameterLength 
   
   send_tx((uint8_t *)&au8SendBuff[0], au8SendBuff[0]); 
 } 
  /**
 * \fn      tcu_MNG_DISCOVER_REMOTE_DEVICE_REQ
 * \brief   Ccommand initializes TC35661 firmware
 * \param   void
 * \return  void
 */
 void tcu_MNG_DISCOVER_REMOTE_DEVICE_REQ(uint8_t Max_Number_of_Reports ){//08 00 00 e1 03 01 00 xx
RTS_high;   
   au8SendBuff[0]=0x08;
   au8SendBuff[1]=0x00;
   au8SendBuff[2]=0x00;   
   au8SendBuff[3]=0xe1;
   au8SendBuff[4]=0x03;
   au8SendBuff[5]=0x01;//ParameterLength 
   au8SendBuff[6]=0x00;//ParameterLength
   au8SendBuff[7]=Max_Number_of_Reports;//
   send_tx((uint8_t *)&au8SendBuff[0], au8SendBuff[0]); 
 }
  /**
 * \fn      tcu_MNG_SSP_SET_REQ
 * \brief   Ccommand initializes TC35661 firmware
 * \param   void
 * \return  void
 */
 void tcu_MNG_SSP_SET_REQ(t_tcu_mng_spp_set_req * param_ ){//08 00 00 e1 03 01 00 xx
   uint32_t count;
RTS_high;   
//   au8SendBuff[0]=0x19;//string lenght of TCU_SPP_SETUP_REQ is 7
//   au8SendBuff[1]=0x00;
//   au8SendBuff[2]=0x00;   
   au8SendBuff[3]=MNG_ServiceID;
   au8SendBuff[4]=TCU_MNG_STANDARD_HCI_SET_REQ;

   au8SendBuff[7]= param_->OpCode & 0x00ff ;
   au8SendBuff[8]= (param_->OpCode)>>8;
   switch (param_->OpCode){
    case HCI_IO_Capability_Request_Reply: 
      au8SendBuff[9]=9; 
      au8SendBuff[16]=param_->param.t_HCI_IO_Capability_Request_Reply_Parameter.IO_Capability;
      au8SendBuff[17]=param_->param.t_HCI_IO_Capability_Request_Reply_Parameter.OOB_Data_Present;
      au8SendBuff[18]=param_->param.t_HCI_IO_Capability_Request_Reply_Parameter.Authentication_Requirement;            
      break;
    case HCI_IO_Capability_Request_Negative_Reply: au8SendBuff[9]=7;break;
    case HCI_User_Confirmation_Negative_Reply: au8SendBuff[9]=6;break;
    case HCI_User_Confirmation_Request_Reply: au8SendBuff[9]=6;break;
    case HCI_Write_Simple_Pairing_Debug_Mode_Command: au8SendBuff[9]=1;break;
   default:break;
   }
   if(param_->OpCode != HCI_Write_Simple_Pairing_Debug_Mode_Command){
    au8SendBuff[10] = param_->param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[0];
    au8SendBuff[11] = param_->param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[1];
    au8SendBuff[12] = param_->param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[2];
    au8SendBuff[13] = param_->param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[3];
    au8SendBuff[14] = param_->param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[4];
    au8SendBuff[15] = param_->param.t_HCI_IO_Capability_Request_Reply_Parameter.BD_ADDR[5];
   }
   count = 3 + au8SendBuff[9];
   au8SendBuff[5]= (uint8_t)count;//ParameterLength 
   au8SendBuff[6]= count>>8;//ParameterLength 
   
   count= 10+au8SendBuff[9];
   au8SendBuff[0]=(0x0000ff & count);
   au8SendBuff[1]=(0x00ff00 & count);
   au8SendBuff[2]=(0xff0000 & count);    

   send_tx((uint8_t *)&au8SendBuff[0], au8SendBuff[0]);  
 }
  /**
 * \fn      tcu_MNG_PIN_WRITE_REQ
 * \brief   
 * \param   void
 * \return  void
 */
 void tcu_MNG_PIN_WRITE_REQ(t_tcu_mng_pin_write_req  * tcu_mng_pin_write_req_){
   uint32_t count;
   uint8_t loop;    
RTS_high;   
   au8SendBuff[3]=MNG_ServiceID;
   au8SendBuff[4]=TCU_MNG_PIN_WRITE_REQ;

   au8SendBuff[7]=(uint8_t)(tcu_mng_pin_write_req_->BD_ADDR[0]);
   au8SendBuff[8]=(uint8_t)(tcu_mng_pin_write_req_->BD_ADDR[1]);
   au8SendBuff[9]=(uint8_t)(tcu_mng_pin_write_req_->BD_ADDR[2]);
   au8SendBuff[10]=(uint8_t)(tcu_mng_pin_write_req_->BD_ADDR[3]);
   au8SendBuff[11]=(uint8_t)(tcu_mng_pin_write_req_->BD_ADDR[4]);
   au8SendBuff[12]=(uint8_t)(tcu_mng_pin_write_req_->BD_ADDR[5]);
   au8SendBuff[13]=(uint8_t)(tcu_mng_pin_write_req_->Length_of_PIN_code);
   count=14;
   for(loop=0;loop<tcu_mng_pin_write_req_->Length_of_PIN_code;loop++){
   	au8SendBuff[count++]=(uint8_t)(tcu_mng_pin_write_req_->PIN_code[loop]);
	}
   count++;			
   count--;
   au8SendBuff[0]=(0x0000ff & count);
   au8SendBuff[1]=(0x00ff00 & count);
   au8SendBuff[2]=(0xff0000 & count);

   /**calculate ParameterLength field */
   loop = count - 7;
   au8SendBuff[5]=(uint8_t)(loop);//ParameterLength low Byte
   au8SendBuff[6]=(uint8_t)(loop>>8);//ParameterLength	high Byte   
   
   send_tx((uint8_t *)&au8SendBuff[0], count); 
 }

/**
* \fn      tcu_MNG_DISCOVER_REMOTE_SERVICE_REQ
* \brief   
* \param   void
* \return  void
*/
void tcu_MNG_DISCOVER_REMOTE_SERVICE_REQ(t_tcu_mng_discover_remote_service_req  * param){
  uint32_t i;
RTS_high;
  
  au8SendBuff[1]=0x00;
  au8SendBuff[2]=0x00; 
  au8SendBuff[3]=MNG_ServiceID;
  au8SendBuff[4]=TCU_MNG_DISCOVER_REMOTE_SERVICE_REQ; 
  
  au8SendBuff[7]=(uint8_t)(param->Security_Mode);  
  au8SendBuff[8]=(uint8_t)(param->BD_ADDR[0]);
  au8SendBuff[9]=(uint8_t)(param->BD_ADDR[1]);
  au8SendBuff[10]=(uint8_t)(param->BD_ADDR[2]);
  au8SendBuff[11]=(uint8_t)(param->BD_ADDR[3]);
  au8SendBuff[12]=(uint8_t)(param->BD_ADDR[4]);
  au8SendBuff[13]=(uint8_t)(param->BD_ADDR[5]);
  au8SendBuff[14]=(uint8_t)(param->Use_of_Link_Key);
  
  if((uint8_t)(param->Use_of_Link_Key)==1){
    for(i=0;i<16;i++){
      au8SendBuff[15+i]=(uint8_t)(param->Link_Key[i]);            
    }
    au8SendBuff[0]=31;//15+16;
  }else{
    au8SendBuff[0]=15;
  }
  au8SendBuff[5]=au8SendBuff[0]-7;//ParameterLength 
  au8SendBuff[6]=0x00;//ParameterLength 
  
  send_tx((uint8_t *)&au8SendBuff[0], au8SendBuff[0]);
}   



  /**
 * \fn      tcu_MNG_CHANGE_LOCAL_DEVICE_PARAM_REQ
 * \brief   This command is used to set name of local device.
 * \param   newDevice_Name
 * \return  void
 */
void tcu_MNG_CHANGE_LOCAL_DEVICE_PARAM_REQ(uint8_t *newDevice_Name){
   uint32_t count;
RTS_high;     
   au8SendBuff[3]=MNG_ServiceID;
   au8SendBuff[4]=TCU_MNG_CHANGE_LOCAL_DEVICE_PARAM_REQ;

   /*calculate Length_of_Device_Name*/
   for(count=0;(*newDevice_Name) != '\0';count++){
    au8SendBuff[count+8]=(uint8_t)(*newDevice_Name++);
   }
   au8SendBuff[7]=(uint8_t)(count);

   /*calculate Parameter Length*/
   count=count+1;
   au8SendBuff[5]=(0x00ff & count);//Parameter Length
   au8SendBuff[6]=(0xff00 & count);//Parameter Length
   
   /*calculate lenght of entire string*/
   count=count+7;   
   au8SendBuff[0]=(0x0000ff & count);
   au8SendBuff[1]=(0x00ff00 & count);
   au8SendBuff[2]=(0xff0000 & count);
   
   send_tx((uint8_t *)&au8SendBuff[0], count);  
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

				