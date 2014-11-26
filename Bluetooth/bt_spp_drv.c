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
#include "bt_spp_drv.h"


/**********************************************
*         Variables and forward declarations  *
**********************************************/
volatile uint16_t BT_Mode = HCI_MODE;//HCI or TCU

/*Data Receive buffer definition*/
int8_t cSPP_DATA_RECEIVE_buff[SPP_DATA_RECEIVE_LENGHT];

/*Definition of SPP requests and Callback event spp_event*/
t_tcu_spp_connect_req tcu_spp_connect_req;
t_tcu_data_transfer_req tcu_data_transfer_req;

t_tcu_spp_disconnect_event tcu_spp_disconnect_event;
t_tcu_spp_connect_event tcu_spp_connect_event;
t_tcu_spp_data_receive_event tcu_spp_data_receive_event;
t_tcu_spp_data_send_event tcu_spp_data_send_event;
t_tcu_spp_line_notify_event tcu_spp_line_notify_event;

t_tcu_spp_setup_resp tcu_spp_setup_resp;
t_tcu_spp_shutdown_resp tcu_spp_shutdown_resp;
t_tcu_spp_uuid_assign_resp tcu_spp_uuid_assign_resp;
t_tcu_uuid_assign_req tcu_uuid_assign_req;

volatile uint32_t gSIOWrBytes;
volatile uint8_t * ptTxUARTbuff;
volatile uint8_t rxUARTbuff[SPP_DATA_RECEIVE_LENGHT];



/**
 * \fn      tcu_SPP_SETUP_REQ
 * \brief   Function to setup SPP device.
 * \param   void
 * \return  void
 */
void tcu_SPP_SETUP_REQ(void){
RTS_high;   
  au8SendBuff[0]=0x07;//string lenght of tcu_SPP_SETUP_REQ is 7
  au8SendBuff[1]=0x00;
  au8SendBuff[2]=0x00;   
  au8SendBuff[3]=SPP_ServiceID;
  au8SendBuff[4]=TCU_SPP_SETUP_REQ;
  au8SendBuff[5]=0x00; 
  au8SendBuff[6]=0x00;    
  
  send_tx((uint8_t *)&au8SendBuff[0], 7L); 
 }
 /**
 * \fn      tcu_SPP_SHUTDOWN_REQ
 * \brief   To shutdown SPP Function.
 * \param   void
 * \return  void
 */
 void tcu_SPP_SHUTDOWN_REQ(void){
RTS_high;   
   au8SendBuff[0]=0x07;//lenght of tcu_SPP_SHUTDOWN_REQ is 7
   au8SendBuff[1]=0x00;
   au8SendBuff[2]=0x00;   
   au8SendBuff[3]=SPP_ServiceID;
   au8SendBuff[4]=TCU_SPP_SHUTDOWN_REQ;
   au8SendBuff[5]=0x00; 
   au8SendBuff[6]=0x00;    
   send_tx((uint8_t *)&au8SendBuff[0], 7L);       
 }   
/**
 * \fn      tcu_SPP_CONNECT_REQ
 * \brief   To establish SPP connection with specified remote device.
 * \param   command parameter 
 * \return  void
 */
 void tcu_SPP_CONNECT_REQ(t_tcu_spp_connect_req * parmSPP_connect){
RTS_high;   
   RTS_high;   
   au8SendBuff[1]=0x00;
   au8SendBuff[2]=0x00; 
   au8SendBuff[3]=SPP_ServiceID;
   au8SendBuff[4]=TCU_SPP_CONNECT_REQ;
   
   au8SendBuff[7]= (uint8_t)(parmSPP_connect->BD_ADDR[0]);
   au8SendBuff[8]= (uint8_t)(parmSPP_connect->BD_ADDR[1]); 
   au8SendBuff[9]= (uint8_t)(parmSPP_connect->BD_ADDR[2]);
   au8SendBuff[10]=(uint8_t)(parmSPP_connect->BD_ADDR[3]);
   au8SendBuff[11]=(uint8_t)(parmSPP_connect->BD_ADDR[4]);
   au8SendBuff[12]=(uint8_t)(parmSPP_connect->BD_ADDR[5]);   
   
   au8SendBuff[13]=(uint8_t)(parmSPP_connect->BaudRate);
   au8SendBuff[14]=(uint8_t)(parmSPP_connect->uDataFormat.DataFormatByte);                            

   au8SendBuff[15]=(uint8_t)(parmSPP_connect->uFlowControl.FlowControlByte);                            
   
   au8SendBuff[16]=(uint8_t)(parmSPP_connect->XonChar); 
   au8SendBuff[17]=(uint8_t)(parmSPP_connect->XoffChar);     

   au8SendBuff[18]= (uint8_t)(parmSPP_connect->uParmMask.ParmMaskbyte); 
   au8SendBuff[19]= (uint8_t)((parmSPP_connect->uParmMask.ParmMaskbyte)>>8);

   au8SendBuff[20]=(uint8_t)(parmSPP_connect->Server_Channel_Validity);    
   au8SendBuff[21]=(uint8_t)(parmSPP_connect->Sever_Channel);
   au8SendBuff[22]=(uint8_t)(parmSPP_connect->Use_of_Link_Key);
   au8SendBuff[0]= 23;//lenght of the tcu_SPP_CONNECT_REQ string 
   if(parmSPP_connect->Use_of_Link_Key != 0){
    au8SendBuff[0]= 39;//lenght of the tcu_SPP_CONNECT_REQ string 
    au8SendBuff[22+1]=(uint8_t)(parmSPP_connect->Link_Key[0]);
    au8SendBuff[22+2]=(uint8_t)((parmSPP_connect->Link_Key[0])>>8); 
    au8SendBuff[22+3]=(uint8_t)((parmSPP_connect->Link_Key[0])>>16);
    au8SendBuff[22+4]=(uint8_t)((parmSPP_connect->Link_Key[0])>>24);
         
    au8SendBuff[26+1]=(uint8_t)(parmSPP_connect->Link_Key[1]);
    au8SendBuff[26+2]=(uint8_t)((parmSPP_connect->Link_Key[1])>>8); 
    au8SendBuff[26+3]=(uint8_t)((parmSPP_connect->Link_Key[1])>>16);
    au8SendBuff[26+4]=(uint8_t)((parmSPP_connect->Link_Key[1])>>24);
    
    au8SendBuff[30+1]=(uint8_t)(parmSPP_connect->Link_Key[2]);
    au8SendBuff[30+2]=(uint8_t)((parmSPP_connect->Link_Key[2])>>8); 
    au8SendBuff[30+3]=(uint8_t)((parmSPP_connect->Link_Key[2])>>16);
    au8SendBuff[30+4]=(uint8_t)((parmSPP_connect->Link_Key[2])>>24);
    
    au8SendBuff[34+1]=(uint8_t)(parmSPP_connect->Link_Key[3]);
    au8SendBuff[34+2]=(uint8_t)((parmSPP_connect->Link_Key[3])>>8); 
    au8SendBuff[34+3]=(uint8_t)((parmSPP_connect->Link_Key[3])>>16);
    au8SendBuff[34+4]=(uint8_t)((parmSPP_connect->Link_Key[3])>>24);
   }
   au8SendBuff[5]= (uint8_t)(au8SendBuff[0]-7);//calculate Parameter Length
   au8SendBuff[6]= 0x00;

   send_tx((uint8_t *)&au8SendBuff[0], au8SendBuff[0]);    
 }
 /**
 * \fn      tcu_SPP_DISCONNECT_REQ
 * \brief   Function to disconnect SPP connection.
 * \param   void
 * \return  void
 */
 void tcu_SPP_DISCONNECT_REQ(void){
RTS_high;   
   au8SendBuff[0]=0x07;//string lenght of tcu_SPP_SETUP_REQ is 7
   au8SendBuff[1]=0x00;
   au8SendBuff[2]=0x00;   
   au8SendBuff[3]=SPP_ServiceID;
   au8SendBuff[4]=TCU_SPP_DISCONNECT_REQ;
   au8SendBuff[5]=0x00; 
   au8SendBuff[6]=0x00; 
   
   send_tx((uint8_t *)&au8SendBuff[0], 7); 
 }
 /**
 * \fn      tcu_SPP_DATA_TRANSFER_REQ
 * \brief   Function to send SPP data to remote device. 
 * \param   command parameter 
 * \return  void
 */
 void tcu_SPP_DATA_TRANSFER_REQ(t_tcu_data_transfer_req * parmTransfData){
   uint16_t ind;
   uint8_t * ptDATA;
   parmTransfData->ParameterLength = 2 + parmTransfData->Length_of_Data;
   
RTS_high;   
   au8SendBuff[0]= (9 + parmTransfData->Length_of_Data);
   au8SendBuff[1]=0x00;
   au8SendBuff[2]=0x00;   
   au8SendBuff[3]=SPP_ServiceID;
   au8SendBuff[4]=TCU_SPP_DATA_TRANSFER_REQ;

   au8SendBuff[5]= (uint8_t)(parmTransfData->ParameterLength); 
   au8SendBuff[6]= (uint8_t)((parmTransfData->ParameterLength)>>8);
   
   au8SendBuff[7]= (uint8_t)(parmTransfData->Length_of_Data); 
   au8SendBuff[8]= (uint8_t)((parmTransfData->Length_of_Data)>>8);   
   
   ptDATA=parmTransfData->Txdata;
   for (ind=9;ind < (9 + parmTransfData->Length_of_Data);ind++){
    au8SendBuff[ind]=(uint8_t)((*ptDATA++));   
   }

   au8SendBuff[0]=(0x0000ff & ind);
   au8SendBuff[1]=(0x00ff00 & ind);
   au8SendBuff[2]=(0xff0000 & ind);
   
   send_tx((uint8_t *)&au8SendBuff[0], ind);   
 }   
 
  /**
 * \fn      tcu_SPP_UUID_ASSIGN_REQ
 * \brief   This command sets Service Class ID (UUID) on SDP for SPP.
 * \param   command parameter 
 * \return  void
 */
 void tcu_SPP_UUID_ASSIGN_REQ(t_tcu_uuid_assign_req * param_uuid_assign){
  uint8_t pos;
RTS_high;   
  au8SendBuff[1]=0x00;
  au8SendBuff[2]=0x00;   
  au8SendBuff[3]=SPP_ServiceID;
  au8SendBuff[4]=TCU_SPP_UUID_ASSIGN_REQ;   
  
  au8SendBuff[7]= (uint8_t)(param_uuid_assign->Initiate_UUID_Data_Type);

  au8SendBuff[7+1]=(uint8_t)(param_uuid_assign->Initiate_UUID_Value[0]);
  au8SendBuff[7+2]=(uint8_t)((param_uuid_assign->Initiate_UUID_Value[0])>>8);

  pos=10;  
  switch (param_uuid_assign->Initiate_UUID_Data_Type){
    case 0x1A:
      au8SendBuff[pos++]=(uint8_t)((param_uuid_assign->Initiate_UUID_Value[0])>>16);
      au8SendBuff[pos++]=(uint8_t)((param_uuid_assign->Initiate_UUID_Value[0])>>24);    
    break;
    case 0x1C:
      au8SendBuff[pos++]=(uint8_t)((param_uuid_assign->Initiate_UUID_Value[0])>>16);
      au8SendBuff[pos++]=(uint8_t)((param_uuid_assign->Initiate_UUID_Value[0])>>24);
      
      au8SendBuff[pos++]=(uint8_t)(param_uuid_assign->Initiate_UUID_Value[1]);
      au8SendBuff[pos++]=(uint8_t)((param_uuid_assign->Initiate_UUID_Value[1])>>8); 
      au8SendBuff[pos++]=(uint8_t)((param_uuid_assign->Initiate_UUID_Value[1])>>16);
      au8SendBuff[pos++]=(uint8_t)((param_uuid_assign->Initiate_UUID_Value[1])>>24);
      
      au8SendBuff[pos++]=(uint8_t)(param_uuid_assign->Initiate_UUID_Value[2]);
      au8SendBuff[pos++]=(uint8_t)((param_uuid_assign->Initiate_UUID_Value[2])>>8); 
      au8SendBuff[pos++]=(uint8_t)((param_uuid_assign->Initiate_UUID_Value[2])>>16);
      au8SendBuff[pos++]=(uint8_t)((param_uuid_assign->Initiate_UUID_Value[2])>>24);
    
      au8SendBuff[pos++]=(uint8_t)(param_uuid_assign->Initiate_UUID_Value[3]);
      au8SendBuff[pos++]=(uint8_t)((param_uuid_assign->Initiate_UUID_Value[3])>>8); 
      au8SendBuff[pos++]=(uint8_t)((param_uuid_assign->Initiate_UUID_Value[3])>>16);
      au8SendBuff[pos++]=(uint8_t)((param_uuid_assign->Initiate_UUID_Value[3])>>24);
      break;
    default:break;
  }
        
  
  au8SendBuff[pos++]= (uint8_t)(param_uuid_assign->Accept_UUID_Data_Type);
  au8SendBuff[pos++]=(uint8_t)(param_uuid_assign->Accept_UUID_Value[0]);
  au8SendBuff[pos++]=(uint8_t)((param_uuid_assign->Accept_UUID_Value[0])>>8);   
  switch((param_uuid_assign->Accept_UUID_Data_Type)){
    case 0x1A:
      au8SendBuff[pos++]=(uint8_t)((param_uuid_assign->Accept_UUID_Value[0])>>16);
      au8SendBuff[pos++]=(uint8_t)((param_uuid_assign->Accept_UUID_Value[0])>>24);     
    break;
    case 0x1C: 
      au8SendBuff[pos++]=(uint8_t)((param_uuid_assign->Accept_UUID_Value[0])>>16);
      au8SendBuff[pos++]=(uint8_t)((param_uuid_assign->Accept_UUID_Value[0])>>24);
      
      au8SendBuff[pos++]=(uint8_t)(param_uuid_assign->Accept_UUID_Value[1]);
      au8SendBuff[pos++]=(uint8_t)((param_uuid_assign->Accept_UUID_Value[1])>>8); 
      au8SendBuff[pos++]=(uint8_t)((param_uuid_assign->Accept_UUID_Value[1])>>16);
      au8SendBuff[pos++]=(uint8_t)((param_uuid_assign->Accept_UUID_Value[1])>>24);    
      
      au8SendBuff[pos++]=(uint8_t)(param_uuid_assign->Accept_UUID_Value[2]);
      au8SendBuff[pos++]=(uint8_t)((param_uuid_assign->Accept_UUID_Value[2])>>8); 
      au8SendBuff[pos++]=(uint8_t)((param_uuid_assign->Accept_UUID_Value[2])>>16);
      au8SendBuff[pos++]=(uint8_t)((param_uuid_assign->Accept_UUID_Value[2])>>24);    
      
      au8SendBuff[pos++]=(uint8_t)(param_uuid_assign->Accept_UUID_Value[3]);
      au8SendBuff[pos++]=(uint8_t)((param_uuid_assign->Accept_UUID_Value[3])>>8); 
      au8SendBuff[pos++]=(uint8_t)((param_uuid_assign->Accept_UUID_Value[3])>>16);
      au8SendBuff[pos++]=(uint8_t)((param_uuid_assign->Accept_UUID_Value[3])>>24); 
        break;
    default:break;    
  }

  au8SendBuff[5]= (uint8_t)(pos-7); 
  au8SendBuff[6]= (uint8_t)0x00;
  au8SendBuff[0]= (uint8_t)pos;  
  
  send_tx((uint8_t *)&au8SendBuff[0], au8SendBuff[0]);  
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
