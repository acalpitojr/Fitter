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

#ifndef _H_BT_MNG_DRV_
#define _H_BT_MNG_DRV_

/**********************************************
*                Include files                *
**********************************************/
#include "bt_uart_transport.h"
#include "bt_spp_drv.h"


/**********************************************
*             Constants & Macros              *
**********************************************/
#define MNG_ServiceID                       0xE1
/*Menagement Interface OpCode part*/
#define TCU_MNG_INIT_REQ              0x01
#define TCU_MNG_STANDARD_HCI_SET_REQ 0x3d
#define TCU_MNG_DISCOVER_REMOTE_DEVICE_REQe     0x03
#define TCU_MNG_DISCOVER_REMOTE_SERVICE_REQ     0x05
#define TCU_MNG_PIN_WRITE_REQ                   0x09
#define TCU_MNG_CHANGE_LOCAL_DEVICE_PARAM_REQ   0x11
#define TCU_MNG_SET_SCAN_REQ          0x0c
#define TCU_MNG_CONNECTION_ACCEPT_REQ 0x13

#define TCU_MNG_INIT_RESP       0x81
//#define TCU_MNG_TCU_ACCEPT      0xf1
#define TCU_ACCEPT      0xf1
#define TCU_MNG_TCU_NOT_ACCEPT  0xf2
#define TCU_MNG_SET_SCAN_RESP   0x8c

#define TCU_MNG_PIN_WRITE_RESP 0x89
#define TCU_MNG_SSP_INFO_EVENT 0x7d
#define TCU_MNG_DISCOVER_REMOTE_DEVICE_RESULT_EVENT   0x44
#define TCU_MNG_DISCOVER_REMOTE_DEVICE_COMPLETE_EVENT 0x43
#define TCU_MNG_DISCOVER_REMOTE_SERVICE_EVENT         0x45
#define TCU_MNG_CONNECTION_STATUS_EVENT 0x47
#define TCU_MNG_CONNECTION_REQUEST_EVENT 0x55
#define TCU_MNG_CHANGE_LOCAL_DEVICE_PARAM_RESP 0x91
#define TCU_MNG_CONNECTION_ACCEPT_RESP 0x93
#define TCU_MNG_PIN_REQUEST_EVENT 0x48
#define TCU_MNG_STANDARD_HCI_SET_RESP 0xbd
#define TCU_MNG_REMOTE_DEVICE_NAME_AUTO_NOTIFY_EVENT 0x6e



/*Supported_Profiles*/
#define SUPPORTED_PROFILE_SPP               (1<<2)

/*Profile Indicator*/
#define PROFILE_INDICATOR_SPP_B_PARTY           0x03

#define Scan_Mode_InquiryScan_OFF_PageScan_OFF  0x00
#define Scan_Mode_InquiryScan_ON_PageScan_OFF   0x01
#define Scan_Mode_InquiryScan_OFF_PageScan_ON   0x02
#define Scan_Mode_InquiryScan_ON_PageScan_ON    0x03


/*TCU_MNG_INIT_REQ*/
typedef struct t_tcu_mng_init_req_{
  uint16_t ParameterLength;
  uint8_t Supported_Profiles;
  uint8_t Paired_Information_Stored;
  uint8_t Length_of_Device_Name;
  uint8_t *Device_Name_;
}t_tcu_mng_init_req;
extern t_tcu_mng_init_req tcu_mng_init_req;

/*TCU_MNG_CONNECTION_ACCEPT_REQ*/
typedef struct {//t_tcu_mng_connection_accept_req_
  uint16_t ParameterLength;
  uint8_t Response_Type;
  uint8_t BD_ADDR[6];
  uint8_t Use_of_Link_Key;
  uint8_t Link_Key[16];  
}t_tcu_mng_connection_accept_req;
extern t_tcu_mng_connection_accept_req tcu_mng_connection_accept_req;

/*TCU_MNG_SSP_SET_REQ*/
#define  HCI_IO_Capability_Request_Reply  0x042B
#define  HCI_IO_Capability_Request_Negative_Reply  0x0434
#define  HCI_User_Confirmation_Request_Reply  0x042C
#define  HCI_User_Confirmation_Negative_Reply  0x042D
#define  HCI_IO_User_Passkey_Request_Reply 0x042E
#define  HCI_Write_Simple_Pairing_Debug_Mode_Command  0x1804

typedef enum{
  DISPLAY_ONLY = 0x00,
  DISPLAY_YES_NO = 0x01,
  KEYBOARD_ONLY = 0x02,
  NO_INPUT_NO_OUTPUT =0x03  
} e_IO_CAPABILITY;

typedef enum{
  OOB_AUTHENTICATION_DATA_NOT_PRESENT = 0x00,
  OOB_AUTHENTICATION_DATA_FROM_REMOTE_PRESENT = 0x01 
} e_OOB_DATA_PRESENT;

typedef enum{
  MITM_Protection_Not_Required_NoBonding = 0x00,
  MITM_Protection_Required_NoBonding_IO_Capabilities = 0x01,
  MITM_Protection_Not_Required_Dedicated_Bonding = 0x02,
  MITM_Protection_Required_Dedicated_BondingIO_Capabilities = 0x03,
  MITM_Protection_Not_RequiredGeneral_Bonding = 0x04,
  MITM_Protection_RequiredGeneral_Bonding_IO_Capabilities = 0x05  
} e_Authentication_Req;

typedef union{
   struct {//HCI_IO_Capability_Request_Reply
    uint8_t Parameter_Length;//9
    uint8_t BD_ADDR[6];
    e_IO_CAPABILITY IO_Capability;
    e_OOB_DATA_PRESENT OOB_Data_Present;
    e_Authentication_Req Authentication_Requirement;
  }t_HCI_IO_Capability_Request_Reply_Parameter;
  
   struct {//HCI_User_Confirmation_Request_Reply
    uint8_t Parameter_Length;//6
    uint8_t BD_ADDR[6];
  }t_HCI_User_Confirmation_Request_Reply_Parameter;

}u_TCU_MNG_SSP_SET_REQ_Parameter_;

typedef struct t_tcu_mng_spp_set_req_{
  uint16_t ParameterLength;
  uint16_t OpCode;
  u_TCU_MNG_SSP_SET_REQ_Parameter_ param;
}t_tcu_mng_spp_set_req;
extern t_tcu_mng_spp_set_req tcu_mng_spp_set_req;


typedef struct t_tcu_mng_discover_remote_service_req_{
  uint16_t ParameterLength;
  uint16_t OpCode;
  uint8_t Security_Mode;
  uint8_t BD_ADDR[6];
  uint8_t Use_of_Link_Key;
  uint8_t Link_Key[16];
}t_tcu_mng_discover_remote_service_req;
extern t_tcu_mng_discover_remote_service_req tcu_mng_discover_remote_service_req;


/*MNG TCU_ACCEPT*/
typedef struct t_tcu_mng_tcu_accept_{
  uint16_t ParameterLength;
  uint8_t Status;
  uint8_t Command_ServiceID;
  uint8_t Command_OpCode;
}t_tcu_mng_tcu_accept;
extern t_tcu_mng_tcu_accept tcu_mng_tcu_accept;

/*MNG TCU_NON_ACCEPT*/
typedef struct t_tcu_mng_tcu_non_accept_{
  uint16_t ParameterLength;
  uint8_t Command_ServiceID;
  uint8_t Command_OpCode;
}t_tcu_mng_tcu_non_accept;
extern t_tcu_mng_tcu_non_accept tcu_mng_tcu_non_accept;

/*TCU_MNG_INIT_RESP*/
typedef struct t_tcu_mng_init_resp_{
  uint16_t ParameterLength;
  uint8_t Status;
  uint8_t BD_ADDR[6];
}t_tcu_mng_init_resp;
extern t_tcu_mng_init_resp tcu_mng_init_resp;

/*TCU_MNG_SET_SCAN_RESP*/
typedef struct t_tcu_mng_set_scan_resp_{
  uint16_t ParameterLength;
  uint8_t Status;
}t_tcu_mng_set_scan_resp;
extern t_tcu_mng_set_scan_resp tcu_mng_set_scan_resp;


/*TCU_MNG_PIN_WRITE_RESP*/
typedef struct t_tcu_mng_pin_write_resp_{
  uint16_t ParameterLength;
  uint8_t status;
  uint8_t BD_ADDR[6];
}t_tcu_mng_pin_write_resp;
extern t_tcu_mng_pin_write_resp tcu_mng_pin_write_resp;

/*TCU_MNG_STANDARD_HCI_SET_RESP*/
typedef struct t_tcu_mng_standard_hci_set_resp_{
  uint16_t ParameterLength;
  uint8_t status;
}t_tcu_mng_standard_hci_set_resp;
extern t_tcu_mng_standard_hci_set_resp tcu_mng_standard_hci_set_resp;

/*TCU_MNG_SSP_INFO_EVENT*/
typedef enum
{
  IO_Capability_Request_Event = 0x31,
  IO_Capability_Response_Event = 0x32,
  IO_User_Confirmation_Request_Event = 0x33,
  Simple_Pairing_Complete_Event = 0x36,
  User_Passkey_Notification = 0x3b
} eTCU_MNG_SSP_INFO_EVENT_OpCode_;

typedef struct {
  uint8_t Parameter_Length;//6
  uint8_t BD_ADDR[6];
}t_HCI_IO_Capability_Request_Event_Parameter;

typedef struct {
  uint8_t Parameter_Length;//10
  uint8_t BD_ADDR[6];
  uint8_t NumericValue[4];  
}t_HCI_IO_User_Confirmation_Request_Event_Parameter;

typedef struct {
  uint16_t ParameterLength;
  eTCU_MNG_SSP_INFO_EVENT_OpCode_ OpCode;
  uint8_t Parameter_Length;
  uint8_t Parameter[16];//1.1.30.1 HCI_IO_Capability_Request_Event in MNG spec
}t_tcu_mng_ssp_info_event_;
extern t_tcu_mng_ssp_info_event_ tcu_mng_ssp_info_event;


/*TCU_MNG_DISCOVER_REMOTE_DEVICE_RESULT_EVENT*/
typedef struct t_tcu_mng_discover_remote_device_result_event_{
  uint16_t ParameterLength;
  uint8_t BD_ADDR[6];
  uint8_t Class_of_Device[3];
  uint8_t Length_of_Device_Name;
  uint8_t Device_Name[24];
}t_tcu_mng_discover_remote_device_result_event;
extern t_tcu_mng_discover_remote_device_result_event tcu_mng_discover_remote_device_result_event;

/*TCU_MNG_DISCOVER_REMOTE_SERVICE_EVENT*/
typedef struct t_tcu_mng_discover_remote_service_event_{
  uint16_t ParameterLength;
  uint8_t status;
  uint8_t BD_ADDR[6];
  uint8_t Service_Count;
  uint8_t Service_Type; 
  uint8_t SPP_Ex_info_Profile;
  uint8_t SPP_Ext_Info_Length;//max is 10 according to SPP manual
  uint8_t Number_of_ServerChannel;
  uint8_t SPP_Server_Channel[10]; 
}t_tcu_mng_discover_remote_service_event;
extern t_tcu_mng_discover_remote_service_event tcu_mng_discover_remote_service_event;

/*TCU_MNG_PIN_REQUEST_EVENT*/
typedef struct t_tcu_mng_pin_request_event_{
  uint16_t ParameterLength;
  uint8_t BD_ADDR[6];
  uint8_t Length_of_Device_Name;
  uint8_t Device_Name[248];
}t_tcu_mng_pin_request_event;
extern t_tcu_mng_pin_request_event tcu_mng_pin_request_event;


/*TCU_MNG_CONNECTION_ACCEPT_RESP*/
typedef struct t_tcu_mng_connection_accept_resp_{
  uint16_t ParameterLength;
  uint8_t Status;//to be checked
}t_tcu_mng_connection_accept_resp;
extern t_tcu_mng_connection_accept_resp tcu_mng_connection_accept_resp;

/*TCU_MNG_CONNECTION_STATUS_EVENT*/
typedef struct t_tcu_mng_connection_status_event_{
  uint16_t ParameterLength;
  uint8_t Status;
  uint8_t BD_ADDR[6];
  uint8_t Connection_Status;
  uint8_t Link_Key[16];
  uint8_t Link_Key_Type;
}t_tcu_mng_connection_status_event;
extern t_tcu_mng_connection_status_event tcu_mng_connection_status_event;


/*TCU_MNG_CONNECTION_REQUEST_EVENT*/
typedef struct t_tcu_mng_connection_request_event_{
  uint16_t ParameterLength;
  uint8_t BD_ADDR[6];
  uint8_t Class_of_Device[3];//to be checked
}t_tcu_mng_connection_request_event;
extern t_tcu_mng_connection_request_event tcu_mng_connection_request_event;

/*TCU_MNG_PIN_WRITE_REQ*/
typedef struct t_tcu_mng_pin_write_req_{
  uint16_t ParameterLength;
  uint8_t BD_ADDR[6];
  uint8_t Length_of_PIN_code;
  uint8_t PIN_code[16];
}t_tcu_mng_pin_write_req;
extern t_tcu_mng_pin_write_req tcu_mng_pin_write_req;

/*TCU_MNG_CHANGE_LOCAL_DEVICE_PARAM_REQ*/
typedef struct {
  uint16_t ParameterLength;
  uint8_t Length_of_Device_Name;
  uint8_t *pt_Device_Name; 
}t_tcu_mng_change_local_device_param_req;
extern t_tcu_mng_change_local_device_param_req tcu_mng_change_local_device_param_req;

void tcu_MNG_CONNECTION_ACCEPT_REQ(t_tcu_mng_connection_accept_req  * param);
void tcu_MNG_CHANGE_LOCAL_DEVICE_PARAM_REQ(uint8_t *newDevice_Name);
void tcu_MNG_SSP_SET_REQ(t_tcu_mng_spp_set_req * param );
void tcu_MNG_INIT_REQ(t_tcu_mng_init_req  * tcu_mng_init_req_);
void tcu_MNG_SET_SCAN_REQ(uint8_t Scan_Mode);
void tcu_MNG_DISCOVER_REMOTE_DEVICE_REQ(uint8_t Max_Number_of_Reports );
void tcu_MNG_DISCOVER_REMOTE_SERVICE_REQ(t_tcu_mng_discover_remote_service_req  * tcu_mng_rem_service);
void tcu_MNG_PIN_WRITE_REQ(t_tcu_mng_pin_write_req  * tcu_mng_pin_write_req_);
#endif                          /* _H_BT_MNG_DRV_ */
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
