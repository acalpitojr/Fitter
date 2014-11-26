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


#ifndef _H_BT_SPP_DRV_
#define _H_BT_SPP_DRV_

/**********************************************
*                Include files                *
**********************************************/
#include "stdint.h"
//#include "main.h"
#include "bt_spp_api.h"
#include "bt_uart_transport.h"  

/**********************************************
*             Constants & Macros              *
**********************************************/
/* -------------------------------------------- SPP OpCode part */
#define SPP_ServiceID                     0xE5
#define TCU_SPP_SETUP_REQ           0x01
#define TCU_SPP_SETUP_RESP          0x81
#define TCU_SPP_SHUTDOWN_REQ        0x02
#define TCU_SPP_SHUTDOWN_RESP       0x82
#define TCU_SPP_CONNECT_REQ         0x03
#define TCU_SPP_DISCONNECT_REQ      0x04
#define TCU_SPP_DATA_TRANSFER_REQ   0x08
#define TCU_SPP_UUID_ASSIGN_REQ     0x20
#define TCU_SPP_UUID_ASSIGN_RESP    0xA0

#define TCU_NO_EVENT                0x00
#define TCU_SPP_DATA_RECEIVE_EVENT  0x48
#define TCU_SPP_DATA_SEND_EVENT     0xF1
#define TCU_SPP_CONNECT_EVENT       0x43
#define TCU_SPP_DISCONNECT_EVENT    0x44
#define TCU_SPP_LINE_NOTIFY_EVENT   0x47


#define SPP_WAIT_RESPONSE_TIME_100ms 100
#define SPP_WAIT_RESPONSE_TIME_10s 10000
#define SPP_WAIT_RESPONSE_TIME_60s 60000

/* ---------------------------------------- Structures/Data Types */
/**
 *  SPP Request Types Section
 */

/*Data structure for TCU_SPP_CONNECT_REQ*/
union uDataFormat_{
  struct {
    uint8_t BIT_0:1;//unused
    uint8_t DataBit:2;
    uint8_t StopBit:1;
    uint8_t Parity:1;
    uint8_t ParityType:2;
    uint8_t BIT_7:1;//unused
  }sDataFormat_;
  uint8_t DataFormatByte;
};

union uFlowControl_{
  struct {
    uint8_t NoFlowControl:1;
    uint8_t XFlowInput:1;
    uint8_t XFlowOutput:1;
    uint8_t RTRInput:1;
    uint8_t RTROutput:1;
    uint8_t RTCInput:1;
    uint8_t RTCOutput:1;
    uint8_t Unused:1;//This bit is ignored. 
  }sFlowControl_;
  uint8_t FlowControlByte;
};  

union uParmMask_{
  struct {
    uint8_t RFCOMM_RPN_MASK_BAUD:1;
    uint8_t RFCOMM_RPN_MASK_DATA:1;
    uint8_t RFCOMM_RPN_MASK_STOP:1;
    uint8_t RFCOMM_RPN_MASK_PARITY:1;
    uint8_t RFCOMM_RPN_MASK_PARITY_TYPE:1;
    uint8_t RFCOMM_RPN_MASK_XON_CHAR:1;
    uint8_t RFCOMM_RPN_MASK_XOFF_CHAR:1;
    uint8_t Unused1:1;//This bit is ignored
    uint8_t RFCOMM_RPN_MASK_FLOW_X_IN:1;
    uint8_t RFCOMM_RPN_MASK_FLOW_X_OUT:1;
    uint8_t RFCOMM_RPN_MASK_FLOW_RTR_IN:1;
    uint8_t RFCOMM_RPN_MASK_FLOW_RTR_OUT:1;  
    uint8_t RFCOMM_RPN_MASK_FLOW_RTC_IN:1;
    uint8_t RFCOMM_RPN_MASK_FLOW_RTC_OUT:1;
    uint8_t Unused2:2;    
  }sParmMask_;
  uint16_t  ParmMaskbyte;
};  

typedef struct t_tcu_spp_connect_req_{
  uint16_t ParameterLength;
  uint8_t BD_ADDR[6];
  uint8_t BaudRate;
  union uDataFormat_ uDataFormat;
  union uFlowControl_ uFlowControl;
  uint8_t XonChar;
  uint8_t XoffChar;
  union uParmMask_ uParmMask;
  uint8_t Server_Channel_Validity;
  uint8_t Sever_Channel; 
  uint8_t Use_of_Link_Key;
  uint32_t Link_Key[4];
}t_tcu_spp_connect_req;

/*TCU_SPP_DATA_TRANSFER_REQ*/
typedef struct t_tcu_data_transfer_req_{
  uint16_t ParameterLength;
  uint16_t Length_of_Data;
  uint8_t *Txdata;
}t_tcu_data_transfer_req;

/*TCU_SPP_UUID_ASSIGN_REQ*/
typedef enum
{    
  UUID16 = 0x19,  
  UUID32 = 0x1A,
  UUID128 = 0x1C   
} SPP_UUID_DATA_TYPE;
typedef struct t_tcu_uuid_assign_req_{
  uint16_t ParameterLength;
  uint16_t Length_of_Data;
  SPP_UUID_DATA_TYPE Initiate_UUID_Data_Type;
  uint32_t Initiate_UUID_Value[4];//16Bytes
  SPP_UUID_DATA_TYPE Accept_UUID_Data_Type;
  uint32_t Accept_UUID_Value[4];
}t_tcu_uuid_assign_req;

/* ------------------------------------------------------- */
/**
 *  SPP Event Types Section
 */
/*EVENT Status result values*/
typedef enum
{    
  SUCCESSFUL_EVENT = 0x00,  
  NO_SDP_SERVICE_SUPPORTED=0x8D,
  SPP_CONNECTION_TIMER_OUT=0xD0,
  SPP_RELEASING_TIMER_OUT=0xD2,  
  SPP_CONNECTION_FAILURE=0xD3     
} SPP_EVENT_STATUS;

typedef enum
{    
  RELEASING_REQUIRED_FROM_LOCAL_HOST=0x01,
  RELEASING_REQUIRED_FROM_REMOTE_DEVICE=0x02,
  DISCONNECTION_ERROR=0x03,    
  LINK_LOSS=0x04,   
} SPP_DISCONNECT_EVENT_REASON;

/*TCU_SPP_CONNECT_EVENT*/
typedef struct t_tcu_spp_connect_event_{ 
  uint16_t ParameterLength;  
  SPP_EVENT_STATUS status;//value of SPP_DISCONNECT_EVENT_STATUS
  uint8_t BD_ADDR[6];
  uint8_t Negotiated_Frame_Size[2];
  uint8_t Length_of_Device_Name;
  uint8_t Device_Name[248];
}t_tcu_spp_connect_event;
typedef struct t_tcu_spp_disconnect_event_{
  uint16_t ParameterLength;  
  uint8_t status;//value of SPP_DISCONNECT_EVENT_STATUS
  uint8_t BD_ADDR[6];
  uint8_t Reason;//value of SPP_DISCONNECT_EVENT_REASON
}t_tcu_spp_disconnect_event;

/*TCU_SPP_LINE_NOTIFY_EVENT*/
typedef struct t_tcu_spp_line_notify_event_{
  uint16_t ParameterLength;//always 0x0001
  uint8_t Line_Status;  
}t_tcu_spp_line_notify_event;

/*TCU_SPP_DATA_RECEIVE_EVENT*/
typedef struct t_tcu_spp_data_receive_event_{
  uint16_t ParameterLength;
  uint16_t Length_of_Data;
  uint8_t *Rxdata;
}t_tcu_spp_data_receive_event;

/*TCU_SPP_DATA_SEND_EVENT*/
typedef struct t_tcu_spp_data_send_event_{
  uint16_t ParameterLength;//always 0x0000
}t_tcu_spp_data_send_event;


/* ------------------------------------------------------- */
/**
 *  SPP Respond Types Section
 */
/*RESPOND Status result values*/
typedef enum
{
  SUCCESSFUL_RESP = 0x00,
  PARAMETER_FAILURE = 0x01,
  NO_DEVICE_INITIALIZATION = 0x03,
  ON_SEARCHING_DEVICE = 0x04,
  ON_SEARCHING_SERVICE = 0x05,
  UNDER_CONNECTION_SETUP_OF_OTHER_PROFILE = 0x0E,
  SETUP_SPP = 0x40,
  NO_SETUP_SPP = 0x41,
  ESTABLISH_SPP = 0x042,
  ON_RELEASING_SPP = 0x043,
  NO_SPP_CONNECTION = 0x44,
  ON_TRANSFERRING_SPP_DATA = 0x46,
  UNDER_SPP_OPERATION = 0x47  
} TCU_SPP_SETUP_RESPOND_STATUS_;

/*TCU_SPP_SETUP_RESP*/
typedef struct t_tcu_spp_setup_resp_{
  uint16_t ParameterLength;//always 0x0001
  TCU_SPP_SETUP_RESPOND_STATUS_ status;
}t_tcu_spp_setup_resp;

/*TCU_SPP_SHUTDOWN_RESP*/
typedef struct t_tcu_spp_shutdown_resp_{
  uint16_t ParameterLength;//always 0x0001
  TCU_SPP_SETUP_RESPOND_STATUS_ status;//
}t_tcu_spp_shutdown_resp;

/*TCU_SPP_UUID_ASSIGN_RESP*/
typedef struct t_tcu_spp_uuid_assign_resp_{
  uint16_t ParameterLength;//always 0x0001
  TCU_SPP_SETUP_RESPOND_STATUS_ status;
}t_tcu_spp_uuid_assign_resp;


/*SPP function declarations for SPP REQUEST commands*/
void tcu_SPP_SETUP_REQ(void);
void tcu_SPP_SHUTDOWN_REQ(void);
void tcu_SPP_CONNECT_REQ(t_tcu_spp_connect_req * parmSPP_connect);
void tcu_SPP_DISCONNECT_REQ(void);
void tcu_SPP_DATA_TRANSFER_REQ(t_tcu_data_transfer_req * parm_dataTx);
void tcu_SPP_UUID_ASSIGN_REQ(t_tcu_uuid_assign_req * param_uuid_assign);


/*SPP variables for SPP REQUEST commands*/
extern t_tcu_spp_connect_req tcu_spp_connect_req;
extern t_tcu_data_transfer_req tcu_data_transfer_req;
extern t_tcu_uuid_assign_req tcu_uuid_assign_req;

extern t_tcu_spp_disconnect_event tcu_spp_disconnect_event;
extern t_tcu_spp_connect_event tcu_spp_connect_event;
extern t_tcu_spp_data_receive_event tcu_spp_data_receive_event;
extern t_tcu_spp_data_send_event tcu_spp_data_send_event;
extern t_tcu_spp_line_notify_event tcu_spp_line_notify_event;

extern t_tcu_spp_setup_resp tcu_spp_setup_resp;
extern t_tcu_spp_shutdown_resp tcu_spp_shutdown_resp;
extern t_tcu_spp_uuid_assign_resp tcu_spp_uuid_assign_resp;

/*eof*/



volatile extern uint8_t rxUARTbuff[];
volatile extern uint8_t * ptTxUARTbuff;
extern volatile uint32_t gSIOWrBytes;

extern int8_t cSPP_DATA_RECEIVE_buff[];
//extern volatile uint8_t au8SendBuff[];
extern uint8_t au8SendBuff[];
volatile extern uint16_t BT_Mode;//HCI or TCU
extern t_tcu_event ul_BT_SPP_rcvValue;
//extern xQueueHandle btQueueEVENT;
#endif /* _H_BT_SPP_DRV_ */

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
