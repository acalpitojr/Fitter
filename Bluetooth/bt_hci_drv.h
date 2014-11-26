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


#ifndef _H_BT_HCI_DRV_
#define _H_BT_HCI_DRV_

/**********************************************
*                Include files                *
**********************************************/
#include "stdint.h"


/**********************************************
*             Constants & Macros              *
**********************************************/
#define HCI_CHANGE_TO_COMPLETE_MODE_TIMEOUT   10000 //
#define HCI_WAIT_RESPONSE_TIME_30ms           30 //

/*HCI packet indicators*/
#define HCI_Command_Packet 0x01
#define HCI_ACL_Data_Packet 0x02
#define HCI_Event_Packet 0x04


#define HCI_OPCODE_EVENT_RESET 0x0e
#define HCI_OPCODE_EVENT_SET_MODE 0x0f
#define HCI_OPCODE_EVENT_CODE 0xff


/*Packet Format in HCI mode -------------------------*/
typedef struct t_hci_event_packet_{
  uint8_t HCI_Event_Packet_Type;//must be 0x04
  uint8_t EventCode;
  uint8_t len;
  uint8_t *Parameter;  
}t_hci_event_packet;
extern t_hci_event_packet hci_event_packet;
extern t_hci_event_packet *pt_hci_event_packet;//Callback HCI Even

typedef struct t_hci_M2_SET_event_packet_{
  uint8_t HCI_Event_Packet_Type;//
  uint8_t EventCode;
  uint8_t len;
  uint8_t OCF;
  uint8_t reserved1;
  uint8_t RequestCommand; 
  uint8_t reserved2;
  uint8_t reserved3;  
  uint8_t reserved4;  
  uint8_t Accepter;
  uint8_t Information_ID;
  uint8_t Command_Result;
  uint8_t Data_Type;
  uint8_t *Parameter;  
}t_hci_M2_SET_event_packet;

/*HCI packet*/
typedef enum
{
    HCI_COMMAND_PACKET = 0x01,
    HCI_ACL_DATA_PACKET = 0x02,
    HCI_EVENT_PACKET = 0x04			    
} eHCI_PACKET_INDICATOR_;
extern eHCI_PACKET_INDICATOR_ eHCI_PACKET_INDICATOR;

typedef enum
{
  HCI_NO_EVENT,
  HCI_EVENT_ERROR,
  HCI_EVENT_HARD_RESET,
  HCI_EVENT_RESET,
  HCI_EVENT_SET_MODE,
  HCI_GETTING_FIRMWARE_VERSION_EVENT,
  HCI_EEPROM_PATCH1_EVENT,
  HCI_EEPROM_PATCH2_EVENT,
  HCI_EEPROM_PATCH3_EVENT,   
  HCI_M2_DEEP_SLEEP_SET,
  HCI_WRITE_BD_ADDR,
  HCI_LE_RF_IMPROVE11,
  HCI_LE_RF_IMPROVE12,
  HCI_LE_RF_IMPROVE13,
  HCI_LE_RF_IMPROVE14,
  HCI_LE_RF_IMPROVE15,  
  HCI_COMMAND_STATUS
} eHCI_EVENT_;

typedef enum
{
  HCI_ERR_1 = 0x20,//Short of receiving packet.
  HCI_ERR_2 = 0x21,//Stop bit error
  HCI_ERR_3 = 0x22,//Over write error
  HCI_ERR_4 = 0x41,//Audio Core version mismatching error
  HCI_ERR_5 = 0x42//Audio Core boot timeout error
} eHCI_ERROR_;

/*Packet Format in HCI mode -------------------------*/
typedef struct t_hci_event_{
  uint16_t event;
  uint8_t status;
  uint8_t ParameterLength;
}t_hci_event;
extern t_hci_event hci_event;
extern t_hci_event * pt_hci_event;

void hci_Get_Version();
void hci_M2_SET_DeepSleep();
void hci_RESET(void);
void hci_SET_MODE(void);
void hci_Write_BD_ADDR(uint8_t *BD_ADDR);
void hci_Patch11_RF_LE_param(uint8_t commRec);
void hci_Patch12_RF_LE_param(void);
void hci_Patch13_RF_LE_param(void);
void hci_Patch14_RF_LE_param(void);
void hci_Patch15_RF_LE_param(void);


#endif /* _H_BT_HCI_DRV_ */