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

#ifndef _H_BT_SPP_API_
#define _H_BT_SPP_API_

/**********************************************
*                Include files                *
**********************************************/
#include "stdint.h"
#include "app_bt.h"

/**********************************************
*         Variables and forward declarations  *
**********************************************/
extern const uint8_t BD_ADDR[];
extern const uint8_t Device_Name[];
extern const uint8_t PIN[];
extern int8_t cSPP_DATA_RECEIVE_buff[];

/* ----------------------------------------------- Structures/Data Types */
typedef struct t_bt_remote_device_{
  uint8_t currDevNo; //current device number
  uint8_t BD_ADDR[6];
  uint32_t Class_of_Device;
  uint8_t Length_of_Device_Name;
  uint8_t Device_Name[24];
}t_bt_remote_device;
extern t_bt_remote_device bt_remote_device[];

typedef struct t_bt_service_information_{
  uint8_t BD_ADDR[6];
  uint8_t Service_Count;
  uint8_t Service_Type;
  uint8_t SPP_Ex_info_Profile;
  uint8_t Number_of_ServerChannel;  
  uint8_t SPP_Server_Channel[10]; 
}t_bt_service_information;
extern t_bt_service_information bt_service_information;


/* ----------------------------------------------- Function Declarations */
/*  API */
API_RESULT BT_hci_init(unsigned char *BD_ADDR, unsigned char *Dev_Name);
API_RESULT BT_spp_discover_remote_device( uint8_t max_Number_of_Reports_, t_bt_remote_device * remote_dev, uint8_t * foundDevices);
API_RESULT BT_spp_discover_remote_service(t_bt_remote_device * param1_, t_bt_service_information * param2_);
API_RESULT BT_spp_connect(void);//establishes service level connection
API_RESULT BT_spp_connect_remote_device(t_bt_remote_device * param1_, t_bt_service_information * param2_);
void BT_spp_disconnect(void);
API_RESULT BT_spp_send(uint8_t *txBuff, uint16_t data_len);
void BT_spp_shutdown(void);
API_RESULT BT_spp_scan(uint8_t parm);
API_RESULT BT_spp_rcv_connect_req(void*  pADDR, uint32_t timeout);
API_RESULT BT_spp_connection_accept_req(uint8_t *BD_ADDR);
/*FreeRTOS */
API_RESULT BT_os_init(void);
#endif
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