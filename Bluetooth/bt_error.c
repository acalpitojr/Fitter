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
#include "stdint.h"
#include "bt_error.h"



uint32_t errorCode(uint32_t err_){
  uint32_t ret;    
  switch (err_){
  case 0x00: ret=BT_NO_ERR;break;
    case 0x01: ret= MNG_ERR_PARAMETER_FAILURE; break;
    case 0x03: ret= MNG_ERR_NO_DEVICE_INITIALIZATION; break;
    case 0x02: ret= MNG_ERR_DEVICE_INITIALIZATION_FINISHED; break;
    case 0x06: ret= MNG_ERR_NO_CONNECTION; break;  
    case 0x08: ret= MNG_ERR_NO_SETUP_PROFILE; break;
    case 0x40: ret= SPP_ERR_SETUP_SPP; break;
  
    case 0x80: ret= MNG_ERR_PAGE_TIMEOUT; break;  
    case 0x81: ret= MNG_ERR_LOCAL_DEVICE_CONNECTION_REJECT; break;
    case 0x82: ret= MNG_ERR_LINK_LOST; break;   
    case 0x83: ret= MNG_ERR_PIN_CODE_INPUT_TIMEOUT; break;    
    case 0x84: ret= MNG_ERR_PIN_CODE_FAILURE; break;    
    case 0x85: ret= MNG_ERR_LOCAL_DEVICE_PIN_CODE_INPUT_REJECT; break;    
    case 0x86: ret= MNG_ERR_REMOTE_DEVICE_PIN_CODE_INPUT_REJECT; break;
    case 0x87: ret= MNG_LINK_KEY_FAILURE ;break;  
    
    default:
      ret = err_; break;
  }
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
