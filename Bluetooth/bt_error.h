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

#ifndef _H_BT_ERROR_
#define _H_BT_ERROR_

/**********************************************
*                Include files                *
**********************************************/
#include "stdint.h"


/**********************************************
*             Constants & Macros              *
**********************************************/
/* Definition of API_RESULT */
typedef unsigned int    API_RESULT;

#define API_SUCCESS             0x0000
#define API_FAILURE             0xFFFF

#define BT_NO_ERR               (0x0000)

/* ======================================= Section 'HCI' */
#define HCI_ERR_CREATE_CONNECTION_FAILED            (0x1001)
#define HCI_ERR_INVALID_COMMAND_PACKET              (0x1002)
#define HCI_ERR_TIMEOUT                             (0x1003)

/* ======================================= Section 'MNG' */
#define MNG_ERR_PARAMETER_FAILURE                   (0x2001)
#define MNG_ERR_DEVICE_INITIALIZATION_FINISHED      (0x2002)
#define MNG_ERR_NO_DEVICE_INITIALIZATION            (0x2003)
#define MNG_ERR_ON_SEARCHING_DEVICE                 (0x2004)
#define MNG_ERR_ON_SEARCHING_DEVICE_SERVICE         (0x2005)
#define MNG_ERR_ON_PROGRESS_OF_OTHER_PROFILE_CONNECTION (0x200E)
#define MNG_ERR_ON_PROGRESS_SPP_CONNECTION_OR_ESTABLISH_SPP (0x2042)
#define MNG_ERR_NO_CONNECTION                       (0x2006)
#define MNG_ERR_NO_SETUP_PROFILE                    (0x2008)
#define MNG_ERR_PAGE_TIMEOUT                        (0x2080)
#define MNG_ERR_LOCAL_DEVICE_CONNECTION_REJECT      (0x2081)
#define MNG_ERR_LINK_LOST                           (0x2082)
#define MNG_ERR_PIN_CODE_INPUT_TIMEOUT              (0x2083)
#define MNG_ERR_PIN_CODE_FAILURE                    (0x2084)
#define MNG_ERR_LOCAL_DEVICE_PIN_CODE_INPUT_REJECT  (0x2085)
#define MNG_ERR_REMOTE_DEVICE_PIN_CODE_INPUT_REJECT (0x2086)
#define MNG_LINK_KEY_FAILURE                        (0x2087)
#define MNG_ERR_SDP_CONNECTION_FAILURE              (0x208C)
#define MNG_ERR_NO_SUPPORTED_SDP                    (0x208D)

/* ======================================= Section 'SPP' */
#define SPP_ERR_PARAMETER_FAILURE               (0x3001)
#define SPP_ERR_NO_DEVICE_INITIALIZATION        (0x3003)
#define SPP_ERR_DATA_EXCEEDS_BUFFER_SIZE        (0x3004)
#define SPP_ERR_SETUP_SPP                       (0x3040)
#define SPP_ERR_NO_SETUP_SPP                    (0x3041)
#define SPP_ERR_ESTABLISH_SPP                   (0x3042)
#define SPP_ERR_ON_RELEASING_SPP                (0x3043)
#define SPP_ERR_NO_SDP_SERVICE_SUPPORTED        (0x308D)
#define SPP_ERR_SPP_CONNECTION_TIME_OUT         (0x30D0)
#define SPP_ERR_CONNECTION_FAILURE              (0x30D3)
/* ======================================= Section TIMEOUT ERROR */
#define BT_ERR_TIMEOUT                          (0x5001)

/* ======================================= Section OS ERROR */
#define BT_ERR_QUEUE_COULD_NOT_BE_CREATED        (0x6002)


uint32_t errorCode(uint32_t err_);

#endif /* _H_BT_ERROR_ */
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