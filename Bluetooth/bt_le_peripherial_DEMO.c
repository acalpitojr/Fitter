/*****************************************************************************
*   This software is copyright protected. All rights reserved
*   Toshiba Electronics Europe GmbH
*   European LSI Design and Engineering Center (ELDEC)
******************************************************************************
*   Description : Bluetooth(SPP) DEMO
******************************************************************************
*   PROJECT     : TC35661 "Bluetooth SPP"
*   MODULE      : Bluetooth SPP Driver
******************************************************************************
*   VERSION     : 1.00
*   DATE        : 1.12.2012
*****************************************************************************/


/**********************************************
*                Include files                *
**********************************************/
#include "bt_common.h"
#include "bt_spp_api.h"
#include "ble_api.h"
#include "HeartRate.h"

/**********************************************
*         Variables and forward declarations  *
**********************************************/
void* qHandle;
uint32_t err;//error code

/*****************************************************************************
*   This software is copyright protected. All rights reserved
*   Toshiba Electronics Europe GmbH
*   European LSI Design and Engineering Center (ELDEC)
******************************************************************************
*   Description : Bluetooth(SPP) DEMO
******************************************************************************
*   PROJECT     : TC35661 "Bluetooth SPP"
*   MODULE      : Bluetooth SPP Driver
******************************************************************************
*   VERSION     : 1.00
*   DATE        : 1.12.2012
*****************************************************************************/


/**********************************************
*                Include files                *
**********************************************/
#include <stdint.h>
#include <stdio.h>
//#include "main.h"
#include "bt_common.h"

/**********************************************
*             Constants & Macros              *
**********************************************/
#define tskIDLE_PRIORITY			( ( unsigned portBASE_TYPE ) 0U )
#define tskLED1_PRIORITY                        ( ( unsigned portBASE_TYPE ) 1U )
#define tskLED2_PRIORITY                        ( ( unsigned portBASE_TYPE ) 2U )
#define mainQUEUE_RECEIVE_TASK_PRIORITY         ( ( unsigned portBASE_TYPE ) 3U )


/**********************************************
*  Static variables and forward declarations  *
**********************************************/
/* The rate at which data is sent to the queue, specified in milliseconds. */
#define mainQUEUE_SEND_FREQUENCY_MS			( 10 / portTICK_RATE_MS )



void* btQueueEVENT = NULL;  /*this is only for freetos code i believe*/
/**************************************
**************************************/
//void vApplicationStackOverflowHook (xTaskHandle *pxTask, signed portCHAR *pcTaskName)
//{
	/* This function will get called if a task overflows its stack. */
 // (void)pxTask;
 // (void)pcTaskName;
 // while(1);
//}

/**************************************
**************************************/
void error(void){
  while(1);
}
/**************************************
**************************************/
#if 0
int main(void)
{
  unsigned portSHORT stackSize = 800;
  pdTASK_CODE pvTask;

  #ifdef  RUN_IN_RAM
  /* Set the Vector Table base location at 0x02000000 */
    SCB->VTOR = ((uint32_t)0x20000000);
  #else  /* VECT_TAB_FLASH  */
  /* Set the Vector Table base location at 0x00000000 */
    SCB->VTOR = 0x0 ;
  #endif

  SystemInit();

#if defined(BOARD_TOPASM369)
  SIO_Configuration(UART_BT_SSP);
  SIO_ChInit(UART_BT_SSP);
  NVIC_ClearPendingIRQ(UART_BT_SSP_TXINT);
  NVIC_ClearPendingIRQ(UART_BT_SSP_RXINT);
#elif defined(BOARD_TOPASM369_BT)
  FUART_GPIOConfig();         /* Congigure GPIO for FUART4 */
  FUART_ChInit();             /* Initialize FUART4 */
  FUART_Enable(FUART4);
  FUART_DisableFIFO(FUART4);
  NVIC_EnableIRQ(INTUART4_IRQn);
  FUART_SetINTMask(FUART4, 0x10);//enable receive interrupt mask
#endif

  pvTask = app_BT_BLE_peripherial_DEMO;

  if(pdPASS != xTaskCreate(pvTask, NULL, stackSize, (void*)0, 0, NULL)){
    while(1){};//endless loop
  }
  vTaskStartScheduler();

  while(1);
}
#endif

/********************************************************************************
 Function		app_BT_SPP_DEMO(void* pvParameters)
 Argumets		:pvParameters
 Returns		:non
 Explanation	:
*********************************************************************************/
void app_BT_BLE_peripherial_DEMO(void)
{
  uint32_t loop;
 // portBASE_TYPE queueRETURN;
  le_api_result_e eLeResult;
  uint16_t u16GAPSvcHandle;
  uint16_t u16DISvcHandle;
  uint16_t u16DICharDeclHndl;
  uint16_t u16DICharValDeclHndl;
  uint16_t u16HRSvcHndl;
  uint16_t u16HRCharDeclHndl;
  uint16_t au16HRCharValDeclHndl[4];
  uint16_t u16ConnHndl;
  uint16_t u16ClientRxMtuSize;
  le_conn_evt_st stConnectInfo;
  le_mtu_exchg_st stMtuEvtData;                       /* MTU Event Data */
  le_mtu_exchg_acc_resp_st stMtuAccResp;              /* MTU Accept Response */
  uint8_t au8DummyCharDescEvntData[50];               /* placeholder for characteristic desc. update evnt. */

  hr_init_st stHRInit;

  char auDevName[]="MY HRS";
  uint8_t au8ManufName[7] = {0x54,0x4F,0x53,0x48,0x49,0x42,0x41}; /* "Toshiba" string in UTF8-format */

 /*initializing the RESET PIN, AND TAKING THE DEVICE OUT OF RESET*/
#if 0
  #if defined(BOARD_TOPASM369_BT)
    /*RESET Chiron*/
    GPIO_SetOpenDrain(GPIO_PD, GPIO_BIT_7, ENABLE);//set PD7 to Open-drain
    GPIO_WriteDataBit(GPIO_PD, GPIO_BIT_7, 0);//set Reset signal to low
    GPIO_SetOutputEnableReg(GPIO_PD, GPIO_BIT_7, ENABLE);
    GPIO_SetInputEnableReg(GPIO_PD, GPIO_BIT_7, DISABLE);
    for(loop=0; loop< 40000;loop++){
    }
    GPIO_WriteDataBit(GPIO_PD, GPIO_BIT_7, 1);//set Reset signal to high
    for(loop=0; loop< 40000;loop++){
    }
    /*RESET Chiron*/
  #endif
#endif

//  /*Initiate OS resources for BT SPP driver*/
 // if(BT_os_init() != API_SUCCESS){
  //    while(1){};//endless loop
 // }

  /*Switch Chiron to TCU Compete mode*/
  //if(BT_hci_init((uint8_t *)(&BD_ADDR), (uint8_t *)&Device_Name) != API_SUCCESS){
  err = BT_hci_init((uint8_t *)(&BD_ADDR), (uint8_t *)&Device_Name);
  if(err != API_SUCCESS){
    while(1){};//endless loop
    }

  /* Prepare Heart Rate Profile Initialization Structure */
  stHRInit.pu8DevName = (uint8_t*)auDevName;                            /* Device Name of the Heart Rate Sensor */
  stHRInit.u8DevNameLen = sizeof(auDevName);                            /* Length of the Devace Name */
  stHRInit.pu16RetDevNameSvcHndl = &u16GAPSvcHandle;                    /* GAP Device Name Service Return Handle */

  stHRInit.pu8ManufName = au8ManufName;                                 /* GATT Manufacturer Name String Characteristic Value (Part of Device Identification Service) */
  stHRInit.pu8ManufNameLen = 7;                                         /* Manufacturer Name String length */

  stHRInit.pu16RetDISvcHndl = &u16DISvcHandle;                          /* GATT Device Identification Service Return Handle */
  stHRInit.pu16RetDICharDeclHndl = &u16DICharDeclHndl;                  /* GATT Device Identification Characteristic Declaration Return Handle */
  stHRInit.pu16RetDICharValDeclHndl = &u16DICharValDeclHndl;            /* GATT Device Identification Characteristic VALUE Declaration Return Handle */

  stHRInit.pu16RetHRSvcHndl = &u16HRSvcHndl;                            /* GATT Heart Rate Service Return Handle */
  stHRInit.pu16RetHRCharDeclHndl = &u16HRCharDeclHndl;                  /* GATT Heart Rate Measurement Characteristic Declaration Return Handle */
  stHRInit.pu16RetHRCharValDeclHndl = au16HRCharValDeclHndl;            /* GATT Heart Rate Measurement Characteristic VALUE Declaration Return Handle */
  stHRInit.pstConnectInfo = &stConnectInfo;                             /* Connection Info. Structure */
  stHRInit.pstMtuEvtData = &stMtuEvtData;                               /* MTU Event Data */
  stHRInit.pstMtuAccResp = &stMtuAccResp;                               /* MTU Accept Response */

  stHRInit.stCharDespEvt.pu8CharDescValue = au8DummyCharDescEvntData;   /* Write characteristic Descriptor placeholder */

  stMtuEvtData.pu16ConnHandle = &u16ConnHndl;
  stMtuEvtData.pu16ClientRxMtuSize = &u16ClientRxMtuSize;

  /* Initialize Heart Rate Profile */
  eLeResult = eHRInitialise(btQueueEVENT, &stHRInit);
  if(LE_API_SUCCCESS == eLeResult)
  {
    /* Start advertizing */
     eLeResult = eHRStartAdvertising(btQueueEVENT);
     if(eLeResult == LE_API_SUCCCESS)
     {
        // Start HR Profile Event Handler
        HRMainRoutine(btQueueEVENT, &stHRInit);
     }
  }
  while(1){};//endless loop
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
