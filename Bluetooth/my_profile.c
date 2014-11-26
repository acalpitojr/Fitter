

/**********************************************
*                Include files                *
**********************************************/
#include "app_bt.h"
#include "le_gatt_command.h"
#include "bt_uart_transport.h"
#include "ble_api.h"
#include "bt_OS.h"
#include "my_profile.h"
#include <stdio.h>

/**********************************************
*               Defs                          *
**********************************************/

/**********************************************
*               Globals                          *
**********************************************/
/* GAP Device Name Service definition structure */
le_srv_service_def_st stGAPService;

/* GATT Heart Rate Service definition structure */
le_srv_service_def_st stMyService;

/* GATT Device Identification Service definition structure */
le_srv_service_def_st stDevIdService;

/* Heart Rate Measurement Characteristic */
le_srv_char_st stMyCharacteristic;

uint8_t fMP_1HZ_tick = 0U;
uint8_t NOTIFY_CLIENT=0U;

volatile uint8_t my_data[]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};

/******************************************************************************
*
*                       GAP/GATT SERVICE REGISTRATION
*
******************************************************************************/

/******************************************************************************/
le_api_result_e eMPAddGAPDevName_Appearance_Service(void* qHandle,
                                                   uint16_t* pu16SvcHandle,
                                                   uint16_t u16DevNameLen,
                                                   uint8_t* pu8DevName)
{
    le_api_result_e eResTemp;
    le_srv_char_st astChar[2];
    uint16_t au16ElementHandles_dev_name[1];
    uint16_t au16ElementHandles_appear[1];
    uint8_t  au8AppearData[2] = {0x00,0x00};


    /********************************************
    *********************************************
              Add Primary (GAP) service
    *********************************************
    ********************************************/
    stGAPService.u16SvcUUID = GAP_SERVICE;                                              /* GAP Service UUID */
    stGAPService.u8InclDefCount =0;                                                     /* No included services */
    stGAPService.u8CharDefCount =2;                                                     /* Two characteristics belong to this service */
    stGAPService.pstIncludes = NULL;                                                    /* No included services */
    stGAPService.pstChars = astChar;                                                    /* Pointer to Array that describes characteristics. */

    /*****************************************
           Characteristic DEVICE_NAME
     *****************************************/
    stGAPService.pstChars[0].u16CharUUID      = (uint16_t)GAP_DEV_NAME;                 /* Characteristic UUID */
    stGAPService.pstChars[0].u8CharProperties = (uint8_t)(LE_READ);                     /* Characteristic is Readable */
    stGAPService.pstChars[0].u8CharDescCnt = 0;                                         /* Only Char. Value Declaration is present and no descriptors */
    stGAPService.pstChars[0].pstCharDesc = NULL;                                        /* No descriptors are available */
    stGAPService.pstChars[0].pu16RetCharDescHandles = au16ElementHandles_dev_name;      /* Pointer to the array where handles of returned char elements shall be stored */

    /* Characteristic VALUE Declaration  */
    stGAPService.pstChars[0].stCharValDecl.u16AttValLen = u16DevNameLen;                /* Length of the actual value (device name) */
    stGAPService.pstChars[0].stCharValDecl.pu8AttVal = pu8DevName;                      /* Pointer to array containing actual value (device name) */
    stGAPService.pstChars[0].stCharValDecl.u16AttPermission = (uint16_t)LE_GATT_READ;   /* Attribute Permission set to readable */


    /*****************************************
           Characteristic APPEARANCE
     *****************************************/
    stGAPService.pstChars[1].u16CharUUID      = (uint16_t)APPEARANCE;
    stGAPService.pstChars[1].u8CharProperties = (uint8_t)(LE_READ);
    stGAPService.pstChars[1].u8CharDescCnt = 0;
    stGAPService.pstChars[1].pstCharDesc = NULL;
    stGAPService.pstChars[1].pu16RetCharDescHandles = au16ElementHandles_appear;

    /* Characteristic VALUE Declaration */
    stGAPService.pstChars[1].stCharValDecl.u16AttValLen = 2;
    stGAPService.pstChars[1].stCharValDecl.pu8AttVal = au8AppearData;
    stGAPService.pstChars[1].stCharValDecl.u16AttPermission = (uint16_t)LE_GATT_READ;

    /* Register as Primary Service */
    eResTemp = eBleSrv_RegisterPrimaryService(qHandle, &stGAPService);

    return eResTemp;
}

/******************************************************************************/
le_api_result_e eMPAddDevIdentService(void* qHandle,
                                       uint8_t* pu8ManufName,
                                       uint8_t pu8ManufNameLen,
                                       uint16_t* pu16SvcHandle,
                                       uint16_t* pu16RetHandleCharDecl,
                                       uint16_t* pu16RetHandleCharValDecl)
{
  le_api_result_e eResTemp;
  le_srv_char_st stChar;
  uint16_t au16ElementHandles[1];


  /********************************************
   ********************************************
           Add Device Information service
   ********************************************
   ********************************************/
   stDevIdService.u16SvcUUID = GATT_DEV_INFO;                                        /* GATT Device information service UUID */
   stDevIdService.u8InclDefCount =0;                                                 /* No included services */
   stDevIdService.u8CharDefCount =1;                                                 /* One characteristic belong to this service */
   stDevIdService.pstIncludes = NULL;                                                /* No included services */
   stDevIdService.pstChars = &stChar;                                                /* Pointer to structure that describes characteristics. */

  /********************************************
       Register Manuf. String characteristic
   *********************************************/

  /* Characteristic Declaration */
  stDevIdService.pstChars->u16CharUUID      = (uint16_t)GATT_CHAR_MANUF_NAME_STRING; /* Characteristic UUID */
  stDevIdService.pstChars->u8CharProperties = (uint8_t)(LE_READ);                    /* Read flag is set */
  stDevIdService.pstChars->u8CharDescCnt = 0;                                        /* Number of descriptors */
  stDevIdService.pstChars->pstCharDesc = NULL;                                       /* No character descriptors */
  stDevIdService.pstChars->pu16RetCharDescHandles = au16ElementHandles;              /* Pointer to the array where handles of returned char elements shall be stored */

  /* Characteristic VALUE Declaration */
  stDevIdService.pstChars->stCharValDecl.u16AttValLen = pu8ManufNameLen;             /* Length of the manufacturer name (in bytes) */
  stDevIdService.pstChars->stCharValDecl.pu8AttVal = pu8ManufName;                   /* Manufacturer name - string array */
  stDevIdService.pstChars->stCharValDecl.u16AttPermission = (uint16_t)LE_GATT_READ;  /* Attribute permissions */

   /* Register as Primary Service */
  eResTemp = eBleSrv_RegisterPrimaryService(qHandle, &stDevIdService);


  return eResTemp;
}

/******************************************************************************/
le_api_result_e eMPAdd_controller_Service(void* qHandle, uint16_t* pu16SvcHandle,
                                     uint16_t* pu16RetHandleCharDecl,
                                     uint16_t* pu16RetHandleCharValDecl)
{
  le_api_result_e eResTemp;
  char_el_req_st astCharElement[2];
  uint16_t au16ElementHandles[2];
  static uint8_t au8MPValue[2] = {0x00,0x00}; /* Default  Value */
  uint8_t auCliProperties[2] = {0x00,0x00}; /*  Value */


  /*******************************************
  ********************************************
           Add  controller Service
  ********************************************
  *******************************************/
  stMyService.u16SvcUUID = GATT_CONTROLLER_UUID;    /* GATT service   UUID */
  stMyService.u8InclDefCount =0;                                                        /* No included services */
  stMyService.pstIncludes = NULL;                                                       /* No included services */
  stMyService.u8CharDefCount =1;                                                        /* One characteristic belong to this service */
  stMyService.pstChars = &stMyCharacteristic;                                                    /* Pointer to structure that describes characteristics. */

  /********************************************
       Register controller. Characteristic
  ********************************************/

  /* Characteristic Declaration */
  stMyCharacteristic.u16CharUUID      = (uint16_t)GATT_CONTROLLER_DATA_UUID;                   /* Service Characteristic UUID */

  stMyCharacteristic.u8CharProperties = (uint8_t)(LE_WRITE | LE_READ|LE_NOTIFY);                                    /* Character can be notified */
  stMyCharacteristic.u8CharDescCnt = 1;                                                          /* Number of character descriptors */
  stMyCharacteristic.pstCharDesc = astCharElement;                                               /* Pointer to the array containing char descriptors */
  stMyCharacteristic.pu16RetCharDescHandles = au16ElementHandles;                                /* Pointer to the array where handles
                                                                                          of returned char descriptors shall be stored */
  /* Characteristic VALUE Declaration */
  stMyService.pstChars->stCharValDecl.u16AttValLen = sizeof(my_data);
  stMyService.pstChars->stCharValDecl.pu8AttVal = &my_data;
  stMyService.pstChars->stCharValDecl.u16AttPermission = (uint16_t)LE_GATT_READ | LE_GATT_WRITE|LE_WRITABLE|LE_REDABLE;

  /* Characteristic DESCRIPTOR Declaration (Client Characteristic Configuration) */
  stMyService.pstChars->pstCharDesc->au8AttType[0] = GET_BYTE_LSB((uint16_t)GATT_CLI_CHARACT_CONFIGURATION);
  stMyService.pstChars->pstCharDesc->au8AttType[1] = GET_BYTE_MSB((uint16_t)GATT_CLI_CHARACT_CONFIGURATION);
  stMyService.pstChars->pstCharDesc->eAttrLen = LE_SHORT_UUID;
  stMyService.pstChars->pstCharDesc->u16AttPermission = (uint16_t)(LE_GATT_READ | LE_GATT_WRITE);
  stMyService.pstChars->pstCharDesc->u16AttValLen = sizeof(auCliProperties);
  stMyService.pstChars->pstCharDesc->pu8AttVal = auCliProperties;

  /* Register as Primary Service */
  eResTemp = eBleSrv_RegisterPrimaryService(qHandle, &stMyService);

  /* Return Characteristic declaration handle */
  *pu16RetHandleCharValDecl = stMyCharacteristic.u16RetCharValDeclHandle;

  return eResTemp;

}

/******************************************************************************
*
*                               ADVERTIZING
*
******************************************************************************/

le_api_result_e eMPStartAdvertising(void* qHandle)
{
  le_api_result_e eResult;
  le_adv_req_st stConfig;

  /* Advertizing configuration */
  stConfig.u16AdvIntervalMin = 0x0040;
  stConfig.u16AdvIntervalMax = 0x0040;
  stConfig.eAdvType = LE_ADV_IND;
  stConfig.eOwnAddrType    =LE_ADDR_PUBLIC;
  stConfig.eDirectAddrType =LE_ADDR_PUBLIC;
  stConfig.au8DirectAddr[0] = 0;
  stConfig.au8DirectAddr[1] = 0;
  stConfig.au8DirectAddr[2] = 0;
  stConfig.au8DirectAddr[3] = 0;
  stConfig.au8DirectAddr[4] = 0;
  stConfig.au8DirectAddr[5] = 0;
  stConfig.u8AdvChannelMap =0x07;
  stConfig.eAdvFilterPolicy = LE_SCAN_ANY_CONN_ANY;



  stConfig.u8AdvDataLen = 0x1f;

  stConfig.au8AdvData[0] = 0x08;                                                        /* Length in bytes */
  stConfig.au8AdvData[1]  =  LE_ADV_COMPLETE_NAME;                          /* AD Type: Complete device name */
  stConfig.au8AdvData[2] = 'R';
  stConfig.au8AdvData[3] = 'e';
  stConfig.au8AdvData[4] = 'm';
  stConfig.au8AdvData[5] = 'o';
  stConfig.au8AdvData[6] = 't';
  stConfig.au8AdvData[7] = 'e';
  stConfig.au8AdvData[8] = '1';

  stConfig.au8AdvData[9] = 21;  /*data length*/

  stConfig.au8AdvData[10] = LE_ADV_MANUF_SPECIFIC_DATA;
  stConfig.au8AdvData[11] = '0';
  stConfig.au8AdvData[12] = '1';
  stConfig.au8AdvData[13] = '2';
  stConfig.au8AdvData[14] = '3';
  stConfig.au8AdvData[15] = '4';
  stConfig.au8AdvData[16] = '5';
  stConfig.au8AdvData[17] = '6';
  stConfig.au8AdvData[18] = '7';
  stConfig.au8AdvData[19] = '8';
  stConfig.au8AdvData[20] = '9';
  stConfig.au8AdvData[21] = 'A';
  stConfig.au8AdvData[22] = 'B';
  stConfig.au8AdvData[23] = 'C';
  stConfig.au8AdvData[24] = 'D';
  stConfig.au8AdvData[25] = 'E';
  stConfig.au8AdvData[26] = 'F';
  stConfig.au8AdvData[27] = 'G';
  stConfig.au8AdvData[28] = 'H';
  stConfig.au8AdvData[29] = 'I';
  stConfig.au8AdvData[30] = 'J';

  stConfig.u8ScanRespDataLen = 0x1f;

  stConfig.au8ScanRespData[0]= 0x02;
  stConfig.au8ScanRespData[1]= 0x01;
  stConfig.au8ScanRespData[2] = 0x00;
  stConfig.au8ScanRespData[3] = 0x00;
  stConfig.au8ScanRespData[4] = 0x00;
  stConfig.au8ScanRespData[5] = 0x00;
  stConfig.au8ScanRespData[6] = 0x00;
  stConfig.au8ScanRespData[7] = 0x00;
  stConfig.au8ScanRespData[8] = 0x00;
  stConfig.au8ScanRespData[9] = 0x00;
  stConfig.au8ScanRespData[10] = 0x00;
  stConfig.au8ScanRespData[11] = 0x00;
  stConfig.au8ScanRespData[12] = 0x00;
  stConfig.au8ScanRespData[13] = 0x00;
  stConfig.au8ScanRespData[14] = 0x00;
  stConfig.au8ScanRespData[15] = 0x00;
  stConfig.au8ScanRespData[16] = 0x00;
  stConfig.au8ScanRespData[17] = 0x00;
  stConfig.au8ScanRespData[18] = 0x00;
  stConfig.au8ScanRespData[19] = 0x00;
  stConfig.au8ScanRespData[20] = 0x00;
  stConfig.au8ScanRespData[21] = 0x00;
  stConfig.au8ScanRespData[22] = 0x00;
  stConfig.au8ScanRespData[23] = 0x00;
  stConfig.au8ScanRespData[24] = 0x00;
  stConfig.au8ScanRespData[25] = 0x00;
  stConfig.au8ScanRespData[26] = 0x00;
  stConfig.au8ScanRespData[27] = 0x00;
  stConfig.au8ScanRespData[28] = 0x00;
  stConfig.au8ScanRespData[29] = 0x00;
  stConfig.au8ScanRespData[30] = 0x00;

  eResult = eBlePerif_StartAdv(qHandle, stConfig);

  return eResult;

}
/******************************************************************************
*
*                           CALLBACK FUNCTIONS
*
******************************************************************************/
/******************************************************************************/
void vPrintUnexpectedCMD(uint8_t u8ServiceID,
                         uint8_t u8CmdCode,
                         uint16_t u16paramLen,
                         uint8_t* pu8ParamData)
{
#if 0
  printf(" ---------- MP PROFILE UNEXPECTED COMMAND RECEIVED - START -------------\n");
  printf("| ServiceID: 0x%.2x \n", u8ServiceID);
  printf("| u8CmdCode: 0x%.2x \n", u8CmdCode);
  printf("| ParamLen: 0x%.2x \n", u16paramLen);
  printf(" ---------- MP PROFILE UNEXPECTED COMMAND RECEIVED - END -------------\n");
#endif
}

/******************************************************************************/
le_api_result_e eUpdateCharValue(void* qHandle,
                                      uint16_t u16ConnHandle,
                                      uint16_t u16CharValHandle)
{

  uint16_t length = 0;

#if 0
  printf("Characteristic VALUE Update Callback Function!\n");
  printf("| u16ConnHandle: 0x%.2x \n", u16ConnHandle);
  printf("| u16CharValHandle: 0x%.2x \n", u16CharValHandle);
#endif
  /*all we have is the "handle of the variable we want to udpate.  We need to match this to the local variable in our system.*/
  if(u16CharValHandle == stMyCharacteristic.u16RetCharValDeclHandle)
  {
      eBleSrv_UpdateCharacteristic(qHandle,   u16CharValHandle,  stMyCharacteristic.stCharValDecl.u16AttValLen,  stMyCharacteristic.stCharValDecl.pu8AttVal);
  }
  else
  {
      /*dont know what they want to update*/
  }




  // eBleSrv_UpdateCharacteristic(qHandle, pstEvtData->u16CharValHandle, pstEvtData->u16ValDataLen, pstEvtData->pu8CharValData);


  return LE_API_SUCCCESS;
}

/******************************************************************************/
le_api_result_e eUpdateCharDescriptor(void* qHandle,
                                      uint16_t u16ConnHandle,
                                      uint16_t u16CharValHandle)
{
 // printf("Characteristic Descriptor Callback Function!\n");

  return LE_API_SUCCCESS;
}
/******************************************************************************/
le_api_result_e eUpdateMulti(void* qHandle,
                               uint16_t u16ConnHandle,
                               uint16_t u16HandleCnt,  uint8_t* pu8Handles)
{
  printf("Character Update Multi - Update Count: %d\n", u16HandleCnt);

  return LE_API_SUCCCESS;
}



/******************************************************************************
*
*                        INIT./ MAIN EVENT HANDLER
*
******************************************************************************/
/******************************************************************************/
le_api_result_e eMPInitialise(void* qHandle, hr_init_st* stInit)
{
  le_api_result_e eResult;
  uint8_t auMyBd_Addr[6];

  /* Initialise SMART API LAYER & register callback functions */
  eBleApiInit(btQueueEVENT,
                vPrintUnexpectedCMD,
                eUpdateCharValue,
                eUpdateCharDescriptor,
                eUpdateMulti);

  /* Initialise Device as GATT Server */
  eResult = eBleDeviceSrv_Init(btQueueEVENT,
                                 stInit->pu8DevName,
                                 stInit->u8DevNameLen,
                                 auMyBd_Addr);




  if(eResult == LE_API_SUCCCESS)
  {
    /* Add GAP Device Name Service */
    eResult =  eMPAddGAPDevName_Appearance_Service(qHandle,
                                                   stInit->pu16RetDevNameSvcHndl,
                                                   stInit->u8DevNameLen,
                                                   stInit->pu8DevName);

    if(eResult == LE_API_SUCCCESS)
    {
        /* Add GATT Device Identification Service */
        eResult = eMPAddDevIdentService(qHandle,
                                        stInit->pu8ManufName,
                                        stInit->pu8ManufNameLen,
                                        stInit->pu16RetDISvcHndl,
                                        stInit->pu16RetDICharDeclHndl,
                                        stInit->pu16RetDICharValDeclHndl);

        if(eResult == LE_API_SUCCCESS)
      {
        /* Add GATT Heart Rate Service */
        eResult = eMPAdd_controller_Service(qHandle,
                                         stInit->pu16RetMPSvcHndl,
                                         stInit->pu16RetMPCharDeclHndl,
                                         stInit->pu16RetMPCharValDeclHndl);
      }
    }
  }
  return eResult;
}


uint8_t CONNECTED = 0; /*if connected to a BLE client*/

/****************************************************************************/
void MPMainRoutine(void* qHandle, hr_init_st* stInit)
{
  uint16_t u16Command;
  le_api_result_e eResult;
  uint16_t u16RetConnHandle;

  static uint32_t delay = 0xFFFFF;  /*how many ms is this?*/

  enum serviceID_opcodes{   TCU_MNG_LE_CONNECTION_COMPLETE_EVENT_SID_OC = 0xD14C,  /*taken from le_gatt_command.h*/
                                                     TCU_LE_GATT_SER_EXG_MTU_EVENT_SID_OC = 0xD3C1,
                                                     TCU_LE_GATT_SER_READ_MULTIPLE_EVENT_SID_OC = 0xD3CA,
                                                     TCU_LE_GATT_SER_READ_CHAR_VAL_EVENT_SID_OC = 0xD3C2,
                                                     TCU_LE_GATT_SER_READ_CHAR_DESP_EVENT_SID_OC = 0xD3C8,
                                                     TCU_LE_GATT_SER_WRITE_CHAR_DESP_EVENT_SID_OC = 0xD3C4,
                                                     TCU_LE_GATT_SER_WRITE_CHAR_VAL_EVENT_SID_OC = 0xD3C3,
                                                     TCU_MNG_LE_DISCONNECT_EVENT_SID_OP = 0xD193,



  };

  extern uint8_t DATA_FROM_BLUETOOTH_UART;

  /* Main MP-Profile Event Handler */
 // while(1)
  //{
    /* At this stage initialization is done and we are waiting on events
    *  Comming from a client. */

    /* Wait for a message to be received (with time-out) */
   // eResult = vSmartWaitForAnyEvent(qHandle, &u16Command);

    if(DATA_FROM_BLUETOOTH_UART == 1)
   // if (LE_API_SUCCCESS == eResult)
    {
        DATA_FROM_BLUETOOTH_UART = 0;

      /*   Message Handler.
       *
       *  Based on message type, the handler calls an appropriate routine
       *  implementing adequate handling.
       */

        uint8_t* pu8Data = (uint8_t*)(tcu_event.ptEvent);
        u16Command = PARSE_CMD_MESSAGE(pu8Data);
        uint16_t connect_delay = 0xfffff;
      switch(u16Command)
      {
        //case GET_CMD_MESSAGE(TCU_MNG_LE_CONNECTION_COMPLETE_EVENT):  /*omg who wrote this*/
          case TCU_MNG_LE_CONNECTION_COMPLETE_EVENT_SID_OC:
          {
            /* Client has sent a Connection Request to the Server,
               therefore a connection handler will be called */
             eResult = eBleSrv_ClientConnRequestHndlr(stInit->pstConnectInfo);
            while(connect_delay--){}
             CONNECTED = 1;
          }
        break;
          case  TCU_MNG_LE_DISCONNECT_EVENT_SID_OP:
              eMPStartAdvertising(qHandle);

        break;
       // case GET_CMD_MESSAGE(TCU_LE_GATT_SER_EXG_MTU_EVENT):
          case TCU_LE_GATT_SER_EXG_MTU_EVENT_SID_OC:
          {
            /*  MTU size negotiation started, so start an MTU Exchange
             *  handler.
             */
             eResult = eBleSrv_ClientMtuExchangeHndlr(qHandle,
                                              stInit->pstConnectInfo->u16ConnHandle,
                                              stInit->pstMtuEvtData,
                                              stInit->pstMtuAccResp);
          }
        break;
        //case GET_CMD_MESSAGE(TCU_LE_GATT_SER_READ_MULTIPLE_EVENT):
          case TCU_LE_GATT_SER_READ_MULTIPLE_EVENT_SID_OC:
          {
            /* Update of multiple values in data-base requested, before
            *  sending those data to client side  */
            eResult = eBleSrv_ReadMultiEventHndlr(qHandle,
                                                    CONN_HANDLE_ANY,
                                                    &(stInit->u16ConnHandle));
          }
        break;
        //case GET_CMD_MESSAGE(TCU_LE_GATT_SER_READ_CHAR_VAL_EVENT):
          case TCU_LE_GATT_SER_READ_CHAR_VAL_EVENT_SID_OC:
          {
            /* Read characteristic VALUE request received */
            eResult = eBleSrv_ReadCharValHndlr(qHandle,
                                                 /*stInit->u16ConnHandle,*/
                                                 stInit->pstConnectInfo->u16ConnHandle,
                                                 &u16RetConnHandle);
          }
        break;
        //case GET_CMD_MESSAGE(TCU_LE_GATT_SER_READ_CHAR_DESP_EVENT):
          case TCU_LE_GATT_SER_READ_CHAR_DESP_EVENT_SID_OC:
          {
            /* Read characteristic DESCRIPTOR request received */
            eResult = eBleSrv_ReadCharDescriptorHndlr(qHandle,
                                                          /* stInit->u16ConnHandle,*/
                                                          stInit->pstConnectInfo->u16ConnHandle,
                                                          &u16RetConnHandle);
          }
        break;


        //case GET_CMD_MESSAGE(TCU_LE_GATT_SER_WRITE_CHAR_DESP_EVENT):
          case TCU_LE_GATT_SER_WRITE_CHAR_DESP_EVENT_SID_OC:
          {
              /* Write characteristic DESCRIPTOR request received */
              eResult = eBleSrv_WriteCharDescriptorHndlr(qHandle,
                                                          /* stInit->u16ConnHandle, */
                                                          stInit->pstConnectInfo->u16ConnHandle,
                                                          &stInit->stCharDespEvt);

              if(eResult==LE_API_SUCCCESS)
              {
                 NOTIFY_CLIENT=1;
              }
          }
        break;

          case TCU_LE_GATT_SER_WRITE_CHAR_VAL_EVENT_SID_OC:
          {
#if 0
              typedef struct tag_le_srv_write_char_desp_event_st
              {
                uint16_t u16ConnHandle;
                uint16_t u16CharDescHandle;
                uint16_t u16CharValueLen;
                uint8_t*  pu8CharDescValue;
              } le_srv_write_char_desp_event_st;
              test
#endif
              le_srv_write_char_desp_event_st       characteristic_to_write;
              uint8_t data_buffer[50] = {0x00};  /*this is used for storing the data from the bluetooth module*/
              characteristic_to_write.pu8CharDescValue = data_buffer;   /*point the buffer to this buffer we have just created*/

              /*this takes the data received from the bluetooth, parses it, and udpates the characteristic value in the bluetooth.  It then returns the characteristic information that was modified.*/
              eResult = eBleSrv_WriteCharValueHndlr(qHandle,   stInit->pstConnectInfo->u16ConnHandle,   &characteristic_to_write);

              if(characteristic_to_write.u16CharDescHandle == stMyCharacteristic.u16RetCharValDeclHandle)  /*match the hanlde of the written characteristic with the handle of the locally stored variable before updating*/
              {
                 /*update our local variable with the value*/
                  memcpy(stMyCharacteristic.stCharValDecl.pu8AttVal, characteristic_to_write.pu8CharDescValue, characteristic_to_write.u16CharValueLen);
                  //memcpy(stMyCharacteristic.stCharValDecl.pu8AttVal, data_buffer, characteristic_to_write.u16CharValueLen);  this should do the same thing
              }
              else
              {
                  /*unknown local varibale to write*/
              }



          }
              break;

      default:
        {

         // printf("MP Profile: Unhandled Message received() \n");
          //printf("| u16Command: 0x%.4x \n", u16Command);


        }
      }
  }
    else
    {
        /*no bluetooth event from uart*/
    }

//#if 0
   // if(delay == 0)
   // {


      //  if(NOTIFY_CLIENT)
    //   {
            //eBleSrv_SendNotification(qHandle,   stInit->pstConnectInfo->u16ConnHandle,   stMyService.pstChars,   my_data);

        //   transmit_bluetooth_packet(my_data);
       //    NOTIFY_CLIENT  = 0;

      //  }
     //   else
       // {
            /*not connected cant send to anyone*/
    //    }


   // else
  //  {
       // delay--;
 //   }

      //#endif
  // if(TIME_TO_SEND == 1)
   //{
      //     transmit_bluetooth_packet(my_data);
   //}

 // }  /*while 1*/

}


/*transmit our 20 byte characteristic*/
uint8_t transmit_bluetooth_packet(uint8_t data_packet[])
{
    static uint8_t TRANSMITTING = 0;
    uint8_t result = 0;


    if(TRANSMITTING == 0)
    {
        TRANSMITTING = 1;
       void*qHandle;  /*used for freetos, unused for us*/
        uint8_t result = 0;
        le_api_result_e eResTemp;

        extern le_srv_service_def_st stMyService;
       extern le_conn_evt_st stConnectInfo;



       if( (CONNECTED == 1) && (NOTIFY_CLIENT == 1) )
       {
         eResTemp = eBleSrv_SendNotification(qHandle,   stConnectInfo.u16ConnHandle  ,    stMyService.pstChars,   data_packet);

          if (eResTemp == LE_API_SUCCCESS)
          {
              result = 1;
          }
          else
          {
              result = 0;
          }
          TRANSMITTING = 0;
       }
       else
       {
           result = 0;
           TRANSMITTING = 0;
       }
    }
    else /* still transmitting*/
    {
        result = 0;
    }


          return result;
}




