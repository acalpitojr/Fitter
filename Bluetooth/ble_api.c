/****************************************************************************
*   COPYRIGHT 2006-2009 All rights reserved
*   Toshiba Electronics Europe GmbH
*   European LSI Design and Engineering Center (ELDEC)
*****************************************************************************
*   File Name   : $RCSfile: booting.h,v $
*   Description : Contains global booting definitions
*****************************************************************************
*   PROJECT     : Bluetooth BLE
*   MODULE      : Smart API
*               : $Name:  $
*   LIBRARIES   : None
*   AUTHOR      : $Author: arelic $
*****************************************************************************
*   VERSION     : $$
*   RELEASE     : Preliminary & Confidential
*   DATE        : $Date: 2012-08-03 13:26:26 $
*   SOURCE MOD. : $Source: /teenet/mpumcu/Capricorn/Capricorn_Bt/ipl/inc/booting.h,v $
*   LOG:        : please have a look at the end of the file ddd
****************************************************************************/

/**********************************************
*                Include files                *
**********************************************/
#include "app_bt.h"
#include "le_gatt_command.h"
#include "bt_uart_transport.h"
#include "ble_api.h"
#include "bt_OS.h"
#include "systick_counter.h"
#include <stdio.h>

//#ifdef FreeRTOS
  //#include "FreeRTOS.h"
  //#include "queue.h"
  //#include "task.h"
//#endif


/**********************************************
*               Defs                          *
**********************************************/
//#define DEBUG_PRINT


/**********************************************
*                  Global                     *
**********************************************/
/* Pointer to a callback function called if unexpected event is received*/
void (*ptUnexEvtCallBack)(uint8_t, uint8_t,uint16_t, uint8_t*) = NULL;

/* Pointer to a callback function called when Character Value needs to be updated */
le_api_result_e (*pCharValUpdCallback)(void*, uint16_t, uint16_t);


/* Pointer to a callback function called when Character Descriptor needs to be updated */
le_api_result_e (*pCharDescUpdCallback)(void*, uint16_t, uint16_t);

/* Pointer to a callback function called when multiple handle values need to be updated */
le_api_result_e (*pUpdMultiCallback)(void*, uint16_t, uint16_t, uint8_t*);


//t_tcu_event stTcuEvent;
extern volatile t_tcu_event tcu_event;

extern const tcu_packet_STRUCT   TCU_LE_ACCEPT_t;
extern const tcu_packet_STRUCT   TCU_LE_NOT_ACCEPT_t;
extern const tcu_packet_STRUCT   TCU_LE_FATAL_ERROR_t;
extern const tcu_packet_STRUCT   TCU_MNG_LE_INIT_REQ_t ;
extern const tcu_packet_STRUCT  TCU_MNG_LE_INIT_RESP_t ;
extern const tcu_packet_STRUCT   TCU_MNG_LE_SET_SCAN_ENABLE_REQ_t ;
extern const tcu_packet_STRUCT   TCU_MNG_LE_SET_SCAN_ENABLE_RESP_t ;
extern const tcu_packet_STRUCT   TCU_MNG_LE_ADV_REPORT_EVENT_t ;
extern const tcu_packet_STRUCT   TCU_MNG_LE_CREATE_CONNECTION_REQ_t ;
extern const tcu_packet_STRUCT  TCU_MNG_LE_CONNECTION_COMPLETE_EVENT_t ;
extern const tcu_packet_STRUCT   TCU_MNG_LE_CREATE_CONNECTION_CANCEL_REQ_t ;
extern const      tcu_packet_STRUCT   TCU_MNG_LE_CREATE_CONNECTION_CANCEL_RESP_t;
extern const    tcu_packet_STRUCT   TCU_MNG_LE_DISCONNECT_REQ_t ;
extern  const    tcu_packet_STRUCT  TCU_MNG_LE_DISCONNECT_EVENT_t ;
extern   const tcu_packet_STRUCT   TCU_MNG_LE_SET_SCAN_DISABLE_REQ_t ;
extern const  tcu_packet_STRUCT  TCU_MNG_LE_SET_SCAN_DISABLE_RESP_t ;
extern const   tcu_packet_STRUCT  TCU_MNG_LE_START_ADVERTISE_REQ_t ;
extern const  tcu_packet_STRUCT   TCU_MNG_LE_START_ADVERTISE_RESP_t ;
extern const  tcu_packet_STRUCT   TCU_MNG_LE_DISABLE_ADVERTISE_REQ_t ;
extern const   tcu_packet_STRUCT   TCU_MNG_LE_DISABLE_ADVERTISE_RESP_t ;
extern const     tcu_packet_STRUCT   TCU_LE_GATT_SDB_ADD_PRIM_SVC_REQ_t ;
extern const     tcu_packet_STRUCT   TCU_LE_GATT_SDB_ADD_PRIM_SVC_RESP_t ;
extern const     tcu_packet_STRUCT   TCU_LE_GATT_SDB_ADD_SEC_SVC_REQ_t ;
extern const   tcu_packet_STRUCT   TCU_LE_GATT_SDB_ADD_SEC_SVC_RESP_t ;
extern const     tcu_packet_STRUCT   TCU_LE_GATT_SDB_ADD_CHAR_DECL_REQ_t ;
extern const      tcu_packet_STRUCT   TCU_LE_GATT_SDB_ADD_CHAR_DECL_RESP_t ;
extern const      tcu_packet_STRUCT   TCU_LE_GATT_SDB_ADD_CHAR_ELE_REQ_t ;
extern const       tcu_packet_STRUCT   TCU_LE_GATT_SDB_ADD_CHAR_ELE_RESP_t ;
extern const    tcu_packet_STRUCT   TCU_LE_GATT_SDB_UPD_CHAR_ELE_REQ_t ;
extern const     tcu_packet_STRUCT  TCU_LE_GATT_SDB_UPD_CHAR_ELE_RESP_t ;
extern const     tcu_packet_STRUCT   TCU_LE_GATT_SDB_ADD_INC_SVC_REQ_t ;
extern const      tcu_packet_STRUCT   TCU_LE_GATT_SDB_ADD_INC_SVC_RESP_t ;
extern const      tcu_packet_STRUCT   TCU_LE_GATT_SDB_RET_END_GRP_HLE_REQ_t ;
extern const      tcu_packet_STRUCT   TCU_LE_GATT_SDB_RET_END_GRP_HLE_RESP_t ;
extern const     tcu_packet_STRUCT   TCU_LE_GATT_SER_INIT_REQ_t ;
extern const    tcu_packet_STRUCT   TCU_LE_GATT_SER_INIT_RESP_t ;
extern    const     tcu_packet_STRUCT   TCU_LE_GATT_SER_WRITE_CHAR_DESP_EVENT_t ;
extern const    tcu_packet_STRUCT   TCU_LE_GATT_SER_WRITE_CHAR_DESP_ACCEPT_REQ_t ;
extern   const       tcu_packet_STRUCT   TCU_LE_GATT_SER_WRITE_CHAR_DESP_ACCEPT_RESP_t ;
extern const     tcu_packet_STRUCT   TCU_LE_GATT_SER_READ_CHAR_VAL_ACCEPT_REQ_t ;
extern const      tcu_packet_STRUCT   TCU_LE_GATT_SER_READ_CHAR_VAL_ACCEPT_RESP_t ;
extern const     tcu_packet_STRUCT   TCU_LE_GATT_CLI_EXG_MTU_REQ_t ;
extern const    tcu_packet_STRUCT   TCU_LE_GATT_CLI_EXG_MTU_EVENT_t ;
extern const      tcu_packet_STRUCT  TCU_LE_GATT_CLI_DISCOVER_PRIM_SVC_BY_UUID_REQ_t ;
extern const     tcu_packet_STRUCT   TCU_LE_GATT_CLI_DISCOVER_PRIM_SVC_BY_UUID_EVENT_t ;
extern const    tcu_packet_STRUCT   TCU_LE_GATT_CLI_DISCOVER_CHAR_DECL_REQ_t ;
extern  const     tcu_packet_STRUCT  TCU_LE_GATT_CLI_DISCOVER_CHAR_DECL_EVENT_t ;
extern  const   tcu_packet_STRUCT   TCU_LE_GATT_CLI_READ_CHAR_VAL_REQ_t ;
extern const   tcu_packet_STRUCT   TCU_LE_GATT_CLI_READ_CHAR_VAL_EVENT_t ;
extern  const   tcu_packet_STRUCT   TCU_LE_GATT_CLI_CHAR_VAL_NOTIFICATION_IND_EVENT_t ;
extern const    tcu_packet_STRUCT   TCU_LE_GATT_SER_EXG_MTU_EVENT_t ;
extern const    tcu_packet_STRUCT   TCU_LE_GATT_SER_EXG_MTU_ACCEPT_REQ_t ;
extern const     tcu_packet_STRUCT   TCU_LE_GATT_SER_EXG_MTU_ACCEPT_RESP_t ;
extern const    tcu_packet_STRUCT   TCU_LE_GATT_SER_READ_CHAR_VAL_EVENT_t ;
extern const      tcu_packet_STRUCT   TCU_LE_GATT_SER_WRITE_CHAR_VAL_EVENT_t ;
extern const    tcu_packet_STRUCT   TCU_LE_GATT_SER_WRITE_CHAR_VAL_ACCEPT_REQ_t ;
extern const    tcu_packet_STRUCT   TCU_LE_GATT_SER_WRITE_CHAR_VAL_ACCEPT_RESP_t ;
extern const     tcu_packet_STRUCT   TCU_LE_GATT_SER_READ_CHAR_DESP_EVENT_t ;
extern const    tcu_packet_STRUCT   TCU_LE_GATT_SER_READ_CHAR_DESP_ACCEPT_REQ_t ;
extern const     tcu_packet_STRUCT   TCU_LE_GATT_SER_READ_CHAR_DESP_ACCEPT_RESP_t ;
extern const    tcu_packet_STRUCT   TCU_LE_GATT_SER_READ_MULTIPLE_EVENT_t ;
extern const    tcu_packet_STRUCT   TCU_LE_GATT_SER_READ_MULTIPLE_ACCEPT_REQ_t ;
extern const      tcu_packet_STRUCT   TCU_LE_GATT_SER_READ_MULTIPLE_ACCEPT_RESP_t ;
extern  const     tcu_packet_STRUCT   TCU_LE_GATT_SER_CHAR_VAL_NOTIFICATION_REQ_t ;
extern const    tcu_packet_STRUCT   TCU_LE_GATT_SER_CHAR_VAL_NOTIFICATION_EVENT_t ;
extern const     tcu_packet_STRUCT   TCU_LE_GATT_SER_CHAR_VAL_INDICATION_REQ_t ;
extern  const     tcu_packet_STRUCT   TCU_LE_GATT_SER_CHAR_VAL_INDICATION_EVENT_t ;
extern  const   tcu_packet_STRUCT  TCU_LE_GATT_SER_WRITE_WITHOUT_RESPONSE_CMD_EVENT_t ;
extern const    tcu_packet_STRUCT   TCU_LE_GATT_CLI_INIT_RESP_t ;
extern const    tcu_packet_STRUCT   TCU_LE_GATT_CLI_DISCOVER_CHAR_DESP_EVENT_t ;
extern const    tcu_packet_STRUCT   TCU_LE_GATT_CLI_DISCOVER_CHAR_DESP_REQ_t ;
/**********************************************
*                Functions                    *
**********************************************/
le_api_result_e eBleWaitForSpecificEvent(void* qHandle, tcu_packet_STRUCT  ExpEvent, uint32_t u32TimeOut, void* pvRetData);
/******************************************************************************/
void vGetCommandCodeFromInputStream(uint8_t* pu8Buff, uint8_t* pu8SvcId, uint8_t* puCmdCode)
{
  *pu8SvcId = PARSE_CMD_SERVICE_ID(pu8Buff);
  *puCmdCode = PARSE_CMD_OPCODE(pu8Buff);
}

/******************************************************************************/
le_api_result_e  eBleRegisterCallback_UnexpEvnt(void (*pCallback)(uint8_t, uint8_t,uint16_t, uint8_t*))
 {
   le_api_result_e eRet = LE_API_SUCCCESS;

    if(pCallback !=NULL)
    {
      ptUnexEvtCallBack =  pCallback;
    }
    else
    {
      eRet = LE_NULL_PTR_ERR;
    }

   return eRet;
 }

/******************************************************************************/
le_api_result_e  eBleRegisterCallback_CharValUpdate(le_api_result_e (*pCallback)(void*, uint16_t, uint16_t))
 {
   le_api_result_e eRet = LE_API_SUCCCESS;

    if(pCallback !=NULL)
    {
      pCharValUpdCallback =  pCallback;
    }
    else
    {
      eRet = LE_NULL_PTR_ERR;
    }

   return eRet;
 }

/******************************************************************************/
le_api_result_e  eBleRegisterCallback_CharDescUpdate(le_api_result_e (*pCallback)(void*, uint16_t, uint16_t))
 {
   le_api_result_e eRet = LE_API_SUCCCESS;

    if(pCallback !=NULL)
    {
      pCharDescUpdCallback =  pCallback;
    }
    else
    {
      eRet = LE_NULL_PTR_ERR;
    }

   return eRet;
 }


/******************************************************************************/

le_api_result_e  eBleRegisterCallback_MultiUpdate(le_api_result_e (*pCallback)(void*, uint16_t, uint16_t, uint8_t*))
 {
   le_api_result_e eRet = LE_API_SUCCCESS;

    if(pCallback !=NULL)
    {
      pUpdMultiCallback =  pCallback;
    }
    else
    {
      eRet = LE_NULL_PTR_ERR;
    }

   return eRet;
 }

/******************************************************************************/
le_api_result_e eBleApiInit(void* qHandle,
                               void (*pUnxEvtFunc)(uint8_t, uint8_t,uint16_t, uint8_t*),
                               le_api_result_e (*pCharValUpdFunc)(void*, uint16_t, uint16_t),
                               le_api_result_e (*pCharDescUpdFunc)(void*, uint16_t, uint16_t),
                               le_api_result_e (*pUpdMultiFunc)(void*, uint16_t, uint16_t, uint8_t*))
{
  le_api_result_e eRet = LE_API_SUCCCESS;

#ifdef DEBUG_PRINT
  printf(" eBleApiInit() \n");
#endif

  /* Register a callback function for unexpected event */
  eRet = eBleRegisterCallback_UnexpEvnt(pUnxEvtFunc);

  if(LE_API_SUCCCESS == eRet)
  {
      /* Register a callback function for characteristic VALUE update */
      eRet = eBleRegisterCallback_CharValUpdate(pCharValUpdFunc);

       if(LE_API_SUCCCESS == eRet)
       {
          /* Register a callback function for characteristic DESCRIPTOR update */
          eRet = eBleRegisterCallback_CharDescUpdate(pCharDescUpdFunc);

          if(LE_API_SUCCCESS == eRet)
          {
              /* Register callback for Multi-Update */
              eRet = eBleRegisterCallback_MultiUpdate(pUpdMultiFunc);
          }
       }


  }

  return eRet;
 }


/******************************************************************************/
le_api_result_e eBleExtractDataEventMng(uint8_t* pu8Buff, void* pvRetData)
{
    le_api_result_e eResult = LE_API_SUCCCESS;
    uint8_t u8OpCode = PARSE_CMD_OPCODE(pu8Buff);
    le_func_status_e eLeDrvRes;

    enum le_opcodes{  TCU_MNG_LE_INIT_RESP_OPCODE = 0x81,  /*found this through macro calls*/
        TCU_MNG_LE_START_ADVERTISE_RESP_OPCODE = 0x88,
        TCU_MNG_LE_DISABLE_ADVERTISE_RESP_OPCODE = 0x89,
        TCU_MNG_LE_CONNECTION_COMPLETE_EVENT_OPCODE = 0x4C,
        TCU_MNG_LE_DISCONNECT_EVENT_OPCODE = 0x93,
        TCU_LE_ACCEPT_OPCODE = 0xF1,
     };


    /* Process returned response/notification */
    switch(u8OpCode)
    {
        //case GET_CMD_OPCODE(TCU_MNG_LE_INIT_RESP):                 eLeDrvRes = eGetLeMngInit_Resp(pu8Buff, (le_mnginit_resp_st*)pvRetData);
        case TCU_MNG_LE_INIT_RESP_OPCODE:
            eLeDrvRes = eGetLeMngInit_Resp(pu8Buff, (le_mnginit_resp_st*)pvRetData);
        break;
        //case GET_CMD_OPCODE(TCU_MNG_LE_START_ADVERTISE_RESP):
        case TCU_MNG_LE_START_ADVERTISE_RESP_OPCODE:
                eLeDrvRes =  eGetLeStartAdvertise_Resp(pu8Buff, (uint8_t*)(pvRetData));
        break;
        //case GET_CMD_OPCODE(TCU_MNG_LE_DISABLE_ADVERTISE_RESP):
        case TCU_MNG_LE_DISABLE_ADVERTISE_RESP_OPCODE:
                eLeDrvRes =  eGetLeDisableAdvertise_Resp(pu8Buff, (uint8_t*)(pvRetData));
        break;
       // case GET_CMD_OPCODE(TCU_MNG_LE_CONNECTION_COMPLETE_EVENT):
        case TCU_MNG_LE_CONNECTION_COMPLETE_EVENT_OPCODE:
                eLeDrvRes = eGetLeMngConnComplete_Event(pu8Buff, (le_conn_evt_st*)(pvRetData));
        break;
       // case GET_CMD_OPCODE(TCU_MNG_LE_DISCONNECT_EVENT):
        case TCU_MNG_LE_DISCONNECT_EVENT_OPCODE:
                eLeDrvRes = eGetLeDisconnect_Event(pu8Buff, (le_disconnect_st*)(pvRetData));
        break;
        //case GET_CMD_OPCODE(TCU_LE_ACCEPT):
        case TCU_LE_ACCEPT_OPCODE:
                eLeDrvRes = eGetLeAccept_Ack(pu8Buff, (le_accept_st*)(pvRetData));
        break;
        default:
        {
            eResult = LE_API_ERR_PARSING;
        }
    }

    if((eResult == LE_API_SUCCCESS) && (eLeDrvRes != LE_FUNC_SUCCCESS))
    {
       eResult = LE_API_ERR_PARSING;
    }

    return eResult;
}

/******************************************************************************/
le_api_result_e eBleExtractDataEventGattSer(uint8_t* pu8Buff, void* pvRetData)
{
    le_api_result_e eResult = LE_API_SUCCCESS;
    uint8_t u8OpCode = PARSE_CMD_OPCODE(pu8Buff);
    le_func_status_e eLeDrvRes;


    enum gatt_opcodes{ TCU_LE_GATT_SER_INIT_RESP_OPCODE = 0x80,
        TCU_LE_GATT_SDB_ADD_PRIM_SVC_RESP_OPCODE = 0xA0,
        TCU_LE_GATT_SDB_ADD_CHAR_DECL_RESP_OPCODE = 0xA2,
        TCU_LE_GATT_SDB_ADD_CHAR_ELE_RESP_OPCODE = 0xA3,
        TCU_LE_GATT_SER_EXG_MTU_EVENT_OPCODE = 0xC1,
        TCU_LE_GATT_SER_EXG_MTU_ACCEPT_RESP_OPCODE = 0x81,
        TCU_LE_GATT_SER_READ_MULTIPLE_EVENT_OPCODE = 0xCA,
        TCU_LE_GATT_SER_READ_MULTIPLE_ACCEPT_RESP_OPCODE = 0x8A,
        TCU_LE_GATT_SER_READ_CHAR_VAL_EVENT_OPCODE = 0xC2,
        TCU_LE_GATT_SER_READ_CHAR_VAL_ACCEPT_RESP_OPCODE = 0x82,
        TCU_LE_GATT_SER_WRITE_CHAR_DESP_EVENT_OPCODE = 0xC4,
        TCU_LE_GATT_SDB_UPD_CHAR_ELE_RESP_OPCODE = 0xA5,
        TCU_LE_GATT_SER_WRITE_CHAR_DESP_ACCEPT_RESP_OPCODE = 0x84,
        TCU_LE_GATT_SER_WRITE_CHAR_VAL_ACCEPT_RESP_OPCODE = 0x83,
        TCU_LE_GATT_SER_CHAR_VAL_NOTIFICATION_EVENT_OPCODE = 0x45,
        TCU_LE_GATT_SDB_RET_END_GRP_HLE_RESP_OPCODE = 0xA6,
        TCU_LE_GATT_SDB_ADD_INC_SVC_RESP_OPCODE = 0xA4,
        TCU_LE_GATT_SER_CHAR_VAL_INDICATION_EVENT_OPCODE = 0x46,
        TCU_LE_GATT_SER_WRITE_WITHOUT_RESPONSE_CMD_EVENT_OPCODE = 0xC9,
        TCU_LE_GATT_SER_READ_CHAR_DESP_EVENT_OPCODE = 0xC8,
        TCU_LE_GATT_SER_READ_CHAR_DESP_ACCEPT_RESP_OPCODE = 0x88,
        TCU_LE_GATT_SER_WRITE_CHAR_VAL_EVENT_OPCODE = 0xC3,


    };


    /* Process returned response/notification */
    switch(u8OpCode)
    {
       // case GET_CMD_OPCODE(TCU_LE_GATT_SER_INIT_RESP):       /*We are changing this because their macro for get_cmd_opcode just uses bitshifts on a 32 bit number.  Since the mcu is only 16 bits, we are having a problem with this.*/
        case TCU_LE_GATT_SER_INIT_RESP_OPCODE:
                eLeDrvRes = eGetServerGattInit_Resp(pu8Buff, (gatt_status_code_e*)pvRetData);
        break;
        //case GET_CMD_OPCODE(TCU_LE_GATT_SDB_ADD_PRIM_SVC_RESP):
        case TCU_LE_GATT_SDB_ADD_PRIM_SVC_RESP_OPCODE:
                eLeDrvRes =  eGetSdbGattAddPrimaryService_Resp(pu8Buff, &(((le_srv_svc_resp_st*)(pvRetData))->eStatus), &(((le_srv_svc_resp_st*)(pvRetData))->u16Handle));
        break;
        //case GET_CMD_OPCODE(TCU_LE_GATT_SDB_ADD_CHAR_DECL_RESP):
        case    TCU_LE_GATT_SDB_ADD_CHAR_DECL_RESP_OPCODE:
                eLeDrvRes =  eGetSdbGattAddCharacterDecl_Resp(pu8Buff, &(((le_srv_char_decl_resp_st*)(pvRetData))->eStatus), &(((le_srv_char_decl_resp_st*)(pvRetData))->u16Handle));
        break;
        //case GET_CMD_OPCODE(TCU_LE_GATT_SDB_ADD_CHAR_ELE_RESP):
        case TCU_LE_GATT_SDB_ADD_CHAR_ELE_RESP_OPCODE:
                eLeDrvRes =  eGetSdbGattAddCharacterEle_Resp(pu8Buff, &(((le_srv_char_decl_resp_st*)(pvRetData))->eStatus), &(((le_srv_char_decl_resp_st*)(pvRetData))->u16Handle));
        break;
        //case GET_CMD_OPCODE(TCU_LE_GATT_SER_EXG_MTU_EVENT):
        case TCU_LE_GATT_SER_EXG_MTU_EVENT_OPCODE:
                eLeDrvRes = eGetServerGattExgMtu_Event(pu8Buff, (le_mtu_exchg_st*)(pvRetData));
        break;
        //case  GET_CMD_OPCODE(TCU_LE_GATT_SER_EXG_MTU_ACCEPT_RESP):
        case TCU_LE_GATT_SER_EXG_MTU_ACCEPT_RESP_OPCODE:
                eLeDrvRes = eGetServerGattExgMtuAccept_Resp(pu8Buff, (le_mtu_exchg_acc_resp_st*) (pvRetData));
        break;
        //case  GET_CMD_OPCODE(TCU_LE_GATT_SER_READ_MULTIPLE_EVENT):
        case TCU_LE_GATT_SER_READ_MULTIPLE_EVENT_OPCODE:
                eLeDrvRes = eGetServerGattReadMulti_Event(pu8Buff, (le_srv_multi_event_st*)(pvRetData));
        break;
        //case GET_CMD_OPCODE(TCU_LE_GATT_SER_READ_MULTIPLE_ACCEPT_RESP):
        case TCU_LE_GATT_SER_READ_MULTIPLE_ACCEPT_RESP_OPCODE:
                eLeDrvRes = eGetServerGattReadMultiAccept_Resp(pu8Buff, (le_srv_multi_acc_event_st*)(pvRetData));
        break;
        //case GET_CMD_OPCODE(TCU_LE_GATT_SER_READ_CHAR_VAL_EVENT):
        case TCU_LE_GATT_SER_READ_CHAR_VAL_EVENT_OPCODE:
                eLeDrvRes = eGetServerGattReadCharVal_Event(pu8Buff, (le_srv_read_char_val_event_st*) (pvRetData));
        break;
        //case GET_CMD_OPCODE(TCU_LE_GATT_SER_READ_CHAR_VAL_ACCEPT_RESP):
        case TCU_LE_GATT_SER_READ_CHAR_VAL_ACCEPT_RESP_OPCODE:
                eLeDrvRes = eSetServerGattReadCharValAccept_Resp(pu8Buff, (le_srv_read_char_acc_event_st*)(pvRetData));
        break;
        //case GET_CMD_OPCODE(TCU_LE_GATT_SER_WRITE_CHAR_DESP_EVENT):
        case TCU_LE_GATT_SER_WRITE_CHAR_DESP_EVENT_OPCODE:
                eLeDrvRes = eGetServerGattWriteCharDesc_Event(pu8Buff, (le_srv_write_char_desp_event_st*)(pvRetData));
        break;
        //case GET_CMD_OPCODE(TCU_LE_GATT_SDB_UPD_CHAR_ELE_RESP):
        case TCU_LE_GATT_SDB_UPD_CHAR_ELE_RESP_OPCODE:
                eLeDrvRes = eGetSdbGattUpdateCharacterEle_Resp(pu8Buff, (gatt_status_code_e*)(pvRetData));
        break;
        //case GET_CMD_OPCODE(TCU_LE_GATT_SER_WRITE_CHAR_DESP_ACCEPT_RESP):
        case TCU_LE_GATT_SER_WRITE_CHAR_DESP_ACCEPT_RESP_OPCODE:
                eLeDrvRes = eGetServerGattWriteCharDespAccept_Resp(pu8Buff, (le_srv_multi_acc_event_st*)(pvRetData));
        break;
        //case GET_CMD_OPCODE(TCU_LE_GATT_SER_WRITE_CHAR_VAL_ACCEPT_RESP):
        case TCU_LE_GATT_SER_WRITE_CHAR_VAL_ACCEPT_RESP_OPCODE:
                eLeDrvRes = eGetServerGattWriteCharValAccept_Resp(pu8Buff, (le_srv_multi_acc_event_st*)(pvRetData));
        break;
        //case GET_CMD_OPCODE(TCU_LE_GATT_SER_CHAR_VAL_NOTIFICATION_EVENT):
        case TCU_LE_GATT_SER_CHAR_VAL_NOTIFICATION_EVENT_OPCODE:
                eLeDrvRes = eGetServerGattValNotify_Event(pu8Buff, (uint16_t*)(pvRetData));
        break;
        //case GET_CMD_OPCODE(TCU_LE_GATT_SDB_RET_END_GRP_HLE_RESP):
        case TCU_LE_GATT_SDB_RET_END_GRP_HLE_RESP_OPCODE:
                eLeDrvRes = eGetSdbGattReturnEndGroupHandle_Resp(pu8Buff, (le_resp_st*)(pvRetData));
        break;
        //case GET_CMD_OPCODE(TCU_LE_GATT_SDB_ADD_INC_SVC_RESP):
        case TCU_LE_GATT_SDB_ADD_INC_SVC_RESP_OPCODE:
                eLeDrvRes = eGetSdbGattAddInclSvc_Resp(pu8Buff, (le_resp_st*)(pvRetData));
        break;
        //case GET_CMD_OPCODE(TCU_LE_GATT_SER_CHAR_VAL_INDICATION_EVENT):
        case TCU_LE_GATT_SER_CHAR_VAL_INDICATION_EVENT_OPCODE:
                eLeDrvRes = eGetServerGattValIndication_Event(pu8Buff, (le_srv_char_cal_ind_st*)(pvRetData));
        break;
       // case GET_CMD_OPCODE(TCU_LE_GATT_SER_WRITE_WITHOUT_RESPONSE_CMD_EVENT):
        case TCU_LE_GATT_SER_WRITE_WITHOUT_RESPONSE_CMD_EVENT_OPCODE:
                eLeDrvRes = eGetServerGattWriteWithoutRespCmd_Event(pu8Buff, (le_srv_wrt_wo_evt_st*)(pvRetData));
        break;
        //case GET_CMD_OPCODE(TCU_LE_GATT_SER_READ_CHAR_DESP_EVENT):
        case TCU_LE_GATT_SER_READ_CHAR_DESP_EVENT_OPCODE:
                eLeDrvRes = eGetServerGattReadCharDescriptor_Event(pu8Buff, (le_srv_read_char_val_event_st*)(pvRetData));
        break;
        //case GET_CMD_OPCODE(TCU_LE_GATT_SER_READ_CHAR_DESP_ACCEPT_RESP):
        case TCU_LE_GATT_SER_READ_CHAR_DESP_ACCEPT_RESP_OPCODE:
                eLeDrvRes = eGetServerGattReadCharDescriptorAccept_Resp(pu8Buff, (le_srv_read_char_acc_event_st*)(pvRetData));
        break;
      case TCU_LE_GATT_SER_WRITE_CHAR_VAL_EVENT_OPCODE:
              eLeDrvRes = eGetServerGattWriteCharVal_Event(pu8Buff,  (le_srv_write_char_desp_event_st*)(pvRetData));
      break;

        default:
        {
            eResult = LE_API_ERR_PARSING;
        }
    }

    if((eResult == LE_API_SUCCCESS) && (eLeDrvRes != LE_FUNC_SUCCCESS))
    {
       eResult = LE_API_ERR_PARSING;
    }

    return eResult;
}

/******************************************************************************/
le_api_result_e eBleExtractDataEventGattCli(uint8_t* pu8Buff, void* pvRetData)
{
    le_api_result_e eResult = LE_API_SUCCCESS;
    uint8_t u8OpCode = PARSE_CMD_OPCODE(pu8Buff);

    /* Process returned response/notification */
    switch(u8OpCode)
    {
        default:
        {
            eResult = LE_API_ERR_PARSING;
        }
    }

    return eResult;
}

/******************************************************************************/
le_api_result_e eBleExtractData(uint8_t* pu8Buff, void* pvRetData)
{
    le_api_result_e eResult = LE_API_SUCCCESS;
    eTCU_SERVICE_ID_ eServId = (eTCU_SERVICE_ID_)PARSE_CMD_SERVICE_ID(pu8Buff);

    /* Select the appropriate parsing function dependent on Service ID */
    switch(eServId)
    {
      case TCU_BT_LE_MNG:
        {
          /* Process MNG Events/Responces */
          eResult = eBleExtractDataEventMng(pu8Buff, pvRetData);
        }
      break;
      case TCU_BT_LE_CLI:
        {
          /* Process GATT Client Events/Responces */
          eResult = eBleExtractDataEventGattCli(pu8Buff, pvRetData);
        }
      break;
      case TCU_BT_LE_SRV:
        {
          /* Process GATT Server Events/Responses */
          eResult = eBleExtractDataEventGattSer(pu8Buff, pvRetData);
        }
      break;
      case TCU_BT_CNTRL:
        {
          /* Process control message used to abort waiting */
          eResult = LE_USR_ABORT;
        }
      break;
      default:
        {
          /* Report unexpected event */
          eResult = LE_API_ERR_EVENT;
        }
    }

    return eResult;
}

/******************************************************************************/
void vAppCallback(uint8_t* pu8Data)
{
#ifdef DEBUG_PRINT
  printf(" vAppCallback() \n");
#endif

  /* Call user registered function over the function pointer */
  ptUnexEvtCallBack(PARSE_CMD_SERVICE_ID(pu8Data),
         PARSE_CMD_OPCODE(pu8Data),
         PARSE_CMD_PARAM_LENGTH(pu8Data),
         &pu8Data[CMD_ARRAY_PARAM_OFFS]);

}

/******************************************************************************/
le_api_result_e vSmartWaitForAnyEvent(void* qHandle, uint16_t* pu16Cmd)
{
  le_api_result_e eResult = LE_API_SUCCCESS;
  int32_t qResult = 0;

  uint32_t timeout = 0xFFFFFFFF;


  //uint8_t* pu8Data = (uint8_t*)(stTcuEvent.ptEvent);
  uint8_t* pu8Data = (uint8_t*)(tcu_event.ptEvent);


  extern uint8_t DATA_FROM_BLUETOOTH_UART;

  /* Wait until any event comes or timeout*/
  //qResult = ReceiveEvent(qHandle, &stTcuEvent, TCU_MNG_TIMEOUT_1Sec);
  DATA_FROM_BLUETOOTH_UART = 0;
  while(DATA_FROM_BLUETOOTH_UART == 0)
  {
      /*wait forever?
       * timeout--;
       *
       * */
  }



  //if(qResult != 0)
  if(DATA_FROM_BLUETOOTH_UART == 1)
  {
    /* Extract the data from input stream */
    *pu16Cmd = PARSE_CMD_MESSAGE(pu8Data);
    eResult = LE_API_SUCCCESS;
  }
  else
  {
      eResult = LE_API_ERR_TIMEOUT;
  }

  return eResult;
}

/******************************************************************************/
//le_api_result_e eBleWaitForSpecificEvent(void* qHandle, uint32_t u32ExpEvent, uint32_t u32TimeOut, void* pvRetData)
le_api_result_e eBleWaitForSpecificEvent(void* qHandle, tcu_packet_STRUCT  ExpEvent, uint32_t u32TimeOut, void* pvRetData)
{
  int32_t qResult = 0;
  le_api_result_e eResult = LE_API_SUCCCESS;
extern uint8_t DATA_FROM_BLUETOOTH_UART;
  /* No time-out, so stay until expected message comes */
  while((qResult != 1) || ( eResult != LE_API_SUCCCESS))
  {
    //qResult = ReceiveEvent(qHandle, &stTcuEvent, u32TimeOut);  /*WAITING FOR SOMETHING DATA FROM THE BLUETOOTH UART.  stTcuEvent seems to be a structure.  WHERE DOES THIS GET FILLED??*/
      while( (DATA_FROM_BLUETOOTH_UART == 0) && (u32TimeOut>0) )
     {
          u32TimeOut--;  /*CHANGE THIS TO BE A ms TIMEOUT*/
     }

    /* If message received, then process it */
    //if(qResult == 1)
     if(DATA_FROM_BLUETOOTH_UART == 1)
    {
         qResult = 1;  /*gets us out of the while loop*/
         DATA_FROM_BLUETOOTH_UART = 0;
      /* Check if event was expected and that was not a control message sent by application */
      //if(((  (ExpEvent.eventType) != stTcuEvent.eventType) || ((ExpEvent.Service_ID) != stTcuEvent.Service_ID)) && ( TCU_BT_CNTRL != stTcuEvent.Service_ID))
      if(((  (ExpEvent.opcode) != tcu_event.eventType) || ((ExpEvent.service_id) != tcu_event.Service_ID)) && ( TCU_BT_CNTRL != tcu_event.Service_ID))
      {
        /* Unexpected command received (not control message),
          so notify application by invoking a callback function.
          No parsing will be done at this level, app needs to take care.
        */
        //vAppCallback((uint8_t*)(stTcuEvent.ptEvent));
        vAppCallback((uint8_t*)(tcu_event.ptEvent));

      }else
      {
        /* Expected or control event */

        /* Extract Data */
//        eResult = eBleExtractData((uint8_t*)(stTcuEvent.ptEvent), pvRetData);
          eResult = eBleExtractData((uint8_t*)(tcu_event.ptEvent), pvRetData);

        /* Check if user abort */
        if(LE_USR_ABORT == eResult)
        {
            break;
        }
      }
    }else if(u32TimeOut != TCU_MNG_NO_TIMEOUT)
   {
     /* If time-out specified, than leave the loop */
     eResult = LE_API_ERR_TIMEOUT;
     break;
   }
  }
  return eResult;
}
/******************************************************************************/
le_api_result_e eBleDeviceSrv_Init(void* qHandle, uint8_t* const pu8DevName, uint8_t u8DevNameLen, uint8_t* pu8RetBd_Addr)
{
    le_api_result_e eResTemp;
    le_mnginit_resp_st stInitResp;
    gatt_status_code_e eStatus;

#ifdef DEBUG_PRINT
  printf(" eBlePerif_Init() \n");
#endif

    /* Write Bluetooth Device Name */
    eSetLeMngInit_Req(pu8DevName, u8DevNameLen);

    /* Check if input pointer is initialized */
    if(NULL == pu8RetBd_Addr)
    {
      return LE_NULL_PTR_ERR;
    }

    /* Initialize BD_ADDR pointer */
    stInitResp.pu8BdAddr = pu8RetBd_Addr;

    /* Wait for event and extract data */

    //eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_MNG_LE_INIT_RESP, TCU_MNG_NO_TIMEOUT, (void*)(&stInitResp));
    eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_MNG_LE_INIT_RESP_t, TCU_MNG_NO_TIMEOUT, (void*)(&stInitResp));

    /* Check if event received */
    if(eResTemp !=LE_API_SUCCCESS)
    {
        return eResTemp;
    }

    /* Check if initialization successful */
    if(LE_MNG_INIT_SUCCESS != stInitResp.eStatus)
    {
        return LE_API_ERR_INIT;
    }

    /* GATT Server initialize request */
    eSetServerGattInit_Req();

    /* Wait for TCU_LE_GATT_SER_INIT_RESP */
     //eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SER_INIT_RESP, TCU_MNG_NO_TIMEOUT, (void*)(&eStatus));
    eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SER_INIT_RESP_t, TCU_MNG_NO_TIMEOUT, (void*)(&eStatus));

    /* Check if event received */
    if(eResTemp !=LE_API_SUCCCESS)
    {
        return eResTemp;
    }
    else if(LE_GATT_SUCCESS != eStatus)
        {
            return LE_SER_INIT_ERROR;
        }

    return LE_API_SUCCCESS;
}

/******************************************************************************/
le_api_result_e eBleSrv_GetEndGroupHandle(void* qHandle,
                                             uint16_t u16SvcHandle,
                                             uint16_t* pu16EndGroupHandle)
{
  le_resp_st stResp;
  le_func_status_e eFncRes;
  le_api_result_e eResTemp;

  /*** Obtain End Group Handle for given service ***/
  eFncRes= eSetSdbGattReturnEndGroupHandle_Req(u16SvcHandle);

  if(LE_FUNC_SUCCCESS != eFncRes)
  {
      return LE_API_ERR_DRV;
  }


  /* Wait for response event and extract End group handle */
//  eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SDB_RET_END_GRP_HLE_RESP, TCU_MNG_NO_TIMEOUT, (void*)(&stResp));
  eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SDB_RET_END_GRP_HLE_RESP_t, TCU_MNG_NO_TIMEOUT, (void*)(&stResp));

  /* Check if event received */
  if(LE_API_SUCCCESS != eResTemp)
  {
      return eResTemp;

  }else if(LE_GATT_SUCCESS == stResp.eStatus)
        {
          /* Return end group handle */
          *pu16EndGroupHandle = stResp.u16Handle;
        }
        else
        {
          return LE_API_ERR_EVENT;
        }

  return LE_API_SUCCCESS;
}

/******************************************************************************/
le_api_result_e eBleSrv_AddIncludeToSvc(void* qHandle,
                                           uint16_t u16ParentSvcHandle,
                                           uint16_t u16InclSvcHandle,
                                           uint16_t u16InclSvcUuid,
                                           uint16_t* pu16RetIncSvcHandle)
{
    le_resp_st stResp;
    le_func_status_e eFncRes;
    le_api_result_e eResTemp;
    uint16_t u16InclSvcEndHandle;
    uint8_t auAttVal[6];

#ifdef DEBUG_PRINT
  printf(" eBleSrv_AddIncludeToSvc() \n");
#endif

    /*** Obtain End Group Handle for included service ***/
    eResTemp = eBleSrv_GetEndGroupHandle(qHandle, u16InclSvcHandle, &u16InclSvcEndHandle);

    if(LE_API_SUCCCESS != eResTemp)
    {
        return eResTemp;
    }

    /* Prepare Attribute Value for service include */
    auAttVal[0] = GET_BYTE_LSB(u16InclSvcHandle);       /* Include Service Start Handle */
    auAttVal[1] = GET_BYTE_MSB(u16InclSvcHandle);
    auAttVal[2] = GET_BYTE_LSB(u16InclSvcEndHandle);    /* Include Service End Handle */
    auAttVal[3] = GET_BYTE_MSB(u16InclSvcEndHandle);
    auAttVal[4] = GET_BYTE_LSB(u16InclSvcUuid);         /* Include Service UUID */
    auAttVal[5] = GET_BYTE_MSB(u16InclSvcUuid);

    /*** Service Inclusion Request ***/
    eFncRes= eSetSdbGattAddInclSvc_Req(u16ParentSvcHandle,
                                       LE_SHORT_UUID,
                                       auAttVal);

    if(LE_FUNC_SUCCCESS != eFncRes)
    {
        return LE_API_ERR_DRV;
    }

    /* Wait for event and extract data */

    //eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SDB_ADD_INC_SVC_RESP, TCU_MNG_NO_TIMEOUT, (void*)(&stResp));
    eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SDB_ADD_INC_SVC_RESP_t, TCU_MNG_NO_TIMEOUT, (void*)(&stResp));

    /* Check if event received */
    if(LE_API_SUCCCESS != eResTemp)
    {
        return eResTemp;
    }else if(LE_GATT_SUCCESS == stResp.eStatus)
          {
            /* Return include service handle */
            *pu16RetIncSvcHandle = stResp.u16Handle;
          }
          else
          {
            return LE_API_ERR_EVENT;
          }

    return LE_API_SUCCCESS;
}

/******************************************************************************/
le_api_result_e eBleSrv_AddPrimServiceAttributes(void* qHandle, uint16_t u16UUID, uint16_t* pu16Handle)
{
    le_srv_svc_resp_st stPrimSvcResp;
    le_func_status_e eFncRes;
    le_api_result_e eResTemp;
    uint8_t au8UUID[2];
#ifdef DEBUG_PRINT
  printf(" eBleSrv_AddPrimServiceAttributes() \n");
#endif
    /* 2-byte UUID */
    au8UUID[0] = GET_BYTE_LSB(u16UUID);
    au8UUID[1] = GET_BYTE_MSB(u16UUID);

    /*** Add Primary Gatt service  ***/
    eFncRes= eSetSdbGattAddPrimaryService_Req(LE_SHORT_UUID, au8UUID);

    if(LE_FUNC_SUCCCESS != eFncRes)
    {
        return LE_API_ERR_DRV;
    }

    /* Wait for event and extract data */



    //eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SDB_ADD_PRIM_SVC_RESP, TCU_MNG_NO_TIMEOUT, (void*)(&stPrimSvcResp));
    eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SDB_ADD_PRIM_SVC_RESP_t, TCU_MNG_NO_TIMEOUT, (void*)(&stPrimSvcResp));
    /* Check if event received */
    if(LE_API_SUCCCESS != eResTemp)
    {
        return eResTemp;
    }

    if( LE_SDB_SUCCESS == stPrimSvcResp.eStatus)
    {
      /* Return primary service handle */
      *pu16Handle = stPrimSvcResp.u16Handle;
    }
    else
    {
       return LE_API_ERR_EVENT;
    }

    return LE_API_SUCCCESS;
}

/******************************************************************************/
le_api_result_e eBleSrv_AddSecServiceAttributes(void* qHandle, uint16_t u16ParentSvcHandle, uint16_t u16UUID, uint16_t* pu16Handle)
{
    le_srv_svc_resp_st stSvcResp;
    le_func_status_e eFncRes;
    le_api_result_e eResTemp;
    uint8_t au8UUID[2];
#ifdef DEBUG_PRINT
  printf(" eBleSrv_AddSecServiceAttributes() \n");
#endif
    /* 2-byte UUID */
    au8UUID[0] = GET_BYTE_LSB(u16UUID);
    au8UUID[1] = GET_BYTE_MSB(u16UUID);

    /*** Add Secondary Gatt service  ***/
    eFncRes = eSetSdbGattAddSecondaryService_Req(LE_SHORT_UUID, u16ParentSvcHandle, au8UUID);

    if(LE_FUNC_SUCCCESS != eFncRes)
    {
        return LE_API_ERR_DRV;
    }

    /* Wait for event and extract data */

    //eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SDB_ADD_SEC_SVC_RESP, TCU_MNG_NO_TIMEOUT, (void*)(&stSvcResp));
    eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SDB_ADD_SEC_SVC_RESP_t, TCU_MNG_NO_TIMEOUT, (void*)(&stSvcResp));
    /* Check if event received */
    if(LE_API_SUCCCESS != eResTemp)
    {
        return eResTemp;
    }

    if( LE_SDB_SUCCESS == stSvcResp.eStatus)
    {
      /* Return primary service handle */
      *pu16Handle = stSvcResp.u16Handle;
    }
    else
    {
       return LE_API_ERR_EVENT;
    }

    return LE_API_SUCCCESS;
}

/******************************************************************************/
le_api_result_e eBleSrv_AddCharacteristic(void* qHandle,
                                              uint16_t u16SvcHandle,
                                              le_srv_char_st* pstCharac,
                                              uint16_t* pu16RetHandleChar,
                                              uint16_t* pu16RetHandleCharEle)
{
    le_api_result_e eResTemp;
    le_func_status_e eResFnc;
    le_srv_char_decl_resp_st stCharDeclResp;
    le_srv_char_decl_resp_st stCharEleResp;
    char_el_req_st stValDecl;
    uint8_t au8Uuid[2];
    uint8_t u8Iter;

    au8Uuid[0] = GET_BYTE_LSB(pstCharac->u16CharUUID);
    au8Uuid[1] = GET_BYTE_MSB(pstCharac->u16CharUUID);

#ifdef DEBUG_PRINT
  printf(" eBleSrv_AddCharacteristic() \n");
#endif

    /*** Add characteristic declaration to the database ***/
    eResFnc = eSetSdbGattAddCharacterDecl_Req(u16SvcHandle, pstCharac->u8CharProperties, LE_SHORT_UUID, au8Uuid);

    /* Check if event received */
    if(eResFnc !=LE_FUNC_SUCCCESS)
    {
        return LE_API_ERR_DRV;
    }

    /* Wait for event and extract data */

    //eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SDB_ADD_CHAR_DECL_RESP, TCU_MNG_NO_TIMEOUT, (void*)(&stCharDeclResp));
    eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SDB_ADD_CHAR_DECL_RESP_t, TCU_MNG_NO_TIMEOUT, (void*)(&stCharDeclResp));

    /* Check if event received */
    if(eResTemp !=LE_API_SUCCCESS)
    {
        return eResTemp;
    }
    else
    {
        /* Check if request successful */
        if(LE_GATT_SUCCESS != stCharDeclResp.eStatus)
        {
            return LE_API_ERR_EVENT;
        }

        /* Return characteristic handle */
        *pu16RetHandleChar = stCharDeclResp.u16Handle;
    }

    /*** Add Character VALUE Declaration ***/
    stValDecl.eAttrLen = LE_SHORT_UUID;
    stValDecl.au8AttType[0] = au8Uuid[0];                       /*  UUID of the char. VALUE must be the same as given in char. declaration*/
    stValDecl.au8AttType[1] = au8Uuid[1];
    stValDecl.pu8AttVal = pstCharac->stCharValDecl.pu8AttVal;
    stValDecl.u16AttPermission = pstCharac->stCharValDecl.u16AttPermission;
    stValDecl.u16AttValLen  = pstCharac->stCharValDecl.u16AttValLen;

    eResFnc = eSetSdbGattAddCharacterEle_Req(stCharDeclResp.u16Handle, stValDecl);

    if(eResFnc !=LE_FUNC_SUCCCESS)
    {
      return LE_API_ERR_DRV;
    }

    /* Wait for event and extract data */


    //eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SDB_ADD_CHAR_ELE_RESP, TCU_MNG_NO_TIMEOUT, (void*)(&stCharEleResp));
    eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SDB_ADD_CHAR_ELE_RESP_t, TCU_MNG_NO_TIMEOUT, (void*)(&stCharEleResp));
    /* Check if event received */
    if(eResTemp !=LE_API_SUCCCESS)
    {
      return eResTemp;
    }
    else
    {
      /* Check if Character element added */
      if(LE_GATT_SUCCESS != stCharEleResp.eStatus)
      {
          return LE_API_ERR_EVENT;
      }

      /* Extract Character VALUE declaration handle */
      pstCharac->u16RetCharValDeclHandle = stCharEleResp.u16Handle;
    }

    /*** Add all Character Descriptors (if any) ***/
    for(u8Iter=0; u8Iter < pstCharac->u8CharDescCnt; u8Iter++)
    {
      /* Add Characteristic element */
      eResFnc = eSetSdbGattAddCharacterEle_Req(stCharDeclResp.u16Handle, pstCharac->pstCharDesc[u8Iter]);

      if(eResFnc !=LE_FUNC_SUCCCESS)
      {
        return LE_API_ERR_DRV;
      }

      /* Wait for event and extract data */

      //eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SDB_ADD_CHAR_ELE_RESP, TCU_MNG_NO_TIMEOUT, (void*)(&stCharEleResp));
      eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SDB_ADD_CHAR_ELE_RESP_t, TCU_MNG_NO_TIMEOUT, (void*)(&stCharEleResp));
      /* Check if event received */
      if(eResTemp !=LE_API_SUCCCESS)
      {
        return eResTemp;
      }
      else
      {
        /* Check if Character element added */
        if(LE_GATT_SUCCESS != stCharEleResp.eStatus)
        {
            return LE_API_ERR_EVENT;
        }

        /* Extract Character element handle */
        pu16RetHandleCharEle[u8Iter] = stCharEleResp.u16Handle;
      }
    }

    return LE_API_SUCCCESS;
}

/******************************************************************************/
le_api_result_e eBleSrv_RegisterPrimaryService(void* qHandle, le_srv_service_def_st* pstSvcDef)
{
  le_api_result_e eResult;
  uint8_t u8Iter;

  /* Check if structure pointer is initialized */
  if(NULL == pstSvcDef)
  {
    return LE_NULL_PTR_ERR;
  }

  /* Add Primary service Declaration */
  delay_cycles_ms(0xffff);  /*when I added a break point here, this passed.  seems there is some timing issue if we run straight through the first time.*/
  eResult = eBleSrv_AddPrimServiceAttributes(qHandle, pstSvcDef->u16SvcUUID, &(pstSvcDef->u16RetSvcHandle));

  /* Check if successful */
  if(eResult != LE_API_SUCCCESS)
  {
    return eResult;
  }

  /* Check if include pointer initialized */
  if((0 != pstSvcDef->u8InclDefCount) && (NULL == pstSvcDef->pstIncludes))
  {
    return LE_NULL_PTR_ERR;
  }

  /* If service contains INCLUDES, add all of them (If exists, INCLUDES must be the first entries) */
  for(u8Iter=0; u8Iter < pstSvcDef->u8InclDefCount; u8Iter++)
  {
    /* Add Include Service */
    eResult = eBleSrv_AddIncludeToSvc(qHandle,
                              pstSvcDef->pstIncludes[u8Iter].u16ParentSvcHandle,
                              pstSvcDef->pstIncludes[u8Iter].u16InclSvcHandle,
                              pstSvcDef->pstIncludes[u8Iter].u16InclSvcUuid,
                              &(pstSvcDef->pstIncludes[u8Iter].u16RetInclHandle));

    /* Check if Include service added successfuly */
    if(eResult != LE_API_SUCCCESS)
    {
      return eResult;
    }

  }

  /* Check if characteristic pointer initialized */
  if((0 != pstSvcDef->u8CharDefCount) && (NULL == pstSvcDef->pstChars))
  {
    return LE_NULL_PTR_ERR;
  }

  /* If service contains CHARACTERISTICS add all of them (If exist, CHARACTERISTICS must follow rigth after INCLUDES)*/
  for(u8Iter=0; u8Iter < pstSvcDef->u8CharDefCount; u8Iter++)
  {
    /* Check if pointer for return handles is initialized */
    if(NULL == pstSvcDef->pstChars[u8Iter].pu16RetCharDescHandles)
    {
      return LE_NULL_PTR_ERR;
    }

    eResult = eBleSrv_AddCharacteristic(qHandle,
                                          pstSvcDef->u16RetSvcHandle,
                                          &(pstSvcDef->pstChars[u8Iter]),
                                          &(pstSvcDef->pstChars[u8Iter].u16RetCharDeclHandle),
                                          pstSvcDef->pstChars[u8Iter].pu16RetCharDescHandles);

    /* Check if characteristic added successfuly */
    if(eResult != LE_API_SUCCCESS)
    {
      return eResult;
    }

  }

  return eResult;
}

/******************************************************************************/
//le_api_result_e eBleSrv_RegisterSecondaryService(void* qHandle, uint16_t u16ParentSvcHandle, uint16_t u16ParentSvcUuid, le_srv_service_def_st* pstSvcDef)
le_api_result_e eBleSrv_RegisterSecondaryService(void* qHandle, le_srv_service_def_st* pstParentSvcDef, le_srv_service_def_st* pstSecSvcDef)
{
  le_api_result_e eResult;
  uint8_t u8Iter;

  /* Check if structure pointer is initialized */
  if((NULL == pstSecSvcDef) || (NULL == pstParentSvcDef))
  {
    return LE_NULL_PTR_ERR;
  }

  /* Add Secondary service Declaration */
  eResult = eBleSrv_AddSecServiceAttributes(qHandle,
                                              pstParentSvcDef->u16RetSvcHandle,
                                              pstParentSvcDef->u16SvcUUID,
                                              &(pstSecSvcDef->u16RetSvcHandle));
  /* Check if successful */
  if(eResult != LE_API_SUCCCESS)
  {
    return eResult;
  }

  /* Check if include pointer initialized */
  if((0 != pstSecSvcDef->u8InclDefCount) && (NULL == pstSecSvcDef->pstIncludes))
  {
    return LE_NULL_PTR_ERR;
  }

  /* If service contains INCLUDES, add all of them */
  for(u8Iter=0; u8Iter < pstSecSvcDef->u8InclDefCount; u8Iter++)
  {
    /* Add Include Service */
    eResult = eBleSrv_AddIncludeToSvc(qHandle,
                              pstSecSvcDef->pstIncludes[u8Iter].u16ParentSvcHandle,
                              pstSecSvcDef->pstIncludes[u8Iter].u16InclSvcHandle,
                              pstSecSvcDef->pstIncludes[u8Iter].u16InclSvcUuid,
                              &(pstSecSvcDef->pstIncludes[u8Iter].u16RetInclHandle));

    /* Check if Include service added successfuly */
    if(eResult != LE_API_SUCCCESS)
    {
      return eResult;
    }
  }

  /* Check if characteristic pointer initialized */
  if((0 != pstSecSvcDef->u8CharDefCount) && (NULL == pstSecSvcDef->pstChars))
  {
    return LE_NULL_PTR_ERR;
  }

  /* If service contains CHARACTERISTICS add all of them */
  for(u8Iter=0; u8Iter < pstSecSvcDef->u8CharDefCount; u8Iter++)
  {

    /* Check if pointer for return handles is initialized */
    if(NULL == pstSecSvcDef->pstChars[u8Iter].pu16RetCharDescHandles)
    {
      return LE_NULL_PTR_ERR;
    }

    eResult = eBleSrv_AddCharacteristic(qHandle,
                                          pstSecSvcDef->u16RetSvcHandle,
                                          &(pstSecSvcDef->pstChars[u8Iter]),
                                          &(pstSecSvcDef->pstChars[u8Iter].u16RetCharDeclHandle),
                                          pstSecSvcDef->pstChars[u8Iter].pu16RetCharDescHandles);

    /* Check if characteristic added successfuly */
    if(eResult != LE_API_SUCCCESS)
    {
      return eResult;
    }

  }

  return eResult;
}

/******************************************************************************/
le_api_result_e eBlePerif_StartAdv(void* qHandle, le_adv_req_st stConfig)
 {
    le_func_status_e eResFnc;
    le_api_result_e eResTemp;
    uint8_t u8Status;

#ifdef DEBUG_PRINT
  printf(" eBlePerif_StartAdv() \n");
#endif

    eResFnc = eSetLeStartAdvertise_Req(stConfig);

    if(eResFnc !=LE_FUNC_SUCCCESS)
    {
        return LE_API_ERR_DRV;
    }

    /* Wait for event and extract data */


//    eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_MNG_LE_START_ADVERTISE_RESP, TCU_MNG_NO_TIMEOUT, (void*)(&u8Status));
    eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_MNG_LE_START_ADVERTISE_RESP_t, TCU_MNG_NO_TIMEOUT, (void*)(&u8Status));

    /* Check if event received */
    if(eResTemp !=LE_API_SUCCCESS)
    {
        return eResTemp;
    }
    else
    {
        if(LE_GATT_SUCCESS != u8Status)
        {
            return LE_API_ERR_EVENT;
        }
    }

    return LE_API_SUCCCESS;

 }

/******************************************************************************/
le_api_result_e eBlePerif_StopAdv(void* qHandle)
 {
    le_func_status_e eResFnc;
    le_api_result_e eResTemp;
    uint8_t u8Status;

#ifdef DEBUG_PRINT
  printf(" eBlePerif_StartAdv() \n");
#endif

    /* Request the device to stop advertizing */
    eResFnc = eSetLeDisableAdvertise_Req();

    if(eResFnc !=LE_FUNC_SUCCCESS)
    {
        return LE_API_ERR_DRV;
    }

    /* Wait for event and extract data */
    //TCU_MNG_LE_DISABLE_ADVERTISE_RESP = 0xD1 89 FFFF

    //eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_MNG_LE_DISABLE_ADVERTISE_RESP, TCU_MNG_NO_TIMEOUT, (void*)(&u8Status));
    eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_MNG_LE_DISABLE_ADVERTISE_RESP_t, TCU_MNG_NO_TIMEOUT, (void*)(&u8Status));

    /* Check if event received */
    if(eResTemp !=LE_API_SUCCCESS)
    {
        return eResTemp;
    }
    else
    {
        if(LE_GATT_SUCCESS != u8Status)
        {
            return LE_API_ERR_EVENT;
        }
    }

    return LE_API_SUCCCESS;

 }

/******************************************************************************/
le_api_result_e eBleDevice_Disconnect(void* qHandle,  uint8_t* const pu8Bd_Addr, uint16_t* pu16RetConnHandle)
{
  le_func_status_e eResFnc;
  le_api_result_e eResTemp = LE_API_UNINITIALIZED;
  le_accept_st stEvntData;
  le_disconnect_st stDisconnect;

#ifdef DEBUG_PRINT
  printf(" eBleDevice_Disconnect() \n");
#endif

  /* Request disconnection */
  eResFnc = eSetLeDisconnect_Req(pu8Bd_Addr);

  if(eResFnc !=LE_FUNC_SUCCCESS)
  {
      return LE_API_ERR_DRV;
  }


  //eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_ACCEPT, TCU_MNG_NO_TIMEOUT, (void*)(&stEvntData));
  eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_ACCEPT_t, TCU_MNG_NO_TIMEOUT, (void*)(&stEvntData));

  if(eResTemp !=LE_API_SUCCCESS)
  {
      return eResTemp;
  }

  /* Check if everything went fine */
//  if((stEvntData.u8OpCode == GET_CMD_OPCODE(TCU_MNG_LE_DISCONNECT_REQ))||   0x13
  if((stEvntData.u8OpCode == TCU_MNG_LE_DISCONNECT_REQ_t.opcode)||
    // (stEvntData.u16ServiceId == GET_CMD_SERVICE_ID(TCU_MNG_LE_DISCONNECT_REQ)) ||  0xD1
          (stEvntData.u16ServiceId ==TCU_MNG_LE_DISCONNECT_REQ_t.service_id) ||
      stEvntData.eStatus == LE_SUCCESS)
  {
    /* Wait for  TCU_MNG_LE_DISCONNECT_EVENT */
      //TCU_MNG_LE_DISCONNECT_EVENT = 0xD1 93 0002

    //eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_MNG_LE_DISCONNECT_EVENT, TCU_MNG_NO_TIMEOUT, (void*)(&stDisconnect));
    eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_MNG_LE_DISCONNECT_EVENT_t, TCU_MNG_NO_TIMEOUT, (void*)(&stDisconnect));

    if(eResTemp !=LE_API_SUCCCESS)
    {
      return eResTemp;
    }
    else if(LE_API_SUCCCESS == stDisconnect.u8Status)
        {
          *pu16RetConnHandle = stDisconnect.u16ConnHandle;
        }
  }
  else
  {
     return LE_API_ERR_DRV;
  }

  return LE_API_SUCCCESS;
}

/******************************************************************************/
le_api_result_e eBleSrv_ClientConnRequestHndlr(le_conn_evt_st* pstConnectInfo)
{
  le_api_result_e eResTemp;

#ifdef DEBUG_PRINT
  printf(" eBleSrv_ClientConnRequestHndlr() \n");
#endif

  //eResTemp = eBleExtractData((uint8_t*)(stTcuEvent.ptEvent), (void*)(pstConnectInfo));
  eResTemp = eBleExtractData((uint8_t*)(tcu_event.ptEvent), (void*)(pstConnectInfo));

  return eResTemp;
}

/******************************************************************************/
le_api_result_e eBleSrv_ClientMtuExchangeHndlr(void* qHandle,
                                                  uint16_t u16ConnHandle,
                                                  le_mtu_exchg_st* pstMtuClientData,
                                                  le_mtu_exchg_acc_resp_st* pstMtuSrvData)
{
  le_func_status_e eResFnc;
  le_api_result_e eResTemp = LE_API_UNINITIALIZED;
  uint8_t  u8Status = 0x00;
  uint16_t u16ServerRxMtuSize =0x80;

#ifdef DEBUG_PRINT
  printf(" eBleSrv_ClientMtuExchangeHndlr() \n");
#endif

  /* Parse MTU exchange request from a client (TCU_LE_GATT_SER_EXG_MTU_EVENT) */
  //eResTemp = eBleExtractData((uint8_t*)(stTcuEvent.ptEvent), (void*)(pstMtuClientData));
  eResTemp = eBleExtractData((uint8_t*)(tcu_event.ptEvent), (void*)(pstMtuClientData));

  if(eResTemp !=LE_API_SUCCCESS)
  {
      return eResTemp;
  }
  else
  {
      /* Check if received connection handle matches the one given from user */
      if((u16ConnHandle != CONN_HANDLE_ANY) && (u16ConnHandle != *(pstMtuClientData->pu16ConnHandle)))
      {
        return LE_CONN_HNDLR;
      }
  }

  /* Send accept request */
  eResFnc  = eGetServerGattExgMtuAccept_Req(*(pstMtuClientData->pu16ConnHandle), u8Status, u16ServerRxMtuSize);

  if(eResFnc !=LE_FUNC_SUCCCESS)
  {
    return LE_API_ERR_DRV;
  }

  /* Wait for MTU accept response */

  //eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SER_EXG_MTU_ACCEPT_RESP, TCU_MNG_NO_TIMEOUT, (void*)(pstMtuSrvData));
  eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SER_EXG_MTU_ACCEPT_RESP_t, TCU_MNG_NO_TIMEOUT, (void*)(pstMtuSrvData));

  if(eResTemp !=LE_API_SUCCCESS)
  {
      return eResTemp;
  }
  else
  {
    if(LE_GATT_SUCCESS != pstMtuSrvData->u8Status)
    {
      return LE_API_ERR_EVENT;
    }
    else if(*(pstMtuClientData->pu16ConnHandle) != pstMtuSrvData->u16ConnHandle)
        {
           return LE_CONN_HNDLR;
        }
  }

  return LE_API_SUCCCESS;

}

/******************************************************************************/
le_api_result_e eBleSrv_ReadMultiEventHndlr(void* qHandle, uint16_t u16ConnHandle, uint16_t* pu16RetConnHandle)
{

  le_api_result_e eResTemp = LE_API_UNINITIALIZED;
  le_func_status_e eResFnc;
  le_srv_multi_event_st stDummyEvntData;
  le_srv_multi_acc_event_st stDummyAccEvntData;
  uint8_t au8HndlrList[10];

#ifdef DEBUG_PRINT
  printf(" eBleSrv_NoUpdate_ReadMultiEventHndlr() \n");
#endif


  /* Assing pointer to avoid exeption by parsing */
  stDummyEvntData.pu8HndlList = au8HndlrList;

  /* Wait until read multiple event comes from a client */
  //eResTemp = eBleExtractData((uint8_t*)(stTcuEvent.ptEvent), (void*)(&stDummyEvntData));
  eResTemp = eBleExtractData((uint8_t*)(tcu_event.ptEvent), (void*)(&stDummyEvntData));

  if(eResTemp !=LE_API_SUCCCESS)
  {
      return eResTemp;
  }
  else  if((u16ConnHandle != CONN_HANDLE_ANY) && (u16ConnHandle != stDummyEvntData.u16ConnHandle))
        {
          return LE_CONN_HNDLR;
        }


  /* Check if function pointer is initialized */
  if(NULL == pUpdMultiCallback)
  {
    return LE_NULL_PTR_ERR;
  }

  /* User Provided Callback function that should update all necessary elements */
  eResTemp = (*pUpdMultiCallback)(qHandle,
                                  stDummyEvntData.u16ConnHandle,
                                  stDummyEvntData.u16HndlCnt,
                                  stDummyEvntData.pu8HndlList);

  /* Check if update was successful */
  if(eResTemp !=LE_API_SUCCCESS)
  {
      return eResTemp;
  }

  /* Respond to Multiple event, green light for Chiron to send data */
  eResFnc = eSetServerGattReadMultiAccept_Req(stDummyEvntData.u16ConnHandle, LE_GATT_SUCCESS, &stDummyEvntData.pu8HndlList[0]);

  if(eResFnc !=LE_FUNC_SUCCCESS)
  {
    return LE_API_ERR_DRV;
  }

  eResTemp = LE_API_UNINITIALIZED;

  /* Wait until TCU_LE_GATT_SER_READ_MULTIPLE_ACCEPT_RESP commes */



  //eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SER_READ_MULTIPLE_ACCEPT_RESP, TCU_MNG_NO_TIMEOUT, (void*)(&stDummyAccEvntData));
  eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SER_READ_MULTIPLE_ACCEPT_RESP_t, TCU_MNG_NO_TIMEOUT, (void*)(&stDummyAccEvntData));

  if(eResTemp !=LE_API_SUCCCESS)
  {
      return eResTemp;
  }

  /* Check if error returned */
  if(stDummyAccEvntData.eStatus != LE_GATT_SUCCESS)
  {
     return LE_API_ERR_EVENT;
  }
  else if(stDummyEvntData.u16ConnHandle != stDummyAccEvntData.u16ConnHandle)
      {
           return LE_CONN_HNDLR;
      }

  /* Return connection handle */
  *pu16RetConnHandle = stDummyAccEvntData.u16ConnHandle;


  return LE_API_SUCCCESS;

}

/******************************************************************************/
le_api_result_e eBleSrv_ReadCharValHndlr(void* qHandle, uint16_t u16ConnHandle, uint16_t* pu16RetConnHandle)
{
  le_func_status_e eResFnc;
  le_api_result_e eResTemp = LE_API_UNINITIALIZED;
  le_srv_read_char_val_event_st stCharValEvent;
  le_srv_read_char_acc_event_st stCharValAccEvntData;
#ifdef DEBUG_PRINT
  printf(" eBleSrv_NoUpdate_ReadCharValHndlr() \n");
#endif

  /* Wait for  TCU_LE_GATT_SER_READ_CHAR_VAL_EVENT */
  //eResTemp = eBleExtractData((uint8_t*)(stTcuEvent.ptEvent), (void*)(&stCharValEvent));
  eResTemp = eBleExtractData((uint8_t*)(tcu_event.ptEvent), (void*)(&stCharValEvent));

  if(eResTemp !=LE_API_SUCCCESS)
  {
      return eResTemp;
  }

  /* Check if received connection handle is the expected one */
  if((u16ConnHandle != CONN_HANDLE_ANY) && (stCharValEvent.u16ConnHandle!= u16ConnHandle))
  {
    return  LE_CONN_HNDLR;
  }

  /* Check if function pointer is initialized */
  if(NULL == pCharValUpdCallback)
  {
    return LE_NULL_PTR_ERR;
  }

  /* User Provided Callback function that should update the characteristic value */
  eResTemp = (*pCharValUpdCallback)(qHandle,
                                 stCharValEvent.u16ConnHandle,
                                 stCharValEvent.u16CharDescHandle);

  /* Check if update was successful */
  if(eResTemp !=LE_API_SUCCCESS)
  {
      return eResTemp;
  }


  /* Give go ahead for sending the value */
  eResFnc = eSetServerGattReadCharValAccept_Req(stCharValEvent.u16ConnHandle, LE_GATT_SUCCESS,stCharValEvent.u16CharDescHandle);

  if(eResFnc !=LE_FUNC_SUCCCESS)
  {
    return LE_API_ERR_DRV;
  }

  /* Wait for  TCU_LE_GATT_SER_READ_CHAR_VAL_ACCEPT_RESP */


  //eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SER_READ_CHAR_VAL_ACCEPT_RESP, TCU_MNG_NO_TIMEOUT, (void*)(&stCharValAccEvntData));
  eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SER_READ_CHAR_VAL_ACCEPT_RESP_t, TCU_MNG_NO_TIMEOUT, (void*)(&stCharValAccEvntData));

  if(eResTemp !=LE_API_SUCCCESS)
  {
      return eResTemp;
  }

  /* Check if response has been sent to client device */
  if (stCharValAccEvntData.eStatus != LE_GATT_SUCCESS)
  {
     return LE_API_ERR_EVENT;
  }
  else if(stCharValEvent.u16ConnHandle != stCharValAccEvntData.u16ConnHandle)
      {
        return LE_CONN_HNDLR;
      }

  /* Return connection handle */
  *pu16RetConnHandle =  stCharValAccEvntData.u16ConnHandle;

  return LE_API_SUCCCESS;
}

/******************************************************************************/
le_api_result_e eBleSrv_ReadCharDescriptorHndlr(void* qHandle, uint16_t u16ConnHandle, uint16_t* pu16RetConnHandle)
{
  le_func_status_e eResFnc;
  le_api_result_e eResTemp = LE_API_UNINITIALIZED;
  le_srv_read_char_val_event_st stCharDescEvent;
  le_srv_read_char_acc_event_st stCharDescAccEvntData;
#ifdef DEBUG_PRINT
  printf(" eBleSrv_NoUpdate_ReadCharValHndlr() \n");
#endif

  /* Wait for  TCU_LE_GATT_SER_READ_CHAR_DESP_EVENT */
  //eResTemp = eBleExtractData((uint8_t*)(stTcuEvent.ptEvent), (void*)(&stCharDescEvent));
  eResTemp = eBleExtractData((uint8_t*)(tcu_event.ptEvent), (void*)(&stCharDescEvent));

  if(eResTemp !=LE_API_SUCCCESS)
  {
      return eResTemp;
  }

  /* Check if received connection handle is the expected one */
  if((u16ConnHandle != CONN_HANDLE_ANY) && (stCharDescEvent.u16ConnHandle!= u16ConnHandle))
  {
    return  LE_CONN_HNDLR;
  }

  /* Check if function pointer is initialized */
  if(NULL == pCharValUpdCallback)
  {
    return LE_NULL_PTR_ERR;
  }

  /* User Provided Callback function that should update the characteristic value */
  eResTemp = (*pCharDescUpdCallback)(qHandle,
                                     stCharDescEvent.u16ConnHandle,
                                     stCharDescEvent.u16CharDescHandle);

  /* Check if update was successful */
  if(eResTemp !=LE_API_SUCCCESS)
  {
      return eResTemp;
  }

  /* Give go ahead for sending the value */
  eResFnc = eSetServerGattReadCharDescriptorAccept_Req(stCharDescEvent.u16ConnHandle, LE_GATT_SUCCESS, stCharDescEvent.u16CharDescHandle);

  if(eResFnc !=LE_FUNC_SUCCCESS)
  {
    return LE_API_ERR_DRV;
  }

  /* Wait for  TCU_LE_GATT_SER_READ_CHAR_DESP_ACCEPT_RESP */

  //eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SER_READ_CHAR_DESP_ACCEPT_RESP, TCU_MNG_NO_TIMEOUT, (void*)(&stCharDescAccEvntData));
  eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SER_READ_CHAR_DESP_ACCEPT_RESP_t, TCU_MNG_NO_TIMEOUT, (void*)(&stCharDescAccEvntData));

  if(eResTemp !=LE_API_SUCCCESS)
  {
      return eResTemp;
  }

  /* Check if response has been sent to client device */
  if (stCharDescAccEvntData.eStatus != LE_GATT_SUCCESS)
  {
     return LE_API_ERR_EVENT;
  }
  else if(stCharDescEvent.u16ConnHandle != stCharDescAccEvntData.u16ConnHandle)
      {
        return LE_CONN_HNDLR;
      }

  /* Return connection handle */
  *pu16RetConnHandle =  stCharDescAccEvntData.u16ConnHandle;

  return LE_API_SUCCCESS;
}

/******************************************************************************/
le_api_result_e eBleSrv_UpdateCharacteristic(void* qHandle, uint16_t u16CharEleHandle, uint16_t u16AttValLen, uint8_t* pu8AttVal)
{
  le_func_status_e eResFnc;
  le_api_result_e eResTemp = LE_API_UNINITIALIZED;
  gatt_status_code_e eStatus;

  /* Request update in database */
  eResFnc = eSetSdbGattUpdateCharacterEle_Req(u16CharEleHandle, u16AttValLen, pu8AttVal);

  if(eResFnc !=LE_FUNC_SUCCCESS)
  {
    return LE_API_ERR_DRV;
  }



  //eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SDB_UPD_CHAR_ELE_RESP, TCU_MNG_NO_TIMEOUT, (void*)(&eStatus));
  eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SDB_UPD_CHAR_ELE_RESP_t, TCU_MNG_NO_TIMEOUT, (void*)(&eStatus));

  if(eResTemp !=LE_API_SUCCCESS)
  {
      return eResTemp;
  }
  else if(eStatus != LE_GATT_SUCCESS)
      {
         return LE_API_ERR_DRV;
      }

  return eResTemp;
}

/******************************************************************************/
le_api_result_e eBleSrv_WriteCharDescriptorHndlr(void* qHandle,
                                                   uint16_t u16ConnHandle,
                                                   le_srv_write_char_desp_event_st* pstCharDespEvt)
{
  le_func_status_e eResFnc;
  gatt_status_code_e eStatus;
  le_srv_multi_acc_event_st stCharDesAccept;
  le_api_result_e eResTemp = LE_API_UNINITIALIZED;

#ifdef DEBUG_PRINT
  printf(" eBleSrv_WriteCharDescriptorHndlr() \n");
#endif

  /* Wait for  TCU_LE_GATT_SER_WRITE_CHAR_DESP_EVENT */
  //eResTemp = eBleExtractData((uint8_t*)(stTcuEvent.ptEvent), (void*)(pstCharDespEvt));
  eResTemp = eBleExtractData((uint8_t*)(tcu_event.ptEvent), (void*)(pstCharDespEvt));

  if(eResTemp !=LE_API_SUCCCESS)
  {
      return eResTemp;
  }

  /* Check if connection handle OK */
  if((u16ConnHandle != CONN_HANDLE_ANY) && (u16ConnHandle != pstCharDespEvt->u16ConnHandle))
  {
    return LE_CONN_HNDLR;
  }

  /* Update Character Element */
  eResFnc = eSetSdbGattUpdateCharacterEle_Req(pstCharDespEvt->u16CharDescHandle,
                                              pstCharDespEvt->u16CharValueLen,
                                              pstCharDespEvt->pu8CharDescValue);
  if(eResFnc !=LE_FUNC_SUCCCESS)
  {
    return LE_API_ERR_DRV;
  }


  //eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SDB_UPD_CHAR_ELE_RESP, TCU_MNG_NO_TIMEOUT, (void*)(&eStatus));
  eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SDB_UPD_CHAR_ELE_RESP_t, TCU_MNG_NO_TIMEOUT, (void*)(&eStatus));

  if(eResTemp !=LE_API_SUCCCESS)
  {
      return eResTemp;
  }

  if((eResFnc !=LE_FUNC_SUCCCESS) || (eStatus != LE_GATT_SUCCESS))
  {
    return LE_API_ERR_EVENT;
  }

  /* Signal to the server that update was OK.
     This will cause server to send reponse to client
     indicating the status of write request
  */
  eResFnc = eSetServerGattWriteCharDescAccept_Req(pstCharDespEvt->u16ConnHandle,LE_GATT_SUCCESS, pstCharDespEvt->u16CharDescHandle);

  if(eResFnc !=LE_FUNC_SUCCCESS)
  {
    return LE_API_ERR_DRV;
  }

  /* Wait for  TCU_LE_GATT_SER_WRITE_CHAR_DESP_ACCEPT_RESP */


  //eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SER_WRITE_CHAR_DESP_ACCEPT_RESP, TCU_MNG_NO_TIMEOUT, (void*)(&stCharDesAccept));
  eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SER_WRITE_CHAR_DESP_ACCEPT_RESP_t, TCU_MNG_NO_TIMEOUT, (void*)(&stCharDesAccept));

  if(eResTemp !=LE_API_SUCCCESS)
  {
      return eResTemp;
  }

  if(stCharDesAccept.eStatus != LE_GATT_SUCCESS)
  {
      return LE_API_ERR_EVENT;
  }
  else if(pstCharDespEvt->u16ConnHandle != stCharDesAccept.u16ConnHandle)
      {
          /* Response does not belong to latest request */
          return LE_CONN_HNDLR;
      }

  return LE_API_SUCCCESS;
}

/******************************************************************************/
le_api_result_e eBleSrv_WriteCharValueHndlr(void* qHandle,
                                              uint16_t u16ConnHandle,
                                              le_srv_write_char_desp_event_st* pstCharValEvt)
{
  le_func_status_e eResFnc;
  gatt_status_code_e eStatus;
  le_srv_multi_acc_event_st stCharValAccept;
  le_api_result_e eResTemp = LE_API_UNINITIALIZED;

#ifdef DEBUG_PRINT
  printf(" eBleSrv_WriteCharDescriptorHndlr() \n");
#endif

  /* Wait for  TCU_LE_GATT_SER_WRITE_CHAR_VAL_EVENT */
  //eResTemp = eBleExtractData((uint8_t*)(stTcuEvent.ptEvent), (void*)(pstCharValEvt));
  eResTemp = eBleExtractData((uint8_t*)(tcu_event.ptEvent), (void*)(pstCharValEvt));

  if(eResTemp !=LE_API_SUCCCESS)
  {
      return eResTemp;
  }

  /* Check if connection handle OK */
  if((u16ConnHandle != CONN_HANDLE_ANY) && (u16ConnHandle != pstCharValEvt->u16ConnHandle))
  {
    return LE_CONN_HNDLR;
  }

  /* Update Character Element */
  eResFnc = eSetSdbGattUpdateCharacterEle_Req(pstCharValEvt->u16CharDescHandle,
                                              pstCharValEvt->u16CharValueLen,
                                              pstCharValEvt->pu8CharDescValue);
  if(eResFnc !=LE_FUNC_SUCCCESS)
  {
    return LE_API_ERR_DRV;
  }



  //eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SDB_UPD_CHAR_ELE_RESP, TCU_MNG_NO_TIMEOUT, (void*)(&eStatus));
  eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SDB_UPD_CHAR_ELE_RESP_t, TCU_MNG_NO_TIMEOUT, (void*)(&eStatus));

  if(eResTemp !=LE_API_SUCCCESS)
  {
      return eResTemp;
  }

  if((eResFnc !=LE_FUNC_SUCCCESS) || (eStatus != LE_GATT_SUCCESS))
  {
    return LE_API_ERR_EVENT;
  }

  /* Signal to the server that update was OK. This will cause server to send
     reponse to client indicating the status of write request.
  */
  eResFnc = eSetServerGattWriteCharValAccept_Req(pstCharValEvt->u16ConnHandle,LE_GATT_SUCCESS, pstCharValEvt->u16CharDescHandle);

  if(eResFnc !=LE_FUNC_SUCCCESS)
  {
    return LE_API_ERR_DRV;
  }

  /* Wait for  TCU_LE_GATT_SER_WRITE_CHAR_VAL_ACCEPT_RESP */


  //eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SER_WRITE_CHAR_VAL_ACCEPT_RESP, TCU_MNG_NO_TIMEOUT, (void*)(&stCharValAccept));
  eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SER_WRITE_CHAR_VAL_ACCEPT_RESP_t, TCU_MNG_NO_TIMEOUT, (void*)(&stCharValAccept));

  if(eResTemp !=LE_API_SUCCCESS)
  {
      return eResTemp;
  }

  if(stCharValAccept.eStatus != LE_GATT_SUCCESS)
  {
      return LE_API_ERR_EVENT;
  }
  else if(pstCharValEvt->u16ConnHandle != stCharValAccept.u16ConnHandle)
      {
          /* Response does not belong to latest request */
          return LE_CONN_HNDLR;
      }

  return LE_API_SUCCCESS;
}


/******************************************************************************/
le_api_result_e eBleSrv_SendNotification(void* qHandle,
                                            uint16_t u16ConnHandle,
                                            le_srv_char_st* pstChar,
                                            uint8_t* pu8NewCharValue)
{
  le_func_status_e eResFnc;
  le_accept_st stEvntData;
  le_api_result_e eResTemp = LE_API_UNINITIALIZED;
  uint16_t u16RetConnHandle;

#ifdef DEBUG_PRINT
  printf(" eBleSrv_SendNotification() \n");
#endif

  /* Send notification request */
  eResFnc = eSetServerGattCharValNotification_Req(u16ConnHandle,
                                                  pstChar->u16RetCharValDeclHandle,
                                                  pstChar->stCharValDecl.u16AttValLen,
                                                  pu8NewCharValue);

  if(eResFnc !=LE_FUNC_SUCCCESS)
  {
    return LE_API_ERR_DRV;
  }

  /* Wait for  TCU_LE_ACCEPT */


  //eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_ACCEPT, TCU_MNG_TIMEOUT_1Sec, (void*)(&stEvntData));
  eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_ACCEPT_t, 0xFFFFFFFF, (void*)(&stEvntData));

  if(eResTemp !=LE_API_SUCCCESS)
  {
      return eResTemp;
  }

  // Check if everything went fine

//  if((stEvntData.u8OpCode == GET_CMD_OPCODE(TCU_LE_GATT_SER_CHAR_VAL_NOTIFICATION_REQ))||   0x05
  if((stEvntData.u8OpCode == 0x05)||
     //(stEvntData.u16ServiceId == GET_CMD_SERVICE_ID(TCU_LE_GATT_SER_CHAR_VAL_NOTIFICATION_REQ)) ||  0xD3
          (stEvntData.u16ServiceId == 0xD3) ||
      stEvntData.eStatus == LE_SUCCESS)
  {
    // Wait for  TCU_LE_GATT_SER_CHAR_VAL_NOTIFICATION_EVENT


    //eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SER_CHAR_VAL_NOTIFICATION_EVENT, TCU_MNG_TIMEOUT_10Sec, (void*)(&u16RetConnHandle));
      eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SER_CHAR_VAL_NOTIFICATION_EVENT_t, TCU_MNG_TIMEOUT_10Sec, (void*)(&u16RetConnHandle));

    if(eResTemp ==LE_API_SUCCCESS)
    {
      if(u16RetConnHandle == u16ConnHandle)
      {
          return LE_API_SUCCCESS;
      }
      else
      {
          return  LE_CONN_HNDLR;
      }
    }
  }
  return eResTemp;
}

/******************************************************************************/
le_api_result_e eBleSrv_SendIndication(void* qHandle,
                                          uint16_t u16ConnHandle,
                                          le_srv_char_st* pstChar,
                                          uint8_t* pu8NewCharValue)
{
  le_func_status_e eResFnc;
  le_accept_st stEvntData;
  le_api_result_e eResTemp = LE_API_UNINITIALIZED;
  le_srv_char_cal_ind_st stIndData;

#ifdef DEBUG_PRINT
  printf(" eBleSrv_SendNotification() \n");
#endif

  /* Send notification request */
  eResFnc = eSetServerGattCharValIndication_Req(u16ConnHandle,
                                                pstChar->u16RetCharValDeclHandle,
                                                pstChar->stCharValDecl.u16AttValLen,
                                                pu8NewCharValue);

  if(eResFnc !=LE_FUNC_SUCCCESS)
  {
    return LE_API_ERR_DRV;
  }

  /* Wait for  TCU_LE_ACCEPT */


  //eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_ACCEPT, TCU_MNG_NO_TIMEOUT, (void*)(&stEvntData));
  eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_ACCEPT_t, TCU_MNG_NO_TIMEOUT, (void*)(&stEvntData));

  if(eResTemp !=LE_API_SUCCCESS)
  {
      return eResTemp;
  }

  /* Check if everything went fine */
  //if((stEvntData.u8OpCode == GET_CMD_OPCODE(TCU_LE_GATT_SER_CHAR_VAL_INDICATION_REQ))|| 0x06
  if((stEvntData.u8OpCode == TCU_LE_GATT_SER_CHAR_VAL_INDICATION_REQ_t.opcode)||
     //(stEvntData.u16ServiceId == GET_CMD_SERVICE_ID(TCU_LE_GATT_SER_CHAR_VAL_INDICATION_REQ)) || 0xD3
     (stEvntData.u16ServiceId == TCU_LE_GATT_SER_CHAR_VAL_INDICATION_REQ_t.service_id) ||
      stEvntData.eStatus == LE_SUCCESS)
  {
    /* Wait for  TCU_LE_GATT_SER_CHAR_VAL_INDICATION_EVENT */


    //eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SER_CHAR_VAL_INDICATION_EVENT, TCU_MNG_TIMEOUT_10Sec, (void*)(&stIndData));
      eResTemp = eBleWaitForSpecificEvent(qHandle, TCU_LE_GATT_SER_CHAR_VAL_INDICATION_EVENT_t, TCU_MNG_TIMEOUT_10Sec, (void*)(&stIndData));

    if(eResTemp ==LE_API_SUCCCESS)
    {
      if(stIndData.eStatus == LE_GATT_SUCCESS)
      {
        /* Check if returned connection handle is OK */
        if(stIndData.u16ConnHandle == u16ConnHandle)
        {
            return LE_API_SUCCCESS;
        }
        else
        {
            return  LE_CONN_HNDLR;
        }
      }
      else
      {
        return LE_GATT_ERR;
      }
    }
  }

  return eResTemp;
}

/******************************************************************************/
le_api_result_e eBleSrv_DisconnectHndlr(le_disconnect_st* pstDisconnInfo)
{
  le_api_result_e eResTemp;

#ifdef DEBUG_PRINT
  printf(" eBleSrv_DisconnectHndlr() \n");
#endif

  //eResTemp = eBleExtractData((uint8_t*)(stTcuEvent.ptEvent), (void*)(pstDisconnInfo));
  eResTemp = eBleExtractData((uint8_t*)(tcu_event.ptEvent), (void*)(pstDisconnInfo));

  return eResTemp;
}

/******************************************************************************/
le_api_result_e eBleSrv_WriteWithoutRespHndlr(void* qHandle, le_srv_wrt_wo_evt_st* pstEvtData)
{
  le_api_result_e eResTemp;

#ifdef DEBUG_PRINT
  printf("eBleSrv_WriteWithoutRespHndlr() \n");
#endif

  /* Extract data returned from TCU_LE_GATT_SER_WRITE_WITHOUT_RESPONSE_CMD_EVENT */
  //eResTemp = eBleExtractData((uint8_t*)(stTcuEvent.ptEvent), (void*)(&pstEvtData));
  eResTemp = eBleExtractData((uint8_t*)(tcu_event.ptEvent), (void*)(&pstEvtData));

  if(eResTemp !=LE_API_SUCCCESS)
  {
      return eResTemp;
  }

  /* Update given characteristics */
  eResTemp = eBleSrv_UpdateCharacteristic(qHandle, pstEvtData->u16CharValHandle, pstEvtData->u16ValDataLen, pstEvtData->pu8CharValData);

  return eResTemp;
}

/******************************************************************************/
void vSmartProcessPacket(uint8_t* pu8Buff, volatile t_tcu_event* pstTcuEvent)
{
    pstTcuEvent->Service_ID = (eTCU_SERVICE_ID_)(PARSE_CMD_SERVICE_ID(pu8Buff));
    pstTcuEvent->eventType =  PARSE_CMD_OPCODE(pu8Buff);

    /* Data starts here */
    pstTcuEvent->ptEvent = (uint32_t *)(pu8Buff);

#ifdef DEBUG_PRINT
    uint16_t u16Count = (uint16_t)(pu8Buff[0]);
    vDebugRcvdCmdPrintf(pu8Buff, u16Count);
#endif
}


/************************* BEGIN of DISCLAIMER   *****************************

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


****************************  END of DISCLAIMER *****************************/
/********************************* BEGIN OF LOG *****************************
*
* $Log:  $
*
*********************************** END OF LOG *****************************/
