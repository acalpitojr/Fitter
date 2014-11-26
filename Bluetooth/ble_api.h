/****************************************************************************
*   COPYRIGHT 2006-2009 All rights reserved
*   Toshiba Electronics Europe GmbH
*   European LSI Design and Engineering Center (ELDEC)
*****************************************************************************
*   File Name   : $RCSfile: booting.h,v $
*   Description : Contains global booting definitions
*****************************************************************************
*   PROJECT     : Bluetooth BLE
*   MODULE      : BLE API
*               : $Name:  $
*   LIBRARIES   : None
*   AUTHOR      : $Author: arelic $
*****************************************************************************
*   VERSION     : $$
*   RELEASE     : Preliminary & Confidential
*   DATE        : $Date: $
*   SOURCE MOD. : $Source:$
*   LOG:        : please have a look at the end of the file
****************************************************************************/
#ifndef _SMART_API_H
#define _SMART_API_H
/**
* @file ble_api.h
* @brief Contains definitions and function declarattions
*
* @author ARE
*
* @date 02/05/2013
*/
/**********************************************
*                Include files                *
**********************************************/
#include "stdint.h"
#include "le_gatt_command.h"
#include "bt_uart_transport.h"

/**********************************************
*               Defs                          *
**********************************************/

/* Time constants */
#define TCU_MNG_TIMEOUT_1Sec    1000
#define TCU_MNG_TIMEOUT_10Sec   10000
#define TCU_MNG_TIMEOUT_30Sec   30000
#define TCU_MNG_TIMEOUT_60Sec   60000
#define TCU_MNG_NO_TIMEOUT      0xFFFFFFFF


/*  Connnection Handle used to tell a function
 *  not to restrict access to some specific
 *  connection handle.
 */
#define CONN_HANDLE_ANY        0xFFFF

/**********************************************
*               Enums                         *
**********************************************/

typedef enum tag_ble_api_result_e
{
  LE_API_SUCCCESS                      = 0x00,  /*!< Success */
  LE_API_ERR_DRV                       = 0x01,  /*!< Low_level GATT driver returned an error */
  LE_API_ERR_TIMEOUT                   = 0x02,  /*!< Queue Timeout reached */
  LE_API_ERR_RESPONSE                  = 0x03,  /*!< Response error */
  LE_API_ERR_INIT                      = 0x04,  /*!< LE device initialization failed */
  LE_API_ERR_EVENT                     = 0x05,  /*!< Unexpected event */
  LE_API_ERR_PARSING                   = 0x06,  /*!< Parsing error */
  LE_SER_INIT_ERROR                    = 0x07,  /*!< Server init. error */
  LE_PRIM_SERVICE_ERR                  = 0x08,  /*!< Service could not be instantiated */
  LE_GAP_SERVICE_ERR                   = 0x09,  /*!< GAP service error */
  LE_CONN_HNDLR                        = 0x0A,  /*!< Unexpected connection handler received */
  LE_USR_ABORT                         = 0x0B,  /*!< Aborted on user request */
  LE_NULL_PTR_ERR                      = 0x0C,  /*!< Null pointer */
  LE_SVC_TYPE_ERR                      = 0x0D,  /*!< Invalid service type */
  LE_GATT_ERR                          = 0x0E,  /*!< GATT response/event returned status error */
  LE_API_UNINITIALIZED                 = 0xFF   /*!< Non-initialized result state */

} le_api_result_e;


typedef enum tag_le_adv_svc_uuid_e
{
  LE_ADV_FLAGS                         = 0x01,  /*!< AD Flags. */
  LE_ADV_16_BIT_MORE                   = 0x02,  /*!< More 16-bit UUIDs available. */
  LE_ADV_16_BIT_COMPLETE               = 0x03,  /*!< Complete list of 16-bit UUIDs available. */
  LE_ADV_32_BIT_MORE                   = 0x04,  /*!< More 32-bit UUIDs available. */
  LE_ADV_32_BIT_COMPLETE               = 0x05,  /*!< Complete list of 32-bit UUIDs available. */
  LE_ADV_128_BIT_MORE                  = 0x06,  /*!< More 128-bit UUIDs available. */
  LE_ADV_128_BIT_COMPLETE              = 0x07,  /*!< Complete list of 128-bit UUIDs available. */
  LE_ADV_SHORT_NAME                    = 0x08,  /*!< Shortened local name. */
  LE_ADV_COMPLETE_NAME                 = 0x09,  /*!< Complete local name. */
  LE_ADV_TXPOWER_LEVEL                 = 0x0A,  /*!< TX Power Level (1 byte). */
  LE_ADV_CLASS_OF_DEV                  = 0x0D,  /*!< Class of device, Format defined in Assigned Numbers. */
  LE_ADV_SIMPLE_PAIRING_HC             = 0x0E,  /*!< Simple Pairing Hash C (16 octets)Format defined in [Vol. 2], Part H Section 7.2.2. */
  LE_ADV_SIMPLE_PAIRING_RAND           = 0x0F,  /*!< Simple Pairing Randomizer R(16 octets)Format defined in[Vol. 2], Part H Section 7.2.2. */
  LE_ADV_SM_TK_VALUE                   = 0x10,  /*!< TK Value Value as used in pairing over LE Physical channel. Format defined in [Vol. 3], Part H Section 2.3. */
  LE_ADV_SM_OOB_FLAGS                  = 0x11,  /*!< Security Manager OOB Flags (1 octet). */
  LE_ADV_SLV_CONN_INT_RANGE            = 0x12,  /*!< Slave Connection Interval Range. */
  LE_ADV_SVC_SOLICITATION_16_BIT       = 0x14,  /*!< Service Solicitation: Service UUIDs List of 16 bit Service UUIDs. */
  LE_ADV_SVC_SOLICITATION_128_BIT      = 0x15,  /*!< Service Solicitation: Service UUIDs List of 128 bit Service UUID. */
  LE_ADV_SERVICE_DATA                  = 0x16,  /*!< Service Data (2 or more octets) The first 2 octets contain the 16 bit Service UUID followed by additional service data. */
  LE_ADV_MANUF_SPECIFIC_DATA           = 0xFF   /*!< Manufacturer Specific Data (2 or more octets)
                                                     The first 2 octets contain the Company Identifier Code
                                                     followed by additional manufacturer specific data. */
} le_adv_svc_uuid_e;


/* Primary service respond structures */
typedef struct tag_le_srv_svc_resp_st
{
  sdb_status_code_e    eStatus;                 /*!< Status Value */
  uint16_t   u16Handle;                         /*!< Handle of added PRIMARY or SECONDARY service */
} le_srv_svc_resp_st;

/* Characteristic declatation response structure */
typedef struct tag_le_srv_char_decl_resp_st
{
  gatt_status_code_e eStatus;                   /*!< Returned status information */
  uint16_t u16Handle;                           /*!< Returned handle */
} le_srv_char_decl_resp_st;

/* Contains infromation required to add an INCLUDE to a Service */
typedef struct tag_le__srv_include_def_st
{
  uint16_t          u16ParentSvcHandle;         /*!< Handle of the service under which this INCLUDE should be added */
  uint16_t          u16InclSvcHandle;           /*!< Handle of included service */
  uint8_t           u16InclSvcUuid;             /*!< UUID of the included service */
  uint16_t          u16RetInclHandle;           /*!< Returned Include Service Handle (should not be modified by the user) */
} le_srv_include_def_st;

/* Contains infromation required to add an INCLUDE to a Service */
typedef struct tag_le_srv_char_val_st
{
    uint16_t          u16AttValLen;             /*!< Value length in bytes */
    uint8_t*          pu8AttVal;                /*!< Pointer to a value or an array */
    uint16_t          u16AttPermission;         /*!< Attribute permitions (please use att_permissions_e masks) */
} le_srv_char_val_st;


/* Contains infromation required to add a new characteristic to the database with belonging elements */
typedef struct tag_le_srv_char_st
{
  uint16_t  u16CharUUID;                        /*!< Characteristic UUID */
  uint8_t   u8CharProperties;                   /*!< Characteristic Properties (please use here combination of gatt_char_prop_mask_e flags) */
  le_srv_char_val_st stCharValDecl;                 /*!< Characteristic Value Declaration */

  uint8_t   u8CharDescCnt;                      /*!< Number of optional characteristic descriptors to be added (corresponds to pstCharDesc array) */
  char_el_req_st* pstCharDesc;                  /*!< Pointer to an array of characteristic descriptors to be added to the characteristic */

  uint16_t  u16RetCharValDeclHandle;            /*!< Characteristic VALUE Declaration Handle returned by GATT Server (should not be modified by the user) */
  uint16_t  u16RetCharDeclHandle;               /*!< Characteristic Declaration Handle returned by GATT Server (should not be modified by the user) */
  uint16_t* pu16RetCharDescHandles;             /*!< Pointer to an array of returned characteristic descriptor handles (should not be modified by the user) */
} le_srv_char_st;

/* Service Definition Structure */
typedef struct tag_le_srv_service_def_st
{
  uint16_t u16SvcUUID;                          /*!< UUID of the Service to be added */
  uint8_t u8InclDefCount;                       /*!< The number of Include definitions in the service */
  uint8_t u8CharDefCount;                       /*!< The number of Characteristic definitions in the service */
  le_srv_include_def_st* pstIncludes;               /*!< Pointer to the array of includes */
  le_srv_char_st*       pstChars;                  /*!< Pointer to the array of characteristics */
  uint16_t u16RetSvcHandle;                     /*!< Returned Service Handle from GATT Server (should not be modified by the user) */
} le_srv_service_def_st;


/*******************************************************************************
********************************************************************************
*             S E R V E R - S I D E    U S E R    F U N C T I O N S
********************************************************************************
*******************************************************************************/

/*******************************************************************************
*                                INITIALIZATION
*******************************************************************************/
/**
 * @brief      Initializes BLE API interface layer and registers callback functions.
 *
 * @param[in]  qHandle OS Message-Queue Handle.
 * @param[in]  pUnxEvtFunc Pointer to the callback function to be called when unexpected message arrives.
 * @param[in]  pCharValUpdFunc Pointer to the callback function to be called when Character value needs update.
 * @param[in]  pCharDescUpdFunc Pointer to the callback function to be called when Character descriptor needs update.
 * @param[in]  pUpdMultiFunc Pointer to the callback function to be called when Multiple values needs update.
 *
 * @return LE_API_SUCCCESS  Completed succesfully.
 * @return LE_NULL_PTR_ERR  Uninitialised pointer to the callback function given.
 */
le_api_result_e eBleApiInit(void* qHandle,
                               void (*pUnxEvtFunc)(uint8_t, uint8_t,uint16_t, uint8_t*),
                               le_api_result_e (*pCharValUpdFunc)(void*, uint16_t, uint16_t),
                               le_api_result_e (*pCharDescUpdFunc)(void*, uint16_t, uint16_t),
                               le_api_result_e (*pUpdMultiFunc)(void*, uint16_t, uint16_t, uint8_t*));

/**
 * @brief      Function initializes the BT Low-Energy device and GATT Server.
 *
 * @param[in]  qHandle OS Message-Queue Handle.
 * @param[in]  pu8DevName Pointer to Bluetooth device name.
 * @param[in]  u8DevNameLen Length (in bytes) of the Bluetooth name.
 * @param[out] pu8RetBd_Addr Returned Bluetooth Address (BD_ADDR).
 *
 * @return LE_API_SUCCCESS  Completed succesfully.
 * @return LE_USR_ABORT     User Aborted wait.
 * @return LE_API_ERR_INIT  LE Device initialization failed.
 * @return LE_SER_INIT_ERROR  LE GATT Server initialization failed.
 */
le_api_result_e eBleDeviceSrv_Init(void* qHandle,
                                      uint8_t* const pu8DevName,
                                      uint8_t u8DevNameLen,
                                      uint8_t* pu8RetBd_Addr);

/*******************************************************************************
*                           GAP (CONNECTION & ADVERTIZING)
*******************************************************************************/
/**
 * @brief      Disconnects from given Bluetooth LE device.
 *
 * @note       If the function returns successfully, you are disconnected.
 *
 * @param[in]  qHandle OS Message-Queue Handle.
 * @param[in]  pu8Bd_Addr BD_ADDR from LE device from which we should disconnect.
 * @param[in]  pu16RetConnHandle Retourned connection handler
 * @param[out] pu8RetBd_Addr Returned Bluetooth Address (BD_ADDR).
 *
 * @return LE_API_SUCCCESS  Completed succesfully.
 * @return LE_USR_ABORT     User Aborted wait.
 * @return LE_API_ERR_DRV   Low-level driver request failed.
 * @return LE_SER_INIT_ERROR  LE GATT Server initialization failed.
 */
le_api_result_e eBleDevice_Disconnect(void* qHandle,
                                         uint8_t* const pu8Bd_Addr,
                                         uint16_t* pu16RetConnHandle);
/**
 * @brief      Configures and starts advertizing procedure.
 *
 * @note       After this function call, the device is visible to other LE devices
 *             performing the scan procedure.
 *
 * @param[in]  qHandle OS Message-Queue Handle
 * @param[in]  stConfig Advertizing configuration structure
 *
 * @return LE_API_SUCCCESS  Completed succesfully.
 * @return LE_USR_ABORT     User Aborted wait.
 * @return LE_API_ERR_DRV   Low-level driver request failed.
 * @return LE_API_ERR_EVENT Advertize response returned error.
*/
le_api_result_e eBlePerif_StartAdv(void* qHandle,
                                      le_adv_req_st stConfig);

/**
 * @brief      Stops advertizing procedure.
 *
 * @note       After this function returns with LE_API_SUCCCESS, it is not anymore
 *             visible by  central devices doing scan procedure.
 *
 * @param[in]  qHandle OS Message-Queue Handle
 * @param[in]  stConfig Advertizing configuration structure
 *
 * @return LE_API_SUCCCESS  Completed succesfully.
 * @return LE_USR_ABORT     User Aborted wait.
 * @return LE_API_ERR_DRV   Low-level driver request failed.
 * @return LE_API_ERR_EVENT Advertize response returned error.
*/
le_api_result_e eBlePerif_StopAdv(void* qHandle);


/*******************************************************************************
*                          GATT-DATABASE OPERATIONS
*******************************************************************************/
/**
 * @brief     Registers given Primary service to GATT database with given parameters.
 *
 *
 * @param[in]  qHandle OS Message-Queue Handle.
 * @param[in,out] pstSvcDef Definition of the PRIMARY service to be added. Please note that the
 *                          service handle structure member together with all characteristic handles
 *                          will be updated by this function.
 *
 * @return LE_API_SUCCCESS  Completed succesfully.
 * @return LE_USR_ABORT     User Aborted wait.
 * @return LE_API_ERR_DRV   Low-level driver request failed.
 * @return LE_API_ERR_EVENT Primary service response returned an error.
 */
le_api_result_e eBleSrv_RegisterPrimaryService(void* qHandle,
                                                  le_srv_service_def_st* pstSvcDef);

/**
 * @brief     Registers given secondary service to GATT database with given parameters.
 *
 * @param[in]  qHandle OS Message-Queue Handle.
 * @param[in]  pstParentSvcDef  Pointer to definition of the PRIMARY or SECONDARY service
 *                              under which this SECONDARY service should be added.
 * @param[in,out] pstSvcDef Definition of the SECONDARY service to be added. Please note that the
 *                          service handle structure member together with all characteristic handles
 *                          will be updated by this function.
 *
 * @return LE_API_SUCCCESS  Completed succesfully.
 * @return LE_USR_ABORT     User Aborted wait.
 * @return LE_API_ERR_DRV   Low-level driver request failed.
 * @return LE_API_ERR_EVENT Primary service response returned an error.
 */
le_api_result_e eBleSrv_RegisterSecondaryService(void* qHandle,
                                                    le_srv_service_def_st* pstParentSvcDef,
                                                    le_srv_service_def_st* pstSecSvcDef);

/**
 * @brief     Updates or changes the attribute value of the Characteristic Elements.
 *
 * @note      Characteristic elements are following
 *            1) Characteristic Value Declaration,
 *            2) Characteristic Descriptor Declaration, etc...
 *
 * @param[in]  qHandle OS Message-Queue Handle.
 * @param[in]  u16CharEleHandle Handle of the Character Element to be updated
 * @param[in]  u16AttValLen Length of the Attribute Value (in bytes)
 * @param[in]  pu8AttVal Pointer to Attribute value
 *
 * @return LE_API_SUCCCESS  Completed succesfully.
 * @return LE_USR_ABORT     User Aborted wait.
 * @return LE_API_ERR_DRV   Low-level driver request failed.
 */
le_api_result_e eBleSrv_UpdateCharacteristic(void* qHandle,
                                                uint16_t u16CharEleHandle,
                                                uint16_t u16AttValLen,
                                                uint8_t* pu8AttVal);

/**
 * @brief      Returns an end handle of the service specified by u16SvcHandle.
 *
 * @note       Each registered serivice has its Service Handle  (here specified
 *             by u16SvcHandle), which is at the same time the start handle and
 *             occupies a certain range of handles (depending on its elements).
 *             This function returns the "end handle" of the service specified
 *             by the u16SvcHandle parameter.
 *
 * @param[in]  qHandle OS Message-Queue Handle.
 * @param[in]  u16SvcHandle Handle of the service for which end handle is required
 * @param[in]  pu16EndGroupHandle Returned end group handle
 *
 * @return LE_API_SUCCCESS  Completed succesfully.
 * @return LE_USR_ABORT     User Aborted wait.
 * @return LE_API_ERR_DRV   Low-level driver request failed.
 */
le_api_result_e eBleSrv_GetEndGroupHandle(void* qHandle,
                                             uint16_t u16SvcHandle,
                                             uint16_t* pu16EndGroupHandle);

/*******************************************************************************
*                        SERVER/PERIPHERAL EVENT HANDLING
*******************************************************************************/

/**
 * @brief      Handles connection request sent by a client.
 *
 *             This function should be called when TCU_MNG_LE_CONNECTION_COMPLETE_EVENT
 *             is received. It will extract connection information and return it back
 *             via given pointer parameter.
 *
 * @param[out] pstConnectInfo Pointer to a structure where connection data should be stored.
 *
 * @return LE_API_SUCCCESS  Completed succesfully
 * @return LE_API_ERR_EVENT Unexpected event received
 */
le_api_result_e eBleSrv_ClientConnRequestHndlr(le_conn_evt_st* pstConnectInfo);

/**
 * @brief      Handles disconnect event.
 *
 *             This function should be called when TCU_MNG_LE_DISCONNECT_EVENT
 *             is received. It will extract connection information and return it back
 *             via given pointer parameter.
 *
 * @param[out] pstDisconnInfo Pointer to a structure where disconnection data should be stored.
 *
 * @return LE_API_SUCCCESS  Completed succesfully
 * @return LE_API_ERR_EVENT Unexpected event received
 */
le_api_result_e eBleSrv_DisconnectHndlr(le_disconnect_st* pstDisconnInfo);

/**
 * @brief      Handles MTU exchange request from client.
 *
 *             This function should be called when TCU_LE_GATT_SER_EXG_MTU_EVENT
 *             is received. If this function returns successfully, it means the MTU exchange was successful
 *             and the minimum of both supported sizes is taken for further data exchange.
 *
 * @param[in]  qHandle OS Message-Queue Handle
 * @param[out] pstMtuClientData - Pointer to Client MTU data.
 * @param[out] pstMtuSrvData - Pointer to Server MTU data.
 *
 * @return LE_API_SUCCCESS  Completed succesfully.
 * @return LE_USR_ABORT     User Aborted wait.
 * @return LE_CONN_HNDLR    Unexpected connection handler received.
 * @return LE_API_ERR_DRV   Low-level driver request failed.
 * @return LE_API_ERR_EVENT Response returned an error.
 */
le_api_result_e eBleSrv_ClientMtuExchangeHndlr(void* qHandle,
                                                  uint16_t u16ConnHandle,
                                                  le_mtu_exchg_st* pstMtuClientData,
                                                  le_mtu_exchg_acc_resp_st* pstMtuSrvData);

/**
 * @brief      Handles read multi event, from a specific or any BT device.
 *
 *             This function should be called when TCU_LE_GATT_SER_READ_MULTIPLE_EVENT
 *             is received.
 *             The function will call user registered pUpdMultiCallback function that
 *             gives user possibility to update all or some of the values, whose handles
 *             are passed through. The list of handles that should be updated is passed
 *             to a callback pUpdMultiFunc function.
 *
 * @note       If as an input for u16ConnHandle parameter the value CONN_HANDLE_ANY is given,
 *             event comming from any device is accepted and handled. If however any valid
 *             connection handler is given (other than CONN_HANDLE_ANY), only read multi
 *             event comming from that device will be processed.
 *
 *
 * @param[in]  qHandle OS Message-Queue Handle
 * @param[in]  u16ConnHandle Connection handle from which messagess should be accepted.
 *                           If CONN_HANDLE_ANY is given, event comming from any device will be accepted.
 * @param[out] pu16RetConnHandle Return connection handler, used to identify device from which event is comming.
 *                               Usefull in case events from any device are accepted.
 *
 * @return LE_API_SUCCCESS  Completed succesfully.
 * @return LE_USR_ABORT     User Aborted wait.
 * @return LE_CONN_HNDLR    Unexpected connection handler received.
 * @return LE_API_ERR_DRV   Low-level driver request failed.
 * @return LE_API_ERR_EVENT Response returned an error.
 */
le_api_result_e eBleSrv_ReadMultiEventHndlr(void* qHandle,
                                               uint16_t u16ConnHandle,
                                               uint16_t* pu16RetConnHandle);

/**
 * @brief      Handles Requests for Reading Character Value.
 *
 *             This function should be called when TCU_LE_GATT_SER_READ_CHAR_VAL_EVENT
 *             is received.
 *             The function will call user registered pCharValUpdCallback function that gives an
 *             application possibility to  update the characteristic value whose handle is passed through.
 *
 * @note       After receiving the request, the function checks if connection handle matches with the one given by the user.
 *             If yes or if user specified CONN_HANDLE_ANY, than it gives GATT server go ahead
 *             for sending the data. When this function completes successfully, it means
 *             the response has been sent to the client device.
 *
 * @param[in]  qHandle OS Message-Queue Handle
 * @param[in]  u16ConnHandle Connection handle from which messagess should be accepted.
 *                           If CONN_HANDLE_ANY is given, event comming from any device will be accepted.
 * @param[out] pu16RetConnHandle Return connection handler, used to identify device from which event is comming.
 *                               Usefull in case events from any device are accepted.
 *
 * @return LE_API_SUCCCESS  Completed succesfully.
 * @return LE_USR_ABORT     User Aborted wait.
 * @return LE_CONN_HNDLR    Unexpected connection handler received.
 * @return LE_API_ERR_EVENT Response returned an error.
 */
le_api_result_e eBleSrv_ReadCharValHndlr(void* qHandle,
                                            uint16_t u16ConnHandle,
                                            uint16_t* pu16RetConnHandle);
/**
 * @brief      Handles Requests for Reading Character Descriptor.
 *
 *             This function should be called when TCU_LE_GATT_SER_READ_CHAR_DESP_EVENT
 *             is received.
 *             The function will call user registered pCharDescUpdCallback function that gives an
 *             application possibility to  update the descriptor, whose handle is passed through.
 *
 * @note       After receiving the request, the function checks if connection handle matches with the one given by the user.
 *             If yes or if user specified CONN_HANDLE_ANY, than it gives GATT server go ahead
 *             for sending the data. When this function completes successfully, it means
 *             the response has been sent to the client device.
 *
 *             Please also note taht for Client Characteristic Configuration Descriptor the Server Application
 *             must write the value stored for the corresponding Client requesting the read (the connection handle information
 *             must be used to identify the client for which descriptor must be updated). It is responsibility of the Server Application
 *             to store the separate instance of Client Characteristic Configuration for each GATT Client, when write descriptor is requested.
 *             The corresponding Client descriptor value must be updated into the database when read is requested. For Characteristic Descriptors
 *             which hold constant values, the value need not be updated in server database.
 *
 * @param[in]  qHandle OS Message-Queue Handle
 * @param[in]  u16ConnHandle Connection handle from which messagess should be accepted.
 *                           If CONN_HANDLE_ANY is given, event comming from any device will be accepted.
 * @param[out] pu16RetConnHandle Return connection handler, used to identify device from which event is comming.
 *                               Usefull in case events from any device are accepted.
 *
 * @return LE_API_SUCCCESS  Completed succesfully.
 * @return LE_USR_ABORT     User Aborted wait.
 * @return LE_CONN_HNDLR    Unexpected connection handler received.
 * @return LE_API_ERR_EVENT Response returned an error.
 */
le_api_result_e eBleSrv_ReadCharDescriptorHndlr(void* qHandle,
                                                  uint16_t u16ConnHandle,
                                                  uint16_t* pu16RetConnHandle);

/**
 * @brief      Handles write characteristic DESCRIPTOR request from the client device,
 *             by updating descriptor as requested by the client. The information about what
 *             value has been updated and how is returned through pstCharValEvt parameter.
 *
 *             This function should be called when TCU_LE_GATT_SER_WRITE_CHAR_DESP_EVENT
 *             is received.
 *
 * @note       If sufficient permission (like write , authorization etc) to write
 *             the characteristic descriptor is not available for the specified
 *             characteristic descriptor handle, then request event will not be sent to
 *             server application and directly GATT server will send the response with
 *             appropriate error code to the client device. In such case the function will
 *             stay in a waiting state until abort signal is sent by the application.
 *
 * @param[in]  qHandle OS Message-Queue Handle
 * @param[in]  u16ConnHandle Connection handle from which messagess should be accepted.
 *                           If CONN_HANDLE_ANY is given, event comming from any device will be accepted.
 * @param[out] pstCharDespEvt Pointer to the structure that will contain returned write request information.
 * @param[out]
 *
 * @return LE_API_SUCCCESS  Completed succesfully.
 * @return LE_USR_ABORT     User Aborted wait.
 * @return LE_CONN_HNDLR    Unexpected connection handler received.
 * @return LE_API_ERR_EVENT Response returned an error.
 */
le_api_result_e eBleSrv_WriteCharDescriptorHndlr(void* qHandle,
                                                   uint16_t u16ConnHandle,
                                                   le_srv_write_char_desp_event_st* pstCharDespEvt);

/**
 * @brief      Handles write characteristic VALUE request from the client device,
 *             by updating the value as requested by the client. The information about what
 *             value has been updated and how is returned through pstCharValEvt parameter.
 *
 *             This function should be called when TCU_LE_GATT_SER_WRITE_CHAR_VAL_EVENT
 *             is received.
 *
 * @note       If sufficient permission (like write , authorization etc) to write
 *             the characteristic descriptor is not available for the specified
 *             characteristic descriptor handle, then request event will not be sent to
 *             server application and directly GATT server will send the response with
 *             appropriate error code to the client device. In such case the function will
 *             stay in a waiting state until abort signal is sent by the application.
 *
 * @param[in]  qHandle OS Message-Queue Handle
 * @param[in]  u16ConnHandle Connection handle from which messagess should be accepted.
 *                           If CONN_HANDLE_ANY is given, event comming from any device will be accepted.
 * @param[out] pstCharValEvt Pointer to the structure that will contain returned write request information.
 * @param[out]
 *
 * @return LE_API_SUCCCESS  Completed succesfully.
 * @return LE_USR_ABORT     User Aborted wait.
 * @return LE_CONN_HNDLR    Unexpected connection handler received.
 * @return LE_API_ERR_EVENT Response returned an error.
 */
le_api_result_e eBleSrv_WriteCharValueHndlr(void* qHandle,
                                              uint16_t u16ConnHandle,
                                              le_srv_write_char_desp_event_st* pstCharValEvt);

/**
 * @brief      Handles write write without response event.
 *
 *             This function should be called when TCU_LE_GATT_SER_WRITE_WITHOUT_RESPONSE_CMD_EVENT
 *             is received.
 *             This event is generated by GATT Server to inform the Server Application
 *             that the Write Without Response  Command has been received from the Client Device.
 *             Handler parses the data from the event and according to this updates
 *             all appropriate characteristic values whose update was requested by a
 *             client device
 *
 *
 * @param[in]  qHandle OS Message-Queue Handle
 * @param[out] pstEvtData Pointer to structure that will hold received event data.
 *                        After the function completes, the caller can check in this structure
 *                        what characteristic was updated and with what value.
 * @param[out]
 *
 * @return LE_API_SUCCCESS  Completed succesfully.
 * @return LE_USR_ABORT     User Aborted wait.
 * @return LE_API_ERR_EVENT Response returned an error.
 */
le_api_result_e eBleSrv_WriteWithoutRespHndlr(void* qHandle,
                                                 le_srv_wrt_wo_evt_st* pstEvtData);


/*******************************************************************************
*                           NOTIFICATION/INDICATION
*******************************************************************************/
/**
 * @brief      Sends notification to a client side.
 *
 *             Sends notification to the specified client.
 *             When the function returns with LE_API_SUCCCESS, the notification has been
 *             successfully sent to the client.
 *
 * @param[in]  qHandle OS Message-Queue Handle.
 * @param[in]  u16ConnHandle Connection handle for the device to be notified.
 * @param[in]  pstChar Pointer to characteristic that should be notified.
 * @param[in]  pu8NewCharValue Pointer to the new characteristic VALUE that should be notified.
 *                             Please note that the value that is sent here is pu8NewCharValue
 *                             and not from the pstValChar.
 *
 * @return LE_API_SUCCCESS  Completed succesfully.
 * @return LE_USR_ABORT     User Aborted wait.
 * @return LE_API_ERR_DRV   Low-level driver request failed.
 * @return LE_CONN_HNDLR    Unexpected connection handler received.
 */
le_api_result_e eBleSrv_SendNotification(void* qHandle,
                                            uint16_t u16ConnHandle,
                                            le_srv_char_st* pstChar,
                                            uint8_t* pu8NewCharValue);

/**
 * @brief      Sends Indication to a client side.
 *
 *             When the function returns with LE_API_SUCCCESS, the indication has been
 *             successfully sent to the client.
 *
 * @param[in]  qHandle OS Message-Queue Handle.
 * @param[in]  u16ConnHandle Connection handle for the device to be indicated.
 * @param[in]  pstChar Pointer to the characteristic that should be indicated.
 * @param[in]  pu8NewCharValue Pointer to the new characteristic VALUE that should be indicated.
 *                             Please note that the value that is sent here is pu8NewCharValue
 *                             and not from the pstValChar.
 *
 * @return LE_API_SUCCCESS  Completed succesfully.
 * @return LE_USR_ABORT     User Aborted wait.
 * @return LE_API_ERR_DRV   Low-level driver request failed.
 * @return LE_GATT_ERR      Error status returned after indication request.
 * @return LE_CONN_HNDLR    Unexpected connection handler received.
 */
le_api_result_e eBleSrv_SendIndication(void* qHandle,
                                            uint16_t u16ConnHandle,
                                            le_srv_char_st* pstChar,
                                            uint8_t* pu8NewCharValue);

/*******************************************************************************
*                           CALLBACK OPERATIONS
*******************************************************************************/
/**
 * @brief      Registers a user defined callback function to be called in case an undexpected
 *             event comes.
 *
 *             In such way the application is given chance to react/process event that
 *             was not expected at the SMART API level.
 *
 * @note       This callback funtion is already registered  by eBleApiInit() during
 *             initialization time, and normally eBleRegisterCallback_UnexpEvnt() function
 *             doesn't need to be called, unless a user wants to change the callback function
 *             during the operation time.
 *
 *
 * @param[in]  qHandle OS Message-Queue Handle.
 * @param[out] pCallback Callback function to be called when unexpected event arrives.
 *                       Parameters of callback function are all of uint8_t type:
 *                       Param 1 (uint8_t): Received Service ID.
 *                       Param 2 (uint8_t): Received Command Op-Code.
 *                       Param 3 (uint8_t): Command Param Length.
 *                       Param 4 (uint8_t): Pointer to the command receive buffer. Points
 *                                to the first command parameter, right after the
 *                                command parameter length.
 *
 * @return LE_API_SUCCCESS  Completed succesfully.
 * @return LE_NULL_PTR_ERR  Uninitialised pointer to the callback function given.
 */
le_api_result_e  eBleRegisterCallback_UnexpEvnt(void (*pCallback)(uint8_t, uint8_t,uint16_t, uint8_t*));

/**
 * @brief      Registers a user defined callback function to be called when Characteristic VALUE Update
 *             is requested by a GATT server.
 *
 *
 * @note       This callback funtion is already registered  by eBleApiInit() during
 *             initialization time, and normally eBleRegisterCallback_UnexpEvnt() function
 *             doesn't need to be called unless a user wants to change the callback function
 *             during an operation time.
 *
 * @param[in]  qHandle OS Message-Queue Handle.
 * @param[out] pCallback Callback function to be called characteristic update is required.
 *                       User must make sure that the current value of the given characteristic is
 *                       obtained and that eBleSrv_UpdateCharacteristic() is called to update
 *                       the requested characteristic element .
 *
 *                       Parameters of callback function are as follows:
 *                       Param 1 (void*): OS Queue Handle
 *                       Param 2 (uint16_t): Connection Handle.
 *                       Param 3 (uint16_t): Characteristic descriptor handle
 *                                           of the element that must be updated.
 *
 * @return LE_API_SUCCCESS  Completed succesfully.
 * @return LE_NULL_PTR_ERR  Uninitialised pointer to the callback function given.
 */
le_api_result_e  eBleRegisterCallback_CharValUpdate(le_api_result_e (*pCallback)(void*, uint16_t, uint16_t));

/**
 * @brief      Registers a user defined callback function to be called when Characteristic
 *             DESCRIPTOR Update is requested by a GATT server.
 *
 * @note       This callback funtion is already registered  by eBleApiInit() during
 *             initialization time, and normally eBleRegisterCallback_UnexpEvnt() function
 *             doesn't need to be called unless a user wants to change the callback function
 *             during an operation time.
 *
 * @param[in]  qHandle OS Message-Queue Handle.
 * @param[out] pCallback Callback function to be called characteristic update is required.
 *                       User must make sure that the current value of the given characteristic is
 *                       obtained and that eBleSrv_UpdateCharacteristic() is called to update
 *                       the requested characteristic element .
 *
 *                       Parameters of callback function are as follows:
 *                       Param 1 (void*): OS Queue Handle
 *                       Param 2 (uint16_t): Connection Handle.
 *                       Param 3 (uint16_t): Characteristic descriptor handle
 *                                           of the element that must be updated.
 *
 * @return LE_API_SUCCCESS  Completed succesfully.
 * @return LE_NULL_PTR_ERR  Uninitialised pointer to the callback function given.
 */
le_api_result_e  eBleRegisterCallback_CharDescUpdate(le_api_result_e (*pCallback)(void*, uint16_t, uint16_t));

/**
 * @brief      Registers a user defined callback function to be update of multiple values
 *             is requested.
 *
 * @note       This callback funtion is already registered  by eBleApiInit() during
 *             initialization time, and normally eBleRegisterCallback_UnexpEvnt() function
 *             doesn't need to be called unless a user wants to change the callback function
 *             during an operation time.
 *
 * @param[in]  qHandle OS Message-Queue Handle.
 * @param[out] pCallback Callback function to be called when multiple values need to be updated.
 *                       User must make sure that the current value of the given characteristic is
 *                       obtained and that eBleSrv_UpdateCharacteristic() is called to update all
 *                       requested characteristic elements.
 *                       Parameters of callback function are all of type:
 *                       Param 1 (void*) : OS Queue handle.
 *                       Param 2 (uint16_t) : Connection handle for which the update is required.
 *                       Param 3 (uint16_t) : Number of handles in a list.
 *                       Param 4 (uint8_t*) : Pointer to the list of handles whose values need to be
 *                                            updated.
 *
 * @return LE_API_SUCCCESS  Completed succesfully.
 * @return LE_NULL_PTR_ERR  Uninitialised pointer to the callback function given.
 */
//le_api_result_e  eBleRegisterCallback_MultiUpdate(le_api_result_e (*pCallback)(void*, uint16_t, uint16_t, uint8_t*));
enum tag_ble_api_result_e        eBleRegisterCallback_MultiUpdate(enum tag_ble_api_result_e (*pCallback)(void*, uint16_t, uint16_t, uint8_t*));


/*******************************************************************************
********************************************************************************
*         H E L P E R   F U N C T I O N S  (Should not be used directly)
********************************************************************************
*******************************************************************************/

/**
 * @brief     Adds primary service attributes to the server database
 *
 * @param[in]  qHandle OS Message-Queue Handle
 * @param[in]  u16UUID Short (16-bit) UUID of the service to be registered
 * @param[out] pu16Handle Pointer to store returned service handle.
 *             This handle shall be used for all subsequent operations related to this particular service.
 *
 * @return LE_API_SUCCCESS  Completed succesfully
 * @return LE_USR_ABORT     User Aborted wait
 * @return LE_API_ERR_DRV   Low-level driver request failed.
 * @return LE_API_ERR_EVENT Primary service response returned an error.
 */
le_api_result_e eBleSrv_AddPrimServiceAttributes(void* qHandle,
                                                    uint16_t u16UUID,
                                                    uint16_t* pu16Handle);

/**
 * @brief     Adds secondary service attributes to the server database
 *
 * @param[in]  qHandle OS Message-Queue Handle
 * @param[in]  u16UUID Short (16-bit) UUID of the service to be registered
 * @param[out] pu16Handle Pointer to store returned service handle.
 *             This handle shall be used for all subsequent operations related to this particular service.
 *
 * @return LE_API_SUCCCESS  Completed succesfully
 * @return LE_USR_ABORT     User Aborted wait
 * @return LE_API_ERR_DRV   Low-level driver request failed.
 * @return LE_API_ERR_EVENT Primary service response returned an error.
 */
le_api_result_e eBleSrv_AddSecServiceAttributes(void* qHandle,
                                                   uint16_t u16ParentSvcHandle,
                                                   uint16_t u16UUID,
                                                   uint16_t* pu16Handle);

 /**
 * @brief      Adds the characteristic to given service and returns Characteristic handles.
 *
 *             This function will add characteristic declaration and all given elements (e.g. value or descriptors),
 *             whereby the service to which the characteristig belongs is specified by u16SvcHandle.
 *             Please note that this function will register any given number of characteristic elements.
 *
 * @param[in]  qHandle OS Message-Queue Handle
 * @param[in]  u16SvcHandle Service handle to which the characteristic belongs
 * @param[in]  stCharac Characteristic structure containing all parameters for given characteristic
 * @param[out] pu16RetHandleChar Pointer to location to write returned characteristic declaration handle
 * @param[out] pu16RetHandleCharEle Pointer to location to write returned characteristic element handles.
 *             If the function completes successfully, the actual number of returned handles is the same
 *             as the number of elements given at stCharac.u8CharElCnt. Please note that the user must
 *             allocate adequate amount of memory to store all element handles.
 *
 * @return LE_API_SUCCCESS  Completed succesfully.
 * @return LE_USR_ABORT     User Aborted wait.
 * @return LE_API_ERR_DRV   Low-level driver request failed.
 * @return LE_API_ERR_EVENT Response returned an error.
 */
le_api_result_e eBleSrv_AddCharacteristic(void* qHandle,
                                             uint16_t u16SvcHandle,
                                             le_srv_char_st* pstCharac,
                                             uint16_t* pu16RetHandleChar,
                                             uint16_t* pu16RetHandleCharEle);

 /**
 * @brief      Adds INCLUDE to a given service.
 *
 * @param[in]  qHandle OS Message-Queue Handle.
 * @param[in]  u16ParentSvcHandle Service handle under which the INCLUDE should be added.
 * @param[in]  u16InclSvcHandle Service handle of the service that is being included.
 * @param[in]  u16InclSvcUuid UUID of the Service that is being included.
 * @param[out] pu16RetIncSvcHandle Pointer where returned INCLUDE handle should be written.
 *
 * @return LE_API_SUCCCESS  Completed succesfully.
 * @return LE_USR_ABORT     User Aborted wait.
 * @return LE_API_ERR_DRV   Low-level driver request failed.
 * @return LE_API_ERR_EVENT Response returned an error.
 */
le_api_result_e eBleSrv_AddIncludeToSvc(void* qHandle,
                                           uint16_t u16ParentSvcHandle,
                                           uint16_t u16InclSvcHandle,
                                           uint16_t u16InclSvcUuid,
                                           uint16_t* pu16RetIncSvcHandle);

/**
 * @brief      Callback function used by message reception routine. It prepares the
 *             TCU callback event structure.
 *
 * @param[in]  qHandle OS Message-Queue Handle
 * @param[out] pstTcuEvent Contains information about received event
 *
 */
void vSmartProcessPacket(uint8_t* pu8Buff,
                         volatile t_tcu_event* pstTcuEvent);

/**
 * @brief      Waits for any event to be recieved. To be used by higher level SW
 *             to wait for an event. Please note that this function will suspend calling
 *             task until some message from Chiron arrives.
 *
 * @param[in]  qHandle OS Message-Queue Handle
 * @param[out] pu16Cmd Pointer to location where received ServiceID will be stored
 */
le_api_result_e  vSmartWaitForAnyEvent(void* qHandle,
                            uint16_t* pu16Cmd);

#endif /* _SMART_API_H */

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
* $Log: $
*
*********************************** END OF LOG *****************************/
