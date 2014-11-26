
#ifndef _MY_PROFILE_H
#define _MY_PROFILE_H


/**********************************************
*                Include files                *
**********************************************/
#include "stdint.h"
#include "le_gatt_command.h"
#include "ble_api.h"

/**********************************************
*               Defs                          *
**********************************************/
/* APPEARANCE value for H/R sensor */
#define MP_APPEARANCE 833


/**********************************************
*               Enums                         *
**********************************************/
/* GAP UUID */
typedef enum tag_gap_uuid_e
{
    GAP_SERVICE                        = 0x1800,
    GAP_DEV_NAME                       = 0x2A00,
    APPEARANCE                         = 0x2A01
} gap_uuid_e;

/* GATT UUID */
typedef enum tag_gatt_uuid_e
{
    GATT_DEV_INFO                      = 0x180A, /* Device Information Service */
    GATT_CHAR_MANUF_NAME_STRING        = 0x2A29, /* Manufacturer String Characteristic */
    GATT_CONTROLLER_UUID                    = 0xAABB, /* made up Service uuid*/
    GATT_CONTROLLER_DATA_UUID        = 0xAACC, /* controller Characteristic uuid made up*/
    GATT_CLI_CHARACT_CONFIGURATION     = 0x2902  /* Client characteristic configuration */

} gatt_uuid_e;

/**********************************************
*               Structs                       *
**********************************************/
/* Heart Rate initialisation structure */
typedef struct tag_hr_init_st
{
  uint8_t*  pu8DevName;                                 /* Device Name of the Heart Rate Sensor */
  uint8_t   u8DevNameLen;                               /* Length of the Device Name */
  uint8_t*  pu8ManufName;                               /* Manufacturer Name of the Heart Rate Sensor */
  uint8_t   pu8ManufNameLen;                            /* Length of the Manufacturer Name */
  uint16_t  u16ConnHandle;                              /* Connection Handle */

  uint16_t* pu16RetDevNameSvcHndl;                      /* Returned GAP Device Name Service  Handle */
  uint16_t* pu16RetDISvcHndl;                           /* Returned GATT Device Identification Service Return Handle */
  uint16_t* pu16RetDICharDeclHndl;                      /* Returned GATT Device Identification Characteristic Declaration Return Handle */
  uint16_t* pu16RetDICharValDeclHndl;                   /* Returned GATT Device Identification Characteristic VALUE Declaration Return Handle */


  uint16_t* pu16RetMPSvcHndl;                           /* Returned GATT Heart Rate Service Return Handle */
  uint16_t* pu16RetMPCharDeclHndl;                      /* Returned GATT Heart Rate Measurement Characteristic Declaration Return Handle */
  uint16_t* pu16RetMPCharValDeclHndl;                   /* Returned GATT Heart Rate Measurement Characteristic VALUE Declaration Return Handle */

  le_conn_evt_st* pstConnectInfo;                       /* Connection information */
  le_mtu_exchg_st* pstMtuEvtData;                       /* MTU Event Data */
  le_mtu_exchg_acc_resp_st* pstMtuAccResp;              /* MTU Accept Response */

  le_srv_write_char_desp_event_st stCharDespEvt;        /* Write Characteristic Descriptor placeholder */

  le_srv_write_char_desp_event_st  stCharValEvt;  /* Write Characteristic VALUE placeholder */

} hr_init_st;

/**********************************************
*                Functions                    *
**********************************************/
/**
 * @brief     Initializes Peripheral device, GATT Server and registers
 *            following services:
 *              1) GAP Device Name,
 *              2) GATT Device Identification,
 *              3) Heart-Rate Services.
 *
 * @param[in]  qHandle OS Message-Queue Handle
 * @param[in]  stInit Pointer to initialization structure
 *
 */
le_api_result_e eMPInitialise(void* qHandle,
                               hr_init_st* stInit);

/**
 * @brief     Main Heart Rate Profil Routine is an event driven routine, which
 *            processes events comming from the local Chiron chip or from a remote
 *            BT Client.
 *
 * @param[in]  qHandle OS Message-Queue Handle
 * @param[in]  stInit Pointer to initialization structure
 *
 */
void MPMainRoutine(void* qHandle, hr_init_st* stInit);

/**
 * @brief     Configures and starts advertizing. After this function is executed,
 *            the device is visible to all central devices performing the scan
 *            procedure
 *
 * @param[in]  qHandle OS Message-Queue Handle
 *
 */
le_api_result_e eMPStartAdvertising(void* qHandle);

/**
 * @brief     Registers GAP services (Device Name & Appearance)
 *
 * @param[in]  qHandle OS Message-Queue Handle
 * @param[in]  stInit Pointer to initialization structure
 *
 */
le_api_result_e eMPAddGAPDevName_Appearance_Service(void* qHandle,
                                                   uint16_t* pu16SvcHandle,
                                                   uint16_t u16DevNameLen,
                                                   uint8_t* pu8DevName);

/**
 * @brief     Registers GATT Device Identification  Service to the GATT data-base
 *
 * @param[in]  qHandle OS Message-Queue Handle
 * @param[in]  stInit Pointer to initialization structure
 * @param[in]  pu8ManufName Manufacturer name string in UTF8 format.
 * @param[in]  pu8ManufNameLen Length of the manufacturer name string array.
 * @param[out]  pu16SvcHandle Returned service handle.
 * @param[out]  pu16RetHandleCharDecl Returned character declaration handle
 * @param[out]  pu16RetHandleCharValDecl Returned character value declaration handles.
 *
 */
le_api_result_e eMPAddDevIdentService(void* qHandle,
                                       uint8_t* pu8ManufName,
                                       uint8_t pu8ManufNameLen,
                                       uint16_t* pu16SvcHandle,
                                       uint16_t* pu16RetHandleCharDecl,
                                       uint16_t* pu16RetHandleCharValDecl);
/**
 * @brief     Registers GAP services (Device Name & Appearance)
 *
 * @param[out]  pu16RetHandleCharDecl Returned character declaration handle
 * @param[out]  pu16RetHandleCharValDecl Returned character value declaration handles.
 *
 */
le_api_result_e eMPAdd_controller_Service(void* qHandle, uint16_t* pu16SvcHandle,
                                     uint16_t* pu16RetHandleCharDecl,
                                     uint16_t* pu16RetHandleCharValDecl);



uint8_t transmit_bluetooth_packet(uint8_t data_packet[]);


#endif /* _HEARTRATE_H */

