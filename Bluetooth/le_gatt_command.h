/****************************************************************************
*   COPYRIGHT 2006-2009 All rights reserved
*   Toshiba Electronics Europe GmbH
*   European LSI Design and Engineering Center (ELDEC)
*****************************************************************************
*   File Name   : $RCSfile: booting.h,v $
*   Description :
*****************************************************************************
*   PROJECT     : Bluetooth BLE
*   MODULE      : LE + GATT Commands
*               : $Name:  $
*   LIBRARIES   : None
*   AUTHOR      : $Author: arelic $
*****************************************************************************
*   VERSION     : $$
*   RELEASE     : Preliminary & Confidential
*   DATE        : $Date: 2012-08-03 13:26:26 $
*   SOURCE MOD. : $Source: /teenet/mpumcu/Capricorn/Capricorn_Bt/ipl/inc/booting.h,v $
*   LOG:        : please have a look at the end of the file
****************************************************************************/

/**
* @file le_gatt_command.h
* @brief Contains definitions and function declarattions
*
* @author ARE
*
* @date 18/07/2013
*/

#ifndef _LE_GATT_COMMAND_H
#define _LE_GATT_COMMAND_H

//#define DEBUG_PRINT
/**********************************************
*                Include files                *
**********************************************/
#include "stdint.h"

/**********************************************
*               Defs                          *
**********************************************/

/*! Max number of advertizing devices */
#define MAX_ADVERTIZERS   16

/*! Bit-fiels offsets in cmd macros */
#define CMD_SER_ID_OFFS 24
#define CMD_OPCODE_OFFS 16

/*!  UART packet offset  */
#define UART_PACKET_OFFSET 3

/*! Offsets in command array (in bytes) */
#define CMD_ARRAY_SER_ID_OFFS (UART_PACKET_OFFSET + 0)
#define CMD_ARRAY_OPCODE_OFFS (UART_PACKET_OFFSET + 1)
#define CMD_ARRAY_LEN_OFFS    (UART_PACKET_OFFSET + 2)
#define CMD_ARRAY_PARAM_OFFS  (UART_PACKET_OFFSET + 4)

/*! Masks for command macros */
#define CMD_SER_ID_MASK (0xFF << CMD_SER_ID_OFFS)
#define CMD_OPCODE_MASK (0xFF << CMD_OPCODE_OFFS)
#define CMD_LENGTH_MASK (0xFFFF)

/*! Variable length value */
#define LEN_VAR 0xFFFF

/*! BLE Message header size (in Bytes) */
#define BLE_MSG_HDR_SIZE 4



/*! Length zero */
#define NO_PARAM 0x0000

/*! BD_ADDR_SIZE in bytes */
#define BD_ADDR_SIZE 6


/*! Command Macro Concatenation */
#define CMD_CONCAT(service_id, opcode, len)(((((uint32_t)((service_id) & 0xFF)) << CMD_SER_ID_OFFS)  |  \
                                            (((uint32_t)((opcode) & 0xFF))      << CMD_OPCODE_OFFS)) |  \
                                            (((uint32_t)(len) & 0xFFFF)))
/*! Returns uint16_t from two bytes */
#define GET_SHORT(lo_byte, hi_byte)((lo_byte & 0xFF) | (((uint16_t)(hi_byte) << 8) & 0xFF00))
//#define GET_SHORT(lo_byte, hi_byte)((hi_byte & 0xFF) | (((uint16_t)(lo_byte) << 8) & 0xFF00))

#define GET_BYTE_LSB(short_data)((uint8_t)(short_data & 0xFF))
#define GET_BYTE_MSB(short_data)((uint8_t)((short_data >> 8) & 0xFF))

/*! Extracts service id from command macro */
#define GET_CMD_SERVICE_ID(command) ((uint8_t)((((uint32_t)(command)) & CMD_SER_ID_MASK) >> CMD_SER_ID_OFFS))

/*! Extracts op code from command */
#define GET_CMD_OPCODE(command)((uint8_t)(((uint32_t)(command) & CMD_OPCODE_MASK) >> CMD_OPCODE_OFFS))

/*! Extracts command length from command */
#define GET_PARAM_LENGTH(command)((uint16_t)((uint32_t)(command) & CMD_LENGTH_MASK))

/*! Extracts service id from input stream */
#define PARSE_CMD_SERVICE_ID(data) ((uint8_t)((data[CMD_ARRAY_SER_ID_OFFS])))

/*! Extracts op code from input stream */
#define PARSE_CMD_OPCODE(data) ((uint8_t)((data[CMD_ARRAY_OPCODE_OFFS])))

/*! Extracts service id and op code from input strea, and returns it as a 16-bit word
  (to be used togehter with GET_CMD_MESSAGE())
*/
#define PARSE_CMD_MESSAGE(data) (GET_SHORT(PARSE_CMD_OPCODE(data), PARSE_CMD_SERVICE_ID(data)))

/*! Extracts service id and op code from command macro, and returns it as a 16-bit word  */
#define GET_CMD_MESSAGE(command) (GET_SHORT(GET_CMD_OPCODE(command), GET_CMD_SERVICE_ID(command)))

/*! Extracts length from command */
#define PARSE_CMD_PARAM_LENGTH(data) ((uint16_t)((uint16_t)((uint16_t)(data[CMD_ARRAY_LEN_OFFS] & 0xFF) |\
                                     ((uint16_t)(data[CMD_ARRAY_LEN_OFFS + 1]) & 0xFF)<<8)))

/*! Returns total message size */
#define GET_TOTAL_LE_MESSAGE_SIZE(data)(PARSE_CMD_PARAM_LENGTH(data) + BLE_MSG_HDR_SIZE)

/*! Sets command service-id in the dest array */
//#define SET_CMD_SVC_ID(data, svc_id)(data[CMD_ARRAY_SER_ID_OFFS] = GET_CMD_SERVICE_ID(svc_id))
#define SET_CMD_SVC_ID(data, svc_id)(data[CMD_ARRAY_SER_ID_OFFS] = (svc_id))

/*! Sets comamnd op-code in the dest array */
//#define SET_CMD_OP_CODE(data, op_code)(data[CMD_ARRAY_OPCODE_OFFS] = GET_CMD_OPCODE(op_code))
#define SET_CMD_OP_CODE(data, op_code)(data[CMD_ARRAY_OPCODE_OFFS] = (op_code))


typedef struct{
                                uint8_t service_id;
                                uint8_t opcode;
                                uint16_t length;
}tcu_packet_STRUCT;;


/**********************************************
*            COMMAND PARAMETERS               *
**********************************************/
#define TCU_LE_ACCEPT                                   (CMD_CONCAT(0xD1, 0xF1, 0x0003))
#define TCU_LE_NOT_ACCEPT                               (CMD_CONCAT(0xD1, 0xF2, 0x0002))
#define TCU_LE_FATAL_ERROR                              (CMD_CONCAT(0xD1, 0xFE, 0x0001))
#define TCU_MNG_LE_INIT_REQ                             (CMD_CONCAT(0xD1, 0x01, 0x00F9))
#define TCU_MNG_LE_INIT_RESP                            (CMD_CONCAT(0xD1, 0x81, 0x0007))
#define TCU_MNG_LE_SET_SCAN_ENABLE_REQ                  (CMD_CONCAT(0xD1, 0x0A, 0x0009))
#define TCU_MNG_LE_SET_SCAN_ENABLE_RESP                 (CMD_CONCAT(0xD1, 0x8A, 0x0001))
#define TCU_MNG_LE_ADV_REPORT_EVENT                     (CMD_CONCAT(0xD1, 0xC1, LEN_VAR))
#define TCU_MNG_LE_CREATE_CONNECTION_REQ                (CMD_CONCAT(0xD1, 0x0C, 0x0019))
#define TCU_MNG_LE_CONNECTION_COMPLETE_EVENT            (CMD_CONCAT(0xD1, 0x4C, 0x0012))
#define TCU_MNG_LE_CREATE_CONNECTION_CANCEL_REQ         (CMD_CONCAT(0xD1, 0x0D, NO_PARAM))
#define TCU_MNG_LE_CREATE_CONNECTION_CANCEL_RESP        (CMD_CONCAT(0xD1, 0x8D, 0x0001))
#define TCU_MNG_LE_DISCONNECT_REQ                       (CMD_CONCAT(0xD1, 0x13, 0x0008))
#define TCU_MNG_LE_DISCONNECT_EVENT                     (CMD_CONCAT(0xD1, 0x93, 0x0002))
#define TCU_MNG_LE_SET_SCAN_DISABLE_REQ                 (CMD_CONCAT(0xD1, 0x0B, NO_PARAM))
#define TCU_MNG_LE_SET_SCAN_DISABLE_RESP                (CMD_CONCAT(0xD1, 0x8B, 0x0001))
#define TCU_MNG_LE_START_ADVERTISE_REQ                  (CMD_CONCAT(0xD1, 0x08, 0x004f))
#define TCU_MNG_LE_START_ADVERTISE_RESP                 (CMD_CONCAT(0xD1, 0x88, 0x0001))
#define TCU_MNG_LE_DISABLE_ADVERTISE_REQ                (CMD_CONCAT(0xD1, 0x09, 0x0000))
#define TCU_MNG_LE_DISABLE_ADVERTISE_RESP               (CMD_CONCAT(0xD1, 0x89, 0x0001))
#define TCU_LE_GATT_SDB_ADD_PRIM_SVC_REQ                (CMD_CONCAT(0xD3, 0x20, 0x0011))
#define TCU_LE_GATT_SDB_ADD_PRIM_SVC_RESP               (CMD_CONCAT(0xD3, 0xA0, 0x0003))
#define TCU_LE_GATT_SDB_ADD_SEC_SVC_REQ                 (CMD_CONCAT(0xD3, 0x21, LEN_VAR))
#define TCU_LE_GATT_SDB_ADD_SEC_SVC_RESP                (CMD_CONCAT(0xD3, 0xA1, LEN_VAR))
#define TCU_LE_GATT_SDB_ADD_CHAR_DECL_REQ               (CMD_CONCAT(0xD3, 0x22, 0x0014))
#define TCU_LE_GATT_SDB_ADD_CHAR_DECL_RESP              (CMD_CONCAT(0xD3, 0xA2, 0x0003))
#define TCU_LE_GATT_SDB_ADD_CHAR_ELE_REQ                (CMD_CONCAT(0xD3, 0x23, 0xFFFE))
#define TCU_LE_GATT_SDB_ADD_CHAR_ELE_RESP               (CMD_CONCAT(0xD3, 0xA3, 0x0003))
#define TCU_LE_GATT_SDB_UPD_CHAR_ELE_REQ                (CMD_CONCAT(0xD3, 0x25, LEN_VAR))
#define TCU_LE_GATT_SDB_UPD_CHAR_ELE_RESP               (CMD_CONCAT(0xD3, 0xA5, 0x0001))
#define TCU_LE_GATT_SDB_ADD_INC_SVC_REQ                 (CMD_CONCAT(0xD3, 0x24, LEN_VAR))
#define TCU_LE_GATT_SDB_ADD_INC_SVC_RESP                (CMD_CONCAT(0xD3, 0xA4, LEN_VAR))
#define TCU_LE_GATT_SDB_RET_END_GRP_HLE_REQ             (CMD_CONCAT(0xD3, 0x26, 0x0002))
#define TCU_LE_GATT_SDB_RET_END_GRP_HLE_RESP            (CMD_CONCAT(0xD3, 0xA6, 0x0003))
#define TCU_LE_GATT_SER_INIT_REQ                        (CMD_CONCAT(0xD3, 0x00, NO_PARAM))
#define TCU_LE_GATT_SER_INIT_RESP                       (CMD_CONCAT(0xD3, 0x80, 0x0001))
#define TCU_LE_GATT_SER_WRITE_CHAR_DESP_EVENT           (CMD_CONCAT(0xD3, 0xC4, LEN_VAR))
#define TCU_LE_GATT_SER_WRITE_CHAR_DESP_ACCEPT_REQ      (CMD_CONCAT(0xD3, 0x04, 0x0005))
#define TCU_LE_GATT_SER_WRITE_CHAR_DESP_ACCEPT_RESP     (CMD_CONCAT(0xD3, 0x84, 0x0003))
#define TCU_LE_GATT_SER_READ_CHAR_VAL_ACCEPT_REQ        (CMD_CONCAT(0xD3, 0x02, 0x0005))
#define TCU_LE_GATT_SER_READ_CHAR_VAL_ACCEPT_RESP       (CMD_CONCAT(0xD3, 0x82, 0x0003))
#define TCU_LE_GATT_CLI_EXG_MTU_REQ                     (CMD_CONCAT(0xD2, 0x01, 0x0004))
#define TCU_LE_GATT_CLI_EXG_MTU_EVENT                   (CMD_CONCAT(0xD2, 0x41, 0x0005))
#define TCU_LE_GATT_CLI_DISCOVER_PRIM_SVC_BY_UUID_REQ   (CMD_CONCAT(0xD2, 0x03, 0x0008))
#define TCU_LE_GATT_CLI_DISCOVER_PRIM_SVC_BY_UUID_EVENT (CMD_CONCAT(0xD2, 0x43, 0x0200))
#define TCU_LE_GATT_CLI_DISCOVER_CHAR_DECL_REQ          (CMD_CONCAT(0xD2, 0x05, 0x0006))
#define TCU_LE_GATT_CLI_DISCOVER_CHAR_DECL_EVENT        (CMD_CONCAT(0xD2, 0x45, 0x0200)) //lwi lenght ???
#define TCU_LE_GATT_CLI_READ_CHAR_VAL_REQ               (CMD_CONCAT(0xD2, 0x08, 0x0004))
#define TCU_LE_GATT_CLI_READ_CHAR_VAL_EVENT             (CMD_CONCAT(0xD2, 0x48, 0x0200))
#define TCU_LE_GATT_CLI_CHAR_VAL_NOTIFICATION_IND_EVENT (CMD_CONCAT(0xD2, 0xC0, LEN_VAR)) //???
#define TCU_LE_GATT_SER_EXG_MTU_EVENT                   (CMD_CONCAT(0xD3, 0xC1, 0x0004))
#define TCU_LE_GATT_SER_EXG_MTU_ACCEPT_REQ              (CMD_CONCAT(0xD3, 0x01, 0x0005)) //T.B.C.!!!
#define TCU_LE_GATT_SER_EXG_MTU_ACCEPT_RESP             (CMD_CONCAT(0xD3, 0x81, 0x0005)) //T.B.C.!!!
#define TCU_LE_GATT_SER_READ_CHAR_VAL_EVENT             (CMD_CONCAT(0xD3, 0xC2, 0x0004))
#define TCU_LE_GATT_SER_WRITE_CHAR_VAL_EVENT            (CMD_CONCAT(0xD3, 0xC3, LEN_VAR))
#define TCU_LE_GATT_SER_WRITE_CHAR_VAL_ACCEPT_REQ       (CMD_CONCAT(0xD3, 0x03, 0x0005))
#define TCU_LE_GATT_SER_WRITE_CHAR_VAL_ACCEPT_RESP      (CMD_CONCAT(0xD3, 0x83, 0x0003))
#define TCU_LE_GATT_SER_READ_CHAR_DESP_EVENT            (CMD_CONCAT(0xD3, 0xC8, 0x0004))
#define TCU_LE_GATT_SER_READ_CHAR_DESP_ACCEPT_REQ       (CMD_CONCAT(0xD3, 0x08, 0x0005))
#define TCU_LE_GATT_SER_READ_CHAR_DESP_ACCEPT_RESP      (CMD_CONCAT(0xD3, 0x88, 0x0003))

#define TCU_LE_GATT_SER_WRITE_CHAR_VAL_ACCEPT_REQ       (CMD_CONCAT(0xD3, 0x03, 0x0005))
#define TCU_LE_GATT_SER_READ_MULTIPLE_EVENT             (CMD_CONCAT(0xD3, 0xCA, LEN_VAR))
#define TCU_LE_GATT_SER_READ_MULTIPLE_ACCEPT_REQ        (CMD_CONCAT(0xD3, 0x0A, 0x0005))
#define TCU_LE_GATT_SER_READ_MULTIPLE_ACCEPT_RESP       (CMD_CONCAT(0xD3, 0x8A, 0x0003))
#define TCU_LE_GATT_SER_CHAR_VAL_NOTIFICATION_REQ       (CMD_CONCAT(0xD3, 0x05, LEN_VAR))
#define TCU_LE_GATT_SER_CHAR_VAL_NOTIFICATION_EVENT     (CMD_CONCAT(0xD3, 0x45, 0x0002))
#define TCU_LE_GATT_SER_CHAR_VAL_INDICATION_REQ         (CMD_CONCAT(0xD3, 0x06, LEN_VAR))
#define TCU_LE_GATT_SER_CHAR_VAL_INDICATION_EVENT       (CMD_CONCAT(0xD3, 0x46, 0x0003))
#define TCU_LE_GATT_SER_WRITE_WITHOUT_RESPONSE_CMD_EVENT (CMD_CONCAT(0xD3, 0xC9, LEN_VAR))


#define TCU_LE_GATT_CLI_INIT_RESP                       (CMD_CONCAT(0xD2, 0x80, 0x0001))
#define TCU_LE_GATT_CLI_DISCOVER_CHAR_DESP_EVENT        (CMD_CONCAT(0xD2, 0x47, LEN_VAR))
#define TCU_LE_GATT_CLI_DISCOVER_CHAR_DESP_REQ          (CMD_CONCAT(0xD2, 0x07, 0x0006))

/**********************************************
*               Enums                         *
**********************************************/

/*! Function call results */
typedef enum tag_le_func_status_e
{
  LE_FUNC_SUCCCESS                      = 0x00,     /*!< Success */
  LE_FUNC_ERR_CMD_SVC_ID                = 0x01,     /*!< Commmand service id mismatch */
  LE_FUNC_ERR_OPCODE                    = 0x02,     /*!< commmand opcode mismatch */
  LE_FUNC_ERR_LENGTH                    = 0x03,     /*!< Wrong Length */
  LE_FUNC_ERR_SEND_FAILED               = 0x04,     /*!< Command sending failed */
  LE_FUNC_ERR_DEV_NAME                  = 0x05      /*!< Device name error */
} le_func_status_e;

/*! TCU_LE_ACCEPT Status */
typedef enum tag_le_accept_status_e
{
  LE_SUCCESS          = 0x00,           /*!< Success */
  LE_PARAM_ERR        = 0x01,           /*!< The abnormalities in a parameter */
  LE_NOT_INIT_ERR     = 0x02,           /*!< Device Is Not -Initialized */
  LE_ALREDY_INIT      = 0x03,           /*!< Device Already Initialized */
  LE_MNG_PROCESSING   = 0x04,           /*!< MNG Processing in progress */
  LE_NO_ACL_LINK      = 0x05,           /*!< No ACL Link*/
  LE_DEV_ROLE_SLAVE   = 0x06,           /*!< Device Role Slave */
  LE_GATT_IN_PROGRESS = 0x07            /*!< GATT Processing in Progress */
} le_accept_status_e;

/*! TCU_MNG_LE_ INIT_RESP Status */
typedef enum tag_mgn_init_resp_status_e
{
  LE_MNG_INIT_SUCCESS       = 0x00,      /*!< Success */
  LE_PARAM_ABN_ERR          = 0x01,      /*!< The abnormalities in a parameter */
  LE_INIT_ERR               = 0x02       /*!< Device Is Already -Initialized */

} mgn_init_resp_status_e;

/*! LE_Scan_Type (TCU_MNG_LE_SET_SCAN_ENABLE_REQ) ***/
typedef enum tag_scan_type_e
{
  LE_PASSIVE  = 0x00,                   /*!< Passive Scanning. No SCAN_REQ packets shall be sent. */
  LE_ACTIVE   = 0x01                    /*!< Active scanning. SCAN_REQ packets may be sent. */
} scan_type_e;

/*! LE_Scan_Type (TCU_MNG_LE_SET_SCAN_ENABLE_REQ) ***/
typedef enum tag_own_addr_type_e
{
  LE_ADDR_PUBLIC  = 0x00,               /*!< Public Device Address */
  LE_ADDR_RANDOM  = 0x01                /*!< Random Device Address */
} own_addr_type_e;

/*! Scanning_Filter_Policy (TCU_MNG_LE_SET_SCAN_ENABLE_REQ) ***/
typedef enum tag_scan_filt_plcy_e
{
  LE_ACCEPT_ALL  = 0x00,               /*!< Accept all advertisement packets */
  LE_WHTLST_ONLY = 0x01                /*!< Ignore advertisement packets from devices not in the White List */

} scan_filt_plcy_e;

/*! Initiator filter policy TCU_MNG_LE_CREATE_CONNECTION_REQ ***/
typedef enum tag_init_filt_plcy_e
{
  LE_NO_WHITE_LIST_PEER_ADDR_USE  = 0x00, /*!< White list is not used to determine which advertiser
                                           * to connect to Peer_Address_Type and  eer_Address shall be used.
                                           */
  LE_WHITE_LIST_PEER_ADDR_IGNORE = 0x01   /*!< White list is used to determine which advertiser to connect
                                             to Peer_Address_Type and Peer_Address shall be ignored.*/

} conn_init_plcy_e;


/* Advertising type (TCU_MNG_LE_START_ADVERTISE_REQ) */
typedef enum tag_adv_type_e
{
    LE_ADV_IND      =0x00,          /*!< Connectable undirected advertising */
    ADV_DIRECT_IND  =0x01,          /*!< Connectable directed advertising */
    ADV_SCAN_IND    =0x02,          /*!< Scannable undirected advertising */
    ADV_NONCONN_IND =0x03           /*!< Non connectable undirected advertising */
} adv_type_e;


/* Advertising filter policy (TCU_MNG_LE_START_ADVERTISE_REQ) */
typedef enum tag_adv_filt_plcy_e
{
    LE_SCAN_ANY_CONN_ANY      =0x00, /*!< Allow Scan Request from Any, Allow Connect Request from Any (default). */
    ADV_SCAN_WLIST_CONN_ANY   =0x01, /*!< Allow Scan Request from White List Only, Allow Connect Request from Any.*/
    ADV_SCAN_ANY_CONN_WLIST   =0x02, /*!< Allow Scan Request from Any, Allow Connect Request from White List Only. */
    ADV_SCAN_WLIST_CONN_WLIST =0x03  /*!< Allow Scan Request from White List Only, Allow Connect Request from White List only. */
} adv_filt_plcy_e;

/*! Enable/disable feature ***/
typedef enum tag_feat_en_e
{
  LE_DISABLE  = 0x00,               /*!< Feature Disable */
  LE_ENABLE   = 0x01                /*!< Feature Enable */

} feat_en_e;


/*! Enable/disable feature ***/
typedef enum tag_role_e
{
  LE_MASTER  = 0x00,               /*!< Master role */
  LE_SLAVE   = 0x01                /*!< Slave role */
} role_e;

/*! UUID Lengths ***/
typedef enum tag_uuid_type_e
{
  LE_SHORT_UUID  = 0x02,               /*!< two-byte form */
  LE_LONG_UUID   = 0x10                /*!< sixteen-byte form */
} uuid_type_e;

/*! GATT Error codes ***/
typedef enum tag_gatt_status_code_e
{
    LE_GATT_SUCCESS                         =0x00,
    LE_ERR_INV_HANDLE                       =0x01,
    LE_ERR_READ_NOT_PERMITTED               =0x02,
    LE_ERR_WRITE_NOT_PERMITTED              =0x03,
    LE_ERR_INVALID_PACKET_DATA_UNIT         =0x04,
    LE_ERR_INSUFFICIENT_AUTHENTICATION      =0x05,
    LE_ERR_REQUEST_NOT_SUPPORTED            =0x06,
    LE_ERR_INVALID_OFFSET                   =0x07,
    LE_ERR_INSUFFICIENT_AUTHORIZATION       =0x08,
    LE_ERR_PREPARE_QUEUE_FULL               =0x09,
    LE_ERR_LOOK_UP_ATTRIBUTE_NOT_FOUND      =0x0A,
    LE_ERR_ATTRIBUTE_NOT_LONG               =0x0B,
    LE_ERR_INSUFFICIENT_ENCRYPTION_KEY_SIZE =0x0C,
    LE_ERR_INVALID_ATTRIBUTE_VALUE_LENGTH   =0x0D,
    LE_ERR_UNLIKELY_ERROR                   =0x0E,
    LE_ERR_INSUFFICIENT_ENCRYPTION          =0x0F,
    LE_ERR_UNSUPPORTED_GROUP_TYPE           =0x10,
    LE_ERR_INSUFFICIENT_RESOURCES           =0x12,
    LE_ERR_INITIALIZATION_ALREADY_DONE      =0xF1,
    LE_ERR_TIMEOUT                          =0xFF,
} gatt_status_code_e;

/*! GATT Error codes ***/
typedef enum tag_sdb_status_code_e
{
    LE_SDB_SUCCESS                         =0x00,
    LE_SDB_MEMORY_NOT_AVAILABLE            =0x01,
    LE_SDB_INVALID_HANDLE                  =0x02,
    LE_SDB_MAX_CHAR_HANDLE_EXCEEDED        =0x03,
    LE_SDB_INVALID_PACKET                  =0x04

} sdb_status_code_e;

/*! GATT Characteristic properties  ***/
typedef enum tag_gatt_char_prop_mask_e
{
    LE_BROADCAST            = 0x01,        /*!< Permits broadcasts of the Characteristic Value using Characteristic ConfigurationDescriptor */
    LE_READ                 = 0x02,        /*!< Permits reads of the Characteristic Value */
    LE_WRITE_WO_RESP        = 0x04,        /*!< Permits writes of the Characteristic Value without response */
    LE_WRITE                = 0x08,        /*!< Permits writes of the Characteristic Value with response */
    LE_NOTIFY               = 0x10,        /*!< Permits notifications of a Characteristic Value without acknowledgement */
    LE_INDICATE             = 0x20,        /*!< Permits indications of a Characteristic Value with acknowledgement */
    LE_AUTH_SIGNED_WRITES   = 0x40,        /*!< Permits signed writes to the Characteristic Value */
    LE_EXT_PROP             = 0x80         /*!< Additional characteristic properties are defined in the Characteristic Extended Properties Descriptor */
} gatt_char_prop_mask_e;

/*! Attribute permissions / Bit def. for permitions ***/
typedef enum tag_gatt_permissions_e
{
  LE_GATT_READ              = 0x0001,       /*!< Masks for Attribute permissions */
  LE_GATT_WRITE             = 0x0002,
  LE_GATT_AUTHORIZATION     = 0x0004,
  LE_GATT_AUTHENTICATION    = 0x0008,
  LE_NO_RW_PERMITION        = 0x0000,       /*!< Masks for Permissions */
  LE_REDABLE                = 0x0100,
  LE_WRITABLE               = 0x0200,
  LE_BOTH_READ_WRITE        = 0x0300,
  LE_AUTHORISATION_NEEDED   = 0x0400,
  LE_AUTHENTICATION_NEEDED  = 0x0800
} gatt_permissions_e;

/*! Continue Flag ***/
typedef enum tag_gatt_continue_e
{
  LE_GATT_COMPLETE          = 0x00,         /*!< Competed, continue flag is not set */
  LE_GATT_INCOMPLETE        = 0x01          /*!< Incomplete transfer continue flag is set */
} gatt_continue_e;




/**********************************************
*               Structs                       *
**********************************************/
/*! contains TCU_LE_ACCEPT data */
typedef struct tag_le_accept_st
{
   le_accept_status_e  eStatus;
   uint16_t            u16ServiceId;
   uint8_t             u8OpCode;
} le_accept_st;

/*! Configuration strucure used for TCU_MNG_LE_SET_SCAN_ENABLE_REQ */
typedef struct tag_set_scan_en_st
{
    uint16_t          u16LE_Scan_Interval;    /*!< Time interval from when the Controller started its
                                                 last LE scan until it begins the subsequent LE scan */
    uint16_t          u16LE_Scan_Window;      /*!< The duration of the LE scan */
    scan_type_e       eLE_Scan_Type;          /*!< Type of the scanning */
    own_addr_type_e   eAddrType;              /*!< Own_Address_Type */
    scan_filt_plcy_e eScanPolicy;             /*!< Scanning_Filter_Policy */
    feat_en_e        eScanEnable;             /*!< Enable scanning */
    feat_en_e        eDuplFiltEnable;         /*!< Enable duplicate filtering */
} set_scan_en_st;

/*! Contains TCU_MNG_LE_ADV_REPORT_EVENT data */
typedef struct tag_le_adv_report_evnt_st
{
    uint8_t au8EvntType;   /*!< Event type */
    uint8_t au8AddrType;   /*!< Describes the Type of PDU Packet obtained */
    uint8_t au8Address[6]; /*!< Addresses */
    uint8_t au8DataLen;    /*!< Length of the Data[i] field for each device which responded. */
    uint8_t* pu8Data;
    int8_t  i8Rssi;
} le_adv_report_evnt_st;

/*! contains TCU_MNG_LE_CREATE_CONNECTION_REQ data */
typedef struct tag_le_create_conn_st
{
    uint16_t          u16ScanInterval;
    uint16_t          u16ScanWindow;
    conn_init_plcy_e  eInitFiltPolicy;
    own_addr_type_e   ePeerAddrType;
    uint8_t           au8PeerAddr[8];
    own_addr_type_e   eOwnAddrType;
    uint16_t          u16ConnIntMin;
    uint16_t          u16ConnIntMax;
    uint16_t          u16ConnLatency;
    uint16_t          u16SupervTimeout;
    uint16_t          u16MinCeLength;
    uint16_t          u16MaxCeLength;
} le_create_conn_st;

/*! contains TCU_MNG_LE_CONNECTION_COMPLETE_EVENT data */
typedef struct tag_le_conn_evt_st
{
    uint8_t           u8Status;
    uint16_t          u16ConnHandle;
    role_e            eRole;
    own_addr_type_e   ePeerAddrType;
    uint8_t           au8PeerAddr[8];
    uint16_t          u16ConnInt;
    uint16_t          u16ConnLatency;
    uint16_t          u16SupervTimeout;
    uint8_t           u8MstrClkAccuracy;
} le_conn_evt_st;

/*! contains TCU_MNG_LE_START_ADVERTISE_REQ data */
typedef struct tag_le_adv_req_st
{
    uint16_t          u16AdvIntervalMin;
    uint16_t          u16AdvIntervalMax;
    adv_type_e        eAdvType;
    own_addr_type_e   eOwnAddrType;
    own_addr_type_e   eDirectAddrType;
    uint8_t           au8DirectAddr[6];
    uint8_t           u8AdvChannelMap;
    adv_filt_plcy_e   eAdvFilterPolicy;
    uint8_t           u8AdvDataLen;
    uint8_t           au8AdvData[31];
    uint8_t           u8ScanRespDataLen;
    uint8_t           au8ScanRespData[31];

} le_adv_req_st;

/*! contains TCU_LE_GATT_SDB_ADD_INC_SVC_REQ data */
typedef struct tag_le_inc_svc_req_st
{
    uint16_t          u16SvcHandle;     /*!< Handle of the container service */
    uuid_type_e       eAttValLen;
    uint8_t*          pu8AttVal;
} le_inc_svc_req_st;

/*! contains Reponse Data data */
typedef struct tag_le_resp_st
{
    gatt_status_code_e eStatus;         /*!< Response status */
    uint16_t           u16Handle;  /*!< Include service Handle*/
} le_resp_st;

/*! contains TCU_LE_GATT_SDB_ADD_CHAR_ELE_REQ data */
typedef struct tag_le_char_el_req_st
{
    uuid_type_e       eAttrLen;
    uint8_t           au8AttType[16];
    uint16_t          u16AttValLen;
    uint8_t*          pu8AttVal;
    uint16_t          u16AttPermission; /*!< please use att_permissions_e masks */
} char_el_req_st;

/*! contains TCU_LE_GATT_CLI_DISCOVER_PRIM_SVC_BY_UUID_EVENT data */
typedef struct tag_le_svc_evt_st
{
    uint16_t            u16ConnHandle;
    gatt_status_code_e  eStatus;
    gatt_continue_e     eContinueFlag;
    uint8_t             u8GroupCnt;
    uint8_t             u8GroupLen;
    uint16_t            u16AttrHandle;
    uint16_t            u16EndGroupHandle;

} le_svc_evt_st;


/*! */
typedef struct tag_le_char_decl_req_st
{
    uint16_t            u16ConnHandle;
    gatt_status_code_e  eStatus;
    gatt_continue_e     eContinueFlag;
    uint8_t             u8GroupCnt[16];
    uint8_t             u8GroupLen[16];
    uint16_t            u16AttrHandle[16];
    uint8_t             u8CharProperties[16]; /*!< please use gatt_char_prop_mask_e */
    uint16_t            u16CharValHandle[16];
    uuid_type_e         eUuidLen[16];
    uint8_t*            pu8CharUUIDVal1[16];  /*!< 2 or 16 -bytes */
} le_char_decl_req_st;

/*! contains TCU_MNG_LE_INIT_RESP */
typedef struct tag_le_mnginit_resp_st
{
    mgn_init_resp_status_e eStatus;
    uint8_t* pu8BdAddr;
} le_mnginit_resp_st;

/*! contains TCU_LE_GATT_SER_EXG_MTU_EVENT */
typedef struct tag_le_mtu_exchg_st
{
  uint16_t* pu16ConnHandle;
  uint16_t* pu16ClientRxMtuSize;
} le_mtu_exchg_st;

/*! contains TCU_LE_GATT_SER_EXG_MTU_ACCEPT_RESP */
typedef struct tag_le_mtu_exchg_acc_resp_st
{
  uint16_t  u16ConnHandle;
  uint8_t   u8Status;
  uint16_t  u16ServerRxMtuSize;
} le_mtu_exchg_acc_resp_st;

/*! contains TCU_LE_GATT_SER_READ_MULTIPLE_EVENT */
typedef struct tag_le_srv_multi_event_st
{
  uint16_t u16ConnHandle;
  uint16_t u16HndlCnt;
  uint8_t* pu8HndlList;
} le_srv_multi_event_st;

/*! contains TCU_LE_GATT_SER_READ_MULTIPLE_ACCEPT_RESP */
typedef struct tag_le_srv_multi_acc_event_st
{
  uint16_t u16ConnHandle;
  gatt_status_code_e eStatus;
} le_srv_multi_acc_event_st;

/*! contains TCU_LE_GATT_SER_READ_CHAR_VAL_EVENT */
typedef struct tag_le_srv_read_char_val_event_st
{
  uint16_t u16ConnHandle;
  uint16_t u16CharDescHandle;
} le_srv_read_char_val_event_st;

/*! contains TCU_LE_GATT_SER_WRITE_CHAR_DESP_EVENT */
typedef struct tag_le_srv_write_char_desp_event_st
{
  uint16_t u16ConnHandle;
  uint16_t u16CharDescHandle;
  uint16_t u16CharValueLen;
  uint8_t*  pu8CharDescValue;
} le_srv_write_char_desp_event_st;

/*! contains TCU_LE_GATT_SER_READ_CHAR_VAL_ACCEPT_RESP */
typedef struct tag_le_srv_read_char_acc_event_st
{
  uint16_t u16ConnHandle;
  gatt_status_code_e eStatus;
} le_srv_read_char_acc_event_st;

/*! contains TCU_MNG_LE_DISCONNECT_EVENT */
typedef struct tag_le_disconnect_st
{
  uint16_t u16ConnHandle;
  uint8_t u8Status;
  uint8_t u8Reason;
} le_disconnect_st;

/*! contains TCU_LE_GATT_SER_CHAR_VAL_INDICATION_EVENT */
typedef struct tag_le_srv_char_cal_ind_st
{
  uint16_t u16ConnHandle;
  gatt_status_code_e eStatus;
} le_srv_char_cal_ind_st;

/*! contains TCU_LE_GATT_SER_WRITE_WITHOUT_RESPONSE_CMD_EVENT */
typedef struct tag_le_srv_wrt_wo_evt_st
{
  uint16_t u16ConnHandle;
  uint16_t u16CharValHandle;
  uint16_t u16ValDataLen;
  uint8_t*  pu8CharValData;
} le_srv_wrt_wo_evt_st;

/**
 * @brief Command Acknowledgement parser.
 *        To be called when TCU_LE_ACCEPT is received.
 *
 * @param[in]  pu8Data pointer to received data array
 * @param[out] pu8ServID Received Service ID
 * @param[out] pu8OpCode Received Op-Code
 * @param[out] eStatus Status of the last sent command
 *
 * @return LE_FUNC_SUCCCESS  Completed succesfully
 * @return LE_ERR_CMD_ERR_OPCODE Op-code missmatch
 * @return LE_ERR_CMD_ERR_SVC_ID Service ID mismatch
 */
le_func_status_e eGetLeAccept_Ack(uint8_t* pu8Data, le_accept_st* pstEvntData);

/**
 * @brief  Command Rejection Acknowledgement parser.
 *        To be called when TCU_LE_NOT_ACCEPT is received.
 *
 * @param[in]  pu8Data Pointer to received data array
 * @param[out] pu8ServID Pointer to Service ID
 * @param[out] pu8OpCode  Command Op-code
 *
 * @return  LE_FUNC_SUCCCESS Completed succesfully
 * @return  LE_ERR_CMD_ERR_OPCODE Op-code missmatch
 * @return  LE_ERR_CMD_ERR_SVC_ID Service ID mismatch
 */
le_func_status_e eGetLeNOTAcceptStatus_Ack(uint8_t* pu8Data, uint8_t* pu8ServID, uint8_t* pu8OpCode);


/**
 * @brief   Fatal Error Notification parser. To be called when TCU_LE_FATAL_ERROR is received.
 *          This error is indicated to host when some abnormalities occur in module which is not expected
 *          inside a module to occur and that cannot be recovered.
 *
 * @param[in]     pu8Data Pointer to received data array
 * @param[out]    pu8ErrorCode Returned Error Code
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return    LE_ERR_CMD_ERR_OPCODE Op-code missmatch
 * @return    LE_ERR_CMD_ERR_SVC_ID Service ID mismatch
 */
le_func_status_e eGetLeFatalError_Notfy(uint8_t* pu8Data, uint8_t* pu8ErrorCode);

/**
 * @brief   BT Low Energy device Initialization. The device is reset to a known state.
 *          The local supported features are read to check that low energy is supported on this Controller. Event mask and LE
 *          event mask are set to enable the events to generate to the Host.
 *          The commands are issued to check the availability of the buffers.
 *          Locally supported LE features are identified. When this command
 *          processing is completed, device initialization response
 *          TCU_MNG_LE_INIT_RESP is obtained.
 *
 * @param[in]   pu8DevName The device name of the local device in UTF-8 format
 * @param[in]   u8NameLen  Length in bytes of the device name
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return    LE_FUNC_ERR_DEV_NAME Invalid device name length
 */
le_func_status_e eSetLeMngInit_Req(uint8_t* pu8DevName, uint8_t u8NameLen);

/**
 * @brief         LE Device Initialization Response parser.
 *
 * @param[in]     pu8Data pointer to received data array
 * @param[out]    peStatus  Returned Result Code
 * @param[out]    pu8BdAddr Local device BD_ADDR
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return    LE_ERR_CMD_ERR_OPCODE Op-code missmatch
 * @return    LE_ERR_CMD_ERR_SVC_ID Service ID mismatch
 */
le_func_status_e eGetLeMngInit_Resp(uint8_t* pu8Data, le_mnginit_resp_st* pstResp);

/**
 * @brief       Sets the scan parameters and set the device in scan mode.
 *              Scanning is used to discover advertising devices nearby. When this command processing is completed,
 *              set scan enable response TCU_MNG_LE_SET_SCAN_ENABLE_RESP is obtained.
 *
 * @param[in]     stConfig Scan configuration
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 */
le_func_status_e eSetLeMngSetScanEnable_Req(set_scan_en_st stConfig);

/**
 * @brief       Scan enable Response parser.
 *              To be called when TCU_MNG_LE_SET_SCAN_ENABLE_RESP is received.
 *
 *
 * @param[in]     pu8Data pointer to received data array
 * @param[out]    pu8Status Result Code
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return    LE_ERR_CMD_ERR_OPCODE Op-code missmatch
 * @return    LE_ERR_CMD_ERR_SVC_ID Sservice ID mismatch
 */
le_func_status_e eGetLeMngSetScanEnable_Resp(uint8_t* pu8Data, uint8_t* pu8Status);

/**
 * @brief      Advertizing Report Event parser. !!!!!! TBD !!!!!!
 *             To be called when TCU_MNG_LE_ADV_REPORT_EVENT is received.
 *
 * @param[in]     pu8Data pointer to received data array

 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return    LE_ERR_CMD_ERR_OPCODE Op-code missmatch
 * @return    LE_ERR_CMD_ERR_SVC_ID Sservice ID mismatch
 */
le_func_status_e eGetLeAdvReport_Event(uint8_t* pu8Data, uint8_t* pu8Status, uint8_t* pu8SubEvtCode, uint8_t* pu8NumReports, le_adv_report_evnt_st* pstReports);

/**
 * @brief      LE Create Connection Request.
 *             This command is used by the device in Scan Mode (LE Central Device)
 *             to create a Link Layer connection to a connectable advertiser
 *             (LE Peripheral Device). When this command processing is started
 *             TCU_LE_ACCEPT is generated.
 *
 *
 *
 * @param[in]   stConfig Connection configuration structure
 *
 * @return      LE_FUNC_SUCCCESS Completed succesfully
 * @return
 * @return
 */
le_func_status_e eSetLeMngCreateConn_Req(le_create_conn_st stConfig);

/**
 * @brief  This response indicates to both of the Hosts (i.e. to both Central and Peripheral Device)
 *         entering the connection that connection has been established when a new connection is established,
 *
 * @param[in]     pu8Data Pointer to received data array
 * @param[out]    pstEvent Pointer to connection event structure
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return    LE_ERR_CMD_ERR_OPCODE Op-code missmatch
 * @return    LE_ERR_CMD_ERR_SVC_ID Sservice ID mismatch
 */
le_func_status_e eGetLeMngConnComplete_Event(uint8_t* pu8Data, le_conn_evt_st* pstEvent);

/**
 * @brief   This command is used by the device (LE Central Device) which
 *          issued the Connection Request to cancel the LE Create Connection
 *          command. When this command processing is completed, LE Create
 *          Connection Cancel response
 *
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 */
le_func_status_e eSetLeMngCreateConnCancel_Req(void);

/**
 * @brief       Response parser.
 *              To be called when TCU_MNG_LE_CREATE_CONNECTION_CANCEL_RESP is received.
 *
 * @param[in]     pu8Data pointer to received data array
 * @param[out]    puStatus Response status
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return    LE_ERR_CMD_ERR_OPCODE Op-code missmatch
 * @return    LE_ERR_CMD_ERR_SVC_ID Sservice ID mismatch
 */
le_func_status_e eGetLeMngCreateConnCancel_Resp(uint8_t* pu8Data, uint8_t* puStatus);

/**
 * @brief   The Disconnection command is used by the local device to terminate
 *          an existing connection.When this command processing is started
 *          TCU_LE_ACCEPT is generated.
 *
 * @param[in]     pu8Data pointer to received data array
 * @param[in,out]
 * @param[in,out]
 * @param[in,out]
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return
 * @return
 */
le_func_status_e eSetLeDisconnect_Req(uint8_t* pu8BdAddr);

/**
 * @brief       Response parser.
 *              To be called when TCU_MNG_LE_DISCONNECT_EVENT is received.
 *
 * @param[in]  pu8Data pointer to received data array
 * @param[out] pstEvntData Pointer to returned status and disconnection reason
 * @param[out] pu8Reason Reason for disconnecting

 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return    LE_ERR_CMD_ERR_OPCODE Op-code missmatch
 * @return    LE_ERR_CMD_ERR_SVC_ID Sservice ID mismatch
 */
le_func_status_e eGetLeDisconnect_Event(uint8_t* pu8Data, le_disconnect_st* pstEvntData);

/**
 * @brief   This request is sent to disable scan in the device which is in
 *          Scanning Mode.Once the command is processed the device stops
 *          Scanning and Set Scan Disable response is obtained.
 *
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return
 * @return
 */
le_func_status_e eSetLeScanDisable_Req(void);

/**
 * @brief       Response parser.
 *              To be called when TCU_MNG_LE_SET_SCAN_DISABLE_RESP is received.
 *
 * @param[in]     pu8Data pointer to received data array
 * @param[out]    puStatus Response status
 * @param[in,out]
 * @param[in,out]
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return    LE_ERR_CMD_ERR_OPCODE Op-code missmatch
 * @return    LE_ERR_CMD_ERR_SVC_ID Sservice ID mismatch
 */
le_func_status_e eGetLeScanDisable_Resp(uint8_t* pu8Data, uint8_t* pu8Status);

/**
 * @brief Start Advertise Request.
 *          This request when issued will carry below functionalities:
 *             - Set the Advertising Parameters.
 *             - Set the data used in Advertising Packets that have a data field.
 *             - Set the data used in Scanning Packets that have a data field.
 *             - Request the Controller to start advertising
 *          When this command processing is completed, start advertise response
 *          TCU_MNG_LE_ START_ADVERTISE_RESP is obtained.
 *
 * @param[in]    stConfig Configuration parameters for advertizing
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return
 * @return
 */
le_func_status_e eSetLeStartAdvertise_Req(le_adv_req_st stConfig);

/**
 * @brief       Response parser.
 *              To be called when TCU_MNG_LE_START_ADVERTISE_RESP is received.
 *
 * @param[in]     pu8Data pointer to received data array
 * @param[out]    puStatus Response status
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return    LE_ERR_CMD_ERR_OPCODE Op-code missmatch
 * @return    LE_ERR_CMD_ERR_SVC_ID Sservice ID mismatch
 */
le_func_status_e eGetLeStartAdvertise_Resp(uint8_t* pu8Data, uint8_t* pu8Status);

/**
 * @brief   Stops Advertising
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 */
le_func_status_e eSetLeDisableAdvertise_Req(void);

/**
 * @brief       Response parser.
 *              To be called when TCU_MNG_LE_DISABLE_ADVERTISE_RESP is received.
 *
 * @param[in]     pu8Data pointer to received data array
 * @param[out]    puStatus Response status
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return    LE_ERR_CMD_ERR_OPCODE Op-code missmatch
 * @return    LE_ERR_CMD_ERR_SVC_ID Sservice ID mismatch
 */
le_func_status_e eGetLeDisableAdvertise_Resp(uint8_t* pu8Data, uint8_t* pu8Status);

/* GATT Server Database commands */

/**
 * @brief  This command is used to add primary service attributes to the server database.
 *         The Attribute type will be set as 0x2800 since the request is for adding primary service.
 *         The Attribute value to be added will be contained in the command.
 *
 * @param[in]     eUuidLen Length of the attribute value
 * @param[in]     pu8UUIDVal This attribute value is a UUID which can either be 2 bytes or 16 bytes
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return
 * @return
 */
le_func_status_e eSetSdbGattAddPrimaryService_Req(uuid_type_e eUuidLen, uint8_t* pu8UUIDVal);

/**
 * @brief       Response parser.
 *              To be called when TCU_LE_GATT_SDB_ADD_PRIM_SVC_RESP is received.
 *
 * @param[in]     pu8Data pointer to received data array
 * @param[out]    pu8Status  Response status.
 * @param[out]    pu16Handle Returned primary service handle.
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return    LE_ERR_CMD_ERR_OPCODE Op-code missmatch
 * @return    LE_ERR_CMD_ERR_SVC_ID Sservice ID mismatch
 */
le_func_status_e eGetSdbGattAddPrimaryService_Resp(uint8_t* pu8Data, uint8_t* pu8Status, uint16_t* pu16Handle);

/**
 * @brief     Inserts a secondary service under given primary service.
 *
 *
 * @param[in]   eUuidLen Length of a UUID Value
 * @param[in]   u16PrimSvcHandle Service Handle of the primary under which the
 *                               secondary service is about to be added.
 * @param[in]   pu8UUIDVal       UUID value of the secondary service to be added.
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return    LE_ERR_CMD_ERR_OPCODE Op-code missmatch
 * @return    LE_ERR_CMD_ERR_SVC_ID Sservice ID mismatch
 */
le_func_status_e eSetSdbGattAddSecondaryService_Req(uuid_type_e eUuidLen, uint16_t u16ParentSvcHandle, uint8_t* pu8UUIDVal);

/**
 * @brief       Response parser.
 *              To be called when TCU_LE_GATT_SDB_ADD_SEC_SVC_RESP is received.
 *
 * @param[in]     pu8Data pointer to received data array.
 * @param[out]    puStatus Response status.
 * @param[out]    pu16Handle Returned secondary service handle.
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return    LE_ERR_CMD_ERR_OPCODE Op-code missmatch
 * @return    LE_ERR_CMD_ERR_SVC_ID Sservice ID mismatch
 */
le_func_status_e eGetSdbGattAddSecondaryService_Resp(uint8_t* pu8Data, uint8_t* pu8Status, uint16_t* pu16Handle);

/**
 * @brief       Used to add characteristic declaration for a particular primary or secondary service.
 *              The service handle for which characteristics needs to be declared is contained in the command format.
 *
 * @param[in]   u16Handle handle of the primary service or secondary service under which the characteristic needs to be declared.
 * @param[in]   u8Properties property of the characteristic added to the server database. Please use gatt_char_prop_mask_e enumerations
 * @param[in]   eUuidLen length of the UUID value of the characteristic declaration to be added to the server database
 * @param[in]   pu8UUIDVal UUID value of the characteristic declaration to be added. This is a UUID which can either be 2 bytes or 16 bytes
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return
 * @return
 */
le_func_status_e eSetSdbGattAddCharacterDecl_Req(uint16_t u16Handle, uint8_t u8Properties, uuid_type_e eUuidLen, uint8_t* pu8UUIDVal);

/**
 * @brief       Response parser.
 *              To be called when TCU_LE_GATT_SDB_ADD_CHAR_DECL_RESP is received.
 *
 * @param[in]   pu8Data pointer to received data array
 * @param[out]  puStatus Response status
 * @param[out]  pu16Handle handle of the characteristic declaration added to the server database
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return    LE_ERR_CMD_ERR_OPCODE Op-code missmatch
 * @return    LE_ERR_CMD_ERR_SVC_ID Sservice ID mismatch
 */
le_func_status_e eGetSdbGattAddCharacterDecl_Resp(uint8_t* pu8Data, gatt_status_code_e* eStatus, uint16_t* pu16Handle);

/**
 * @brief       Response parser.
 *              To be called when TCU_LE_GATT_SER_READ_CHAR_VAL_ACCEPT_RESP is received.
 *
 * @param[in]   pu8Data pointer to received data array
 * @param[out]  puStatus Response status
 * @param[out]  pu16Handle handle of the characteristic declaration added to the server database
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return    LE_ERR_CMD_ERR_OPCODE Op-code missmatch
 * @return    LE_ERR_CMD_ERR_SVC_ID Sservice ID mismatch
 */
le_func_status_e eSetServerGattReadCharValAccept_Resp(uint8_t* pu8Data, le_srv_read_char_acc_event_st* pstEvntData);

/**
 * @brief       Used to add elements like Characteristic Value Declaration, Characteristic Descriptor Declaration etc for the Characteristic declared.
 *              Handle of the characteristic for which elements need to be declared is contained in the command format.
 *
 * @param[in]   u16Handle Characteristic handle to which element belongs
 * @param[in]   stParams Character element structure
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 */
le_func_status_e eSetSdbGattAddCharacterEle_Req(uint16_t  u16Handle, char_el_req_st stParams);

/**
 * @brief       Response parser.
 *              To be called when TCU_LE_GATT_SDB_ADD_CHAR_ELE_RESP is received.
 *
 * @param[in]     pu8Data pointer to received data array
 * @param[out]    peStatus Response status
 * @param[out]    pu16Handle handle of the characteristic element added to the server database
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return    LE_ERR_CMD_ERR_OPCODE Op-code missmatch
 * @return    LE_ERR_CMD_ERR_SVC_ID Sservice ID mismatch
 */
le_func_status_e eGetSdbGattAddCharacterEle_Resp(uint8_t* pu8Data, gatt_status_code_e* peStatus, uint16_t* pu16Handle);

/**
 * @brief         Used to update or change the attribute value for Characteristic Value Declaration,
 *                Characteristic Descriptor Declaration etc for the Characteristic Elements.
 *                Using the handle the server database will identify the attribute element
 *                (like Characteristic Value, Characteristic Format, and Characteristic User Descriptor etc) whose value need to be changed/Updated.
 *
 * @param[in]     u16Handle handle of the characteristic element to be updated.
 * @param[in]     u16AttValLen length of the characteristic element to be updated
 * @param[in]     pu8AttVal attribute value to be updated in the server database
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 */
le_func_status_e eSetSdbGattUpdateCharacterEle_Req(uint16_t u16Handle, uint16_t u16AttValLen, uint8_t* pu8AttVal);

/**
 * @brief       Response parser.
 *              To be called when TCU_LE_GATT_SDB_UPD_CHAR_ELE_RESP is received.
 *
 * @param[in]     pu8Data pointer to received data array
 * @param[out]    puStatus Response status
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return    LE_ERR_CMD_ERR_OPCODE Op-code missmatch
 * @return    LE_ERR_CMD_ERR_SVC_ID Sservice ID mismatch
 */
le_func_status_e eGetSdbGattUpdateCharacterEle_Resp(uint8_t* pu8Data, gatt_status_code_e* eStatus);

/**
 * @brief
 *
 * @param[in]  u16Handle handle of the service for which the end handle is needed to be found
 * @param[out] stIncParam
 *
 * @return     LE_FUNC_SUCCCESS Completed succesfully
 */
le_func_status_e eSetSdbGattAddInclSvc_Req(uint16_t u16SvcHandle,
                                           uuid_type_e eAttValLen,
                                           uint8_t* pu8AttVal);

/**
 * @brief       Response parser.
 *              To be called when TCU_LE_GATT_SDB_ADD_INC_SVC_RESP is received.
 *
 * @param[in]  pu8Data
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return    LE_ERR_CMD_ERR_OPCODE Op-code missmatch
 * @return    LE_ERR_CMD_ERR_SVC_ID Sservice ID mismatch
 */
le_func_status_e eGetSdbGattAddInclSvc_Resp(uint8_t* pu8Data,
                                            le_resp_st* pstRespData);

/**
 * @brief       Used to obtain the end group handle for a specified Service.
 *              The service for which end group handle is desired is specified as a part of the command.
 *              The Handle specified in the command can be the handle of any primary or secondary service
 *              (even nested secondary service) which is already declared in the server database
 *
 * @param[in]     u16Handle handle of the service for which the end handle is needed to be found
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 */
le_func_status_e eSetSdbGattReturnEndGroupHandle_Req(uint16_t u16Handle);

/**
 * @brief       Response parser.
 *              To be called when TCU_LE_GATT_SDB_RET_END_GRP_HLE_RESP is received.
 *
 * @param[in]   pu8Data pointer to received data array
 * @param[out]  peStatus returned status
 * @param[out]  pu16EndGroupHandle  end group handle of the requested service
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return    LE_ERR_CMD_ERR_OPCODE Op-code missmatch
 * @return    LE_ERR_CMD_ERR_SVC_ID Sservice ID mismatch
 */
le_func_status_e eGetSdbGattReturnEndGroupHandle_Resp(uint8_t* pu8Data, le_resp_st* pstRespData);

/*** Bluetooth LE GATT  ***/

/* Server side commands */
/**
 * @brief Used by the host application to initialize the GATT server.
 * The GATT server must be initialized before any GATT command is issued to the server device
 *
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return
 * @return
 */
le_func_status_e eSetServerGattInit_Req(void);

/**
 * @brief       Response parser.
 *              To be called when TCU_LE_GATT_SER_INIT_RESP is received.
 *
 * @param[in]     pu8Data pointer to received data array
 * @param[out]    puStatus Response status
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return    LE_ERR_CMD_ERR_OPCODE Op-code missmatch
 * @return    LE_ERR_CMD_ERR_SVC_ID Sservice ID mismatch
 */
le_func_status_e eGetServerGattInit_Resp(uint8_t* pu8Data, gatt_status_code_e* eStatus);

/**
 * @brief       Response parser. 
 *              To be called when TCU_LE_GATT_SER_WRITE_CHAR_DESP_EVENT is received.
 *
 * @param[in]     pu8Data pointer to received data array
 * @param[out]   pstEvntData pointer to returned data structure
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return    LE_ERR_CMD_ERR_OPCODE Op-code missmatch
 * @return    LE_ERR_CMD_ERR_SVC_ID Sservice ID mismatch
 */
le_func_status_e eGetServerGattWriteCharDesc_Event(uint8_t* pu8Data, le_srv_write_char_desp_event_st* pstEvntData);

/**
 * @brief   The Accept Request is generated by server application
 *          when it writes the characteristic descriptor to server database.
 *          On receiving this request the GATT server will send the response to client
 *          device indicating the status of write request.
 *
 * @param[in]     u16ConnHandle index for accessing the remote device.
 * @param[in]     eStatus result code
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 */
le_func_status_e eSetServerGattWriteCharDescAccept_Req(uint16_t u16ConnHandle,gatt_status_code_e eStatus, uint16_t u16DescHle);

/**
 * @brief       Response parser. 
 *              To be called when TCU_LE_GATT_SER_WRITE_CHAR_VAL_EVENT is received.
 *
 * @param[in]     pu8Data pointer to received data array
 * @param[out]   pstEvntData pointer to returned data structure
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return    LE_ERR_CMD_ERR_OPCODE Op-code missmatch
 * @return    LE_ERR_CMD_ERR_SVC_ID Sservice ID mismatch
 */
le_func_status_e eGetServerGattWriteCharVal_Event(uint8_t* pu8Data,le_srv_write_char_desp_event_st* pstEvntData);

/**
 * @brief       Response parser.
 *              To be called when TCU_LE_GATT_SER_READ_CHAR_VAL_EVENT is received.
 *
 * @param[in]   pu8Data pointer to received data array
 * @param[out]  pstEvntData data returned by event
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return    LE_ERR_CMD_ERR_OPCODE Op-code missmatch
 * @return    LE_ERR_CMD_ERR_SVC_ID Sservice ID mismatch
 */
le_func_status_e eGetServerGattReadCharVal_Event(uint8_t* pu8Data, le_srv_read_char_val_event_st* pstEvntData);

/**
 * @brief       The server application will send this response to GATT server, once it updates the characteristic value.
 *              The status flag must indicate whether updation was successful or resulted in error.
 *
 * @param[in]   u16ConnHandle index for accessing the remote device.
 * @param[in]   eStatus  result code
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return
 * @return
 */
le_func_status_e eSetServerGattReadCharValAccept_Req(uint16_t u16ConnHandle, gatt_status_code_e eStatus, uint16_t u16ReadCharHle);

/**
 * @brief       Response parser.
 *              To be called when TCU_LE_GATT_SER_EXG_MTU_EVENT is received.
 *
 * @param[in]   pu8Data pointer to received data array
 * @param[out]  pstMtuData MTU connection data (Connection handler & Clients max. receive MTU size)
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return    LE_ERR_CMD_ERR_OPCODE Op-code missmatch
 * @return    LE_ERR_CMD_ERR_SVC_ID Sservice ID mismatch
 */
le_func_status_e eGetServerGattExgMtu_Event(uint8_t* pu8Data, le_mtu_exchg_st* pstMtuData);



/**
 * @brief       Sends MTU Size to the client side TBC !!!
 *
 *
 * @param[in]   pu8param pointer to parameter array
 * @param[out]
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 */
le_func_status_e eGetServerGattExgMtuAccept_Req(uint16_t u16ConnHandle, uint8_t  u8Status, uint16_t u16ServerRxMtuSize);

/**
 * @brief       The server application will send this response to GATT server,
 *              once it updates all the handles requested in read multiple events.
 *              The status flag must indicate whether updation was successful or resulted in error.
 *              On receiving this response the GATT server will read the updated values using the handles
 *              and will send the response to client device.
 *
 * @param[in]   u16ConnHandle Connection handle
 * @param[in]   eStatus Status code reported to the server
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 */
le_func_status_e eSetServerGattReadMultiAccept_Req(uint16_t u16ConnHandle, gatt_status_code_e eStatus, uint8_t *pu8ReadMulHle);

/**
 * @brief       Response Parser TBC!!!!
 *              To be called when TCU_LE_GATT_SER_EXG_MTU_ACCEPT_RESP has been received
 *
 * @param[in]   pu8Data pointer to received data array
 * @param[out]
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 */
le_func_status_e eGetServerGattExgMtuAccept_Resp(uint8_t* pu8Data, le_mtu_exchg_acc_resp_st* stResp);

/**
 * @brief       Response parser.
 *              To be called when TCU_LE_GATT_SER_WRITE_CHAR_VAL_EVENT is received.
 *
 * @param[in]   pu8Data pointer to received data array
 * @param[out]  pstEvntData returned event data
 * @param[out]  pu16CharValHandle characteristic Value Handle to read the value of the characteristic descriptors
 * @param[out]  pu8CharValData characteristic Attribute value to be written onto server
 * @param[out]  pu16ValDataLen data length in bytes
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return    LE_ERR_CMD_ERR_OPCODE Op-code missmatch
 * @return    LE_ERR_CMD_ERR_SVC_ID Sservice ID mismatch
 */
le_func_status_e eGetServerGattWriteCharVal_Event(uint8_t* pu8Data,le_srv_write_char_desp_event_st* pstEvntData);

/**
 * @brief       Generated by server application when it updates the characteristic value
 *              send by GATT server onto server database. On receiving this Accept Request
 *              the GATT server will send the response to client device indicating the status of write reques.
 *
 * @param[in]   u16ConnHandle ndex for accessing the remote device
 * @param[in]   eStatus result code
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 */
le_func_status_e eSetServerGattWriteCharValAccept_Req(uint16_t u16ConnHandle, gatt_status_code_e eStatus, uint16_t u16ValHle);

/**
 * @brief       On receiving the Accept Request from application, the GATT server will send the response to client device
 *              indicating the status of write request.
 *              Also the GATT server will send the Accept Response to Server Application
 *
 * @param[in]   pu8Data pointer to received data array
 * @param[in]   pstEvntData pointer to returned response data
 *
 * @return      LE_FUNC_SUCCCESS Completed succesfully
 */
le_func_status_e eGetServerGattWriteCharValAccept_Resp(uint8_t* pu8Data, le_srv_multi_acc_event_st* pstEvntData);

/**
 * @brief       On receiving the Accept Request from application, the GATT server will send the response to client device
 *              indicating the status of write request.
 *              Also the GATT server will send the Accept Response to Server Application
 *
 * @param[in]   pu8Data pointer to received data array
 * @param[in]   pstEvntData pointer to returned response data
 *
 * @return      LE_FUNC_SUCCCESS Completed succesfully
 */
le_func_status_e eGetServerGattWriteCharDespAccept_Resp(uint8_t* pu8Data, le_srv_multi_acc_event_st* pstEvntData);


/**
 * @brief       Response parser.
 *              To be called when TCU_LE_GATT_SER_READ_MULTIPLE_ACCEPT_RESP is received.
 *
 * @param[in]     pu8Data pointer to received data array
 * @param[out]    peStatus Response status
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return    LE_ERR_CMD_ERR_OPCODE Op-code missmatch
 * @return    LE_ERR_CMD_ERR_SVC_ID Sservice ID mismatch
 */
le_func_status_e eGetServerGattMultiAccept_Resp(uint8_t* pu8Data, uint16_t* pu16ConnHandle, gatt_status_code_e* peStatus);

/**
 * @brief       Response parser.
 *              To be called when TCU_LE_GATT_SER_READ_MULTIPLE_ACCEPT_RESP is received.
 *
 * @param[in]     pu8Data pointer to received data array
 * @param[out]    pstEvntData Response data
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return    LE_ERR_CMD_ERR_OPCODE Op-code missmatch
 * @return    LE_ERR_CMD_ERR_SVC_ID Sservice ID mismatch
 */
le_func_status_e eGetServerGattReadMultiAccept_Resp(uint8_t* pu8Data, le_srv_multi_acc_event_st* pstEvntData);

/**
 * @brief       Response parser.
 *              To be called when TCU_LE_GATT_SER_READ_MULTIPLE_EVENT is received.
 *
 * @param[in]     pu8Data pointer to received data array
 * @param[out]    pu16ConnHandle connection handle
 * @param[out]    pu16HndlCnt number of handles in the list
 * @param[out]    pu8HndlList list of returned handles
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return    LE_ERR_CMD_ERR_OPCODE Op-code missmatch
 * @return    LE_ERR_CMD_ERR_SVC_ID Sservice ID mismatch
 */
le_func_status_e eGetServerGattReadMulti_Event(uint8_t* pu8Data, le_srv_multi_event_st* pstEvntData);

/**
 * @brief T.B.D.
 *
 * @param[in]  u16ConnHandle
 * @param[in]  u16CharValHandle
 * @param[in]  pu8CharValData
 * @param[in]  u16ValDataLen
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return
 * @return
 */
le_func_status_e eSetServerGattValNotify_Req(uint16_t u16ConnHandle, uint16_t u16CharValHandle,  uint8_t* pu8CharValData, uint16_t u16ValDataLen);

/**
 * @brief       Response parser.
 *              To be called when TCU_LE_GATT_SER_CHAR_VAL_NOTIFICATION_EVENT is received.
 *
 * @param[in]   pu8Data pointer to received data array
 * @param[out]  pu16ConnHandle index for accessing the remote device
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return    LE_ERR_CMD_ERR_OPCODE Op-code missmatch
 * @return    LE_ERR_CMD_ERR_SVC_ID Sservice ID mismatch
 */
le_func_status_e eGetServerGattValNotify_Event(uint8_t* pu8Data, uint16_t* pu16ConnHandle);


/**
 * @brief       Response parser.
 *              To be called when TCU_LE_GATT_SER_WRITE_WITHOUT_RESPONSE_CMD_EVENT is received.
 *
 * @param[in]   pu8Data pointer to received data array
 * @param[out]  pstEvtData Pointer to structure in which returned event data should be copied
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully
 * @return    LE_ERR_CMD_ERR_OPCODE Op-code missmatch
 * @return    LE_ERR_CMD_ERR_SVC_ID Sservice ID mismatch
 */
le_func_status_e eGetServerGattWriteWithoutRespCmd_Event(uint8_t* pu8Data, le_srv_wrt_wo_evt_st* pstEvtData);

 /**
 * @brief       Sends Notification request
 *
 * @param[in]   u16ConnHandle Connection handle.
 * @param[out]  u16CharValHandle Characteristic value handle.
 * @param[out]  u16CharValLen Length of Characteristic value in bytes.
 * @param[out]  pu8UCharVal Pointer to characteristic value.
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully.
 * @return    LE_ERR_CMD_ERR_OPCODE Op-code missmatch.
 * @return    LE_ERR_CMD_ERR_SVC_ID Sservice ID mismatch.
 */
 le_func_status_e eSetServerGattCharValNotification_Req(uint16_t u16ConnHandle, uint16_t u16CharValHandle, uint16_t u16CharValLen, uint8_t* pu8UCharVal);

 /**
 * @brief       Sends Indication request
 *
 * @param[in]   u16ConnHandle Connection handle.
 * @param[out]  u16CharValHandle Characteristic value handle.
 * @param[out]  u16CharValLen Length of Characteristic value in bytes.
 * @param[out]  pu8UCharVal Pointer to characteristic value.
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully.
 * @return    LE_ERR_CMD_ERR_OPCODE Op-code missmatch.
 * @return    LE_ERR_CMD_ERR_SVC_ID Sservice ID mismatch.
 */
 le_func_status_e eSetServerGattCharValIndication_Req(uint16_t u16ConnHandle, uint16_t u16CharValHandle, uint16_t u16CharValLen, uint8_t* pu8UCharVal);

 /**
 * @brief       Response parser.
 *              To be called when TCU_LE_GATT_SER_CHAR_VAL_INDICATION_EVENT is received.
 *
 * @param[in]   pu8Data pointer to received data array.
 * @param[out]  pstEvtData return data from indication event.
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully.
 * @return    LE_ERR_CMD_ERR_OPCODE Op-code missmatch.
 * @return    LE_ERR_CMD_ERR_SVC_ID Sservice ID mismatch.
 */
 le_func_status_e eGetServerGattValIndication_Event(uint8_t* pu8Data, le_srv_char_cal_ind_st* pstEvtData);

 
 /**
 * @brief       Response parser.
 *              To be called when TCU_LE_GATT_SER_READ_CHAR_DESP_EVENT is received.
 *
 * @param[in]   pu8Data pointer to received data array.
 * @param[out]  pstEvtData Return data from read descriptor event.
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully.
 * @return    LE_ERR_CMD_ERR_OPCODE Op-code missmatch.
 * @return    LE_ERR_CMD_ERR_SVC_ID Sservice ID mismatch.
 */
 le_func_status_e eGetServerGattReadCharDescriptor_Event(uint8_t* pu8Data, 
                                                         le_srv_read_char_val_event_st* pstEvntData);
/**
 * @brief       The Accept Request is generated by server application when 
 *              it writes the updated value of characteristic descriptor to server database.
 *             
 *
 * @param[in]   u16ConnHandle Connection handle.
 * @param[out]  u8Status Status information.
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully.
 * @return    LE_ERR_CMD_ERR_OPCODE Op-code missmatch.
 * @return    LE_ERR_CMD_ERR_SVC_ID Sservice ID mismatch.
 */
 le_func_status_e eSetServerGattReadCharDescriptorAccept_Req(uint16_t u16ConnHandle,  uint8_t u8Status, uint16_t u16DescHle);
/**
 * @brief       Response parser.
 *              To be called when TCU_LE_GATT_SER_READ_CHAR_DESP_ACCEPT_RESP is received.
 *
 * @param[in]   pu8Data pointer to received data array.
 * @param[out]  pstEvtData return data from read accept descriptor event.
 *
 * @return    LE_FUNC_SUCCCESS Completed succesfully.
 * @return    LE_ERR_CMD_ERR_OPCODE Op-code missmatch.
 * @return    LE_ERR_CMD_ERR_SVC_ID Sservice ID mismatch.
 */
 le_func_status_e eGetServerGattReadCharDescriptorAccept_Resp(uint8_t* pu8Data, 
                                                              le_srv_read_char_acc_event_st* pstEvntData);
/**
 * @brief       Debugging Printf funtion.
 *
 * @param[in]   pu8Data pointer to received data array.
 * @param[in]   u16Count number of bytes.
 *
 */
#ifdef DEBUG_PRINT
void vDebugRcvdCmdPrintf(uint8_t* pu8Data, uint16_t u16Count);
void vPrintCmd(uint8_t u8ServId, uint8_t u8OpCode);
#endif /* DEBUG_PRINT */


#endif /* _LE_GATT_COMMAND_H */

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

