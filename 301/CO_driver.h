#ifndef CO_DRIVER_H
#define CO_DRIVER_H

#include <string.h>

#include "CO_config.h"
#include "CO_driver_target.h"

// ---------- HANDMADE BEGIN ----------

struct can_file_desc {
    int fd;
    unsigned int status;
};

#ifdef __cplusplus

#include <errno.h>
#include <fcntl.h>
#include <nuttx/board.h>
#include <nuttx/can/can.h>
#include <nuttx/config.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>

#define CAN_DATA_MAX 8
#ifndef LOG
#define LOG(format, ...) { if (logger != NULL) { fprintf(logger, format, ##__VA_ARGS__); fflush(logger); }}
#endif

class canmsg {
        int id_;
        unsigned char len_;
        unsigned char data_[CAN_DATA_MAX];
        FILE* logger_;

    public:
        canmsg(int id) : canmsg(id, NULL) {};
        canmsg(int id, FILE* logger) {
            id_ = id;
            len_ = 0;
            logger_ = logger;
            LOG("[CANMSG] Initialized.\r\n");
        }

        canmsg(int id, unsigned char l, unsigned char *data) : canmsg(id, l, data, NULL) {};
        canmsg(int id, unsigned char l, unsigned char *data, FILE* logger) {
            id_ = id;
            len_ = 0;
            for (size_t i = 0; i < l; i++) data_[i] = data[i];
            logger_ = logger;
            LOG("[CANMSG] Initialized.\r\n");
        }

        canmsg(CO_CAN_t in) : canmsg(in, NULL) {};
        canmsg(CO_CAN_t in, FILE* logger) {
            id_ = in.ident;
            len_ = in.DLC;
            for (size_t i = 0; i < len_; i++) data_[i] = in.data[i];
            logger_ = logger;
            LOG("[CANMSG] Initialized.\r\n");
        }

        void add_data(int data) {
            auto logger = logger_;
            if (len_ == CAN_DATA_MAX) {
                LOG("[CANMSG] Full. Cannot add more data.\r\n");
            } else {
                data_[len_++] = data;
                LOG("[CANMSG] Added more data.\r\n");
            }
        }

        int get_id(void) { return id_; }
        unsigned char get_len(void) { return len_; }
        unsigned char* get_data(void) { return data_; }
        unsigned char get_data(int i) { return data_[i]; }

        CO_CAN_t get_rtx(void) { CO_CAN_t a = {.ident = id_, .DLC = len_, .data = { 0 }, .bufferFull = false, .syncFlag = true}; for (size_t i = 0; i < len_; i++) a.data[i] = data_[i]; return a; }

        void reset(void) {
            len_ = 0;
            memset(data_, 0, CAN_DATA_MAX);
        }

        void reset(int id) {
            id_ = id;
            len_ = 0;
            memset(data_, 0, CAN_DATA_MAX);
        }
};

struct can_file_desc can_init(std::string path, FILE* logger);
void can_deinit(struct can_file_desc fd, FILE* logger);
int can_read(struct can_file_desc fd, canmsg& msg, FILE* logger);
int can_send(struct can_file_desc fd, canmsg& msg, FILE* logger);
int can_setbaud(struct can_file_desc fd, int bauds, FILE* logger);
#endif /* __cplusplus */
#ifndef __cplusplus
typedef void canmsg;
#endif /* ! __cplusplus */
// ---------- HANDMADE END ----------

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** Major version number of CANopenNode */
#define CO_VERSION_MAJOR 4
/** Minor version number of CANopenNode */
#define CO_VERSION_MINOR 0

/* Macros and declarations in following part are only used for documentation. */
#ifdef CO_DOXYGEN
#define CO_LITTLE_ENDIAN                 /**< CO_LITTLE_ENDIAN or CO_BIG_ENDIAN must be defined */
#define CO_SWAP_16(x)    x               /**< Macro must swap bytes, if CO_BIG_ENDIAN is defined */
#define CO_SWAP_32(x)    x               /**< Macro must swap bytes, if CO_BIG_ENDIAN is defined */
#define CO_SWAP_64(x)    x               /**< Macro must swap bytes, if CO_BIG_ENDIAN is defined */
#define NULL             (0)             /**< NULL, for general usage */
#define true             1               /**< Logical true, for general use */
#define false            0               /**< Logical false, for general use */
typedef uint_fast8_t bool_t;             /**< Boolean data type for general use */
typedef signed char int8_t;              /**< INTEGER8 in CANopen (0002h), 8-bit signed integer */
typedef signed int int16_t;              /**< INTEGER16 in CANopen (0003h), 16-bit signed integer */
typedef signed long int int32_t;         /**< INTEGER32 in CANopen (0004h), 32-bit signed integer */
typedef signed long long int int64_t;    /**< INTEGER64 in CANopen (0015h), 64-bit signed integer */
typedef unsigned char uint8_t;           /**< UNSIGNED8 in CANopen (0005h), 8-bit unsigned integer */
typedef unsigned int uint16_t;           /**< UNSIGNED16 in CANopen (0006h), 16-bit unsigned integer */
typedef unsigned long int uint32_t;      /**< UNSIGNED32 in CANopen (0007h), 32-bit unsigned integer */
typedef unsigned long long int uint64_t; /**< UNSIGNED64 in CANopen (001Bh), 64-bit unsigned integer */
typedef float float32_t;  /**< REAL32 in CANopen (0008h), single precision floating point value, 32-bit */
typedef double float64_t; /**< REAL64 in CANopen (0011h), double precision floating point value, 64-bit */

/* Stack configuration default global values. For more information see file CO_config.h. */
#ifndef CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE
#define CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE (0)
#endif
#ifndef CO_CONFIG_GLOBAL_RT_FLAG_CALLBACK_PRE
#define CO_CONFIG_GLOBAL_RT_FLAG_CALLBACK_PRE (0)
#endif
#ifndef CO_CONFIG_GLOBAL_FLAG_TIMERNEXT
#define CO_CONFIG_GLOBAL_FLAG_TIMERNEXT (0)
#endif
#ifndef CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC
#define CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC CO_CONFIG_FLAG_OD_DYNAMIC
#endif
#ifdef CO_DEBUG_COMMON
#if (CO_CONFIG_DEBUG) & CO_CONFIG_DEBUG_SDO_CLIENT
#define CO_DEBUG_SDO_CLIENT(msg) CO_DEBUG_COMMON(msg)
#endif
#if (CO_CONFIG_DEBUG) & CO_CONFIG_DEBUG_SDO_SERVER
#define CO_DEBUG_SDO_SERVER(msg) CO_DEBUG_COMMON(msg)
#endif
#endif

/**
 * CAN receive callback function which pre-processes received CAN message
 *
 * It is called by fast CAN receive thread. Each \ref CO_obj "CANopenNode Object" defines its own and registers it with
 * CO_CANrxBufferInit(), by passing function pointer.
 *
 * @param object pointer to specific \ref CO_obj "CANopenNode Object", registered with CO_CANrxBufferInit()
 * @param rxMsg pointer to received CAN message
 */
void CANrx_callback(void* object, void* rxMsg);

/**
 * CANrx_callback() can read CAN identifier from received CAN message
 *
 * Must be defined in the **CO_driver_target.h** file.
 *
 * This is target specific function and is specific for specific microcontroller. It is best to implement it by using
 * inline function or macro. `rxMsg` parameter should cast to a pointer to structure. For best efficiency structure may
 * have the same alignment as CAN registers inside CAN module.
 *
 * @param rxMsg Pointer to received message
 * @return 11-bit CAN standard identifier.
 */
static inline uint16_t
CO_CANrxMsg_readIdent(void* rxMsg) {
    return 0;
}

/**
 * CANrx_callback() can read Data Length Code from received CAN message
 *
 * See also CO_CANrxMsg_readIdent():
 *
 * @param rxMsg Pointer to received message
 * @return data length in bytes (0 to 8)
 */
static inline uint8_t
CO_CANrxMsg_readDLC(void* rxMsg) {
    return 0;
}

/**
 * CANrx_callback() can read pointer to data from received CAN message
 *
 * See also CO_CANrxMsg_readIdent():
 *
 * @param rxMsg Pointer to received message
 * @return pointer to data buffer
 */
static inline const uint8_t*
CO_CANrxMsg_readData(void* rxMsg) {
    return NULL;
}

/**
 * Configuration object for CAN received message for specific \ref CO_obj "CANopenNode Object".
 *
 * Must be defined in the **CO_driver_target.h** file.
 *
 * Data fields of this structure are used exclusively by the driver. Usually it has the following data fields, but they
 * may differ for different microcontrollers. Array of multiple CO_CANrx_t objects is included inside CO_CANmodule_t.
 */
typedef struct {
    uint16_t ident; /**< Standard CAN Identifier (bits 0..10) + RTR (bit 11) */
    uint16_t mask;  /**< Standard CAN Identifier mask with the same alignment as ident */
    void* object;   /**< \ref CO_obj "CANopenNode Object" initialized in from CO_CANrxBufferInit() */
    void (*pCANrx_callback)(void* object, void* message); /**< Pointer to CANrx_callback() initialized in CO_CANrxBufferInit() */
} CO_CANrx_t;

/**
 * Configuration object for CAN transmit message for specific \ref CO_obj "CANopenNode Object".
 *
 * Must be defined in the **CO_driver_target.h** file.
 *
 * Data fields of this structure are used exclusively by the driver. Usually it has the following data fields, but they
 * may differ for different microcontrollers. Array of multiple CO_CANtx_t objects is included inside CO_CANmodule_t.
 */
typedef struct {
    uint32_t ident;             /**< CAN identifier as aligned in CAN module */
    uint8_t DLC;                /**< Length of CAN message */
    uint8_t data[CAN_DATA_MAX];            /**< 8 data bytes */
    volatile bool_t bufferFull; /**< True if previous message is still in the buffer */
    volatile bool_t syncFlag;   /**< Synchronous PDO messages has this flag set. It prevents them to be sent outside the synchronous window */
} CO_CANtx_t;

/**
 * Complete CAN module object.
 *
 * Must be defined in the **CO_driver_target.h** file.
 *
 * Usually it has the following data fields, but they may differ for different microcontrollers.
 */
typedef struct {
    void* CANptr;                    /**< From CO_CANmodule_init() */
    CO_CANrx_t* rxArray;             /**< From CO_CANmodule_init() */
    uint16_t rxSize;                 /**< From CO_CANmodule_init() */
    CO_CANtx_t* txArray;             /**< From CO_CANmodule_init() */
    uint16_t txSize;                 /**< From CO_CANmodule_init() */
    uint16_t CANerrorStatus;         /**< CAN error status bitfield, see @ref CO_CAN_ERR_status_t */
    volatile bool_t CANnormal;       /**< CAN module is in normal mode */
    volatile bool_t useCANrxFilters; /**< Value different than zero indicates, that CAN module hardware filters are used
                                        for CAN reception. If there is not enough hardware filters, they won't be used.
                                        In this case will be *all* received CAN messages processed by software. */
    volatile bool_t bufferInhibitFlag; /**< If flag is true, then message in transmit buffer is synchronous PDO message,
                                          which will be aborted, if CO_clearPendingSyncPDOs() function will be called by
                                          application. This may be necessary if Synchronous window time was expired. */
    volatile bool_t
        firstCANtxMessage; /**< Equal to 1, when the first transmitted message (bootup message) is in CAN TX buffers */
    volatile uint16_t
        CANtxCount;  /**< Number of messages in transmit buffer, which are waiting to be copied to the CAN module */
    uint32_t errOld; /**< Previous state of CAN errors */
} CO_CANmodule_t;

/**
 * Data storage object for one entry.
 *
 * Must be defined in the **CO_driver_target.h** file.
 *
 * For more information on Data storage see @ref CO_storage or **CO_storage.h** file. Structure members documented here
 * are always required or required with @ref CO_storage_eeprom. Target system may add own additional, hardware specific
 * variables.
 */
typedef struct {
    void* addr;         /**< Address of data to store, always required. */
    size_t len;         /**< Length of data to store, always required. */
    uint8_t subIndexOD; /**< Sub index in OD objects 1010 and 1011, from 2 to 127. Writing 0x65766173 to 1010,subIndexOD
                           will store data to non-volatile memory Writing 0x64616F6C to 1011,subIndexOD will restore
                           default data, always required. */
    uint8_t attr;       /**< Attribute from @ref CO_storage_attributes_t, always required. */
    void* storageModule; /**< Pointer to storage module, target system specific usage, required with @ref
                            CO_storage_eeprom. */
    uint16_t crc; /**< CRC checksum of the data stored in eeprom, set on store, required with @ref CO_storage_eeprom. */
    size_t eepromAddrSignature; /**< Address of entry signature inside eeprom, set by init, required with @ref
                                   CO_storage_eeprom. */
    size_t eepromAddr; /**< Address of data inside eeprom, set by init, required with @ref CO_storage_eeprom. */
    size_t offset; /**< Offset of next byte being updated by automatic storage, required with @ref CO_storage_eeprom. */
    void* additionalParameters; /**< Additional target specific parameters, optional. */
} CO_storage_entry_t;

#define CO_LOCK_CAN_SEND(CAN_MODULE)   /**< Lock critical section in CO_CANsend() */
#define CO_UNLOCK_CAN_SEND(CAN_MODULE) /**< Unlock critical section in CO_CANsend() */
#define CO_LOCK_EMCY(CAN_MODULE)       /**< Lock critical section in CO_errorReport() or CO_errorReset() */
#define CO_UNLOCK_EMCY(CAN_MODULE)     /**< Unlock critical section in CO_errorReport() or CO_errorReset() */
#define CO_LOCK_OD(CAN_MODULE)         /**< Lock critical section when accessing Object Dictionary */
#define CO_UNLOCK_OD(CAN_MODULE)       /**< Unock critical section when accessing Object Dictionary */

/** Check if new message has arrived */
#define CO_FLAG_READ(rxNew)            ((rxNew) != NULL)
/** Set new message flag */
#define CO_FLAG_SET(rxNew)                                                                                             \
    {                                                                                                                  \
        __sync_synchronize();                                                                                          \
        rxNew = (void*)1L;                                                                                             \
    }
/** Clear new message flag */
#define CO_FLAG_CLEAR(rxNew)                                                                                           \
    {                                                                                                                  \
        __sync_synchronize();                                                                                          \
        rxNew = NULL;                                                                                                  \
    }

#endif /* ! CO_DOXYGEN */

#define CO_CAN_ID_NMT_SERVICE 0x000U /**< 0x000 Network management */
#define CO_CAN_ID_GFC         0x001U /**< 0x001 Global fail-safe command */
#define CO_CAN_ID_SYNC        0x080U /**< 0x080 Synchronous message */
#define CO_CAN_ID_EMERGENCY   0x080U /**< 0x080 Emergency messages (+nodeID) */
#define CO_CAN_ID_TIME        0x100U /**< 0x100 Time message */
#define CO_CAN_ID_SRDO_1      0x0FFU /**< 0x0FF Default SRDO1 (+2*nodeID) */
#define CO_CAN_ID_TPDO_1      0x180U /**< 0x180 Default TPDO1 (+nodeID) */
#define CO_CAN_ID_RPDO_1      0x200U /**< 0x200 Default RPDO1 (+nodeID) */
#define CO_CAN_ID_TPDO_2      0x280U /**< 0x280 Default TPDO2 (+nodeID) */
#define CO_CAN_ID_RPDO_2      0x300U /**< 0x300 Default RPDO2 (+nodeID) */
#define CO_CAN_ID_TPDO_3      0x380U /**< 0x380 Default TPDO3 (+nodeID) */
#define CO_CAN_ID_RPDO_3      0x400U /**< 0x400 Default RPDO3 (+nodeID) */
#define CO_CAN_ID_TPDO_4      0x480U /**< 0x480 Default TPDO4 (+nodeID) */
#define CO_CAN_ID_RPDO_4      0x500U /**< 0x500 Default RPDO5 (+nodeID) */
#define CO_CAN_ID_SDO_SRV     0x580U /**< 0x580 SDO response from server (+nodeID) */
#define CO_CAN_ID_SDO_CLI     0x600U /**< 0x600 SDO request from client (+nodeID) */
#define CO_CAN_ID_HEARTBEAT   0x700U /**< 0x700 Heartbeat message */
#define CO_CAN_ID_LSS_SLV     0x7E4U /**< 0x7E4 LSS response from slave */
#define CO_CAN_ID_LSS_MST     0x7E5U /**< 0x7E5 LSS request from master */

/**
 * Restricted CAN-IDs
 *
 * Macro for verifying 'Restricted CAN-IDs', as specified by standard CiA301. They shall not be used for SYNC, TIME,
 * EMCY, PDO and SDO.
 */
#ifndef CO_IS_RESTRICTED_CAN_ID
#define CO_IS_RESTRICTED_CAN_ID(CAN_ID)                                                                                \
    (((CAN_ID) <= 0x7FU) || (((CAN_ID) >= 0x101U) && ((CAN_ID) <= 0x180U))                                             \
     || (((CAN_ID) >= 0x581U) && ((CAN_ID) <= 0x5FFU)) || (((CAN_ID) >= 0x601U) && ((CAN_ID) <= 0x67FU))               \
     || (((CAN_ID) >= 0x6E0U) && ((CAN_ID) <= 0x6FFU)) || ((CAN_ID) >= 0x701U))
#endif

#define CO_CAN_ERRTX_WARNING    0x0001U /**< 0x0001 CAN transmitter warning */
#define CO_CAN_ERRTX_PASSIVE    0x0002U /**< 0x0002 CAN transmitter passive */
#define CO_CAN_ERRTX_BUS_OFF    0x0004U /**< 0x0004 CAN transmitter bus off */
#define CO_CAN_ERRTX_OVERFLOW   0x0008U /**< 0x0008 CAN transmitter overflow */
#define CO_CAN_ERRTX_PDO_LATE   0x0080U /**< 0x0080 TPDO is outside sync window */
#define CO_CAN_ERRRX_WARNING    0x0100U /**< 0x0100 CAN receiver warning */
#define CO_CAN_ERRRX_PASSIVE    0x0200U /**< 0x0200 CAN receiver passive */
#define CO_CAN_ERRRX_OVERFLOW   0x0800U /**< 0x0800 CAN receiver overflow */
#define CO_CAN_ERR_WARN_PASSIVE 0x0303U /**< 0x0303 combination */
#define CO_CAN_GOOD             0x1000U /**< HANDMADE */
#define CO_CAN_DESTROYED        0x2000U /**< HANDMADE */

/**
 * Return values of some CANopen functions. If function was executed successfully it returns 0 otherwise it returns <0.
 */
typedef enum {
    CO_ERROR_NO = 0,                /**< Operation completed successfully */
    CO_ERROR_ILLEGAL_ARGUMENT = -1, /**< Error in function arguments */
    CO_ERROR_OUT_OF_MEMORY = -2,    /**< Memory allocation failed */
    CO_ERROR_TIMEOUT = -3,          /**< Function timeout */
    CO_ERROR_ILLEGAL_BAUDRATE = -4, /**< Illegal baudrate passed to function CO_CANmodule_init() */
    CO_ERROR_RX_OVERFLOW = -5,      /**< Previous message was not processed yet */
    CO_ERROR_RX_PDO_OVERFLOW = -6,  /**< previous PDO was not processed yet */
    CO_ERROR_RX_MSG_LENGTH = -7,    /**< Wrong receive message length */
    CO_ERROR_RX_PDO_LENGTH = -8,    /**< Wrong receive PDO length */
    CO_ERROR_TX_OVERFLOW = -9,      /**< Previous message is still waiting, buffer full */
    CO_ERROR_TX_PDO_WINDOW = -10,   /**< Synchronous TPDO is outside window */
    CO_ERROR_TX_UNCONFIGURED = -11, /**< Transmit buffer was not configured properly */
    CO_ERROR_OD_PARAMETERS = -12,   /**< Error in Object Dictionary parameters */
    CO_ERROR_DATA_CORRUPT = -13,    /**< Stored data are corrupt */
    CO_ERROR_CRC = -14,             /**< CRC does not match */
    CO_ERROR_TX_BUSY = -15,         /**< Sending rejected because driver is busy. Try again */
    CO_ERROR_WRONG_NMT_STATE = -16, /**< Command can't be processed in current state */
    CO_ERROR_SYSCALL = -17,         /**< Syscall failed */
    CO_ERROR_INVALID_STATE = -18,   /**< Driver not ready */
    CO_ERROR_NODE_ID_UNCONFIGURED_LSS = -19 /**< Node-id is in LSS unconfigured state. If objects are handled properly, this may not be an error. */
} CO_ReturnError_t;

void CO_CANsetConfigurationMode(void* CANptr);
void CO_CANsetNormalMode(CO_CANmodule_t* CANmodule);
CO_ReturnError_t CO_CANmodule_init(CO_CANmodule_t* CANmodule, void* CANptr, CO_CANrx_t rxArray[], uint16_t rxSize, CO_CANtx_t txArray[], uint16_t txSize, uint16_t CANbitRate);
void CO_CANmodule_disable(CO_CANmodule_t* CANmodule);
CO_ReturnError_t CO_CANrxBufferInit(CO_CANmodule_t* CANmodule, uint16_t index, uint16_t ident, uint16_t mask, bool_t rtr, void* object, void (*CANrx_callback)(void* object, void* message));
CO_CANtx_t* CO_CANtxBufferInit(CO_CANmodule_t* CANmodule, uint16_t index, uint16_t ident, bool_t rtr, uint8_t noOfBytes, bool_t syncFlag);
CO_ReturnError_t CO_CANsend(CO_CANmodule_t* CANmodule, CO_CANtx_t* buffer);
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t* CANmodule);
void CO_CANmodule_process(CO_CANmodule_t* CANmodule);

/**
 * Get uint8_t value from memory buffer
 *
 * @param buf Memory buffer to get value from.
 *
 * @return Value
 */
static inline uint8_t
CO_getUint8(const void* buf) {
    uint8_t value;
    (void)memmove((void*)&value, buf, sizeof(value));
    return value;
}

/** Get uint16_t value from memory buffer, see @ref CO_getUint8 */
static inline uint16_t
CO_getUint16(const void* buf) {
    uint16_t value;
    (void)memmove((void*)&value, buf, sizeof(value));
    return value;
}

/** Get uint32_t value from memory buffer, see @ref CO_getUint8 */
static inline uint32_t
CO_getUint32(const void* buf) {
    uint32_t value;
    (void)memmove((void*)&value, buf, sizeof(value));
    return value;
}

/**
 * Write uint8_t value into memory buffer
 *
 * @param buf Memory buffer.
 * @param value Value to be written into buf.
 *
 * @return number of bytes written.
 */
static inline uint8_t
CO_setUint8(void* buf, uint8_t value) {
    (void)memmove(buf, (const void*)&value, sizeof(value));
    return (uint8_t)(sizeof(value));
}

/** Write uint16_t value into memory buffer, see @ref CO_setUint8 */
static inline uint8_t
CO_setUint16(void* buf, uint16_t value) {
    (void)memmove(buf, (const void*)&value, sizeof(value));
    return (uint8_t)(sizeof(value));
}

/** Write uint32_t value into memory buffer, see @ref CO_setUint8 */
static inline uint8_t
CO_setUint32(void* buf, uint32_t value) {
    (void)memmove(buf, (const void*)&value, sizeof(value));
    return (uint8_t)(sizeof(value));
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CO_DRIVER_H */
