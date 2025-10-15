#ifndef CO_DRIVER_TARGET_H
#define CO_DRIVER_TARGET_H

/* This file contains device and application specific definitions.
 * It is included from CO_driver.h, which contains documentation
 * for common definitions below. */

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#define CAN_DATA_MAX 8

#ifdef __cplusplus
extern "C" {
#endif /* ! __cplusplus */

/* Stack configuration override default values.
 * For more information see file CO_config.h. */

/* Basic definitions. If big endian, CO_SWAP_xx macros must swap bytes. */
#define CO_LITTLE_ENDIAN
#define CO_SWAP_16(x) x
#define CO_SWAP_32(x) x
#define CO_SWAP_64(x) x

/* NULL is defined in stddef.h */
/* true and false are defined in stdbool.h */
/* int8_t to uint64_t are defined in stdint.h */
typedef uint_fast8_t            bool_t;
typedef float                   float32_t;
typedef double                  float64_t;

typedef struct {
    int ident;                              /**< CAN identifier as aligned in CAN module */
    uint8_t DLC;                            /**< Length of CAN message */
    uint8_t data[CAN_DATA_MAX];             /**< 8 data bytes */
    volatile bool_t bufferFull; /**< True if previous message is still in the buffer */
    volatile bool_t syncFlag;   /**< Synchronous PDO messages has this flag set. It prevents them to be sent outside the synchronous window */
    // void (*pCANrx_callback)(void* object, void* message); /**< Pointer to CANrx_callback() initialized in CO_CANrxBufferInit() */
} CO_CAN_t;

// typedef struct {
    // int ident;                  /**< CAN identifier as aligned in CAN module */
    // uint8_t DLC;                /**< Length of CAN message */
    // uint8_t data[CAN_DATA_MAX]; /**< 8 data bytes */
    // volatile bool_t bufferFull; /**< True if previous message is still in the buffer */
    // volatile bool_t syncFlag;   /**< Synchronous PDO messages has this flag set. It prevents them to be sent outside the synchronous window */
// } CO_CANtx_t;

#define CO_CANrx_t CO_CAN_t
#define CO_CANtx_t CO_CAN_t

/* Access to received CAN message */
#define CO_CANrxMsg_readIdent(msg) ((uint16_t)((CO_CAN_t*)(msg))->ident)
#define CO_CANrxMsg_readDLC(msg)   ((uint8_t)((CO_CAN_t*)(msg))->DLC)
#define CO_CANrxMsg_readData(msg)  ((uint8_t*)((CO_CAN_t*)(msg))->data)

/* CAN module object */
typedef struct {
    void *CANptr;
    CO_CANrx_t *rxArray;
    uint16_t rxSize;
    CO_CANtx_t *txArray;
    uint16_t txSize;
    uint16_t CANerrorStatus;
    volatile bool_t CANnormal;
    volatile bool_t useCANrxFilters;
    volatile bool_t bufferInhibitFlag;
    volatile bool_t firstCANtxMessage;
    volatile uint16_t CANtxCount;
    uint32_t errOld;
} CO_CANmodule_t;

/* Data storage object for one entry */
typedef struct {
    void *addr;
    size_t len;
    uint8_t subIndexOD;
    uint8_t attr;
    /* Additional variables (target specific) */
    void *addrNV;
} CO_storage_entry_t;

/* (un)lock critical section in CO_CANsend() */
#define CO_LOCK_CAN_SEND(CAN_MODULE)
#define CO_UNLOCK_CAN_SEND(CAN_MODULE)

/* (un)lock critical section in CO_errorReport() or CO_errorReset() */
#define CO_LOCK_EMCY(CAN_MODULE)
#define CO_UNLOCK_EMCY(CAN_MODULE)

/* (un)lock critical section when accessing Object Dictionary */
#define CO_LOCK_OD(CAN_MODULE)
#define CO_UNLOCK_OD(CAN_MODULE)

/* Synchronization between CAN receive and message processing threads. */
#define CO_MemoryBarrier()
#define CO_FLAG_READ(rxNew)                     ((rxNew) != NULL)
#define CO_FLAG_SET(rxNew)                      do { CO_MemoryBarrier(); rxNew = (void*)1L; } while (0)
#define CO_FLAG_CLEAR(rxNew)                    do { CO_MemoryBarrier(); rxNew = NULL; } while (0)

#ifdef __cplusplus
}
#endif /* ! __cplusplus */

#endif /* ! CO_DRIVER_TARGET_H */
