#include "301/CO_driver.h"

FILE* GLOBAL_LOGGER = fopen("/dev/ttyS2", "w+"); // open in reading and writing mode

bool can_start(struct can_file_desc fd, FILE* logger) {
    if (logger == NULL) logger = GLOBAL_LOGGER;
    if (fd.status == CO_CAN_DESTROYED) return false;
    LOG("[CAN] Starting !\r\n");
    fd.status = CO_CAN_GOOD;
    return true;
}

bool can_stop(struct can_file_desc fd, FILE* logger) {
    if (logger == NULL) logger = GLOBAL_LOGGER;
    if (fd.status == CO_CAN_DESTROYED) return false;
    LOG("[CAN] Stopping !\r\n");
    fd.status = CO_CAN_ERRTX_BUS_OFF;
    return true;
}

struct can_file_desc can_init(std::string path, FILE* logger) {
    if (logger == NULL) logger = GLOBAL_LOGGER;
    /* Open device */
    struct can_file_desc fd = { .fd = open(path.c_str(), O_RDWR), .status = CO_CAN_DESTROYED };
    if (fd.fd < 0) {
        LOG("[CAN] Failed to open %s %d.\r\n", path.c_str(), -errno);
        return fd;
    }
    fd.status = CO_CAN_ERR_WARN_PASSIVE;
    LOG("[CAN] Initialized.\r\n");
    can_start(fd, logger);
    return fd;
}

void can_deinit(struct can_file_desc fd, FILE* logger) {
    if (logger == NULL) logger = GLOBAL_LOGGER;
    if (fd.status == CO_CAN_DESTROYED) return;
    can_stop(fd, logger);
    LOG("[CAN] Closing !\r\n");
    close(fd.fd);
    fd.status = CO_CAN_DESTROYED;
    LOG("[CAN] Closed.\r\n");
}

int can_read(struct can_file_desc fd, canmsg& msg, FILE* logger) {
    if (logger == NULL) logger = GLOBAL_LOGGER;
    if (fd.status == CO_CAN_DESTROYED) return -2;
    if (fd.status == CO_CAN_ERRTX_BUS_OFF) return -1;
    struct can_msg_s frame;
    size_t             ret;

    /* Read frame */
    size_t msgsize = sizeof(struct can_msg_s);
    ret = read(fd.fd, &frame, msgsize);
    if (ret < CAN_MSGLEN(0) || ret > msgsize) {
        LOG("[CAN] Reading %d returned %d bytes.\r\n", msgsize, ret);
        return -errno;
    }

    /* Check for error reports */
    if (frame.cm_hdr.ch_error != 0) {
        LOG("[CAN] An error has been detected: [%#06lX]\r\n", frame.cm_hdr.ch_id);
        if ((frame.cm_hdr.ch_id & CAN_ERROR_TXTIMEOUT) != 0) LOG("- Transmission timeout\r\n");
        if ((frame.cm_hdr.ch_id & CAN_ERROR_LOSTARB) != 0) LOG("- Lost arbitration: %02x\r\n", frame.cm_data[0]);
        if ((frame.cm_hdr.ch_id & CAN_ERROR_CONTROLLER) != 0) LOG("- Controller error: %02x\r\n", frame.cm_data[1]);
        if ((frame.cm_hdr.ch_id & CAN_ERROR_PROTOCOL) != 0) LOG("- Protocol error: %02x %02x\r\n", frame.cm_data[2], frame.cm_data[3]);
        if ((frame.cm_hdr.ch_id & CAN_ERROR_TRANSCEIVER) != 0) LOG("- Transceiver error: %02x\r\n", frame.cm_data[4]);
        if ((frame.cm_hdr.ch_id & CAN_ERROR_NOACK) != 0) LOG("- No ACK received on transmission\r\n");
        if ((frame.cm_hdr.ch_id & CAN_ERROR_BUSOFF) != 0) LOG("- Bus off\r\n");
        if ((frame.cm_hdr.ch_id & CAN_ERROR_BUSERROR) != 0) LOG("- Bus error\r\n");
        if ((frame.cm_hdr.ch_id & CAN_ERROR_RESTARTED) != 0) LOG("- Controller restarted\r\n");
        return -1;
    }

    /* Convert frame to common format */
    msg.reset(frame.cm_hdr.ch_id);
    // msg.len_ = can_dlc2bytes(frame.cm_hdr.ch_dlc);
    for (int i = 0; i < can_dlc2bytes(frame.cm_hdr.ch_dlc); i++) { // As in send
        msg.add_data(frame.cm_data[i]);
    }
    LOG("[CAN] Message read successfully.\r\n");

    return ret;
}

int can_send(struct can_file_desc fd, canmsg& msg, FILE* logger) {
    if (logger == NULL) logger = GLOBAL_LOGGER;
    if (fd.status == CO_CAN_DESTROYED) return -2;
    if (fd.status == CO_CAN_ERRTX_BUS_OFF) return -1;
    struct can_msg_s frame;
    int                ret;

    /* Convert from common format */
    frame.cm_hdr.ch_id     = msg.get_id();
    frame.cm_hdr.ch_rtr    = false;
    frame.cm_hdr.ch_dlc    = can_bytes2dlc(msg.get_len());
#ifdef CONFIG_CAN_ERRORS
    frame.cm_hdr.ch_error  = 0;
#endif
#ifdef CONFIG_CAN_EXTID
    frame.cm_hdr.ch_extid  = 1;
#endif
    frame.cm_hdr.ch_tcf    = 0;
    for (int i = 0; i < msg.get_len(); i++) { // As in the nuttx example
        frame.cm_data[i] = msg.get_data(i);
    }

    /* Send frame */
    int len = CAN_MSGLEN(msg.get_len());
    ret = write(fd.fd, &frame, len);
    if (ret < 0) {
        LOG("[CAN] Write completely failed %d\r\n", -errno);
        return -1;
    } else if (ret != len) {
        LOG("[CAN] Write partially failed. Wrote %d bytes out of %d. Errno: %d\r\n", ret, len, errno);
        return -1;
    }
    LOG("[CAN] Successfully sent message.\r\n");
    return ret;
}

int can_setbaud(struct can_file_desc fd, int bauds, FILE* logger) { // https://gitlab.fel.cvut.cz/lencmich/nuttx-teensy/-/blob/master/apps/canutils/canlib/canlib_setbaud.c
    if (logger == NULL) logger = GLOBAL_LOGGER;
    if (fd.status == CO_CAN_DESTROYED) return -2;
    if (fd.status == CO_CAN_ERRTX_BUS_OFF) return -1;
    int ret;
    struct canioc_bittiming_s timings;

    ret = ioctl(fd.fd, CANIOC_GET_BITTIMING, (unsigned long)&timings);
    if (ret != OK) {
        LOG("[CAN] CANIOC_GET_BITTIMING failed, errno=%d\n", errno);
        return ret;
    }

    timings.bt_baud = bauds;

    ret = ioctl(fd.fd, CANIOC_SET_BITTIMING, (unsigned long)&timings);
    if (ret != OK) { LOG("[CAN] CANIOC_SET_BITTIMING failed, errno=%d\n", errno); }
    else { LOG("[CAN] Successfully changed baudrate"); }

    return ret;
}

// After this point of the file, all the remaining is heavily inspired by https://github.com/CANopenNode/CanOpenSTM32/blob/master/CANopenNode_STM32/CO_driver_STM32.c
// It is also good to remember that it wasn't tested much more than "It's compiling"
// Note: Update above comment if more testing is done XD

/**
 * CAN receive callback function which pre-processes received CAN message
 *
 * It is called by fast CAN receive thread. Each \ref CO_obj "CANopenNode Object" defines its own and registers it with
 * CO_CANrxBufferInit(), by passing function pointer.
 *
 * @param object pointer to specific \ref CO_obj "CANopenNode Object", registered with CO_CANrxBufferInit()
 * @param rxMsg pointer to received CAN message
 */
void CANrx_callback(void* object, void* rxMsg) {
    auto logger = GLOBAL_LOGGER;
    LOG("[ATTENTION] This function IS indeed called ! [ATTENTION]");
}

/**
 * Request CAN configuration (stopped) mode and *wait* until it is set.
 *
 * @param CANptr Pointer to CAN device
 */
void CO_CANsetConfigurationMode(void* CANptr) {
    if (CANptr)
        can_stop(*static_cast<struct can_file_desc*>(CANptr), NULL);
}

/**
 * Request CAN normal (operational) mode and *wait* until it is set.
 *
 * @param CANmodule CO_CANmodule_t object.
 */
void CO_CANsetNormalMode(CO_CANmodule_t* CANmodule) {
    if (CANmodule && CANmodule->CANptr)
        if (can_start(*static_cast<struct can_file_desc*>(CANmodule->CANptr), NULL))
            CANmodule->CANnormal = true;
}

/**
 * Initialize CAN module object.
 *
 * Function must be called in the communication reset section. CAN module must be in Configuration Mode before.
 *
 * @param CANmodule This object will be initialized.
 * @param CANptr Pointer to CAN device.
 * @param rxArray Array for handling received CAN messages
 * @param rxSize Size of the above array. Must be equal to number of receiving CAN objects.
 * @param txArray Array for handling transmitting CAN messages
 * @param txSize Size of the above array. Must be equal to number of transmitting CAN objects.
 * @param CANbitRate Valid values are (in kbps): 10, 20, 50, 125, 250, 500, 800, 1000. If value is illegal, bitrate
 * defaults to 125.
 *
 * Return #CO_ReturnError_t: CO_ERROR_NO or CO_ERROR_ILLEGAL_ARGUMENT.
 */
CO_ReturnError_t CO_CANmodule_init(CO_CANmodule_t* CANmodule, void* CANptr, CO_CANrx_t rxArray[], uint16_t rxSize, CO_CANtx_t txArray[], uint16_t txSize, uint16_t CANbitRate) {
    if (CANmodule == NULL || rxArray == NULL || txArray == NULL) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }
    switch (CANbitRate) {
        case 10:
        case 20:
        case 50:
        case 125:
        case 250:
        case 500:
        case 800:
        case 1000:
            break;
        default:
            CANbitRate = 125;
            break;
    }
    if (!can_start(*static_cast<struct can_file_desc*>(CANptr), NULL)) return CO_ERROR_ILLEGAL_ARGUMENT;
    CANmodule->CANptr = CANptr;
    CANmodule->rxArray = rxArray;
    CANmodule->rxSize = rxSize;
    CANmodule->txArray = txArray;
    CANmodule->txSize = txSize;
    can_setbaud(*static_cast<struct can_file_desc*>(CANptr), CANbitRate * 1000, NULL);
    CANmodule->CANnormal = false;
    CANmodule->useCANrxFilters = false;
    CANmodule->bufferInhibitFlag = false;
    CANmodule->firstCANtxMessage = true;
    CANmodule->CANtxCount = 0U;
    CANmodule->CANerrorStatus = 0;
    CANmodule->errOld = 0U;

    for (int tmp = 0; tmp < 2; tmp++) {
        auto size = tmp == 0 ? rxSize : txSize;
        auto arr = tmp == 0 ? rxArray : txArray;
        for (uint16_t i = 0U; i < size; i++) {
            arr[i].ident = 0U;
            arr[i].DLC = 0U;
            for (size_t j = 0; j < CAN_DATA_MAX; j++)
                arr[i].data[j] = 0;
            arr[i].bufferFull = false;
            arr[i].syncFlag = false;
        }
    }

    return CO_ERROR_NO;
}

/**
 * Switch off CANmodule. Call at program exit.
 *
 * @param CANmodule CAN module object.
 */
void CO_CANmodule_disable(CO_CANmodule_t* CANmodule) {
    can_deinit(*static_cast<struct can_file_desc*>(CANmodule->CANptr), NULL);
}

/**
 * Configure CAN message receive buffer.
 *
 * Function configures specific CAN receive buffer. It sets CAN identifier and connects buffer with specific object.
 * Function must be called for each member in _rxArray_ from CO_CANmodule_t.
 *
 * @param CANmodule This object.
 * @param index Index of the specific buffer in _rxArray_.
 * @param ident 11-bit standard CAN Identifier. If two or more CANrx buffers have the same _ident_, then buffer with
 * lowest _index_ has precedence and other CANrx buffers will be ignored.
 * @param mask 11-bit mask for identifier. Most usually set to 0x7FF. Received message (rcvMsg) will be accepted if the
 * following condition is true: (((rcvMsgId ^ ident) & mask) == 0).
 * @param rtr If true, 'Remote Transmit Request' messages will be accepted.
 * @param object CANopen object, to which buffer is connected. It will be used as an argument to CANrx_callback. Its
 * type is (void), CANrx_callback will change its type back to the correct object type.
 * @param CANrx_callback Pointer to function, which will be called, if received CAN message matches the identifier. It
 * must be fast function.
 *
 * Return #CO_ReturnError_t: CO_ERROR_NO CO_ERROR_ILLEGAL_ARGUMENT or CO_ERROR_OUT_OF_MEMORY (not enough masks for
 * configuration).
 */
CO_ReturnError_t CO_CANrxBufferInit(CO_CANmodule_t* CANmodule, uint16_t index, uint16_t ident, uint16_t mask, bool_t rtr, void* object, void (*CANrx_callback)(void* object, void* message)) {
    (void) mask;
    (void) rtr;
    (void*) object;
    (void*) CANrx_callback;
    CO_ReturnError_t ret = CO_ERROR_NO;

    if (CANmodule != NULL && object != NULL && CANrx_callback != NULL && index < CANmodule->rxSize) {
        CO_CANrx_t* buffer = &CANmodule->rxArray[index];

        /*
         * Configure global identifier, including RTR bit
         *
         * This is later used for RX operation match case
         */
        buffer->ident = ident;

        /* Set CAN hardware module filter and mask. */
        if (CANmodule->useCANrxFilters) {
            __asm__("nop"); // For fun's sake
        }
    } else {
        ret = CO_ERROR_ILLEGAL_ARGUMENT;
    }

    return ret;
}

/**
 * Configure CAN message transmit buffer.
 *
 * Function configures specific CAN transmit buffer. Function must be called for each member in _txArray_ from
 * CO_CANmodule_t.
 *
 * @param CANmodule This object.
 * @param index Index of the specific buffer in _txArray_.
 * @param ident 11-bit standard CAN Identifier.
 * @param rtr If true, 'Remote Transmit Request' messages will be transmitted.
 * @param noOfBytes Length of CAN message in bytes (0 to 8 bytes).
 * @param syncFlag This flag bit is used for synchronous TPDO messages. If it is set, message will not be sent, if
 * current time is outside synchronous window.
 *
 * @return Pointer to CAN transmit message buffer. 8 bytes data array inside buffer should be written, before
 * CO_CANsend() function is called. Zero is returned in case of wrong arguments.
 */
CO_CANtx_t* CO_CANtxBufferInit(CO_CANmodule_t* CANmodule, uint16_t index, uint16_t ident, bool_t rtr, uint8_t noOfBytes, bool_t syncFlag) {
    (void)rtr;
    CO_CANtx_t* buffer = NULL;
    if (CANmodule != NULL && index < CANmodule->txSize) {
        buffer = &CANmodule->txArray[index];
        buffer->ident = ident;
        buffer->DLC = noOfBytes;
        buffer->bufferFull = false;
        buffer->syncFlag = syncFlag;
    }
    return buffer;
}

/**
 * Send CAN message.
 *
 * @param CANmodule This object.
 * @param buffer Pointer to transmit buffer, returned by CO_CANtxBufferInit(). Data bytes must be written in buffer
 * before function call.
 *
 * @return #CO_ReturnError_t: CO_ERROR_NO, CO_ERROR_TX_OVERFLOW or CO_ERROR_TX_PDO_WINDOW (Synchronous TPDO is outside
 * window).
 */
CO_ReturnError_t CO_CANsend(CO_CANmodule_t* CANmodule, CO_CANtx_t* buffer) {
    CO_ReturnError_t err = CO_ERROR_NO;

    /* Verify overflow */
    if (buffer->bufferFull) {
        if (!CANmodule->firstCANtxMessage) {
            /* don't set error, if bootup message is still on buffers */
            CANmodule->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
        }
        err = CO_ERROR_TX_OVERFLOW;
    }

    /*
     * Send message to CAN network
     *
     * Lock interrupts for atomic operation
     */
    CO_LOCK_CAN_SEND(CANmodule);
    canmsg msg(*buffer);
    if (can_send(*static_cast<struct can_file_desc*>(CANmodule->CANptr), msg, NULL)) {
        CANmodule->bufferInhibitFlag = buffer->syncFlag;
    } else {
        /* Only increment count if buffer wasn't already full */
        if (!buffer->bufferFull) {
            buffer->bufferFull = true;
            CANmodule->CANtxCount++;
        }
    }
    CO_UNLOCK_CAN_SEND(CANmodule);

    return err;
}

/**
 * Clear all synchronous TPDOs from CAN module transmit buffers.
 *
 * CANopen allows synchronous PDO communication only inside time between SYNC message and SYNC Window. If time is
 * outside this window, new synchronous PDOs must not be sent and all pending sync TPDOs, which may be on CAN TX
 * buffers, may optionally be cleared.
 *
 * This function checks (and aborts transmission if necessary) CAN TX buffers when it is called. Function should be
 * called by the stack in the moment, when SYNC time was just passed out of synchronous window.
 *
 * @param CANmodule This object.
 */
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t* CANmodule) {
    uint32_t tpdoDeleted = 0U;

    CO_LOCK_CAN_SEND(CANmodule);
    /* Abort message from CAN module, if there is synchronous TPDO.
     * Take special care with this functionality. */
    if (CANmodule->bufferInhibitFlag) {
        /* Clear TXREQ */
        CANmodule->bufferInhibitFlag = false;
        tpdoDeleted = 1U;
    }
    /* Delete also pending synchronous TPDOs in TX buffers */
    if (CANmodule->CANtxCount > 0)
        for (uint16_t i = CANmodule->txSize; i > 0U; --i)
            if (CANmodule->txArray[i].bufferFull)
                if (CANmodule->txArray[i].syncFlag) {
                    CANmodule->txArray[i].bufferFull = false;
                    CANmodule->CANtxCount--; // How does this delete anything ???
                    tpdoDeleted = 2U;
                }
    CO_UNLOCK_CAN_SEND(CANmodule);
    if (tpdoDeleted) {
        CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
    }
}

/**
 * Process can module - verify CAN errors
 *
 * Function must be called cyclically. It should calculate CANerrorStatus bitfield for CAN errors defined in @ref
 * CO_CAN_ERR_status_t.
 *
 * @param CANmodule This object.
 */
void CO_CANmodule_process(CO_CANmodule_t* CANmodule) {
    uint32_t err = 0;

    // CANOpen just care about Bus_off, Warning, Passive and Overflow
    err = static_cast<struct can_file_desc*>(CANmodule->CANptr)->status;

    if (CANmodule->errOld != err) {

        uint16_t status = CANmodule->CANerrorStatus;

        CANmodule->errOld = err;

        /* recalculate CANerrorStatus, first clear some flags */
        status &= 0xFFFF ^ (CO_CAN_ERRTX_BUS_OFF | CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_WARNING | CO_CAN_ERRTX_PASSIVE);

        if (err & CO_CAN_ERRTX_BUS_OFF) {
            status |= CO_CAN_ERRTX_BUS_OFF;
        }

        if (err & (CO_CAN_ERRRX_WARNING | CO_CAN_ERRTX_WARNING)) {
            status |= CO_CAN_ERRRX_WARNING | CO_CAN_ERRTX_WARNING;
        }

        if (err & (CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_PASSIVE)) {
            status |= CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_PASSIVE;
        }

        CANmodule->CANerrorStatus = status;
    }
}
