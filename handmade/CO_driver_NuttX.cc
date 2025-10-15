#include "301/CO_driver.h"

int can_init(std::string path, FILE* logger)
{
    /* Open device */
    int fd = open(path.c_str(), O_RDWR);
    if (fd < 0)
    {
        LOG("[CAN] Failed to open %s %d.\r\n", path.c_str(), -errno);
        return -1;
    }
    LOG("[CAN] Initialized.\r\n");
    return fd;
}

void can_deinit(int fd, FILE* logger)
{
    LOG("[CAN] Closing !\r\n");
    close(fd); // TODO: Probable runtime error since second log call never get sent.
    LOG("[CAN] Closed.\r\n");
}

int can_read(int fd, canmsg& msg, FILE* logger)
{
    struct can_msg_s frame;
    size_t             ret;

    /* Read frame */
    size_t msgsize = sizeof(struct can_msg_s);
    ret = read(fd, &frame, msgsize);
    if (ret < CAN_MSGLEN(0) || ret > msgsize)
    {
        LOG("[CAN] Reading %d returned %d bytes.\r\n", msgsize, ret);
        return -errno;
    }

    /* Check for error reports */
    if (frame.cm_hdr.ch_error != 0)
    {
        LOG("[CAN] An error has been detected: [0x%04\]\r\n", frame.cm_hdr.ch_id);
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
    for (int i = 0; i < msg.len_; i++) // As in send
    {
        msg.add_data(frame.cm_data[i]);
    }
    LOG("[CAN] Message read successfully.\r\n");

    return ret;
}

int can_send(int fd, canmsg& msg, FILE* logger)
{
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
    for (int i = 0; i < msg.get_len(); i++) // As in the nuttx example
    {
        frame.cm_data[i] = msg.get_data(i);
    }

    /* Send frame */
    int len = CAN_MSGLEN(msg.get_len());
    ret = write(fd, &frame, len);
    if (ret < 0)
    {
        LOG("[CAN] Write completely failed %d\r\n", -errno);
        return -1;
    }
    else if (ret != len)
    {
        LOG("[CAN] Write partially failed. Wrote %d bytes out of %d. Errno: %d\r\n", ret, len, errno);
        return -1;
    }
    LOG("[CAN] Successfully sent message.\r\n");
    return ret;
}

/**
 * CAN receive callback function which pre-processes received CAN message
 *
 * It is called by fast CAN receive thread. Each \ref CO_obj "CANopenNode Object" defines its own and registers it with
 * CO_CANrxBufferInit(), by passing function pointer.
 *
 * @param object pointer to specific \ref CO_obj "CANopenNode Object", registered with CO_CANrxBufferInit()
 * @param rxMsg pointer to received CAN message
 */
// void CANrx_callback(void* object, void* rxMsg);

/**
 * Request CAN configuration (stopped) mode and *wait* until it is set.
 *
 * @param CANptr Pointer to CAN device
 */
// void CO_CANsetConfigurationMode(void* CANptr);

/**
 * Request CAN normal (operational) mode and *wait* until it is set.
 *
 * @param CANmodule CO_CANmodule_t object.
 */
// void CO_CANsetNormalMode(CO_CANmodule_t* CANmodule);

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
// CO_ReturnError_t CO_CANmodule_init(CO_CANmodule_t* CANmodule, void* CANptr, CO_CANrx_t rxArray[], uint16_t rxSize,
                                   CO_CANtx_t txArray[], uint16_t txSize, uint16_t CANbitRate);

/**
 * Switch off CANmodule. Call at program exit.
 *
 * @param CANmodule CAN module object.
 */
// void CO_CANmodule_disable(CO_CANmodule_t* CANmodule);

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
// CO_ReturnError_t CO_CANrxBufferInit(CO_CANmodule_t* CANmodule, uint16_t index, uint16_t ident, uint16_t mask,
                                    bool_t rtr, void* object, void (*CANrx_callback)(void* object, void* message));

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
// CO_CANtx_t* CO_CANtxBufferInit(CO_CANmodule_t* CANmodule, uint16_t index, uint16_t ident, bool_t rtr, uint8_t noOfBytes, bool_t syncFlag);

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
// CO_ReturnError_t CO_CANsend(CO_CANmodule_t* CANmodule, CO_CANtx_t* buffer);

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
// void CO_CANclearPendingSyncPDOs(CO_CANmodule_t* CANmodule);

/**
 * Process can module - verify CAN errors
 *
 * Function must be called cyclically. It should calculate CANerrorStatus bitfield for CAN errors defined in @ref
 * CO_CAN_ERR_status_t.
 *
 * @param CANmodule This object.
 */
// void CO_CANmodule_process(CO_CANmodule_t* CANmodule);
