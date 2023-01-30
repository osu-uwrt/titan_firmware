#include <stdint.h>
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware/sync.h"
#include "pico/binary_info.h"

#include "canmore/protocol.h"

#include "can_mcp251XFD_bridge.h"
#include "mcp251Xfd/CRC16_CMS.h"
#include "mcp251Xfd/MCP251XFD.h"

// ========================================
// Preprocessor Defines
// ========================================

#ifndef UWRT_ROBOT_DEFINED
#error Robot must be defined to retrieve CAN bus configuration
#endif

#ifndef UWRT_BOARD_DEFINED
#error UWRT board must be defined to retrieve CAN bus configuration
#endif

#ifndef CAN_BUS_NAME
#error No CAN bus defined in board file
#endif

// Set configurations for can bus
// Note that these are pulled from the board and robot configurations (hence the error check above)
// This pulls the CAN_BUS_NAME from the board files, and looks up that name in the robot definition file
// So for example, if the board has CAN_BUS_NAME=ALPHA the vehicle should have a ALPHA_ENABLE_FD=1 define
#define CAN_BUS_ENABLE_FD   __CONCAT(CAN_BUS_NAME, _ENABLE_FD)
#define CAN_BUS_RATE        __CONCAT(CAN_BUS_NAME, _RATE)
#if CAN_BUS_ENABLE_FD
#define CAN_BUS_FD_RATE     __CONCAT(CAN_BUS_NAME, _FD_RATE)
#else
#define CAN_BUS_FD_RATE     MCP251XFD_NO_CANFD
#endif

#define MCP2517FD_OSC_RATE  4000000     // OSC must run at 4MHz
#define MCP2517FD_OSC_SRC   clk_usb     // Use 48 MHz clock source so it'll evenly divide into 4 MHz

#if MCP2517FD_OSC_SRC == clk_usb
#define MCP2517FD_CLK_GPOUT_AUXSRC CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_VALUE_CLK_USB
#elif MCP2517FD_OSC_SRC == clk_adc
#define MCP2517FD_CLK_GPOUT_AUXSRC CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_VALUE_CLK_ADC
#else
#error Invalid clock source
#endif

#define MCP2517FD_SPI_INST  __CONCAT(spi, MCP2517FD_SPI)

canmore_msg_decoder_t msg_decoder;

void report_canmore_msg_decode_error(__unused void* arg, unsigned int error_code) {
    canbus_call_receive_error_cb(CANBUS_RECVERR_DECODE_ERROR_BASE + error_code);
}

// ========================================
// Driver SPI Callbacks
// ========================================

eERRORRESULT can_mcp251x_spi_config(void *pIntDev, uint8_t chipSelect, const uint32_t sckFreq){
    spi_set_baudrate((spi_inst_t*) pIntDev, sckFreq);

    return ERR_OK;
}

static eERRORRESULT __not_in_flash_func(can_mcp251x_spi_transfer)(void *pIntDev, uint8_t chipSelect, uint8_t *txData, uint8_t *rxData, size_t size) {
    if (!txData) {
        return ERR__SPI_PARAMETER_ERROR;
    }

    // Disable interrupts from CAN controller in the middle of transfer
    gpio_set_irq_enabled(MCP2517FD_INT_PIN, GPIO_IRQ_LEVEL_LOW, false);

    // Make sure this function is not interrupted and called in the middle of a transfer, using CS pin as a lock
    uint32_t prev_interrupt = save_and_disable_interrupts();
    if (!gpio_get_out_level(chipSelect)){
        restore_interrupts(prev_interrupt);
        return ERR__SPI_BUSY;
    }

    gpio_put(chipSelect, false);
    restore_interrupts(prev_interrupt);

    if (rxData) {
        spi_write_read_blocking((spi_inst_t*) pIntDev, txData, rxData, size);
    } else {
        spi_write_blocking((spi_inst_t*) pIntDev, txData, size);
    }

    gpio_put(chipSelect, true);
    gpio_set_irq_enabled(MCP2517FD_INT_PIN, GPIO_IRQ_LEVEL_LOW, true);

    return ERR_OK;
}

static uint32_t __not_in_flash_func(can_mcp251x_get_current_ms)(void) {
    return to_ms_since_boot(get_absolute_time());
}

// ========================================
// Driver Configuration
// ========================================

MCP251XFD mcp251xfd_device =
{
    .UserDriverData = NULL,
    //--- Driver configuration ---
    .DriverConfig    = MCP251XFD_DRIVER_SAFE_RESET
                     | MCP251XFD_DRIVER_USE_READ_WRITE_CRC
                     | MCP251XFD_DRIVER_USE_SAFE_WRITE
                     | MCP251XFD_DRIVER_ENABLE_ECC
                     | MCP251XFD_DRIVER_INIT_CHECK_RAM
                     | MCP251XFD_DRIVER_INIT_SET_RAM_AT_0
                     | MCP251XFD_DRIVER_CLEAR_BUFFER_BEFORE_READ,
    //--- IO configuration ---
    .GPIOsOutState   = MCP251XFD_GPIO0_LOW | MCP251XFD_GPIO1_LOW,
    //--- Interface driver call functions ---
    .SPI_ChipSelect  = MCP2517FD_NCS_PIN,   // Passed to spi functions
    .InterfaceDevice = MCP2517FD_SPI_INST,  // Passed to spi functions
    .fnSPI_Init      = can_mcp251x_spi_config,
    .fnSPI_Transfer  = can_mcp251x_spi_transfer,
    //--- Time call function ---
    .fnGetCurrentms  = can_mcp251x_get_current_ms,
    //--- CRC16-CMS call function ---
    .fnComputeCRC16  = ComputeCRC16CMS,
    //--- Interface clocks ---
    .SPIClockSpeed   = 20000000,  // 20MHz (must be at most mcp251x sysclk/2)
};

MCP251XFD_BitTimeStats mcp251xfd_device_btstats;
uint32_t mcp251xfd_device_sysclk;
MCP251XFD_Config mcp251xfd_device_config =
{
    //--- Controller clocks ---
    .XtalFreq       = 0,                                // CLKIN is not a crystal
    .OscFreq        = MCP2517FD_OSC_RATE,               // CLKIN is a 4MHz oscillator generated from the RP2040
    .SysclkConfig   = MCP251XFD_SYSCLK_IS_CLKIN_MUL_10, // Generate 40 MHz clock
    .ClkoPinConfig  = MCP251XFD_CLKO_SOF,
    .SYSCLK_Result  = &mcp251xfd_device_sysclk,
    //--- CAN configuration ---
    .NominalBitrate = CAN_BUS_RATE,   // Nominal Bitrate to 1Mbps
    .DataBitrate    = CAN_BUS_FD_RATE,   // Data Bitrate to 2Mbps
    .BitTimeStats   = &mcp251xfd_device_btstats,
    .Bandwidth      = MCP251XFD_NO_DELAY,
    .ControlFlags   = MCP251XFD_CAN_RESTRICTED_MODE_ON_ERROR
                    | MCP251XFD_CAN_ESI_REFLECTS_ERROR_STATUS
                    | MCP251XFD_CAN_RESTRICTED_RETRANS_ATTEMPTS
                    | MCP251XFD_CANFD_BITRATE_SWITCHING_ENABLE
                    | MCP251XFD_CAN_PROTOCOL_EXCEPT_AS_FORM_ERROR
                    | MCP251XFD_CANFD_USE_ISO_CRC
                    | MCP251XFD_CANFD_DONT_USE_RRS_BIT_AS_SID11,
    //--- GPIOs and Interrupts pins ---
    .GPIO0PinMode   = MCP251XFD_PIN_AS_GPIO0_IN,
    .GPIO1PinMode   = MCP251XFD_PIN_AS_GPIO1_IN,
    .INTsOutMode    = MCP251XFD_PINS_PUSHPULL_OUT,
    .TXCANOutMode   = MCP251XFD_PINS_PUSHPULL_OUT,
    //--- Interrupts ---
    .SysInterruptFlags = MCP251XFD_INT_TX_EVENT                 // Enable global TX interrupts (controlled via FIFO)
                       | MCP251XFD_INT_RX_EVENT                 // Enable global RX interrupts (controlled via FIFO)
                       // Device Errors
                       | MCP251XFD_INT_RX_OVERFLOW_EVENT        // Device error if message dropped from code being slow
                       | MCP251XFD_INT_RAM_ECC_EVENT            // Report any RAM ECC errors
                       | MCP251XFD_INT_SPI_CRC_EVENT            // Report any SPI CRC errors
                       | MCP251XFD_INT_SYSTEM_ERROR_EVENT       // Report any errors on device
                       // Bus Errors
                       | MCP251XFD_INT_BUS_ERROR_EVENT          // Notify on bus errors to know when entering bus off state
                       | MCP251XFD_INT_RX_INVALID_MESSAGE_EVENT // Notify on invalid message reception
                       | MCP251XFD_INT_TX_ATTEMPTS_EVENT,       // Notify on message reaching retransmit limit
};

// FIFO definitions
#define mcp251xfd_msg_tx_fifo            MCP251XFD_FIFO1
#define mcp251xfd_msg_rx_fifo            MCP251XFD_FIFO2
#define mcp251xfd_utility_tx_fifo        MCP251XFD_FIFO3
#define mcp251xfd_utility_rx_fifo        MCP251XFD_FIFO4

#define mcp251xfd_msg_rx_filter_num      MCP251XFD_FILTER0
#define mcp251xfd_msg_rx_crc_filter_num  MCP251XFD_FILTER1
#define mcp251xfd_utility_rx_filter_num  MCP251XFD_FILTER2

MCP251XFD_RAMInfos mcp251xfd_msg_tx_raminfo;
MCP251XFD_RAMInfos mcp251xfd_msg_rx_raminfo;
MCP251XFD_RAMInfos mcp251xfd_utility_tx_raminfo;
MCP251XFD_RAMInfos mcp251xfd_utility_rx_raminfo;

MCP251XFD_FIFO mcp251xfd_msg_tx_fifo_config = {
    .Name = mcp251xfd_msg_tx_fifo, .Size = MCP251XFD_FIFO_8_MESSAGE_DEEP, .Payload = MCP251XFD_PAYLOAD_64BYTE,
    .Direction = MCP251XFD_TRANSMIT_FIFO, .Attempts = MCP251XFD_THREE_ATTEMPTS,
    .Priority = MCP251XFD_MESSAGE_TX_PRIORITY17, .ControlFlags = MCP251XFD_FIFO_NO_RTR_RESPONSE,
    .InterruptFlags = MCP251XFD_FIFO_TX_ATTEMPTS_EXHAUSTED_INT + MCP251XFD_FIFO_TRANSMIT_FIFO_NOT_FULL_INT,
    .RAMInfos = &mcp251xfd_msg_tx_raminfo,
};

MCP251XFD_FIFO mcp251xfd_msg_rx_fifo_config = {
    .Name = mcp251xfd_msg_rx_fifo, .Size = MCP251XFD_FIFO_8_MESSAGE_DEEP, .Payload = MCP251XFD_PAYLOAD_64BYTE,
    .Direction = MCP251XFD_RECEIVE_FIFO, .ControlFlags = MCP251XFD_FIFO_ADD_TIMESTAMP_ON_RX,
    .InterruptFlags = MCP251XFD_FIFO_OVERFLOW_INT + MCP251XFD_FIFO_RECEIVE_FIFO_NOT_EMPTY_INT,
    .RAMInfos = &mcp251xfd_msg_rx_raminfo,
};

MCP251XFD_FIFO mcp251xfd_utility_tx_fifo_config = {
    // NOTE: Don't notify on TX failures
    .Name = mcp251xfd_utility_tx_fifo, .Size = MCP251XFD_FIFO_4_MESSAGE_DEEP, .Payload = MCP251XFD_PAYLOAD_8BYTE,
    .Direction = MCP251XFD_TRANSMIT_FIFO, .Attempts = MCP251XFD_THREE_ATTEMPTS,
    .Priority = MCP251XFD_MESSAGE_TX_PRIORITY16, .ControlFlags = MCP251XFD_FIFO_NO_RTR_RESPONSE,
    .InterruptFlags = MCP251XFD_FIFO_TRANSMIT_FIFO_NOT_FULL_INT,
    .RAMInfos = &mcp251xfd_utility_tx_raminfo,
};

MCP251XFD_FIFO mcp251xfd_utility_rx_fifo_config = {
    .Name = mcp251xfd_utility_rx_fifo, .Size = MCP251XFD_FIFO_4_MESSAGE_DEEP, .Payload = MCP251XFD_PAYLOAD_8BYTE,
    .Direction = MCP251XFD_RECEIVE_FIFO, .ControlFlags = MCP251XFD_FIFO_ADD_TIMESTAMP_ON_RX,
    .InterruptFlags = MCP251XFD_FIFO_OVERFLOW_INT + MCP251XFD_FIFO_RECEIVE_FIFO_NOT_EMPTY_INT,
    .RAMInfos = &mcp251xfd_utility_rx_raminfo,
};

// NOTE: Ensure that any new fifos or changed interrupt flags are properly handled in the IRQ handler
// If not, the IRQ handler might get stuck

MCP251XFD_Filter mcp251xfd_msg_rx_filter =
{   // Filter for standard CANmore message frames
    // Match only standard CANmore message frames for this client from the agent
    .Filter = mcp251xfd_msg_rx_filter_num, .EnableFilter = true, .Match = MCP251XFD_MATCH_ONLY_SID,
    .AcceptanceID = 0, .AcceptanceMask = CANMORE_CALC_FILTER_MASK(true, true, true, false),
    .PointTo = mcp251xfd_msg_rx_fifo,
};

MCP251XFD_Filter mcp251xfd_msg_rx_crc_filter =
{   // Filter for extended (crc) CANmore message frames
    // Match only extended CANmore message frames for this client from the agent
    .Filter = mcp251xfd_msg_rx_crc_filter_num, .EnableFilter = true, .Match = MCP251XFD_MATCH_ONLY_EID,
    .AcceptanceID = 0, .AcceptanceMask = CANMORE_CALC_EXT_FILTER_MASK(true, true, true, false, false),
    .PointTo = mcp251xfd_msg_rx_fifo,
};

MCP251XFD_Filter mcp251xfd_utility_rx_filter =
{   // Filter for standard CANmore utility frames
    // Match only standard CANmore utility frames for this client from the agent
    .Filter = mcp251xfd_utility_rx_filter_num, .EnableFilter = true, .Match = MCP251XFD_MATCH_ONLY_SID,
    .AcceptanceID = 0, .AcceptanceMask = CANMORE_CALC_FILTER_MASK(true, true, true, false),
    .PointTo = mcp251xfd_utility_rx_fifo,
};


// ========================================
// Interrupt Pin Handling
// ========================================

static bool check_fifo_event(eMCP251XFD_FIFO name, eMCP251XFD_FIFOstatus event) {
    eMCP251XFD_FIFOstatus fifo_status;
    eERRORRESULT error_code = MCP251XFD_GetFIFOStatus(&mcp251xfd_device, name, &fifo_status);
    if (error_code != ERR_OK){
        canbus_report_driver_error(error_code);
        return false;
    }

    return (fifo_status & event) != 0;
}

/**
 * @brief Set the tx/rx fifo's not full/not empty interrupt enable bit
 *
 * @param name Fifo to configure
 * @param enabled Whether to enable the fifo interrupt
 */
static void set_fifo_not_full_empty_interrupt(eMCP251XFD_FIFO name, bool enabled) {
    // Calculate FIFO interrupt configuration address
    uint16_t address = RegMCP251XFD_CiFIFOCONm_CONFIG + (MCP251XFD_FIFO_REG_SIZE * ((uint16_t)name - 1u));

    uint8_t register_data;
    eERRORRESULT error_code = MCP251XFD_ReadSFR8(&mcp251xfd_device, address, &register_data);
    if (error_code != ERR_OK) {
        canbus_report_driver_error(error_code);
        return;
    }

    // Mask off interrupt enabled
    register_data &= ~MCP251XFD_CAN_CiFIFOCONm8_TFNRFNIE;

    // Set flag if enabed
    register_data |= (enabled ? MCP251XFD_CAN_CiFIFOCONm8_TFNRFNIE : 0);

    // Write back
    error_code = MCP251XFD_WriteSFR8(&mcp251xfd_device, address, register_data);
    if (error_code != ERR_OK) {
        canbus_report_driver_error(error_code);
        return;
    }
}

/**
 * @brief Check fifo for interrupt event, and clear interrupt event if set
 *
 * @param name FIFO to check
 * @param event Event to check for/clear
 * @return true Interrupt event found and cleared
 * @return false Interrupt event not found
 */
static bool check_and_clear_fifo_event(eMCP251XFD_FIFO name, eMCP251XFD_FIFOstatus event) {
    if (check_fifo_event(name, event)) {
        eERRORRESULT error_code = MCP251XFD_ClearFIFOEvents(&mcp251xfd_device, name, event);
        if (error_code != ERR_OK) canbus_report_driver_error(error_code);
        return true;
    } else {
        return false;
    }
}

static void __not_in_flash_func(can_mcp251xfd_interrupt_cb)(__unused uint gpio, __unused uint32_t events) {
    setMCP251XFD_InterruptEvents active_interrupts;
    eERRORRESULT error_code = MCP251XFD_GetInterruptEvents(&mcp251xfd_device, &active_interrupts);

    if (error_code != ERR_OK) {
        canbus_report_driver_error(error_code);

        // Can't process interrupt since the register failed to read
        // If the interrupt is still present after this returns, it'll retry the call above
        return;
    }

    bool interrupt_cleared = false;

    // Device Errors
    if (active_interrupts & MCP251XFD_INT_RX_OVERFLOW_EVENT) {
        canbus_report_library_error(CANBUS_LIBERR_RX_OVERFLOW);

        if (check_and_clear_fifo_event(mcp251xfd_msg_rx_fifo, MCP251XFD_RX_FIFO_OVERFLOW)){
            interrupt_cleared = true;
        }
        if (check_and_clear_fifo_event(mcp251xfd_utility_rx_fifo, MCP251XFD_RX_FIFO_OVERFLOW)){
            interrupt_cleared = true;
        }
    }
    if (active_interrupts & MCP251XFD_INT_RAM_ECC_EVENT) {
        canbus_report_library_error(CANBUS_LIBERR_DEVICE_MALFUNCTION);

        interrupt_cleared = true;
        error_code = MCP251XFD_ClearECCEvents(&mcp251xfd_device);
        if (error_code != ERR_OK) canbus_report_driver_error(error_code);
    }
    if (active_interrupts & MCP251XFD_INT_SPI_CRC_EVENT) {
        canbus_report_library_error(CANBUS_LIBERR_COMM_MALFUNCTION);

        interrupt_cleared = true;
        error_code = MCP251XFD_ClearCRCEvents(&mcp251xfd_device);
        if (error_code != ERR_OK) canbus_report_driver_error(error_code);
    }
    if (active_interrupts & MCP251XFD_INT_SYSTEM_ERROR_EVENT) {
        canbus_report_library_error(CANBUS_LIBERR_DEVICE_MALFUNCTION);

        interrupt_cleared = true;
        error_code = MCP251XFD_ClearInterruptEvents(&mcp251xfd_device, MCP251XFD_INT_SYSTEM_ERROR_EVENT);
        if (error_code != ERR_OK) canbus_report_driver_error(error_code);
    }

    // Bus Errors
    if (active_interrupts & MCP251XFD_INT_BUS_ERROR_EVENT) {
        //TODO: Handle bus off events to report CAN bus has gone offline

        interrupt_cleared = true;
        error_code = MCP251XFD_ClearInterruptEvents(&mcp251xfd_device, MCP251XFD_INT_BUS_ERROR_EVENT);
        if (error_code != ERR_OK) canbus_report_driver_error(error_code);
    }
    if (active_interrupts & MCP251XFD_INT_RX_INVALID_MESSAGE_EVENT) {
        canbus_call_receive_error_cb(CANBUS_RECVERR_CORRUPT_MESSAGE);

        interrupt_cleared = true;
        error_code = MCP251XFD_ClearInterruptEvents(&mcp251xfd_device, MCP251XFD_INT_RX_INVALID_MESSAGE_EVENT);
        if (error_code != ERR_OK) canbus_report_driver_error(error_code);
    }
    if (active_interrupts & MCP251XFD_INT_TX_ATTEMPTS_EVENT) {
        if (check_and_clear_fifo_event(mcp251xfd_msg_tx_fifo, MCP251XFD_TX_FIFO_ATTEMPTS_EXHAUSTED)){
            interrupt_cleared = true;
        }
    }

    // Handle FIFO Events
    if (active_interrupts & MCP251XFD_INT_TX_EVENT) {
        if (check_fifo_event(mcp251xfd_msg_tx_fifo, MCP251XFD_TX_FIFO_NOT_FULL)) {
            // TODO: Figure out a better way to prevent race conditions than this
            uint32_t prev_interrupts = save_and_disable_interrupts();
            if (!canmore_msg_encode_done(&encoding_buffer)) {
                restore_interrupts(prev_interrupts);

                uint8_t buffer[MCP251XFD_PAYLOAD_MAX];
                bool is_extended;
                MCP251XFD_CANMessage msg;

                canmore_msg_encode_next(&encoding_buffer, buffer, &msg.DLC, &msg.MessageID, &is_extended);

                // TODO: Configure to allow FD support
                msg.ControlFlags = MCP251XFD_CAN20_FRAME;

                if (is_extended) {
                    msg.ControlFlags |= MCP251XFD_EXTENDED_MESSAGE_ID;
                }
                msg.MessageSEQ = 0;
                msg.PayloadData = buffer;

                interrupt_cleared = true;
                error_code = MCP251XFD_TransmitMessageToFIFO(&mcp251xfd_device, &msg, mcp251xfd_msg_tx_fifo, true);
                if (error_code != ERR_OK) canbus_report_driver_error(error_code);
            } else {
                interrupt_cleared = true;
                set_fifo_not_full_empty_interrupt(mcp251xfd_msg_tx_fifo, false);
                restore_interrupts(prev_interrupts);
            }
        }

        if (check_fifo_event(mcp251xfd_utility_tx_fifo, MCP251XFD_TX_FIFO_NOT_FULL)) {
            // TODO: Figure out a better way to prevent race conditions than this
            interrupt_cleared = true;
            set_fifo_not_full_empty_interrupt(mcp251xfd_utility_tx_fifo, false);
        }
    }

    if (active_interrupts & MCP251XFD_INT_RX_EVENT) {
        if (check_fifo_event(mcp251xfd_msg_rx_fifo, MCP251XFD_RX_FIFO_NOT_EMPTY)) {
            // TODO: Figure out a better way to prevent race conditions than this
            uint32_t prev_interrupts = save_and_disable_interrupts();
            if (!canmore_received_msg.waiting) {
                restore_interrupts(prev_interrupts);

                uint32_t timestamp = 0;
                uint8_t payload[MCP251XFD_PayloadToByte(mcp251xfd_msg_rx_fifo_config.Payload)];

                MCP251XFD_CANMessage msg;
                msg.PayloadData = payload;

                // that will be received
                interrupt_cleared = true;
                error_code = MCP251XFD_ReceiveMessageFromFIFO(&mcp251xfd_device, &msg, mcp251xfd_msg_rx_fifo_config.Payload,
                                                              &timestamp, mcp251xfd_msg_rx_fifo);
                if (error_code != ERR_OK) {
                    canbus_report_driver_error(error_code);
                } else {
                    canmore_id_t client_id = {.identifier = msg.MessageID};

                    if ((msg.ControlFlags & MCP251XFD_EXTENDED_MESSAGE_ID) == 0) {
                        canmore_msg_decode_frame(&msg_decoder, client_id.pkt_std.noc, msg.PayloadData, msg.DLC);
                    } else {
                        size_t msg_size = canmore_msg_decode_last_frame(&msg_decoder, client_id.pkt_ext.noc,
                                                                        msg.PayloadData, msg.DLC, client_id.pkt_ext.crc,
                                                                        canmore_received_msg.data);

                        // Only queue the data if the decode was successful
                        if (msg_size > 0) {
                            canmore_received_msg.length = msg_size;
                            canmore_received_msg.waiting = true;
                        }
                    }
                }
            } else {
                interrupt_cleared = true;
                set_fifo_not_full_empty_interrupt(mcp251xfd_msg_rx_fifo, false);
                restore_interrupts(prev_interrupts);
            }
        }

        if (check_fifo_event(mcp251xfd_utility_rx_fifo, MCP251XFD_RX_FIFO_NOT_EMPTY)) {
            printf("Utility RX Not Empty!\n");
            // TODO: Figure out a better way to prevent race conditions than this
            //set_fifo_not_full_empty_interrupt(mcp251xfd_utility_rx_fifo, false);
            //interrupt_cleared = true;

            uint32_t timestamp = 0;
            uint8_t payload[MCP251XFD_PayloadToByte(mcp251xfd_msg_rx_fifo_config.Payload)];

            MCP251XFD_CANMessage msg;
            msg.PayloadData = payload;

            // that will be received
            interrupt_cleared = true;
            error_code = MCP251XFD_ReceiveMessageFromFIFO(&mcp251xfd_device, &msg, mcp251xfd_utility_rx_fifo_config.Payload,
                                                            &timestamp, mcp251xfd_utility_rx_fifo);
            if (error_code != ERR_OK) {
                canbus_report_driver_error(error_code);
            } else {
                printf("Payload Data: ");
                for (int i = 0; i < msg.DLC; i++) {
                    printf("0x%02X%s", msg.PayloadData[i], (i+1 == msg.DLC ? "\n" : ", "));
                }
            }
        }
    }

    // Report if an interrupt was raised but was unable to be cleared
    if (!interrupt_cleared) {
        canbus_report_library_error(CANBUS_LIBERR_UNEXPECTED_INTERRUPT);
    }
}

#define TIMESTAMP_TICK_us       ( 25 ) // TimeStamp tick is 25µs
#define TIMESTAMP_TICK(sysclk)  ( ((sysclk) / 1000000) * TIMESTAMP_TICK_us )

// ========================================
// Driver Exports
// ========================================

bool can_mcp251xfd_configure(unsigned int client_id)
{
    canmore_msg_decode_init(&msg_decoder, report_canmore_msg_decode_error, NULL);

    //--- Compute Filter Values ---
    // Mask was set up in initialization (0 all fields not being checked, set ID to be from agent)
    mcp251xfd_msg_rx_filter.AcceptanceID = CANMORE_CALC_MSG_ID_A2C(client_id, 0);
    mcp251xfd_msg_rx_crc_filter.AcceptanceID = CANMORE_CALC_MSG_EXT_ID_A2C(client_id, 0, 0);
    mcp251xfd_utility_rx_filter.AcceptanceID = CANMORE_CALC_UTIL_ID_A2C(client_id, 0);

    //--- Initialize Int pins or GPIOs ---
    // Initialize CS Pin
    gpio_init(MCP2517FD_NCS_PIN);
    gpio_put(MCP2517FD_NCS_PIN, true);
    gpio_set_dir(MCP2517FD_NCS_PIN, GPIO_OUT);
    bi_decl_if_func_used(bi_1pin_with_name(MCP2517FD_NCS_PIN, "MCP251XFD CHIP SELECT"));

    // Initialize SPI
    // Stealing SPI Clock Speed just for setup
    // This is because the initialization function is called several times, and all real setup is performed here
    spi_init(MCP2517FD_SPI_INST, mcp251xfd_device.SPIClockSpeed);

    gpio_set_function(MCP2517FD_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MCP2517FD_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MCP2517FD_MISO_PIN, GPIO_FUNC_SPI);
    bi_decl_if_func_used(bi_3pins_with_func(MCP2517FD_MISO_PIN, MCP2517FD_MOSI_PIN, MCP2517FD_SCK_PIN, GPIO_FUNC_SPI));

    // Setup Clock Output
    uint32_t clk_src_freq = clock_get_hz(MCP2517FD_OSC_SRC);
    hard_assert(clk_src_freq % MCP2517FD_OSC_RATE == 0);  // Ensure that clock divides evenly
    clock_gpio_init(MCP2517FD_CANCLK_PIN, MCP2517FD_CLK_GPOUT_AUXSRC, clk_src_freq / MCP2517FD_OSC_RATE);
    bi_decl_if_func_used(bi_1pin_with_name(MCP2517FD_CANCLK_PIN, "MCP251XFD CLOCK REF"));

    // Setup IRQ Pin
    gpio_init(MCP2517FD_INT_PIN);
    gpio_set_dir(MCP2517FD_INT_PIN, GPIO_IN);
    gpio_pull_up(MCP2517FD_INT_PIN);
    bi_decl_if_func_used(bi_1pin_with_name(MCP2517FD_INT_PIN, "MCP251XFD Interrupt Notify"));

    sleep_ms(10);   // Give time for clocks on device to stabalize

    //--- Configure Device ---
    eERRORRESULT error_code = ERR__NO_DEVICE_DETECTED;
    error_code = Init_MCP251XFD(&mcp251xfd_device, &mcp251xfd_device_config);
    if (error_code != ERR_OK) {
        canbus_report_driver_error(error_code);
        return false;
    }

    error_code = MCP251XFD_ConfigureTimeStamp(&mcp251xfd_device, true, MCP251XFD_TS_CAN20_SOF_CANFD_SOF,
                                             TIMESTAMP_TICK(mcp251xfd_device_sysclk), false);
    if (error_code != ERR_OK) {
        canbus_report_driver_error(error_code);
        return false;
    }

    // Configure FIFOs
    error_code = MCP251XFD_ConfigureFIFO(&mcp251xfd_device, &mcp251xfd_msg_tx_fifo_config);
    if (error_code != ERR_OK) {
        canbus_report_driver_error(error_code);
        return false;
    }

    error_code = MCP251XFD_ConfigureFIFO(&mcp251xfd_device, &mcp251xfd_msg_rx_fifo_config);
    if (error_code != ERR_OK) {
        canbus_report_driver_error(error_code);
        return false;
    }

    error_code = MCP251XFD_ConfigureFIFO(&mcp251xfd_device, &mcp251xfd_utility_tx_fifo_config);
    if (error_code != ERR_OK) {
        canbus_report_driver_error(error_code);
        return false;
    }

    error_code = MCP251XFD_ConfigureFIFO(&mcp251xfd_device, &mcp251xfd_utility_rx_fifo_config);
    if (error_code != ERR_OK) {
        canbus_report_driver_error(error_code);
        return false;
    }

    // Configure Filters
    error_code = MCP251XFD_ConfigureFilter(&mcp251xfd_device, &mcp251xfd_msg_rx_filter);
    if (error_code != ERR_OK) {
        canbus_report_driver_error(error_code);
        return false;
    }

    error_code = MCP251XFD_ConfigureFilter(&mcp251xfd_device, &mcp251xfd_msg_rx_crc_filter);
    if (error_code != ERR_OK) {
        canbus_report_driver_error(error_code);
        return false;
    }

    error_code = MCP251XFD_ConfigureFilter(&mcp251xfd_device, &mcp251xfd_utility_rx_filter);
    if (error_code != ERR_OK) {
        canbus_report_driver_error(error_code);
        return false;
    }

    // Start CAN
    #if CAN_BUS_ENABLE_FD
    error_code = MCP251XFD_StartCANFD(&mcp251xfd_device);
    #else
    error_code = MCP251XFD_StartCAN20(&mcp251xfd_device);
    #endif
    if (error_code != ERR_OK) {
        canbus_report_driver_error(error_code);
        return false;
    }

    // TODO: Fix only one irq allowed at a time
    gpio_set_irq_enabled_with_callback(MCP2517FD_INT_PIN, GPIO_IRQ_LEVEL_LOW, true, &can_mcp251xfd_interrupt_cb);

    return true;
}

void can_mcp251xfd_report_msg_tx_fifo_ready(void) {
    set_fifo_not_full_empty_interrupt(mcp251xfd_msg_tx_fifo, true);
}

void can_mcp251xfd_report_msg_rx_fifo_ready(void) {
    set_fifo_not_full_empty_interrupt(mcp251xfd_msg_rx_fifo, true);
}

void can_mcp251xfd_report_utility_tx_fifo_ready(void) {
    set_fifo_not_full_empty_interrupt(mcp251xfd_utility_tx_fifo, true);
}

void can_mcp251xfd_report_utility_rx_fifo_ready(void) {
    set_fifo_not_full_empty_interrupt(mcp251xfd_utility_rx_fifo, true);
}

void test_transmit(uint8_t data) {
    if (check_fifo_event(mcp251xfd_msg_tx_fifo, MCP251XFD_TX_FIFO_NOT_FULL)) {
        uint8_t buffer[5] = {0xDE, 0xAD, 0xBE, 0xEF, data};

        MCP251XFD_CANMessage msg;
        msg.DLC = 5;
        msg.MessageID = 0x123;
        msg.ControlFlags = MCP251XFD_CAN20_FRAME;
        msg.MessageSEQ = 0;
        msg.PayloadData = buffer;

        eERRORRESULT error_code = MCP251XFD_TransmitMessageToFIFO(&mcp251xfd_device, &msg, mcp251xfd_msg_tx_fifo, true);
        if (error_code != ERR_OK) canbus_report_driver_error(error_code);
        printf("Sent!\n");
    } else {
        canbus_call_receive_error_cb(69);
    }
}