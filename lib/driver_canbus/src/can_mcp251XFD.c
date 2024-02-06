#include "can_mcp251XFD_bridge.h"
#include "mcp251Xfd/CRC16_CMS.h"
#include "mcp251Xfd/MCP251XFD.h"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware/structs/iobank0.h"
#include "hardware/sync.h"
#include "pico/binary_info.h"
#include "pico/mutex.h"
#include "titan/canmore.h"

#include <stdint.h>

// ========================================
// Preprocessor Defines
// ========================================

// PICO_CONFIG: CAN_MCP251XFD_CONNECT_ATTEMPTS, Number of connection attempts when initializing the MCP251x Device, type=int, min=1, default=3, group=driver_canbus
#ifndef CAN_MCP251XFD_CONNECT_ATTEMPTS
#define CAN_MCP251XFD_CONNECT_ATTEMPTS 3
#endif

// PICO_CONFIG: CAN_MCP251XFD_CONNECT_DELAY, Delay between attempts of initializing the MCP251x Device in milliseconds, type=int, default=10, group=driver_canbus
#ifndef CAN_MCP251XFD_CONNECT_DELAY_MS
#define CAN_MCP251XFD_CONNECT_DELAY_MS 10
#endif

#if !defined(UWRT_ROBOT_DEFINED)
#error UWRT_ROBOT must be defined to retrieve CAN bus configuration for that robot

#elif !defined(UWRT_BOARD_DEFINED)
#error Selected PICO_BOARD is not a UWRT board - only UWRT boards have CAN bus configurations defined

#elif !defined(CAN_BUS_NAME)
#error No CAN bus defined in selected PICO_BOARD - does this board support CAN bus?

#else

// Set configurations for can bus
// Note that these are pulled from the board and robot configurations (hence the error check above)
// This pulls the CAN_BUS_NAME from the board files, and looks up that name in the robot definition file
// So for example, if the board has CAN_BUS_NAME=ALPHA the vehicle should have a ALPHA_ENABLE_FD=1 define
#define CAN_BUS_ENABLE_FD __CONCAT(CAN_BUS_NAME, _ENABLE_FD)
#define CAN_BUS_RATE __CONCAT(CAN_BUS_NAME, _RATE)
#if CAN_BUS_ENABLE_FD
#define CAN_BUS_FD_RATE __CONCAT(CAN_BUS_NAME, _FD_RATE)
#else
#define CAN_BUS_FD_RATE MCP251XFD_NO_CANFD
#endif

#define MCP2517FD_OSC_RATE 4000000  // OSC must run at 4MHz
#define MCP2517FD_OSC_SRC clk_usb   // Use 48 MHz clock source so it'll evenly divide into 4 MHz

#if MCP2517FD_OSC_SRC == clk_usb
#define MCP2517FD_CLK_GPOUT_AUXSRC CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_VALUE_CLK_USB
#elif MCP2517FD_OSC_SRC == clk_adc
#define MCP2517FD_CLK_GPOUT_AUXSRC CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_VALUE_CLK_ADC
#else
#error Invalid clock source
#endif

#define MCP2517FD_SPI_INST __CONCAT(spi, MCP2517FD_SPI)

#define UTILITY_MSG_PAYLOAD_SIZE_ENUM MCP251XFD_PAYLOAD_8BYTE
static_assert(CANMORE_FRAME_SIZE == 8, "Utility payload size enum does not match configured Utility Frame Size");
// Note if the static assert above fails, this requires a few things to be changed
//  1. The UTILITY_MSG_PAYLOAD_SIZE_ENUM so it is the right size
//  2. The DLC calculations when constructing a utility frame, as they assume 8 byte max (since DLC gets more
//  complicated above 8)
//  3. The utility message frame type, as it currently sends as a CAN 2.0 frame, not CAN FD frame which is needed when
//  size > 8

uint32_t saved_client_id;

// ========================================
// Utility Functions
// ========================================

/**
 * @brief Returns if the specified GPIO IRQ is enabled
 *
 * @param gpio The GPIO pin to check
 * @param event The GPIO IRQ event to check
 * @return true IRQ is enabled for the event on the specified pin
 * @return false IRQ is not enabled
 */
static bool gpio_is_irq_enabled(unsigned int gpio, unsigned int event) {
    // Separate mask/force/status per-core, so check which core called, and
    // set the relevant IRQ controls.
    io_irq_ctrl_hw_t *irq_ctrl_base = get_core_num() ? &iobank0_hw->proc1_irq_ctrl : &iobank0_hw->proc0_irq_ctrl;
    io_rw_32 *en_reg = &irq_ctrl_base->inte[gpio / 8];
    event <<= 4 * (gpio % 8);

    return ((*en_reg) & event) == event;
}

/**
 * @brief Check if the specified FIFO name has the specified event pending
 *
 * @param pComp The pointed structure of the device to be used
 * @param name FIFO to check events
 * @param event The FIFO status event to check
 * @return true Event is pending
 * @return false Event is not pending
 */
static bool check_fifo_event(MCP251XFD *pComp, eMCP251XFD_FIFO name, eMCP251XFD_FIFOstatus event) {
    eMCP251XFD_FIFOstatus fifo_status;
    eERRORRESULT error_code = MCP251XFD_GetFIFOStatus(pComp, name, &fifo_status);
    if (error_code != ERR_OK) {
        canbus_report_driver_error(error_code);
        return false;
    }

    return (fifo_status & event) != 0;
}

/**
 * @brief Set the tx/rx fifo's not full/not empty interrupt enable bit
 *
 * @param pComp The pointed structure of the device to be used
 * @param name Fifo to configure
 * @param enabled Whether to enable the fifo interrupt
 */
static void set_fifo_not_full_empty_interrupt(MCP251XFD *pComp, eMCP251XFD_FIFO name, bool enabled) {
    // Calculate FIFO interrupt configuration address
    uint16_t address = RegMCP251XFD_CiFIFOCONm_CONFIG + (MCP251XFD_FIFO_REG_SIZE * ((uint16_t) name - 1u));

    uint8_t register_data;
    eERRORRESULT error_code = MCP251XFD_ReadSFR8(pComp, address, &register_data);
    if (error_code != ERR_OK) {
        canbus_report_driver_error(error_code);
        return;
    }

    // Mask off interrupt enabled
    register_data &= ~MCP251XFD_CAN_CiFIFOCONm8_TFNRFNIE;

    // Set flag if enabed
    register_data |= (enabled ? MCP251XFD_CAN_CiFIFOCONm8_TFNRFNIE : 0);

    // Write back
    error_code = MCP251XFD_WriteSFR8(pComp, address, register_data);
    if (error_code != ERR_OK) {
        canbus_report_driver_error(error_code);
        return;
    }
}

/**
 * @brief Check fifo for interrupt event, and clear interrupt event if set
 *
 * @param pComp The pointed structure of the device to be used
 * @param name FIFO to check
 * @param event Event to check for/clear
 * @return true Interrupt event found and cleared
 * @return false Interrupt event not found
 */
static bool check_and_clear_fifo_event(MCP251XFD *pComp, eMCP251XFD_FIFO name, eMCP251XFD_FIFOstatus event) {
    if (check_fifo_event(pComp, name, event)) {
        eERRORRESULT error_code = MCP251XFD_ClearFIFOEvents(pComp, name, event);
        if (error_code != ERR_OK)
            canbus_report_driver_error(error_code);
        return true;
    }
    else {
        return false;
    }
}

// ========================================
// Driver SPI Callbacks
// ========================================

auto_init_mutex(mcp251Xfd_spi_mutex);

eERRORRESULT can_mcp251x_spi_config(void *pIntDev, __unused uint8_t chipSelect, const uint32_t sckFreq) {
    // SPI is already configured before MCP251XFD driver is called, to avoid re-initializing hardware just set the
    // appropriate frequency
    spi_set_baudrate((spi_inst_t *) pIntDev, sckFreq);

    return ERR_OK;
}

static eERRORRESULT __not_in_flash_func(can_mcp251x_spi_transfer)(void *pIntDev, uint8_t chipSelect, uint8_t *txData,
                                                                  uint8_t *rxData, size_t size) {
    if (!txData) {
        return ERR__SPI_PARAMETER_ERROR;
    }

    // Disable interrupts from CAN controller in the middle of transfer
    uint32_t prev_state = save_and_disable_mcp251Xfd_irq();

    // Try to enter mutex to ensure transaction isn't interrupted recurisvely
    // This *shouldn't* happen since CAN transmit shouldn't be called in an interrupt, but just incase
    if (!mutex_try_enter(&mcp251Xfd_spi_mutex, NULL)) {
        restore_mcp251Xfd_irq(prev_state);

        // ***NOTE FROM AUTHOR***
        // If you are debugging and you are getting this return value YOU ARE DOING SOMETHING WRONG
        // The CAN functions should not be called in interrupts (as mentioned in the docs)
        // This can only occur if a CAN function is called DURING another transaction
        // Change your program to avoid this case
        return ERR__SPI_BUSY;
    }

    gpio_put(chipSelect, false);

    if (rxData) {
        spi_write_read_blocking((spi_inst_t *) pIntDev, txData, rxData, size);
    }
    else {
        spi_write_blocking((spi_inst_t *) pIntDev, txData, size);
    }

    gpio_put(chipSelect, true);

    mutex_exit(&mcp251Xfd_spi_mutex);
    restore_mcp251Xfd_irq(prev_state);

    return ERR_OK;
}

static uint32_t __not_in_flash_func(can_mcp251x_get_current_ms)(void) {
    return to_ms_since_boot(get_absolute_time());
}

// ========================================
// Driver Configuration
// ========================================

MCP251XFD mcp251xfd_device = {
    .UserDriverData = NULL,
    //--- Driver configuration ---
    .DriverConfig = MCP251XFD_DRIVER_SAFE_RESET | MCP251XFD_DRIVER_USE_READ_WRITE_CRC |
                    MCP251XFD_DRIVER_USE_SAFE_WRITE | MCP251XFD_DRIVER_ENABLE_ECC | MCP251XFD_DRIVER_INIT_CHECK_RAM |
                    MCP251XFD_DRIVER_INIT_SET_RAM_AT_0 | MCP251XFD_DRIVER_CLEAR_BUFFER_BEFORE_READ,
    //--- IO configuration ---
    .GPIOsOutState = MCP251XFD_GPIO0_LOW | MCP251XFD_GPIO1_LOW,
    //--- Interface driver call functions ---
    .SPI_ChipSelect = MCP2517FD_NCS_PIN,    // Passed to spi functions
    .InterfaceDevice = MCP2517FD_SPI_INST,  // Passed to spi functions
    .fnSPI_Init = can_mcp251x_spi_config,
    .fnSPI_Transfer = can_mcp251x_spi_transfer,
    //--- Time call function ---
    .fnGetCurrentms = can_mcp251x_get_current_ms,
    //--- CRC16-CMS call function ---
    .fnComputeCRC16 = ComputeCRC16CMS,
    //--- Interface clocks ---
    .SPIClockSpeed = 20000000,  // 20MHz (must be at most mcp251x sysclk/2)
};

MCP251XFD_BitTimeStats mcp251xfd_device_btstats;
uint32_t mcp251xfd_device_sysclk;
MCP251XFD_Config mcp251xfd_device_config = {
    //--- Controller clocks ---
    .XtalFreq = 0,                                     // CLKIN is not a crystal
    .OscFreq = MCP2517FD_OSC_RATE,                     // CLKIN is a 4MHz oscillator generated from the RP2040
    .SysclkConfig = MCP251XFD_SYSCLK_IS_CLKIN_MUL_10,  // Generate 40 MHz clock
    .ClkoPinConfig = MCP251XFD_CLKO_SOF,
    .SYSCLK_Result = &mcp251xfd_device_sysclk,
    //--- CAN configuration ---
    .NominalBitrate = CAN_BUS_RATE,
    .DataBitrate = CAN_BUS_FD_RATE,
    .BitTimeStats = &mcp251xfd_device_btstats,
    .Bandwidth = MCP251XFD_NO_DELAY,
    .ControlFlags = MCP251XFD_CAN_RESTRICTED_MODE_ON_ERROR | MCP251XFD_CAN_ESI_REFLECTS_ERROR_STATUS |
                    MCP251XFD_CAN_RESTRICTED_RETRANS_ATTEMPTS | MCP251XFD_CANFD_BITRATE_SWITCHING_ENABLE |
                    MCP251XFD_CAN_PROTOCOL_EXCEPT_AS_FORM_ERROR | MCP251XFD_CANFD_USE_ISO_CRC |
                    MCP251XFD_CANFD_DONT_USE_RRS_BIT_AS_SID11,
    //--- GPIOs and Interrupts pins ---
    .GPIO0PinMode = MCP251XFD_PIN_AS_GPIO0_IN,
    .GPIO1PinMode = MCP251XFD_PIN_AS_GPIO1_IN,
    .INTsOutMode = MCP251XFD_PINS_PUSHPULL_OUT,
    .TXCANOutMode = MCP251XFD_PINS_PUSHPULL_OUT,
    //--- Interrupts ---
    .SysInterruptFlags = MCP251XFD_INT_TX_EVENT             // Enable global TX interrupts (controlled via FIFO)
                         | MCP251XFD_INT_RX_EVENT           // Enable global RX interrupts (controlled via FIFO)
                         | MCP251XFD_INT_TEF_EVENT          // Enable TEF events (so we can watch when we send messages)
                                                            // Device Errors
                         | MCP251XFD_INT_RX_OVERFLOW_EVENT  // Device error if message dropped from code being slow
                         | MCP251XFD_INT_RAM_ECC_EVENT      // Report any RAM ECC errors
                         | MCP251XFD_INT_SPI_CRC_EVENT      // Report any SPI CRC errors
                         | MCP251XFD_INT_SYSTEM_ERROR_EVENT  // Report any errors on device
                                                             // Bus Errors
                         | MCP251XFD_INT_BUS_ERROR_EVENT  // Notify on transitioning for bus errors (avoids spamming on
                                                          // stuck bus for RX_INVALID_MESSAGE_EVENT)
                         | MCP251XFD_INT_TX_ATTEMPTS_EVENT,  // Notify on message reaching retransmit limit
};

// FIFO definitions
#define mcp251xfd_tx_fifo                                                                                              \
    MCP251XFD_FIFO1  // Utility and msg tx fifos joined to ensure that if the msg fifo stalls so do heartbeats, causing
                     // the device to reset
#define mcp251xfd_msg_rx_fifo MCP251XFD_FIFO2
#define mcp251xfd_utility_rx_fifo MCP251XFD_FIFO3

#define mcp251xfd_msg_rx_filter_num MCP251XFD_FILTER0
#define mcp251xfd_msg_rx_crc_filter_num MCP251XFD_FILTER1
#define mcp251xfd_utility_rx_filter_num MCP251XFD_FILTER2

MCP251XFD_RAMInfos mcp251xfd_transmit_event_raminfo;
MCP251XFD_RAMInfos mcp251xfd_tx_raminfo;
MCP251XFD_RAMInfos mcp251xfd_msg_rx_raminfo;
MCP251XFD_RAMInfos mcp251xfd_utility_rx_raminfo;

MCP251XFD_FIFO mcp251xfd_transmit_event_fifo_config = {
    .Name = MCP251XFD_TEF,
    .Size = MCP251XFD_FIFO_12_MESSAGE_DEEP,
    .ControlFlags = MCP251XFD_FIFO_ADD_TIMESTAMP_ON_OBJ,
    .InterruptFlags = MCP251XFD_FIFO_OVERFLOW_INT + MCP251XFD_FIFO_EVENT_FIFO_NOT_EMPTY_INT,
    .RAMInfos = &mcp251xfd_transmit_event_raminfo,
};

MCP251XFD_FIFO mcp251xfd_tx_fifo_config = {
    .Name = mcp251xfd_tx_fifo,
    .Size = MCP251XFD_FIFO_12_MESSAGE_DEEP,
    .Payload = MCP251XFD_PAYLOAD_8BYTE,
    .Direction = MCP251XFD_TRANSMIT_FIFO,
    .Attempts = MCP251XFD_THREE_ATTEMPTS,
    .Priority = MCP251XFD_MESSAGE_TX_PRIORITY16,
    .ControlFlags = MCP251XFD_FIFO_NO_RTR_RESPONSE,
    .InterruptFlags = MCP251XFD_FIFO_TX_ATTEMPTS_EXHAUSTED_INT + MCP251XFD_FIFO_TRANSMIT_FIFO_NOT_FULL_INT,
    .RAMInfos = &mcp251xfd_tx_raminfo,
};

MCP251XFD_FIFO mcp251xfd_msg_rx_fifo_config = {
    .Name = mcp251xfd_msg_rx_fifo,
    .Size = MCP251XFD_FIFO_32_MESSAGE_DEEP,
    .Payload = MCP251XFD_PAYLOAD_8BYTE,
    .Direction = MCP251XFD_RECEIVE_FIFO,
    .ControlFlags = MCP251XFD_FIFO_ADD_TIMESTAMP_ON_RX,
    .InterruptFlags = MCP251XFD_FIFO_OVERFLOW_INT + MCP251XFD_FIFO_RECEIVE_FIFO_NOT_EMPTY_INT,
    .RAMInfos = &mcp251xfd_msg_rx_raminfo,
};

MCP251XFD_FIFO mcp251xfd_utility_rx_fifo_config = {
    .Name = mcp251xfd_utility_rx_fifo,
    .Size = MCP251XFD_FIFO_32_MESSAGE_DEEP,
    .Payload = UTILITY_MSG_PAYLOAD_SIZE_ENUM,
    .Direction = MCP251XFD_RECEIVE_FIFO,
    .ControlFlags = MCP251XFD_FIFO_ADD_TIMESTAMP_ON_RX,
    .InterruptFlags = MCP251XFD_FIFO_OVERFLOW_INT + MCP251XFD_FIFO_RECEIVE_FIFO_NOT_EMPTY_INT,
    .RAMInfos = &mcp251xfd_utility_rx_raminfo,
};

// NOTE: Ensure that any new fifos or changed interrupt flags are properly handled in the IRQ handler
// If not, the IRQ handler might get stuck

MCP251XFD_Filter mcp251xfd_msg_rx_filter = {
    // Filter for standard CANmore message frames
    // Match only standard CANmore message frames for this client from the agent
    .Filter = mcp251xfd_msg_rx_filter_num,
    .EnableFilter = true,
    .Match = MCP251XFD_MATCH_ONLY_SID,
    .AcceptanceID = 0,
    .AcceptanceMask = CANMORE_CALC_FILTER_MASK(true, true, true, false),
    .PointTo = mcp251xfd_msg_rx_fifo,
};

MCP251XFD_Filter mcp251xfd_msg_rx_crc_filter = {
    // Filter for extended (crc) CANmore message frames
    // Match only extended CANmore message frames for this client from the agent
    .Filter = mcp251xfd_msg_rx_crc_filter_num,
    .EnableFilter = true,
    .Match = MCP251XFD_MATCH_ONLY_EID,
    .AcceptanceID = 0,
    .AcceptanceMask = CANMORE_CALC_EXT_FILTER_MASK(true, true, true, false, false),
    .PointTo = mcp251xfd_msg_rx_fifo,
};

MCP251XFD_Filter mcp251xfd_utility_rx_filter = {
    // Filter for standard CANmore utility frames
    // Match only standard CANmore utility frames for this client from the agent
    .Filter = mcp251xfd_utility_rx_filter_num,
    .EnableFilter = true,
    .Match = MCP251XFD_MATCH_ONLY_SID,
    .AcceptanceID = 0,
    .AcceptanceMask = CANMORE_CALC_FILTER_MASK(true, true, true, false),
    .PointTo = mcp251xfd_utility_rx_fifo,
};

#if MCP2517FD_TERM_SENSE_ON_INT0

/**
 * @brief Gets the termination resistor state
 *
 * @param term_state_out Pointer to write termination state if valid, writes true if termination is enabled, false if
 * not
 * @return true The termination state is valid
 * @return false Could not read termination state
 */
bool can_mcp251x_get_term_state(bool *term_state_out) {
    uint8_t pin_state;
    eERRORRESULT err = MCP251XFD_GetGPIOPinsInputLevel(&mcp251xfd_device, &pin_state);

    if (err == ERR_OK) {
        if (pin_state & MCP251XFD_GPIO0_HIGH) {
            *term_state_out = true;
        }
        else {
            *term_state_out = false;
        }

        return true;
    }
    else {
        return false;
    }
}

#else

bool can_mcp251x_get_term_state(bool *term_state_out) {
    return false;
}

#endif

// ========================================
// Interrupt Pin Handling
// ========================================

void can_mcp251xfd_interrupt_cb(uint gpio, uint32_t events) {
    if (gpio != CAN_MCP251XFD_EXTERNAL_INTERRUPT_PIN || events != CAN_MCP251XFD_EXTERNAL_INTERRUPT_EVENTS) {
        return;
    }

    if (!gpio_is_irq_enabled(CAN_MCP251XFD_EXTERNAL_INTERRUPT_PIN, CAN_MCP251XFD_EXTERNAL_INTERRUPT_EVENTS)) {
        return;
    }

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
        if (check_and_clear_fifo_event(&mcp251xfd_device, mcp251xfd_msg_rx_fifo, MCP251XFD_RX_FIFO_OVERFLOW)) {
            canbus_report_library_error(CANBUS_LIBERR_RX_OVERFLOW);
            interrupt_cleared = true;
        }
        if (check_and_clear_fifo_event(&mcp251xfd_device, mcp251xfd_utility_rx_fifo, MCP251XFD_RX_FIFO_OVERFLOW)) {
            canbus_report_library_error(CANBUS_LIBERR_RX_OVERFLOW);
            interrupt_cleared = true;
        }
    }
    if (active_interrupts & MCP251XFD_INT_RAM_ECC_EVENT) {
        canbus_report_library_error(CANBUS_LIBERR_DEVICE_MALFUNCTION);

        interrupt_cleared = true;
        error_code = MCP251XFD_ClearECCEvents(&mcp251xfd_device);
        if (error_code != ERR_OK)
            canbus_report_driver_error(error_code);
    }
    if (active_interrupts & MCP251XFD_INT_SPI_CRC_EVENT) {
        canbus_report_library_error(CANBUS_LIBERR_COMM_MALFUNCTION);

        interrupt_cleared = true;
        error_code = MCP251XFD_ClearCRCEvents(&mcp251xfd_device);
        if (error_code != ERR_OK)
            canbus_report_driver_error(error_code);
    }
    if (active_interrupts & MCP251XFD_INT_SYSTEM_ERROR_EVENT) {
        canbus_report_library_error(CANBUS_LIBERR_DEVICE_MALFUNCTION);

        interrupt_cleared = true;
        error_code = MCP251XFD_ClearInterruptEvents(&mcp251xfd_device, MCP251XFD_INT_SYSTEM_ERROR_EVENT);
        if (error_code != ERR_OK)
            canbus_report_driver_error(error_code);
    }

    // Bus Errors
    if (active_interrupts & MCP251XFD_INT_BUS_ERROR_EVENT) {
        canbus_call_receive_error_cb(CANBUS_RECVERR_BUS_ERROR);

        interrupt_cleared = true;
        error_code = MCP251XFD_ClearInterruptEvents(&mcp251xfd_device, MCP251XFD_INT_BUS_ERROR_EVENT);
        if (error_code != ERR_OK)
            canbus_report_driver_error(error_code);
    }
    if (active_interrupts & MCP251XFD_INT_TX_ATTEMPTS_EVENT) {
        if (check_and_clear_fifo_event(&mcp251xfd_device, mcp251xfd_tx_fifo, MCP251XFD_TX_FIFO_ATTEMPTS_EXHAUSTED)) {
            canbus_call_receive_error_cb(CANBUS_RECVERR_MSG_TX_FAILURE);
            interrupt_cleared = true;
        }
    }

    // Handle FIFO Events
    if (active_interrupts & MCP251XFD_INT_TX_EVENT) {
        if (check_fifo_event(&mcp251xfd_device, mcp251xfd_tx_fifo, MCP251XFD_TX_FIFO_NOT_FULL)) {
            if (!canmore_msg_encode_done(&encoding_buffer)) {
                uint8_t buffer[MCP251XFD_PAYLOAD_MAX];
                bool is_extended;
                MCP251XFD_CANMessage msg;

                // If this assert no longer holds true, then the DLC must be looked up by the eMCP251XFD_DataLength enum
                // This is because CAN FD compresses the DLC by only allowing specific sizes above 8 bytes
                // The canmore encoding must be modified to account for this, as canmore messages support arbitrary
                // lengths
                static_assert(CANMORE_FRAME_SIZE <= 8, "Message encoding does not fit into simple DLC");
                canmore_msg_encode_next(&encoding_buffer, buffer, &msg.DLC, &msg.MessageID, &is_extended);

                msg.ControlFlags = MCP251XFD_CAN20_FRAME;

                if (is_extended) {
                    msg.ControlFlags |= MCP251XFD_EXTENDED_MESSAGE_ID;
                }
                msg.MessageSEQ = MESSAGE_SEQ_MSG_FRAME;
                msg.PayloadData = buffer;

                interrupt_cleared = true;
                error_code = MCP251XFD_TransmitMessageToFIFO(&mcp251xfd_device, &msg, mcp251xfd_tx_fifo, true);
                if (error_code != ERR_OK)
                    canbus_report_driver_error(error_code);
            }
            else if (utility_tx_buf.waiting) {
                MCP251XFD_CANMessage msg;

                // Note this only holds when max utility_tx_buf.length is 8
                msg.DLC = utility_tx_buf.length;
                msg.MessageID = CANMORE_CALC_UTIL_ID_C2A(saved_client_id, utility_tx_buf.channel);
                msg.ControlFlags = MCP251XFD_CAN20_FRAME;
                msg.MessageSEQ = utility_tx_buf.seq;
                msg.PayloadData = utility_tx_buf.data;

                interrupt_cleared = true;
                error_code = MCP251XFD_TransmitMessageToFIFO(&mcp251xfd_device, &msg, mcp251xfd_tx_fifo, true);
                if (error_code != ERR_OK)
                    canbus_report_driver_error(error_code);

                utility_tx_buf.waiting = false;
            }
            else {
                interrupt_cleared = true;
                set_fifo_not_full_empty_interrupt(&mcp251xfd_device, mcp251xfd_tx_fifo, false);
            }
        }
    }

    if (active_interrupts & MCP251XFD_INT_RX_EVENT) {
        if (check_fifo_event(&mcp251xfd_device, mcp251xfd_msg_rx_fifo, MCP251XFD_RX_FIFO_NOT_EMPTY)) {
            if (canbus_msg_driver_space_in_rx()) {
                uint32_t timestamp = 0;
                uint8_t payload[MCP251XFD_PayloadToByte(mcp251xfd_msg_rx_fifo_config.Payload)];

                MCP251XFD_CANMessage msg;
                msg.PayloadData = payload;

                interrupt_cleared = true;
                error_code = MCP251XFD_ReceiveMessageFromFIFO(
                    &mcp251xfd_device, &msg, mcp251xfd_msg_rx_fifo_config.Payload, &timestamp, mcp251xfd_msg_rx_fifo);
                if (error_code != ERR_OK) {
                    canbus_report_driver_error(error_code);
                }
                else {
                    bool is_extended = (msg.ControlFlags & MCP251XFD_EXTENDED_MESSAGE_ID) != 0;
                    bool is_canfd = (msg.ControlFlags & MCP251XFD_CANFD_FRAME) != 0;
                    canbus_msg_driver_post_rx(msg.MessageID, is_extended, MCP251XFD_DLCToByte(msg.DLC, is_canfd),
                                              msg.PayloadData);
                }
            }
            else {
                interrupt_cleared = true;
                set_fifo_not_full_empty_interrupt(&mcp251xfd_device, mcp251xfd_msg_rx_fifo, false);
            }
        }

        if (check_fifo_event(&mcp251xfd_device, mcp251xfd_utility_rx_fifo, MCP251XFD_RX_FIFO_NOT_EMPTY)) {
            // Check if space available
            if (!utility_rx_buf.waiting) {
                uint32_t timestamp = 0;
                MCP251XFD_CANMessage msg;
                msg.PayloadData = utility_rx_buf.data;
                // Ensure that the utility_rx_buf data won't overflow
                static_assert(UTILITY_MSG_PAYLOAD_SIZE_ENUM == MCP251XFD_PAYLOAD_8BYTE &&
                                  sizeof(utility_rx_buf.data) == 8,
                              "Utility payload size does not match buffer size");

                // that will be received
                interrupt_cleared = true;
                error_code = MCP251XFD_ReceiveMessageFromFIFO(&mcp251xfd_device, &msg, UTILITY_MSG_PAYLOAD_SIZE_ENUM,
                                                              &timestamp, mcp251xfd_utility_rx_fifo);
                if (error_code != ERR_OK) {
                    canbus_report_driver_error(error_code);
                }
                else {
                    bool is_extended = (msg.ControlFlags & MCP251XFD_EXTENDED_MESSAGE_ID) != 0;
                    if (!is_extended) {
                        canmore_id_t id = { .identifier = msg.MessageID };
                        bool is_canfd = (msg.ControlFlags & MCP251XFD_CANFD_FRAME) != 0;
                        unsigned int length = MCP251XFD_DLCToByte(msg.DLC, is_canfd);
                        if (length > sizeof(utility_rx_buf.data)) {
                            length = sizeof(utility_rx_buf.data);
                        }

                        utility_rx_buf.length = length;
                        utility_rx_buf.channel = id.pkt_std.noc;
                        utility_rx_buf.waiting = true;
                    }
                }
            }
            else {
                set_fifo_not_full_empty_interrupt(&mcp251xfd_device, mcp251xfd_utility_rx_fifo, false);
                interrupt_cleared = true;
            }
        }
    }

    if (active_interrupts & MCP251XFD_INT_TEF_EVENT) {
        eMCP251XFD_TEFstatus tefStatus;
        error_code = MCP251XFD_GetTEFStatus(&mcp251xfd_device, &tefStatus);
        if (error_code != ERR_OK) {
            canbus_report_driver_error(error_code);
        }
        else {
            if (tefStatus & MCP251XFD_TEF_FIFO_OVERFLOW) {
                canbus_report_library_error(CANBUS_LIBERR_TEF_OVERFLOW);

                eERRORRESULT error_code =
                    MCP251XFD_ClearFIFOEvents(&mcp251xfd_device, MCP251XFD_TEF, MCP251XFD_TEF_FIFO_OVERFLOW);
                if (error_code != ERR_OK)
                    canbus_report_driver_error(error_code);

                interrupt_cleared = true;
            }
            if (tefStatus & MCP251XFD_TEF_FIFO_NOT_EMPTY) {
                MCP251XFD_CANMessage msg;
                uint32_t timestamp;
                MCP251XFD_ReceiveMessageFromTEF(&mcp251xfd_device, &msg, &timestamp);

                // If its a heartbeat transmission, keep track when we received it
                if (msg.MessageSEQ == MESSAGE_SEQ_UTILITY_HEARTBEAT) {
                    heartbeat_transmit_timeout = make_timeout_time_ms(CAN_MCP251XFD_HEARTBEAT_TIMEOUT_MS);
                }
                interrupt_cleared = true;
            }
        }
    }

    // Report if an interrupt was raised but was unable to be cleared
    if (!interrupt_cleared) {
        canbus_report_library_error(CANBUS_LIBERR_UNEXPECTED_INTERRUPT);
    }
}

#define TIMESTAMP_TICK_us (25)  // TimeStamp tick is 25µs
#define TIMESTAMP_TICK(sysclk) (((sysclk) / 1000000) * TIMESTAMP_TICK_us)

// ========================================
// Driver Exports
// ========================================

static bool can_mcp251xfd_configure_chip(void) {
    // First disable the irq to ensure that we are configuring this in a safety context
    gpio_set_irq_enabled(CAN_MCP251XFD_EXTERNAL_INTERRUPT_PIN, CAN_MCP251XFD_EXTERNAL_INTERRUPT_EVENTS, false);

    //--- Configure Device ---
    eERRORRESULT error_code = ERR__NO_DEVICE_DETECTED;

    for (int try = 0; try < CAN_MCP251XFD_CONNECT_ATTEMPTS; try++) {
        error_code = Init_MCP251XFD(&mcp251xfd_device, &mcp251xfd_device_config);
        if (error_code == ERR_OK)
            break;
        sleep_ms(CAN_MCP251XFD_CONNECT_DELAY_MS);
    }

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
    error_code = MCP251XFD_ConfigureTEF(&mcp251xfd_device, true, &mcp251xfd_transmit_event_fifo_config);
    if (error_code != ERR_OK) {
        canbus_report_driver_error(error_code);
        return false;
    }

    error_code = MCP251XFD_ConfigureFIFO(&mcp251xfd_device, &mcp251xfd_tx_fifo_config);
    if (error_code != ERR_OK) {
        canbus_report_driver_error(error_code);
        return false;
    }

    error_code = MCP251XFD_ConfigureFIFO(&mcp251xfd_device, &mcp251xfd_msg_rx_fifo_config);
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

#if CAN_MCP251XFD_USE_EXTERNAL_INTERRUPT_CB
    gpio_set_irq_enabled(CAN_MCP251XFD_EXTERNAL_INTERRUPT_PIN, CAN_MCP251XFD_EXTERNAL_INTERRUPT_EVENTS, true);
#else
    gpio_set_irq_enabled_with_callback(CAN_MCP251XFD_EXTERNAL_INTERRUPT_PIN, CAN_MCP251XFD_EXTERNAL_INTERRUPT_EVENTS,
                                       true, &can_mcp251xfd_interrupt_cb);
#endif

    return true;
}

bool can_mcp251xfd_configure(unsigned int client_id) {
    saved_client_id = client_id;

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

    sleep_ms(CAN_MCP251XFD_CONNECT_DELAY_MS);  // Give time for clocks on device to stabalize

    return can_mcp251xfd_configure_chip();
}

static bool reset_now = false;

void can_mcp251xfd_check_offline_reset(void) {
    // Check if we need to reset the chip after being offline for too long
    // Handles edge cases where the chip will randomly stop transmitting in the event of certain edge cases
    if (absolute_time_diff_us(heartbeat_transmit_timeout, get_absolute_time()) >
            (CAN_MCP251XFD_OFFLINE_RESET_TIMEOUT_MS * 1000) ||
        reset_now) {
        if (can_mcp251xfd_configure_chip()) {
            reset_now = false;
            // Only clear the heartbeat transmit timeout (but not put it in the future, marking the bus online)
            // if the initialization was successful. If not, then next time the canbus ticks this function will be
            // called and this function should retry
            heartbeat_transmit_timeout = get_absolute_time();
        }
    }
}

void can_mcp251xfd_report_msg_tx_fifo_ready(void) {
    // Ensure that readback/writing to FIFO configuration isn't interrupted
    uint32_t prev_state = save_and_disable_mcp251Xfd_irq();
    set_fifo_not_full_empty_interrupt(&mcp251xfd_device, mcp251xfd_tx_fifo, true);
    restore_mcp251Xfd_irq(prev_state);
}

void can_mcp251xfd_report_msg_rx_fifo_ready(void) {
    // Ensure that readback/writing to FIFO configuration isn't interrupted
    uint32_t prev_state = save_and_disable_mcp251Xfd_irq();
    set_fifo_not_full_empty_interrupt(&mcp251xfd_device, mcp251xfd_msg_rx_fifo, true);
    restore_mcp251Xfd_irq(prev_state);
}

void can_mcp251xfd_report_utility_tx_fifo_ready(void) {
    // Ensure that readback/writing to FIFO configuration isn't interrupted
    uint32_t prev_state = save_and_disable_mcp251Xfd_irq();
    set_fifo_not_full_empty_interrupt(&mcp251xfd_device, mcp251xfd_tx_fifo, true);
    restore_mcp251Xfd_irq(prev_state);
}

void can_mcp251xfd_report_utility_rx_fifo_ready(void) {
    // Ensure that readback/writing to FIFO configuration isn't interrupted
    uint32_t prev_state = save_and_disable_mcp251Xfd_irq();
    set_fifo_not_full_empty_interrupt(&mcp251xfd_device, mcp251xfd_utility_rx_fifo, true);
    restore_mcp251Xfd_irq(prev_state);
}

uint32_t save_and_disable_mcp251Xfd_irq(void) {
    unsigned int was_enabled =
        (gpio_is_irq_enabled(CAN_MCP251XFD_EXTERNAL_INTERRUPT_PIN, CAN_MCP251XFD_EXTERNAL_INTERRUPT_EVENTS) ? 1 : 0);
    gpio_set_irq_enabled(CAN_MCP251XFD_EXTERNAL_INTERRUPT_PIN, CAN_MCP251XFD_EXTERNAL_INTERRUPT_EVENTS, false);

    return was_enabled;
}

void restore_mcp251Xfd_irq(uint32_t prev_interrupt_state) {
    gpio_set_irq_enabled(CAN_MCP251XFD_EXTERNAL_INTERRUPT_PIN, CAN_MCP251XFD_EXTERNAL_INTERRUPT_EVENTS,
                         prev_interrupt_state != 0);
}

void canbus_reenable_intr(void) {
    uint32_t prev_state = save_and_disable_mcp251Xfd_irq();
    set_fifo_not_full_empty_interrupt(&mcp251xfd_device, mcp251xfd_tx_fifo, true);
    set_fifo_not_full_empty_interrupt(&mcp251xfd_device, mcp251xfd_msg_rx_fifo, true);
    set_fifo_not_full_empty_interrupt(&mcp251xfd_device, mcp251xfd_utility_rx_fifo, true);
    restore_mcp251Xfd_irq(prev_state);
}

void canbus_fifo_clear(void) {
    uint32_t prev_state = save_and_disable_mcp251Xfd_irq();
    MCP251XFD_ResetFIFO(&mcp251xfd_device, mcp251xfd_msg_rx_fifo);
    restore_mcp251Xfd_irq(prev_state);
}

void canbus_reset(void) {
    reset_now = true;
}

#endif
