#include "bl_interface.h"

#include <stdint.h>
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/structs/iobank0.h"
#include "hardware/spi.h"
#include "pico/time.h"

#include "canmore_titan/protocol.h"
#include "CRC16_CMS.h"
#include "MCP251XFD.h"

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

#define UTILITY_MSG_PAYLOAD_SIZE_ENUM MCP251XFD_PAYLOAD_8BYTE

uint32_t saved_client_id;
uint32_t saved_channel;


// ========================================
// Driver SPI Callbacks
// ========================================

eERRORRESULT can_mcp251x_spi_config(void *pIntDev, __unused uint8_t chipSelect, const uint32_t sckFreq){
    // SPI is already configured before MCP251XFD driver is called, to avoid re-initializing hardware just set the appropriate frequency
    spi_set_baudrate((spi_inst_t*) pIntDev, sckFreq);

    return ERR_OK;
}

static eERRORRESULT __not_in_flash_func(can_mcp251x_spi_transfer)(void *pIntDev, uint8_t chipSelect, uint8_t *txData, uint8_t *rxData, size_t size) {
    if (!txData) {
        return ERR__SPI_PARAMETER_ERROR;
    }

    gpio_put(chipSelect, false);

    if (rxData) {
        spi_write_read_blocking((spi_inst_t*) pIntDev, txData, rxData, size);
    } else {
        spi_write_blocking((spi_inst_t*) pIntDev, txData, size);
    }

    gpio_put(chipSelect, true);

    return ERR_OK;
}

static uint32_t can_mcp251x_get_current_ms(void) {
    return us_to_ms(time_us_64());
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
    .NominalBitrate = CAN_BUS_RATE,
    .DataBitrate    = CAN_BUS_FD_RATE,
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
    .SysInterruptFlags = MCP251XFD_INT_RX_EVENT                 // Enable global RX interrupts (controlled via FIFO)
                         // No need to handle any other interrupts, if CAN goes down, the bootloader will timeout and the device will reset
};

// FIFO definitions
#define mcp251xfd_utility_tx_fifo        MCP251XFD_FIFO1
#define mcp251xfd_utility_rx_fifo        MCP251XFD_FIFO2

#define mcp251xfd_utility_rx_filter_num  MCP251XFD_FILTER0

MCP251XFD_RAMInfos mcp251xfd_utility_tx_raminfo;
MCP251XFD_RAMInfos mcp251xfd_utility_rx_raminfo;

MCP251XFD_FIFO mcp251xfd_utility_tx_fifo_config = {
    // NOTE: Don't notify on TX failures
    .Name = mcp251xfd_utility_tx_fifo, .Size = MCP251XFD_FIFO_8_MESSAGE_DEEP, .Payload = UTILITY_MSG_PAYLOAD_SIZE_ENUM,
    .Direction = MCP251XFD_TRANSMIT_FIFO, .Attempts = MCP251XFD_THREE_ATTEMPTS,
    .Priority = MCP251XFD_MESSAGE_TX_PRIORITY16, .ControlFlags = MCP251XFD_FIFO_NO_RTR_RESPONSE,
    .InterruptFlags = MCP251XFD_FIFO_NO_INTERRUPT_FLAGS,
    .RAMInfos = &mcp251xfd_utility_tx_raminfo,
};

MCP251XFD_FIFO mcp251xfd_utility_rx_fifo_config = {
    .Name = mcp251xfd_utility_rx_fifo, .Size = MCP251XFD_FIFO_8_MESSAGE_DEEP, .Payload = UTILITY_MSG_PAYLOAD_SIZE_ENUM,
    .Direction = MCP251XFD_RECEIVE_FIFO, .ControlFlags = MCP251XFD_FIFO_ADD_TIMESTAMP_ON_RX,
    .InterruptFlags = MCP251XFD_FIFO_RECEIVE_FIFO_NOT_EMPTY_INT,
    .RAMInfos = &mcp251xfd_utility_rx_raminfo,
};

MCP251XFD_Filter mcp251xfd_utility_rx_filter =
{   // Filter for standard CANmore utility frames
    // Match only standard CANmore utility frames for this client from the agent
    .Filter = mcp251xfd_utility_rx_filter_num, .EnableFilter = true, .Match = MCP251XFD_MATCH_ONLY_SID,
    .AcceptanceID = 0, .AcceptanceMask = CANMORE_CALC_FILTER_MASK(true, true, true, true),
    .PointTo = mcp251xfd_utility_rx_fifo,
};

#if MCP2517FD_TERM_SENSE_ON_INT0

/**
 * @brief Gets the termination resistor state
 *
 * @param term_state_out Pointer to write termination state if valid, writes true if termination is enabled, false if not
 * @return true The termination state is valid
 * @return false Could not read termination state
 */
static bool can_mcp251x_get_term_state(bool *term_state_out) {
    uint8_t pin_state;
    eERRORRESULT err = MCP251XFD_GetGPIOPinsInputLevel(&mcp251xfd_device, &pin_state);

    if (err == ERR_OK) {
        if (pin_state & MCP251XFD_GPIO0_HIGH) {
            *term_state_out = false;
        } else {
            *term_state_out = true;
        }

        return true;
    } else {
        return false;
    }
}

#else

static bool can_mcp251x_get_term_state(bool *term_state_out) {
    return false;
}

#endif

// ========================================
// Driver Exports
// ========================================

bool bl_interface_init(void)
{
    saved_client_id = 1;
    saved_channel = 1;

    //--- Compute Filter Values ---
    // Mask was set up in initialization (use configured client_id and channel, set ID to be from agent)
    mcp251xfd_utility_rx_filter.AcceptanceID = CANMORE_CALC_UTIL_ID_A2C(saved_client_id, saved_channel);

    //--- Initialize Int pins or GPIOs ---
    // Initialize CS Pin
    gpio_init(MCP2517FD_NCS_PIN);
    gpio_put(MCP2517FD_NCS_PIN, true);
    gpio_set_dir(MCP2517FD_NCS_PIN, GPIO_OUT);

    // Initialize SPI
    // Stealing SPI Clock Speed just for setup
    // This is because the initialization function is called several times, and all real setup is performed here
    spi_init(MCP2517FD_SPI_INST, mcp251xfd_device.SPIClockSpeed);

    gpio_set_function(MCP2517FD_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MCP2517FD_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MCP2517FD_MISO_PIN, GPIO_FUNC_SPI);

    // Setup Clock Output
    uint32_t clk_src_freq = clock_get_hz(MCP2517FD_OSC_SRC);
    hard_assert(clk_src_freq % MCP2517FD_OSC_RATE == 0);  // Ensure that clock divides evenly
    clock_gpio_init(MCP2517FD_CANCLK_PIN, MCP2517FD_CLK_GPOUT_AUXSRC, clk_src_freq / MCP2517FD_OSC_RATE);

    // Setup IRQ Pin
    gpio_init(MCP2517FD_INT_PIN);
    gpio_set_dir(MCP2517FD_INT_PIN, GPIO_IN);
    gpio_pull_up(MCP2517FD_INT_PIN);

    busy_wait_ms(10);   // Give time for clocks on device to stabalize

    //--- Configure Device ---
    eERRORRESULT error_code = ERR__NO_DEVICE_DETECTED;
    error_code = Init_MCP251XFD(&mcp251xfd_device, &mcp251xfd_device_config);
    if (error_code != ERR_OK) {
        return false;
    }

    // Configure FIFOs
    error_code = MCP251XFD_ConfigureFIFO(&mcp251xfd_device, &mcp251xfd_utility_tx_fifo_config);
    if (error_code != ERR_OK) {
        return false;
    }

    error_code = MCP251XFD_ConfigureFIFO(&mcp251xfd_device, &mcp251xfd_utility_rx_fifo_config);
    if (error_code != ERR_OK) {
        return false;
    }

    // Configure Filters
    error_code = MCP251XFD_ConfigureFilter(&mcp251xfd_device, &mcp251xfd_utility_rx_filter);
    if (error_code != ERR_OK) {
        return false;
    }

    // Start CAN
    #if CAN_BUS_ENABLE_FD
    error_code = MCP251XFD_StartCANFD(&mcp251xfd_device);
    #else
    error_code = MCP251XFD_StartCAN20(&mcp251xfd_device);
    #endif
    if (error_code != ERR_OK) {
        return false;
    }

    return true;
}

void can_mcp251x_send_heartbeat(int mode) {
    MCP251XFD_CANMessage msg;
    static canmore_titan_heartbeat_t heartbeat = {.data = 0};

    heartbeat.pkt.cnt += 1;
    heartbeat.pkt.error = 0;
    heartbeat.pkt.mode = mode;

    bool term_enabled;
    if (can_mcp251x_get_term_state(&term_enabled)) {
        heartbeat.pkt.term_valid = 1;
        heartbeat.pkt.term_enabled = (term_enabled ? 1 : 0);
    } else {
        heartbeat.pkt.term_valid = 0;
        heartbeat.pkt.term_enabled = 0;
    }

    msg.DLC = MCP251XFD_DLC_1BYTE;
    msg.MessageID = CANMORE_CALC_UTIL_ID_C2A(saved_client_id, CANMORE_CHAN_HEARTBEAT);
    msg.ControlFlags = MCP251XFD_CAN20_FRAME;
    msg.MessageSEQ = 0;
    msg.PayloadData = &heartbeat.data;

    MCP251XFD_TransmitMessageToFIFO(&mcp251xfd_device, &msg, mcp251xfd_utility_tx_fifo, true);
}

void bl_interface_heartbeat(void) {
    can_mcp251x_send_heartbeat(CANMORE_TITAN_HEARTBEAT_MODE_BOOTLOADER);
}

void bl_interface_notify_boot(void) {
    can_mcp251x_send_heartbeat(CANMORE_TITAN_HEARTBEAT_MODE_BOOT_DELAY);
}

bool bl_interface_try_receive(uint8_t *msg_out, size_t *len_out) {
    // First check if a message is pending
    if (gpio_get(MCP2517FD_INT_PIN)) {
        // INT high, no message pening
        return false;
    }

    uint32_t timestamp = 0;
    MCP251XFD_CANMessage msg;
    msg.PayloadData = msg_out;

    // that will be received
    eERRORRESULT error_code = MCP251XFD_ReceiveMessageFromFIFO(&mcp251xfd_device, &msg, UTILITY_MSG_PAYLOAD_SIZE_ENUM,
                                                    &timestamp, mcp251xfd_utility_rx_fifo);
    if (error_code != ERR_OK) {
        return false;
    }

    if ((msg.ControlFlags & MCP251XFD_EXTENDED_MESSAGE_ID) != 0) {
        // Utility messages can't have an extended ID
        return false;
    }
    bool is_canfd = (msg.ControlFlags & MCP251XFD_CANFD_FRAME) != 0;
    *len_out = MCP251XFD_DLCToByte(msg.DLC, is_canfd);

    return true;
}

void bl_interface_transmit(uint8_t *msg, size_t len) {
    // Don't allow messages > CAN frame size
    if (len > 8 || len < 1)
        return;

    // Don't need to worry about checking if the buffer is full, we have no handling, it'll just drop the message
    MCP251XFD_CANMessage can_msg;
    can_msg.DLC = len;
    can_msg.MessageID = CANMORE_CALC_UTIL_ID_C2A(saved_client_id, saved_channel);
    can_msg.ControlFlags = MCP251XFD_CAN20_FRAME;
    can_msg.MessageSEQ = 0;
    can_msg.PayloadData = msg;

    MCP251XFD_TransmitMessageToFIFO(&mcp251xfd_device, &can_msg, mcp251xfd_utility_tx_fifo, true);
}