#ifndef DSHOT_H
#define DSHOT_H

#include "hardware/pio.h"

#include <stdbool.h>
#include <stdint.h>

// ========================================
// Configuration
// ========================================

// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_DSHOT, Enable/disable assertions in the dshot module, type=bool, default=0, group=talos/ESCBoard
#ifndef PARAM_ASSERTIONS_ENABLED_DSHOT
#define PARAM_ASSERTIONS_ENABLED_DSHOT 0
#endif

// The number of thrusters controlled by the board - Max value of 4 (as this assumes all dshot SMs are on the same PIO)
#define NUM_THRUSTERS 4

// The minimum rate that new commands must be sent from ROS. If this rate isn't met, the thrusters will be disabled
#define DSHOT_MIN_UPDATE_RATE_MS 200

// The amount of time in milliseconds thrusters will be disabled for if DSHOT_MIN_UPDATE_RATE_MS isn't met. Prevents
// jittering from commands being sent slightly slower than the minimum update rate
#define DSHOT_UPDATE_DISABLE_TIME_MS 1000

// The delay in milliseconds that thrusters will be disabled for on ESC power-up
#define ESC_WAKEUP_DELAY_MS 5000

// The dshot rate in KHz, must be a valid dshot rate (DSHOT150, DSHOT300, DSHOT600, DSHOT1200)
#define DSHOT_RATE 600

// The conversion factor from the raw adc value to the vcc in millivolts
#define VCC_CONVERSION_MULT_MV 36300  // (3300 * (10000 + 1000))
#define VCC_CONVERSION_DIV_MV 4096    // (4096 * 1000)

// The voltage threshold upon which passing the ESCs will be considered powered
#define ESC_POWER_THRESHOLD_MV 9000

// Number of dshot packet sent between each telemetry request
// It will take dshot_packet_period * TELEM_PACKET_DELAY * NUM_THRUSTERS microseconds to refreh all ESC telemetry
// where dshot_packet_period is the time between each packet's transmission
#define TELEM_PACKET_DELAY 100

// Number of packets that can be missed before the telemetry packet is marked as invalid
#define TELEM_MAX_MISSED_PACKETS 10

// The PIO Block to reserve for DShot communication
#define DSHOT_PIO_BLOCK pio0

// The PIO block to reserve for uart telemetry from ESCs
#define DSHOT_TELEM_PIO_BLOCK pio1

// Minimum time per dshot tick. Sending it too fast will result in packets being dropped by the ESC
// This implies the maximum time that we'd expect a bidirectional dshot message to take, since we can't transmit while
// expecting the ESC might still respond, as this would cause contention
#define DSHOT_MIN_FRAME_TIME_US 1000

// Constant for converting electrical RPM to mechanical RPM
// Found on forum: https://discuss.bluerobotics.com/t/t200-thruster-questions-poles-max-voltage-e-bike-controller/2442/2
#define ESC_NUM_POLE_PAIRS 7

// ========================================
// Data Types
// ========================================

struct dshot_uart_telemetry {
    int missed_count;      // Missed packet counter (incremented every TX attempt, zeroed after successful rx)
    bool valid;            // If the packet is valid
    bool rpm_reversed;     // True if the RPM value is negative (guessed by software, as telemetry gives absolute value)
    uint8_t temperature;   // Temperature in C
    uint16_t voltage;      // Voltage in hundredths of volts
    uint16_t current;      // Current in hundredths of amps
    uint16_t consumption;  // Consumption of mAh
    uint16_t rpm;          // Electrical RPM in 100s of RPM
};

// ========================================
// Exported Variables - Do not write
// ========================================

/**
 * @brief Boolean if dshot_init has been called
 */
extern bool dshot_initialized;

/**
 * @brief VCC Voltage Reading in millivolts
 *
 * @note Safe to be read from either core.
 */
extern volatile uint32_t vcc_reading_mv;

/**
 * @brief Report if the ESC VCC rail is considered 'powered on' determined
 *
 * @note Safe to be read from either core.
 */
#define esc_board_on (vcc_reading_mv > ESC_POWER_THRESHOLD_MV)

/**
 * @brief Set to true if the ADC gets a measurement which considers the ESC board to be off.
 *
 * This prevents cases where the ESC board might have turned off, but the control loop didn't update in time to see it.
 * After the code acknowledges that the ESC board turned off, it can set this to false.
 *
 * @attention This should only be modified by *one* piece of code.
 * @attention Care should be taken for race conditions where this is set again between a read and write.
 *
 */
extern volatile bool esc_board_was_off;

/**
 * @brief Bool reporting if the ESCs are commanding a non-neutral value.
 * If this is true, the thrusters are considered to be on (used for error reporting timeouts)
 *
 * @note Safe to be read from either core.
 */
extern volatile bool dshot_thrusters_on;

// ========================================
// Exported Methods (Available to Core 0)
// ========================================

/**
 * @brief Attempts to retreive telemetry for the requested thruster number.
 *
 * @note This function is safe to be called from any context, including both cores and in interrupts.
 *
 * @param thruster_num The thruster number to get telemetry for (must be less than NUM_THRUSTERS)
 * @param telem_out Pointer to write telemetry data
 * @return true Telemetry was successfully copied into telem_out
 * @return false No valid telemetry data was available for that ESC
 */
bool dshot_get_telemetry(int thruster_num, struct dshot_uart_telemetry *telem_out);

// ========================================
// Core 1 Only Methods
// ========================================

/**
 * @brief Initialize dshot and starts outputting thruster neutral commands
 *
 * @attention Only call this on core 1 (running the controller)
 */
void dshot_init(void);

/**
 * @brief Updates thruster values to the specified values.
 * This will convert the value range -999 to 999 to the proper dshot command
 *
 * @attention Only call this on core 1 (running the controller)
 * @attention DShot must be initialized before calling this function
 * @attention You must wait dshot_min_frame_time between calls of this function
 *
 * @param throttle_values A NUM_THRUSTERS long array of throttle values in range (-999 to 999) [one for each thruster]
 */
void dshot_update_thrusters(const int16_t *throttle_values);

/**
 * @brief Reports if an RPM measurement is available for the requested thruster
 *
 * @attention Only call this on core 1 (running the controller)
 * @param thruster The thruster number
 * @return true if dshot_rpm_read can return RPM without blocking
 */
static inline bool dshot_rpm_available(unsigned int thruster) {
    return pio_sm_get_rx_fifo_level(DSHOT_PIO_BLOCK, thruster) > 0;
}

/**
 * @brief Attempts to read the mechanical RPM for the requested thruster.
 * RPM will be sent back after each dshot update thruster call, unless the ESC is busy or off.
 * If the RPM was corrupted during transit, this function will return false and not modify rpm-out
 *
 * @attention Only call this on core 1 (running the controller)
 * @note This function will block until data is read. Use dshot_rpm_available to check if data is pending
 *
 * @param thruster The thruster number
 * @param rpm_out Pointer to store computed RPM on successful decode
 * @return true RPM successfully decoded, stored into rpm_out
 * @return false RPM failed to decode, rpm_out is preserved
 */
bool dshot_get_rpm(unsigned int thruster, int *rpm_out);

#endif  // DSHOT_H
