#ifndef DYNAMIXEL_SCHEDULER_H
#define DYNAMIXEL_SCHEDULER_H

#include "dxl_packet.h"
#include "dynamixel_comms.h"

#include "driver/dynamixel.h"

struct dynamixel_state {
    dynamixel_id id;
    struct dynamixel_eeprom eeprom;
    struct dynamixel_ram ram;
    uint8_t missed_ping_counter;
    volatile bool connected;
    bool alert_notified;
    bool first_connect_attempted;
};

/**
 * @brief Initializes the dynamixel scheduler
 *
 * @param id_list Array of servos to monitor
 * @param id_cnt Count of servos to monitor
 * @param error_cb Callback to report an error (events which cannot be gracefully handled by the dynamixel driver)
 * @param event_cb Callback to report any scheduler events
 */
void dynamixel_schedule_init(const dynamixel_id *id_list, size_t id_cnt, dynamixel_error_cb error_cb,
                             dynamixel_event_cb event_cb);

/**
 * @brief Schedules a dynamixel write packet
 *
 * @attention Not safe to be called from interrupts
 *
 * @param id ID to write to
 * @param start_address The starting address for the data array to write
 * @param data Array of data to write
 * @param data_len Length of data ot write
 */
void dynamixel_schedule_write_packet(dynamixel_id id, uint16_t start_address, uint8_t *data, size_t data_len);

/**
 * @brief Schedules a full EEPROM read command and updates EEPROM upon successful completion.
 *
 * @param id ID to read
 */
void dynamixel_schedule_eeprom_read(dynamixel_id id);

/**
 * @brief Gets the requested IDs state
 *
 * @param id The ID to look up
 * @return struct dynamixel_state* Pointer to the dynamixel state, or NULL if the ID is not tracked
 */
struct dynamixel_state *dynamixel_schedule_get_state_ptr(dynamixel_id id);

/**
 * @brief Returns pointer to the dynamixel state array, and writes the number of servos in the array.
 *
 * @param num_servos_out Pointer to write the number of servos in the state array
 * @return struct dynamixel_state* Pointer to the state array, or NULL if no servos are available
 */
struct dynamixel_state *dynamixel_schedule_get_state_array(size_t *num_servos_out);

// ========================================
// Low Level Queueing
// ========================================
// These functions are to queue custom packets into the buffer

/**
 * @brief Function to perform all the common steps after a transfer is done and ready to release the trasnfer lock.
 *
 * @attention This function should only be called at the end of your callback function for
 *
 * @attention This function must be called with transfer lock held dynamixel_schedule_raw_packet
 */
void dynamixel_schedule_next_transfer_or_release(void);

/**
 * @brief Schedule a raw dynamixel packet to be sent.
 *
 * @note This should only be used for low-level debug systems (like canmore commands). This directly calls the callback
 * function, and requires that the callback handle all error conditions. If you are writing code to interact with the
 * dynamixel, you should instead use the functions defined above, which will properly call the error_cb set during
 * initialization. Only use this if you know what you're doing!
 *
 * @attention The callback MUST call dynamixel_schedule_next_transfer_or_release() at the end of the callback. Failure
 * to do so will stall the dynamixel driver, preventing any future transfers from firing.
 *
 * @param packet The dynamixel packet info to transmit (created by dynamixel create packet calls)
 * @param callback The callback to fire when the transfer completes (called on both success and failure)
 * @return true The packet was successfully scheduled
 * @return false The packet failed to schedule (packet buffer was larger than DYNAMIXEL_PACKET_BUFFER_SIZE, or the queue
 * was full)
 */
bool dynamixel_schedule_raw_packet(InfoToMakeDXLPacket_t *packet, dynamixel_request_cb callback);

#endif
