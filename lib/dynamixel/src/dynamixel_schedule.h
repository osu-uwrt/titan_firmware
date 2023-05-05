#ifndef DYNAMIXEL_SCHEDULER_H
#define DYNAMIXEL_SCHEDULER_H

#include "dynamixel/dynamixel.h"

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
void dynamixel_schedule_init(const dynamixel_id *id_list, size_t id_cnt,
                             dynamixel_error_cb error_cb,
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
 * @brief Gets the requested IDs state
 *
 * @param id The ID to look up
 * @return struct dynamixel_state* Pointer to the dynamixel state, or NULL if the ID is not tracked
 */
struct dynamixel_state* dynamixel_schedule_get_state_ptr(dynamixel_id id);

#endif