#include <stdlib.h>
#include <string.h>
#include "pico/sync.h"
#include "pico/time.h"

#include "basic_queue/queue.h"
#include "dynamixel/dynamixel.h"

#include "dynamixel_schedule.h"
#include "dynamixel_comms.h"
#include "dynamixel_controls.h"
#include "dynamixel_reg.h"

// TODO: Figure out logging
#include "basic_logger/logging.h"
#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME "dynamixel_schedule"

// PICO_CONFIG: DYNAMIXEL_MAX_CMDS, Size of dynamixel command queue, min=1, default=8, group=dynamixel
#ifndef DYNAMIXEL_MAX_CMDS
#define DYNAMIXEL_MAX_CMDS 8
#endif

// PICO_CONFIG: DYNAMIXEL_REFRESH_INTERVAL_MS, Interval of dynamixel ping/ram read requests in milliseconds, min=1, default=200, group=dynamixel
#ifndef DYNAMIXEL_REFRESH_INTERVAL_MS
#define DYNAMIXEL_REFRESH_INTERVAL_MS 200
#endif

// PICO_CONFIG: DYNAMIXEL_MAX_MISSED_PINGS, Max number of missed pings before dynamixel is marked as disconnected, min=1, default=3, group=dynamixel
#ifndef DYNAMIXEL_MAX_MISSED_PINGS
#define DYNAMIXEL_MAX_MISSED_PINGS 3
#endif

#define dynamixel_report_error_with_arg(error_code, arg) do { \
        dynamixel_error_t error_msg = {.fields = {.error = error_code, .error_source = DYNAMIXEL_SOURCE_SCHEDULE, \
                                        .line = __LINE__, .wrapped_error_code = arg}};    \
        inst->error_cb(error_msg); \
    } while(0)

#define dynamixel_report_error(error_code) dynamixel_report_error_with_arg(error_code, 0)

#define retcheck_refresh_dxl_call(ret_code) do {\
        enum DXLLibErrorCode ret = ret_code; \
        if (ret != DXL_LIB_OK) { \
            LOG_DEBUG("Error calling dxl function during refresh: %d (line %d)", ret, __LINE__); \
            dynamixel_report_error_with_arg(DYNAMIXEL_DRIVER_ERROR, ret); \
            handle_refresh_transfer_done(); \
            return; \
        } \
    } while(0)

struct internal_cmd {
    InfoToMakeDXLPacket_t packet;
    uint8_t packet_buf[DYNAMIXEL_PACKET_BUFFER_SIZE];
};

struct dynamixel_scheduler_instance {
    dynamixel_error_cb error_cb;
    dynamixel_event_cb event_cb;

    // Dynamixel Tracking
    struct dynamixel_state *servo_states;
    size_t servo_count;

    // Packet Storage
    struct QUEUE_DEFINE(struct internal_cmd, DYNAMIXEL_MAX_CMDS) cmd_queue;
    struct internal_cmd refresh_cmd;

    // State Tracking
    volatile bool transfer_active;
    volatile bool refresh_active;
    volatile bool refresh_pending;

    // Refresh Control
    repeating_timer_t refresh_timer;
    size_t next_index_to_refresh;
} dynamixel_inst = {0};

// Define global instance pointer (so we don't have to dereference all that much)
static struct dynamixel_scheduler_instance *const inst = &dynamixel_inst;

// Static Function Declarations

static bool refresh_timer_cb(repeating_timer_t *timer);
static void trigger_next_refresh(void);
static void handle_refresh_transfer_done(void);
static void periodic_ram_read_cb(dynamixel_error_t err, struct dynamixel_req_result *result);
static void connect_ping_cb(dynamixel_error_t err, struct dynamixel_req_result *result);
static void initial_eeprom_read_cb(dynamixel_error_t err, struct dynamixel_req_result *result);
static void initial_ram_read_cb(dynamixel_error_t err, struct dynamixel_req_result *result);
static void initial_torque_disable_cb(dynamixel_error_t err, struct dynamixel_req_result *result);

static void process_next_transfer_or_release(void);
static void queued_cmd_cb(dynamixel_error_t err, struct dynamixel_req_result *result);

// ========================================
// Synchronization Primitives
// ========================================

/**
 * @brief Attempts to get the transfer lock
 *
 * @return true Successfully locked transfer lock
 * @return false Failed to get lock (already held by someone else)
 */
static inline bool try_get_transfer_lock(void) {
    bool got_lock = false;

    uint32_t prev_interrupts = save_and_disable_interrupts();
    if (!inst->transfer_active) {
        inst->transfer_active = true;
        got_lock = true;
    }
    restore_interrupts(prev_interrupts);

    return got_lock;
}

/**
 * @brief Releases your hold on the transfer lock
 *
 * @attention This function must be called with transfer lock held
 */
static inline void release_transfer_lock(void) {
    assert(inst->transfer_active);  // If this fails, then multiple functions thought they had a lock and then two tried to release
    inst->transfer_active = false;
}

// ========================================
// Dynamixel Initialization
// ========================================

void dynamixel_schedule_init(const dynamixel_id *id_list, size_t id_cnt,
                             dynamixel_error_cb _error_cb,
                             dynamixel_event_cb _event_cb) {
    // Save callbacks
    inst->error_cb = _error_cb;
    inst->event_cb = _event_cb;

    // Initialize the servo states
    inst->servo_states = malloc(sizeof(*inst->servo_states) * id_cnt);
    inst->servo_count = id_cnt;
    for (size_t i = 0; i < id_cnt; i++) {
        inst->servo_states[i].id = id_list[i];
        inst->servo_states[i].connected = false;
        inst->servo_states[i].missed_ping_counter = 0;
    }

    // Begin the background refresh
    inst->transfer_active = true;   // We can just set the lock, as no other code should be acessing inst right now
    inst->refresh_active = true;
    inst->next_index_to_refresh = 0;

    // Add the timer first, then we manually trigger the refresh (as the timer won't fire until at least one interval elapses)
    // Note timer duration is multiplied by -1 for a start->start delay rather than end->start delay
    hard_assert(add_repeating_timer_ms(-DYNAMIXEL_REFRESH_INTERVAL_MS, refresh_timer_cb, NULL, &inst->refresh_timer));
    trigger_next_refresh();
}

// ========================================
// Dynamixel Refreshing
// ========================================

/*
 * Perform Ping Under Lock Theory of Operation:
 *  - If servo index is >= num servos
 *    - If transfer queue not empty
 *      - Call begin next transfer under lock
 *    - Else release lock and return
 *  - If servo index is valid, perform on the following servo
 *    - If connected, perform RAM read
 *     - If RAM read successful, report ram read, increase index, call ping common
 *
 *    - If not connected, perform ping
 *     - If ping successful, perform eeprom read
 *      - If eeprom read successful, read RAM
 *        - If RAM read successful, send disable torque command
 *          - If disable torque successful, report new servo, incrase index, call ping common
 *          - If disable torque unsuccessful, incrase index, call ping common
 *        - If RAM read unsuccessful, incrase index, call ping common
 *      - If eeprom read unsuccessful, incrase index, call ping common
 *     - If ping unsuccessful, incrase index, call ping common
 */

/**
 * @brief Repeating timer callback
 *
 * @param timer Timer passed to callback
 * @return true Reschedules alarm
 * @return false Cancels repeating alarm
 */
static bool refresh_timer_cb(__unused repeating_timer_t *timer) {
    if (inst->refresh_active || inst->refresh_pending) {
        LOG_DEBUG("Driver Error: Refresh timer fired before previous refresh completed");
        dynamixel_report_error(DYNAMIXEL_DRIVER_ERROR);
        return true;
    }

    inst->next_index_to_refresh = 0;

    if (try_get_transfer_lock()) {
        inst->refresh_active = true;
        trigger_next_refresh();
    }
    else {
        inst->refresh_pending = true;
    }

    return true;
}

/**
 * @brief Sends a refresh packet for the next servo in the list to refresh determined by `next_index_to_refresh`
 *
 * @attention This function must be called with transfer lock held
 * @attention This function must be called with the refresh active lock held
 */
static void trigger_next_refresh(void) {
    assert(inst->transfer_active);
    assert(inst->refresh_active);

    if (inst->next_index_to_refresh >= inst->servo_count) {
        inst->refresh_active = false;
        process_next_transfer_or_release();
        return;
    }

    struct dynamixel_state *servo = &inst->servo_states[inst->next_index_to_refresh];
    if (servo->connected) {
        retcheck_refresh_dxl_call(dynamixel_reg_ram_gen_request(&inst->refresh_cmd.packet, inst->refresh_cmd.packet_buf, servo->id));
        dynamixel_send_packet(periodic_ram_read_cb, &inst->refresh_cmd.packet);
    }
    else {
        retcheck_refresh_dxl_call(dynamixel_create_ping_packet(&inst->refresh_cmd.packet, inst->refresh_cmd.packet_buf, servo->id));
        dynamixel_send_packet(connect_ping_cb, &inst->refresh_cmd.packet);
    }
}

static void handle_refresh_transfer_done(void) {
    assert(inst->transfer_active);
    assert(inst->refresh_active);

    // Send disconnect if first ping did not work
    struct dynamixel_state *servo = &inst->servo_states[inst->next_index_to_refresh];
    if (!servo->first_connect_attempted) {
        servo->first_connect_attempted = true;
        if (!servo->connected) {
            inst->event_cb(DYNAMIXEL_EVENT_DISCONNECTED, servo->id);
        }
    }

    // Go to next device in list
    inst->next_index_to_refresh++;
    trigger_next_refresh();
}

static bool check_refresh_response_valid(dynamixel_error_t err, struct dynamixel_req_result *result) {
    struct dynamixel_state *servo = &inst->servo_states[inst->next_index_to_refresh];

    // Check that a driver error wasn't reported
    if (err.fields.error != DYNAMIXEL_ERROR_NONE) {
        LOG_DEBUG("[ID: %d, INSTR: %d] Error during refresh: 0x%08lx", servo->id, result->instr, err.data);
        return false;
    }

    // Make sure the device we want is the one that responded
    assert(servo->id == result->request_id);    // Ensure the state didn't get desynchronized
    if (result->packet->id != servo->id) {
        LOG_DEBUG("[ID: %d, INSTR: %d] Unexpected device %d responded", servo->id, result->instr, result->packet->id);
        return false;
    }

    // Check that the packet didn't report an error (alert bit is separate)
    if (result->packet->err_idx & 0x7F) {
        LOG_DEBUG("[ID: %d, INSTR: %d] Error field set in response: %d", servo->id, result->instr, result->packet->err_idx);
        return false;
    }

    return true;
}

static void periodic_ram_read_cb(dynamixel_error_t err, struct dynamixel_req_result *result) {
    struct dynamixel_state *servo = &inst->servo_states[inst->next_index_to_refresh];

    if (!check_refresh_response_valid(err, result)) {
        uint8_t missed = ++servo->missed_ping_counter;
        if (missed >= DYNAMIXEL_MAX_MISSED_PINGS) {
            servo->connected = false;
            inst->event_cb(DYNAMIXEL_EVENT_DISCONNECTED, servo->id);
            LOG_DEBUG("Dynamixel Disconnected - ID: %d", servo->id);
        }
        handle_refresh_transfer_done();
        return;
    }

    servo->missed_ping_counter = 0;
    dynamixel_reg_ram_decode(result->packet, &servo->ram);
    inst->event_cb(DYNAMIXEL_EVENT_RAM_READ, servo->id);

    // Report if alert bit is set
    if (result->packet->err_idx & 0x80) {
        if (!servo->alert_notified) {
            inst->event_cb(DYNAMIXEL_EVENT_ALERT, servo->id);
            servo->alert_notified = true;
        }
    }
    else {
        servo->alert_notified = false;
    }

    handle_refresh_transfer_done();
    return;
}

static void connect_ping_cb(dynamixel_error_t err, struct dynamixel_req_result *result) {
    if (!check_refresh_response_valid(err, result)) {
        handle_refresh_transfer_done();
        return;
    }

    struct dynamixel_state *servo = &inst->servo_states[inst->next_index_to_refresh];

    // Make sure the model matches what we expect
    uint16_t discovered_model_num = result->packet->p_param_buf[0] | (result->packet->p_param_buf[0] << 8);
    if (discovered_model_num != DYNAMIXEL_MODEL_NUMBER) {
        LOG_DEBUG("Invalid Model: %d, expected " __XSTRING(DYNAMIXEL_MODEL_NUMBER), discovered_model_num);
        handle_refresh_transfer_done();
        return;
    }

    // Now that we've found the servo, try reading its state
    // First read eeprom
    retcheck_refresh_dxl_call(dynamixel_reg_eeprom_gen_request(&inst->refresh_cmd.packet, inst->refresh_cmd.packet_buf, servo->id));
    dynamixel_send_packet(initial_eeprom_read_cb, &inst->refresh_cmd.packet);
}

static void initial_eeprom_read_cb(dynamixel_error_t err, struct dynamixel_req_result *result) {
    if (!check_refresh_response_valid(err, result)) {
        handle_refresh_transfer_done();
        return;
    }

    struct dynamixel_state *servo = &inst->servo_states[inst->next_index_to_refresh];

    // Save the EEPROM response
    dynamixel_reg_eeprom_decode(result->packet, &servo->eeprom);

    // Now read the RAM
    retcheck_refresh_dxl_call(dynamixel_reg_eeprom_gen_request(&inst->refresh_cmd.packet, inst->refresh_cmd.packet_buf, servo->id));
    dynamixel_send_packet(initial_ram_read_cb, &inst->refresh_cmd.packet);
}

static void initial_ram_read_cb(dynamixel_error_t err, struct dynamixel_req_result *result) {
    if (!check_refresh_response_valid(err, result)) {
        handle_refresh_transfer_done();
        return;
    }

    struct dynamixel_state *servo = &inst->servo_states[inst->next_index_to_refresh];

    // Save the RAM response
    dynamixel_reg_ram_decode(result->packet, &servo->ram);

    // Disable torque on connect (in case a disconnect, but not power cycle, occurred)
    uint8_t torque_enable = 0;
    retcheck_refresh_dxl_call(dynamixel_create_write_packet(&inst->refresh_cmd.packet, inst->refresh_cmd.packet_buf,
                                    servo->id, DYNAMIXEL_CTRL_TABLE_TORQUE_ENABLE_ADDR, &torque_enable, 1));
    dynamixel_send_packet(initial_torque_disable_cb, &inst->refresh_cmd.packet);
}

static void initial_torque_disable_cb(dynamixel_error_t err, struct dynamixel_req_result *result) {
    if (!check_refresh_response_valid(err, result)) {
        handle_refresh_transfer_done();
        return;
    }

    struct dynamixel_state *servo = &inst->servo_states[inst->next_index_to_refresh];

    // We have finished connecting to the new device
    // Set up the state and ping the next thing
    servo->missed_ping_counter = 0;
    servo->connected = true;
    inst->event_cb(DYNAMIXEL_EVENT_CONNECTED, servo->id);
    LOG_DEBUG("New Dynamixel Connected - ID: %d", servo->id);
    handle_refresh_transfer_done();
}

// ========================================
// Command Queue
// ========================================

/**
 * @brief Utility function to perform all the common steps after a transfer is done and ready to release the trasnfer
 * lock.
 *
 * @attention This function must be called with transfer lock held
 */
static void process_next_transfer_or_release() {
    assert(inst->transfer_active);

    if (inst->refresh_pending) {
        inst->refresh_active = true;
        inst->refresh_pending = false;
        trigger_next_refresh();
    }
    else if (!QUEUE_EMPTY(&inst->cmd_queue)) {
        struct internal_cmd *next_cmd = QUEUE_CUR_READ_ENTRY(&inst->cmd_queue);
        dynamixel_send_packet(queued_cmd_cb, &next_cmd->packet);
        QUEUE_MARK_READ_DONE(&inst->cmd_queue);
    }
    else {
        release_transfer_lock();
    }
}

/**
 * @brief Callback for dynamixel_send_packet for all queued transfers.
 * The primary purpose of this callback is to report any errors out from this library, as queued transfers
 * shouldn't have any feedback (they are assummed to succeed unless error_cb is fired, and rely on the periodic feedback
 * from the dynamixel refresh to ensure that the desired behavior occurs)
 *
 * @param ec Error code
 * @param result The transfer result,
 */
static void queued_cmd_cb(dynamixel_error_t err, struct dynamixel_req_result *result) {
    // Check that a driver error wasn't reported
    if (err.fields.error != DYNAMIXEL_ERROR_NONE) {
        LOG_DEBUG("Error executing queued command: 0x%08lx", err.data)
        inst->error_cb(err);
    }

    // Check that the packet didn't report an error
    if (result->packet->err_idx & 0x7F) {
        LOG_DEBUG("Error response set in queued command: %d", result->packet->err_idx);
        dynamixel_report_error_with_arg(DYNAMIXEL_HARDWARE_ERROR, result->packet->err_idx);
    }

    if (result->request_id != result->packet->id) {
        LOG_DEBUG("Mismatched ID: %d expected, %d received", result->request_id, result->packet->id);
        dynamixel_report_error_with_arg(DYNAMIXEL_INVALID_ID, result->packet->id);
    }

    process_next_transfer_or_release();
}

void dynamixel_schedule_write_packet(dynamixel_id id, uint16_t start_address, uint8_t *data, size_t data_len) {
    // Not safe to be called from interrupts
    // TODO: Move this to a parameter assertion linked with safety
    // This is required as QUEUE_CUR_WRITE_ENTRY is not atmoic, and an interrupt firing during dynamixel_create_write_packet
    // which also calls write packet will corrupt the buffer state
    assert(__get_current_exception() == 0);

    if (QUEUE_FULL(&inst->cmd_queue)) {
        LOG_DEBUG("Failed to queue write command into full queue");
        dynamixel_report_error(DYNAMIXEL_CMD_QUEUE_FULL_ERROR);
        return;
    }

    struct internal_cmd *entry = QUEUE_CUR_WRITE_ENTRY(&inst->cmd_queue);
    enum DXLLibErrorCode ret = dynamixel_create_write_packet(&entry->packet, entry->packet_buf, id, start_address, data, data_len);

    // Make sure the packet was created ok
    if (ret != DXL_LIB_OK) {
        LOG_DEBUG("Failed to create write packet: %d", ret);
        dynamixel_report_error_with_arg(DYNAMIXEL_REQUEST_ERROR, ret);
        return;
    }

    QUEUE_MARK_WRITE_DONE(&inst->cmd_queue);

    // If we can get the lock, begin the transfer
    // If we couldn't then whoever has it will queue us when its ready
    if (try_get_transfer_lock()) {
        process_next_transfer_or_release();
    }
}

struct dynamixel_state* dynamixel_schedule_get_state_ptr(dynamixel_id id) {
    for (size_t i = 0; i < inst->servo_count; i++) {
        if (id == inst->servo_states[i].id) {
            return &inst->servo_states[i];
        }
    }

    return NULL;
}
