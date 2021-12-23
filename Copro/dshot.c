#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"

#include "safety.h"
#include "dshot.pio.h"

// Thruster lookup macros
// Thruster id is a value 1-8
#define THRUSTER_PIO(thruster_id) (thruster_id > 4 ? pio1 : pio0)
#define THRUSTER_SM(thruster_id) (((thruster_id-1) % 4))
#define THRUSTER_PRGM_OFFSET(thruster_id) (thruster_id > 4 ? offset_pio1 : offset_pio0)

bool dshot_initialized = false;

/**
 * @brief Sends the dshot command with the specified parameters
 * 
 * INITIALIZATION REQUIRED
 * 
 * @param thruster_id Thruster ID 1-8
 * @param pwm_value   
 * @param request_telemetry 
 * @param send_once 
 */
static inline void dshot_send_internal(uint8_t thruster_id, uint16_t throttle_value, bool request_telemetry, bool send_once) {
    uint32_t cmd = (throttle_value & 0x7FF) << 1;

    if (request_telemetry) {
        cmd |= 1;
    }

    uint32_t crc = (cmd ^ (cmd >> 4) ^ (cmd >> 8)) & 0x0F;
    cmd <<= 4;
    cmd |= crc;

    // PIO reads from MSB first, so it needs to have the top 16 bits be the data to send
    cmd <<= 16;
    if (!send_once) {
        cmd |= (1 << 15);
    }

    pio_sm_put_blocking(THRUSTER_PIO(thruster_id), THRUSTER_SM(thruster_id), cmd);
}


void dshot_stop_thrusters(void) {
    // This command needs to be able to be called from kill switch callbacks
    // So this can be called at any point, so just ignore call if dshot is not initialized yet
    if (dshot_initialized) {
        for (int i = 1; i <= 8; i++){
            dshot_send_internal(i, 0, false, false);
        }
    }
}


#define init_thruster_pio(thruster_id) dshot_program_init(THRUSTER_PIO(thruster_id), THRUSTER_SM(thruster_id), THRUSTER_PRGM_OFFSET(thruster_id), \
                                                          clock_get_hz(clk_sys) / DSHOT_RATE(300), THRUSTER_##thruster_id##_PIN)
void dshot_init(void) {
    uint offset_pio0 = pio_add_program(pio0, &dshot_program);
    uint offset_pio1 = pio_add_program(pio1, &dshot_program);

    init_thruster_pio(1);
    init_thruster_pio(2);
    init_thruster_pio(3);
    init_thruster_pio(4);
    init_thruster_pio(5);
    init_thruster_pio(6);
    init_thruster_pio(7);
    init_thruster_pio(8);
    
    dshot_initialized = true;

    dshot_stop_thrusters();
}