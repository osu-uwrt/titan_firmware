#ifdef MICRO_ROS_TRANSPORT_USB
#include "micro_ros_pico/transport_usb.h"
#endif

#ifdef MICRO_ROS_TRANSPORT_CAN
#include "micro_ros_pico/transport_can.h"
#endif

#include "basic_logger/logging.h"
#include "build_version.h"

#include "ros.h"
#include "safety_interface.h"

#undef LOGGING_UNIT_NAME
#define LOGGING_UNIT_NAME ROBOT_NAMESPACE "_main"
#define HEARTBEAT_TIME_MS 80

absolute_time_t next_heartbeat;

#define PROCESS_TIMER(next_time, timer_interval, callback) do { \
        if (time_reached(next_time)) { \
            delayed_by_us(next_time, timer_interval); \
            if (time_reached(next_time)) { \
                unsigned int i = 0; \
                while(time_reached(next_time)) { \
                    delayed_by_us(next_time, timer_interval); \
                    i++; \
                } \
                LOG_WARN("Missed %u Runs of Timer " # callback, i); \
            } \
            if (callback() != RCL_RET_OK){ \
                return; \
            } \
        } \
    } while(0)

void tick_ros_timers() { 
    PROCESS_TIMER(next_heartbeat, HEARTBEAT_TIME_MS, ros_heartbeat_pulse);
}

void tick_background_timers() { 

}


int main() { 
    #ifdef MICRO_ROS_TRANSPORT_USB
    transport_usb_serial_init_early();
    #endif 
    
    #ifdef MICRO_ROS_TRANSPORT_CAN
    uint can_id = 0; 
    transport_can_init(can_id);
    #endif

    LOG_INFO("%s", FULL_BUILD_TAG);

    safety_setup();
    ros_rmw_init();

    #ifdef MICRO_ROS_TRANSPORT_USB
    transport_usb_init();
    #endif

    bool ros_initialized = false; 

    while(true) { 
        if(is_ros_connected()) { 
            if(!ros_initialized) { 
                LOG_DEBUG("ROS connected");
                ros_initialized = ros_init();

                if(ros_initialized) { 
                    safety_init();
                    next_heartbeat = make_timeout_time_ms(HEARTBEAT_TIME_MS);
                } else {
                    LOG_ERROR("ROS failed to initialize.");
                    ros_fini();
                }
            } else { 
                ros_update();
                tick_ros_timers();
            }
        } else if(ros_initialized){ 
            LOG_DEBUG("Lost connection to ROS")
            safety_deinit();
            ros_fini();
            ros_initialized = false;
        } else {
            ros_ping();
        }

        safety_tick();
        tick_background_timers();
    }

    return 0;
}