#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "micro_ros_pico/custom_allocator.h"
#include "micro_ros_pico/transport_usb.h"

#include "build_version.h"

#include <uxr/client/client.h>
#include <uxr/client/util/ping.h>

absolute_time_t total_latency_start;

const uint LED_PIN = 25;

rcl_timer_t timer;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

#define RCRETCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on in " __FILE__ ":%d : %d. Aborting.\n",__LINE__,(int)temp_rc); return false;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on in " __FILE__ ":%d : %d. Continuing.\n",__LINE__,(int)temp_rc);}}

extern int timer_task_count;

bool disconnected = false;
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    total_latency_start = get_absolute_time();
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    if (ret != RCL_RET_OK) {
        //RCSOFTCHECK(ret);
        disconnected = true;
    }
    msg.data++;
}

void ros_ping_until_connect(void) {
    // Why does RMW need to be initialized? All the other examples don't...
    // Well you see, the only way to clear the RMW context is to re-initialize it, so if the previous session
    // got cleared, rmw still has some static memory allocated. Calling rmw_init will clear it, but rmw_uros_ping_agent
    // won't. So rmw_uros_ping_agent will try to use the old session and fail to ping the agent
    // Of course calling fini will fix this, but then it would also try to communicate with the agent during fini,
    // and wait for several timeouts which could trigger the watchdog... So instead we just nuke and re-initialize
    // Fingers crossed there's no memory leaks

    /*extern char session_memory;
    extern char custom_sessions[];
    extern void rmw_uxrce_init_session_memory(void*, void*, size_t);
    rmw_uxrce_init_session_memory(&session_memory, custom_sessions, RMW_UXRCE_MAX_SESSIONS);*/

    /*extern rmw_uxrce_transport_params_t rmw_uxrce_transport_default_params;

    uxrCustomTransport transport;
    transport.framing = rmw_uxrce_transport_default_params.framing;
    transport.args = rmw_uxrce_transport_default_params.args;
    transport.open = rmw_uxrce_transport_default_params.open_cb;
    transport.close = rmw_uxrce_transport_default_params.close_cb;
    transport.write = rmw_uxrce_transport_default_params.write_cb;
    transport.read = rmw_uxrce_transport_default_params.read_cb;
    extern rmw_ret_t rmw_uxrce_transport_init(
        void * context_impl,
        void * init_options_impl,
        void * override_transport);

    RCSOFTCHECK(rmw_uxrce_transport_init(NULL, NULL, (void *)&transport));

    bool success;
    do {
        success = uxr_ping_agent_attempts(&transport.comm, 1000, 1);
    } while (!success);
    uxr_close_custom_transport(&transport);*/

    rcl_ret_t ret;
    do {
        ret = rmw_uros_ping_agent(1000, 1);
    } while (ret != RCL_RET_OK);
}

bool ros_init(void) {
    allocator = rcl_get_default_allocator();
    RCRETCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    RCRETCHECK(rclc_node_init_default(&node, "pico_node", "", &support));
    sleep_ms(3000);
    RCRETCHECK(rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "pico_publisher"));

    RCRETCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback));

    RCRETCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCRETCHECK(rclc_executor_add_timer(&executor, &timer));

    return true;
}

const char * const entity_lookup_table[] = {
    "RMW_UROS_ERROR_ON_UNKNOWN",
    "RMW_UROS_ERROR_ON_NODE",
    "RMW_UROS_ERROR_ON_SERVICE",
    "RMW_UROS_ERROR_ON_CLIENT",
    "RMW_UROS_ERROR_ON_SUBSCRIPTION",
    "RMW_UROS_ERROR_ON_PUBLISHER",
    "RMW_UROS_ERROR_ON_GRAPH",
    "RMW_UROS_ERROR_ON_GUARD_CONDITION",
    "RMW_UROS_ERROR_ON_TOPIC",
};
const char * const source_lookup_table[] = {
    "RMW_UROS_ERROR_ENTITY_CREATION",
    "RMW_UROS_ERROR_ENTITY_DESTRUCTION",
    "RMW_UROS_ERROR_CHECK",
    "RMW_UROS_ERROR_NOT_IMPLEMENTED",
    "RMW_UROS_ERROR_MIDDLEWARE_ALLOCATION",
};

#define lookup_string_enum(value, list) ((value < sizeof(list)/sizeof(*list)) && (value >= 0) ? list[value] : "Out-of-Bounds")
#define lookup_entity_enum(value) lookup_string_enum(value, entity_lookup_table)
#define lookup_source_enum(value) lookup_string_enum(value, source_lookup_table)

void my_rmw_error_cb(
  const rmw_uros_error_entity_type_t entity,
  const rmw_uros_error_source_t source,
  const rmw_uros_error_context_t context,
  const char * file,
  const int line) {
    printf("RMW UROS Error:\n\tEntity: %s\n\tSource: %s\n\tDesc: %s\n\tLocation: %s:%d\n", lookup_entity_enum(entity), lookup_source_enum(source), context.description, file, line);
}

int main()
{
    transport_usb_serial_init_early();
    printf("%s\n", FULL_BUILD_TAG);
    sleep_ms(1000);

    //custom_allocator_set_default();
    transport_usb_init();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    total_latency_start = get_absolute_time();

    rmw_uros_set_error_handling_callback(my_rmw_error_cb);

    while (true) {
        gpio_put(LED_PIN, 0);
        sleep_ms(500);

        printf("\n\nTotal latency: %lld ms\n", absolute_time_diff_us(total_latency_start, get_absolute_time())/1000);
        //printf("Connecting...\n");

        ros_ping_until_connect();

        //printf("Connected to ROS\n");

        gpio_put(LED_PIN, 1);

        if (ros_init()) {

            disconnected = false;
            msg.data = 0;
            while (!disconnected)
            {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            }
        } else {
            printf("Failed to init!\n");
        }

        //printf("Cleaning up ROS\n");

        absolute_time_t start = get_absolute_time();
        RCSOFTCHECK(rcl_publisher_fini(&publisher, &node));
        RCSOFTCHECK(rcl_timer_fini(&timer));
        RCSOFTCHECK(rclc_executor_fini(&executor));
        RCSOFTCHECK(rcl_node_fini(&node));
        RCSOFTCHECK(rclc_support_fini(&support));
        //printf("Time to stop publisher: %lld us\n", absolute_time_diff_us(start, get_absolute_time()));

    }

    return 0;
}
