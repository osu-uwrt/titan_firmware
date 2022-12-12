#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <riptide_msgs2/msg/pwm_stamped.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_can_transport.h"

#include "build_version.h"

const uint LED_PIN = 25;

rcl_publisher_t publisher;
riptide_msgs2__msg__PwmStamped msg;

extern int timer_task_count;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    //msg.data++;
    msg.pwm[0]++;
}

int main()
{
    stdio_init_all();
    printf("%s\n", FULL_BUILD_TAG);

    pico_can_transport_init(0, 1000000, 5, 4, 5, -1);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000;
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        printf("Unreachable Agent\n");
        // Unreachable agent, exiting program.
        do {} while(1);
        return ret;
    }

    printf("Connected to ROS\n");

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);
    rclc_publisher_init_best_effort(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, PwmStamped),
        "pico_publisher");

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1),
        timer_callback);

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    gpio_put(LED_PIN, 1);

    bool state = true;
    absolute_time_t next_blink = make_timeout_time_ms(500);

    msg.pwm[0] = 0;
    while (true)
    {
        if (absolute_time_diff_us(next_blink, get_absolute_time()) > 0) {
            state = !state;
            gpio_put(LED_PIN, state);
            next_blink = make_timeout_time_ms(500);
        }
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
