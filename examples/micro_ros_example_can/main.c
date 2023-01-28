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

#include "safety_interface.h"

const uint LED_PIN = 25;

rcl_publisher_t publisher;
riptide_msgs2__msg__PwmStamped msg_out;

static rcl_subscription_t pwm_subscriber;
static riptide_msgs2__msg__PwmStamped pwm_msg;

static void pwm_subscription_callback(const void * msgin)
{
	const riptide_msgs2__msg__PwmStamped * msg = (const riptide_msgs2__msg__PwmStamped *)msgin;
    rcl_publish(&publisher, &msg_out, NULL);
    msg_out.pwm[0]++;
}

extern int timer_task_count;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    rcl_ret_t ret = rcl_publish(&publisher, &msg_out, NULL);
    //msg.data++;
    msg_out.pwm[0]++;
}

int main()
{
    stdio_init_all();
    printf("%s\n", FULL_BUILD_TAG);

    pico_can_transport_init(1000000, 5, ETH_SPI, ETH_CS_PIN, ETH_MOSI_PIN, ETH_MISO_PIN, ETH_CLK_PIN, 10000000, ETH_RST_PIN);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    safety_setup();

    const int timeout_ms = 1000;

    rcl_ret_t ret;
    do {
        ret = rmw_uros_ping_agent(timeout_ms, 1);
		safety_tick();
    } while (ret != RCL_RET_OK);

    printf("Connected to ROS\n");

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);
    rclc_publisher_init_best_effort(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, PwmStamped),
        "pico_publisher");

    rclc_subscription_init_best_effort(
		&pwm_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(riptide_msgs2, msg, PwmStamped),
		"command/pwm");

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback);

    rclc_executor_init(&executor, &support.context, 2, &allocator);
    //rclc_executor_add_timer(&executor, &timer);
    rclc_executor_add_subscription(&executor, &pwm_subscriber, &pwm_msg, &pwm_subscription_callback, ON_NEW_DATA);

    gpio_put(LED_PIN, 1);
    safety_init();

    bool state = true;
    absolute_time_t next_blink = make_timeout_time_ms(500);

    msg_out.pwm[0] = 0;
    while (true)
    {
        if (absolute_time_diff_us(next_blink, get_absolute_time()) > 0) {
            state = !state;
            gpio_put(LED_PIN, state);
            next_blink = make_timeout_time_ms(500);
        }
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        safety_tick();
    }
    return 0;
}
