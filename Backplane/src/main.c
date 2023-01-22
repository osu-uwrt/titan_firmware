#include <stdio.h>
#include "pico/stdlib.h"
#include <stdlib.h>

#include "build_version.h"

// micro-ros stuff 
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <riptide_msgs2/msg/pwm_stamped.h>
#include <rmw_microros/rmw_microros.h>
#include "pico_eth_transport.h"

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


int main() {
    stdio_init_all();

    // GPIO init
    gpio_init(LED_RGB_R_PIN);
    gpio_set_dir(LED_RGB_R_PIN, GPIO_OUT);
    gpio_init(LED_RGB_G_PIN);
    gpio_set_dir(LED_RGB_G_PIN, GPIO_OUT);
    gpio_init(LED_RGB_B_PIN);
    gpio_set_dir(LED_RGB_B_PIN, GPIO_OUT);

    // Eth init
    uint8_t xavier_ip[] = {192, 168, 1, 23};
    uint16_t xavier_port = 8888;
    pico_eth_transport_init(0, *((uint32_t*)(&xavier_ip)), xavier_port);

    // ROS init 
    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    safety_setup();

    // wait for micro-ros connect
    const int timeout_ms = 1000;

    rcl_ret_t ret;
    do {
        ret = rmw_uros_ping_agent(timeout_ms, 1);
		safety_tick();
    } while (ret != RCL_RET_OK);

    printf("%s\n", FULL_BUILD_TAG);
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

    unsigned char count = 7;

    while (true) {
        printf("Hello World!\n");

        gpio_put(LED_RGB_R_PIN, count & 0x01);
        gpio_put(LED_RGB_G_PIN, count & 0x02);
        gpio_put(LED_RGB_B_PIN, count & 0x04);

        count = count > 0 ? 7 : count - 1;

        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        safety_tick();
        
        sleep_ms(1000);
    }
    return 0;
}