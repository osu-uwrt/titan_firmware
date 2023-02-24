#include <stdio.h>
#include "pico/stdlib.h"
#include <stdlib.h>

#include "build_version.h"
// adc test
#include "mcp3426.h"

// micro-ros stuff
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <riptide_msgs2/msg/pwm_stamped.h>
#include <rmw_microros/rmw_microros.h>
#include "micro_ros_pico/transport_eth.h"
#include "eth_networking.h"

#include "safety_interface.h"

rcl_publisher_t publisher;
riptide_msgs2__msg__PwmStamped msg_out;

static rcl_subscription_t pwm_subscriber;
static riptide_msgs2__msg__PwmStamped pwm_msg;

static void pwm_subscription_callback(__unused const void * msgin)
{
	const riptide_msgs2__msg__PwmStamped * msg = (const riptide_msgs2__msg__PwmStamped *)msgin;
    rcl_publish(&publisher, &msg_out, NULL);
    msg_out.pwm[0]++;
}

extern int timer_task_count;

void timer_callback(__unused rcl_timer_t *timer, __unused int64_t last_call_time)
{
    rcl_ret_t ret = rcl_publish(&publisher, &msg_out, NULL);
    //msg.data++;
    msg_out.pwm[0]++;
}


int main() {
    stdio_init_all();

    safety_setup();
    // safety_init();

    //safety_setup();
    unsigned char count = 7;
    uint8_t gpio_base_status_reg = 0;
    
    // Computer power on
    gpio_init(PWR_CTL_CPU);
    gpio_init(LED_RGB_R_PIN);
    gpio_init(LED_RGB_G_PIN);
    gpio_init(LED_RGB_B_PIN);
    gpio_init(ETH_CS_PIN);
    gpio_init(ETH_RST_PIN);

    gpio_set_dir(ETH_RST_PIN, GPIO_OUT);
    gpio_set_dir(ETH_CS_PIN, GPIO_OUT);
    gpio_set_dir(LED_RGB_R_PIN, GPIO_OUT);
    gpio_set_dir(LED_RGB_G_PIN, GPIO_OUT);
    gpio_set_dir(LED_RGB_B_PIN, GPIO_OUT);
    gpio_set_dir(PWR_CTL_CPU, GPIO_OUT);


    // GPIO init
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_put(PWR_CTL_CPU, 1);
    gpio_put(LED_RGB_R_PIN, 0);
    gpio_put(LED_RGB_G_PIN, 1);
    gpio_put(LED_RGB_B_PIN, 1);
    gpio_put(ETH_CS_PIN, 1);

	//reset routine
    gpio_put(ETH_RST_PIN, 0);
    busy_wait_ms(50);
    gpio_put(ETH_RST_PIN, 1);
    busy_wait_ms(1000);


    sleep_ms(1000);

    // Eth init

    transport_eth_init();

    // ROS init
    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

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
    rclc_executor_add_timer(&executor, &timer);
    rclc_executor_add_subscription(&executor, &pwm_subscriber, &pwm_msg, &pwm_subscription_callback, ON_NEW_DATA);

    mcp3426_init();

    while (true) {
        count = 7;
        
        // Basic GPIO management
        // Kill switches and inputs
        if(!gpio_get(KILL_SW_SENSE)) {
            printf("Kill Switch Fired!\n");
            count ^= 0x1;
            gpio_base_status_reg |= 0x2;
        }
        if(!gpio_get(AUX_SW_SENSE)) {
            printf("AUX Switch Fired!\n");
            count ^= 0x4;
            gpio_base_status_reg |= 0x4;
        }

        mcp3426_read();
        sleep_us(1000);

        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        //safety_tick();
    }
    return 0;
}