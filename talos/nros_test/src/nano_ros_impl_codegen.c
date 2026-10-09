#include "nano_ros_codegen.h"

bool nros_publish_imu_data(const sensor_msgs__msg__Imu_t *msg) {
    return nros_publish(IMU_DATA_ID, (nros_serialize_fn) sensor_msgs__msg__Imu__serialize, msg);
}

static void (*bit_sub_cb)(const std_msgs__msg__UInt8_t *) = NULL;

static void nros_bit_sub_subscriber_handler(ucdrBuffer *buf) {
    std_msgs__msg__UInt8_t msg;
    if (std_msgs__msg__UInt8__deserialize(buf, &msg) && bit_sub_cb) {
        bit_sub_cb(&msg);
    }
}

void nros_set_bit_sub_subscriber_cb(void (*cb)(const std_msgs__msg__UInt8_t *msg)) {
    bit_sub_cb = cb;
}

bool nros_publish_firmware_status(const riptide_msgs2__msg__FirmwareStatus_t *msg) {
    return nros_publish(FIRMWARE_STATUS_ID, (nros_serialize_fn) riptide_msgs2__msg__FirmwareStatus__serialize, msg);
}

const uint8_t nros_topic_count = 3u;
const nros_topic_t nros_topics[3] = {
    {
        .topic_id = IMU_DATA_ID,
        .topic_name = "imu_data",
        .topic_type = "sensor_msgs/msg/Imu",
        .cb = NULL,
    },
    {
        .topic_id = BIT_SUB_ID,
        .topic_name = "bit_sub",
        .topic_type = "std_msgs/msg/UInt8",
        .cb = nros_bit_sub_subscriber_handler,
    },
    {
        .topic_id = FIRMWARE_STATUS_ID,
        .topic_name = "firmware_status",
        .topic_type = "riptide_msgs2/msg/FirmwareStatus",
        .cb = NULL,
    },
};

