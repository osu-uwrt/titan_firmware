#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "nano_ros.h"


#define IMU_DATA_ID 127u
#define BIT_SUB_ID 0u
#define FIRMWARE_STATUS_ID 128u

typedef struct {
    int32_t sec;
    uint32_t nanosec;
} builtin_interfaces__msg__Time_t;

typedef struct {
    builtin_interfaces__msg__Time_t stamp;
    char frame_id[256];
} std_msgs__msg__Header_t;

typedef struct {
    double x;
    double y;
    double z;
    double w;
} geometry_msgs__msg__Quaternion_t;

typedef struct {
    double x;
    double y;
    double z;
} geometry_msgs__msg__Vector3_t;

typedef struct {
    std_msgs__msg__Header_t header;
    geometry_msgs__msg__Quaternion_t orientation;
    double orientation_covariance[9];
    geometry_msgs__msg__Vector3_t angular_velocity;
    double angular_velocity_covariance[9];
    geometry_msgs__msg__Vector3_t linear_acceleration;
    double linear_acceleration_covariance[9];
} sensor_msgs__msg__Imu_t;

typedef struct {
    uint8_t data;
} std_msgs__msg__UInt8_t;

typedef struct {
    char board_name[256];
    uint8_t bus_id;
    uint8_t client_id;
    uint8_t version_major;
    uint8_t version_minor;
    uint8_t version_release_type;
    uint32_t uptime_ms;
    uint32_t faults;
    uint32_t kill_switches_enabled;
    uint32_t kill_switches_asserting_kill;
    uint32_t kill_switches_needs_update;
    uint32_t kill_switches_timed_out;
} riptide_msgs2__msg__FirmwareStatus_t;

bool nros_publish_imu_data(const sensor_msgs__msg__Imu_t *msg);
void nros_set_bit_sub_subscriber_cb(void (*cb)(const std_msgs__msg__UInt8_t *msg));
bool nros_publish_firmware_status(const riptide_msgs2__msg__FirmwareStatus_t *msg);
bool builtin_interfaces__msg__Time__serialize(ucdrBuffer* buf, const builtin_interfaces__msg__Time_t* msg);
bool builtin_interfaces__msg__Time__deserialize(ucdrBuffer* buf, builtin_interfaces__msg__Time_t* msg);
bool std_msgs__msg__Header__serialize(ucdrBuffer* buf, const std_msgs__msg__Header_t* msg);
bool std_msgs__msg__Header__deserialize(ucdrBuffer* buf, std_msgs__msg__Header_t* msg);
bool geometry_msgs__msg__Quaternion__serialize(ucdrBuffer* buf, const geometry_msgs__msg__Quaternion_t* msg);
bool geometry_msgs__msg__Quaternion__deserialize(ucdrBuffer* buf, geometry_msgs__msg__Quaternion_t* msg);
bool geometry_msgs__msg__Vector3__serialize(ucdrBuffer* buf, const geometry_msgs__msg__Vector3_t* msg);
bool geometry_msgs__msg__Vector3__deserialize(ucdrBuffer* buf, geometry_msgs__msg__Vector3_t* msg);
bool sensor_msgs__msg__Imu__serialize(ucdrBuffer* buf, const sensor_msgs__msg__Imu_t* msg);
bool sensor_msgs__msg__Imu__deserialize(ucdrBuffer* buf, sensor_msgs__msg__Imu_t* msg);
bool std_msgs__msg__UInt8__serialize(ucdrBuffer* buf, const std_msgs__msg__UInt8_t* msg);
bool std_msgs__msg__UInt8__deserialize(ucdrBuffer* buf, std_msgs__msg__UInt8_t* msg);
bool riptide_msgs2__msg__FirmwareStatus__serialize(ucdrBuffer* buf, const riptide_msgs2__msg__FirmwareStatus_t* msg);
bool riptide_msgs2__msg__FirmwareStatus__deserialize(ucdrBuffer* buf, riptide_msgs2__msg__FirmwareStatus_t* msg);
