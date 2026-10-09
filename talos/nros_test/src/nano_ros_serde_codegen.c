#include "nano_ros_codegen.h"

bool builtin_interfaces__msg__Time__serialize(ucdrBuffer* buf, const builtin_interfaces__msg__Time_t* msg) {
    bool ok = true;
    ok &= ucdr_serialize_int32_t(buf, msg->sec);
    ok &= ucdr_serialize_uint32_t(buf, msg->nanosec);
    return ok;
}

bool builtin_interfaces__msg__Time__deserialize(ucdrBuffer* buf, builtin_interfaces__msg__Time_t* msg) {
    bool ok = true;
    ok &= ucdr_deserialize_int32_t(buf, &msg->sec);
    ok &= ucdr_deserialize_uint32_t(buf, &msg->nanosec);
    return ok;
}

bool std_msgs__msg__Header__serialize(ucdrBuffer* buf, const std_msgs__msg__Header_t* msg) {
    bool ok = true;
    ok &= builtin_interfaces__msg__Time__serialize(buf, &msg->stamp);
    ok &= ucdr_serialize_string(buf, msg->frame_id);
    return ok;
}

bool std_msgs__msg__Header__deserialize(ucdrBuffer* buf, std_msgs__msg__Header_t* msg) {
    bool ok = true;
    ok &= builtin_interfaces__msg__Time__deserialize(buf, &msg->stamp);
    ok &= ucdr_deserialize_string(buf, msg->frame_id, sizeof(msg->frame_id));
    return ok;
}

bool geometry_msgs__msg__Quaternion__serialize(ucdrBuffer* buf, const geometry_msgs__msg__Quaternion_t* msg) {
    bool ok = true;
    ok &= ucdr_serialize_double(buf, msg->x);
    ok &= ucdr_serialize_double(buf, msg->y);
    ok &= ucdr_serialize_double(buf, msg->z);
    ok &= ucdr_serialize_double(buf, msg->w);
    return ok;
}

bool geometry_msgs__msg__Quaternion__deserialize(ucdrBuffer* buf, geometry_msgs__msg__Quaternion_t* msg) {
    bool ok = true;
    ok &= ucdr_deserialize_double(buf, &msg->x);
    ok &= ucdr_deserialize_double(buf, &msg->y);
    ok &= ucdr_deserialize_double(buf, &msg->z);
    ok &= ucdr_deserialize_double(buf, &msg->w);
    return ok;
}

bool geometry_msgs__msg__Vector3__serialize(ucdrBuffer* buf, const geometry_msgs__msg__Vector3_t* msg) {
    bool ok = true;
    ok &= ucdr_serialize_double(buf, msg->x);
    ok &= ucdr_serialize_double(buf, msg->y);
    ok &= ucdr_serialize_double(buf, msg->z);
    return ok;
}

bool geometry_msgs__msg__Vector3__deserialize(ucdrBuffer* buf, geometry_msgs__msg__Vector3_t* msg) {
    bool ok = true;
    ok &= ucdr_deserialize_double(buf, &msg->x);
    ok &= ucdr_deserialize_double(buf, &msg->y);
    ok &= ucdr_deserialize_double(buf, &msg->z);
    return ok;
}

bool sensor_msgs__msg__Imu__serialize(ucdrBuffer* buf, const sensor_msgs__msg__Imu_t* msg) {
    bool ok = true;
    ok &= std_msgs__msg__Header__serialize(buf, &msg->header);
    ok &= geometry_msgs__msg__Quaternion__serialize(buf, &msg->orientation);
    ok &= ucdr_serialize_array_double(buf, msg->orientation_covariance, 9);
    ok &= geometry_msgs__msg__Vector3__serialize(buf, &msg->angular_velocity);
    ok &= ucdr_serialize_array_double(buf, msg->angular_velocity_covariance, 9);
    ok &= geometry_msgs__msg__Vector3__serialize(buf, &msg->linear_acceleration);
    ok &= ucdr_serialize_array_double(buf, msg->linear_acceleration_covariance, 9);
    return ok;
}

bool sensor_msgs__msg__Imu__deserialize(ucdrBuffer* buf, sensor_msgs__msg__Imu_t* msg) {
    bool ok = true;
    ok &= std_msgs__msg__Header__deserialize(buf, &msg->header);
    ok &= geometry_msgs__msg__Quaternion__deserialize(buf, &msg->orientation);
    ok &= ucdr_deserialize_array_double(buf, msg->orientation_covariance, 9);
    ok &= geometry_msgs__msg__Vector3__deserialize(buf, &msg->angular_velocity);
    ok &= ucdr_deserialize_array_double(buf, msg->angular_velocity_covariance, 9);
    ok &= geometry_msgs__msg__Vector3__deserialize(buf, &msg->linear_acceleration);
    ok &= ucdr_deserialize_array_double(buf, msg->linear_acceleration_covariance, 9);
    return ok;
}

bool std_msgs__msg__UInt8__serialize(ucdrBuffer* buf, const std_msgs__msg__UInt8_t* msg) {
    bool ok = true;
    ok &= ucdr_serialize_uint8_t(buf, msg->data);
    return ok;
}

bool std_msgs__msg__UInt8__deserialize(ucdrBuffer* buf, std_msgs__msg__UInt8_t* msg) {
    bool ok = true;
    ok &= ucdr_deserialize_uint8_t(buf, &msg->data);
    return ok;
}

bool riptide_msgs2__msg__FirmwareStatus__serialize(ucdrBuffer* buf, const riptide_msgs2__msg__FirmwareStatus_t* msg) {
    bool ok = true;
    ok &= ucdr_serialize_string(buf, msg->board_name);
    ok &= ucdr_serialize_uint8_t(buf, msg->bus_id);
    ok &= ucdr_serialize_uint8_t(buf, msg->client_id);
    ok &= ucdr_serialize_uint8_t(buf, msg->version_major);
    ok &= ucdr_serialize_uint8_t(buf, msg->version_minor);
    ok &= ucdr_serialize_uint8_t(buf, msg->version_release_type);
    ok &= ucdr_serialize_uint32_t(buf, msg->uptime_ms);
    ok &= ucdr_serialize_uint32_t(buf, msg->faults);
    ok &= ucdr_serialize_uint32_t(buf, msg->kill_switches_enabled);
    ok &= ucdr_serialize_uint32_t(buf, msg->kill_switches_asserting_kill);
    ok &= ucdr_serialize_uint32_t(buf, msg->kill_switches_needs_update);
    ok &= ucdr_serialize_uint32_t(buf, msg->kill_switches_timed_out);
    return ok;
}

bool riptide_msgs2__msg__FirmwareStatus__deserialize(ucdrBuffer* buf, riptide_msgs2__msg__FirmwareStatus_t* msg) {
    bool ok = true;
    ok &= ucdr_deserialize_string(buf, msg->board_name, sizeof(msg->board_name));
    ok &= ucdr_deserialize_uint8_t(buf, &msg->bus_id);
    ok &= ucdr_deserialize_uint8_t(buf, &msg->client_id);
    ok &= ucdr_deserialize_uint8_t(buf, &msg->version_major);
    ok &= ucdr_deserialize_uint8_t(buf, &msg->version_minor);
    ok &= ucdr_deserialize_uint8_t(buf, &msg->version_release_type);
    ok &= ucdr_deserialize_uint32_t(buf, &msg->uptime_ms);
    ok &= ucdr_deserialize_uint32_t(buf, &msg->faults);
    ok &= ucdr_deserialize_uint32_t(buf, &msg->kill_switches_enabled);
    ok &= ucdr_deserialize_uint32_t(buf, &msg->kill_switches_asserting_kill);
    ok &= ucdr_deserialize_uint32_t(buf, &msg->kill_switches_needs_update);
    ok &= ucdr_deserialize_uint32_t(buf, &msg->kill_switches_timed_out);
    return ok;
}

