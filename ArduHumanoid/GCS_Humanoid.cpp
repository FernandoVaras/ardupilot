#include "GCS_Humanoid.h"

#include "ArduHumanoid.h"

const char* GCS_Humanoid::frame_string() const
{
    return humanoid.get_frame_string();
}

void GCS_Humanoid::update_vehicle_sensor_status_flags(void)
{
    // mode-specific flags:
    control_sensors_present |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION;

    control_sensors_enabled |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION;

    control_sensors_health |=
        MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL |
        MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION |
        MAV_SYS_STATUS_SENSOR_YAW_POSITION;

    control_sensors_present |= MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL;
    control_sensors_present |= MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL;
}
