#include "encos_motor.h"
#include "encos_interface.h"

size_t motor_c_layout(unsigned index)
{
    const size_t layout[] = {
        sizeof(dcu_can_frame_t), _Alignof(dcu_can_frame_t),
        sizeof(mc_axis_rx_t), _Alignof(mc_axis_rx_t),
        sizeof(mc_axis_tx_t), _Alignof(mc_axis_tx_t),
        sizeof(mc_imu_t), _Alignof(mc_imu_t),
        offsetof(mc_imu_t, TimestampNs), sizeof(mc_field_t), sizeof(mc_object_t)
    };
    return index < sizeof(layout) / sizeof(layout[0]) ? layout[index] : SIZE_MAX;
}

bool motor_c_query(dcu_can_frame_t *frame)
{
    return encos_query(5, 0, frame) && mc_object_count != 0;
}
