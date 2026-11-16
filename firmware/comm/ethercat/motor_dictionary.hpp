#ifndef DCU_MOTOR_DICTIONARY_HPP
#define DCU_MOTOR_DICTIONARY_HPP

#include "encos_interface.h"

namespace dcu::ethercat::motor_dictionary {

mc_object_t *find(uint16_t index);
template <typename Object>
Object *object(uint16_t index)
{
    return static_cast<Object *>(mc_object(index));
}
inline mc_axis_tx_t *axis_feedback(unsigned axis) { return object<mc_axis_tx_t>(0x6200 + axis); }
inline mc_diag_t *axis_diagnostics(unsigned axis) { return object<mc_diag_t>(0x2400 + 16 * axis); }
constexpr uint16_t axis_object_base(unsigned axis) { return 0x3000 + 16 * axis; }

}
#endif
