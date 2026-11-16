#include "motor_dictionary.hpp"

namespace dcu::ethercat::motor_dictionary {
mc_object_t *find(uint16_t index)
{
    unsigned lo = 0, hi = mc_object_count;
    while (lo < hi) {
        unsigned m = (lo + hi) / 2;
        if (mc_objects[m].index < index)
            lo = m + 1;
        else
            hi = m;
    }
    return lo < mc_object_count && mc_objects[lo].index == index ? &mc_objects[lo] : NULL;
}
}

void *mc_object(uint16_t index)
{
    mc_object_t *o = dcu::ethercat::motor_dictionary::find(index);
    return o ? o->data : NULL;
}
