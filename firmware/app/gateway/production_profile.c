#include "production_profile.h"
#include "dcu_can_profile_generated.h"

const dcu_production_profile_t *dcu_production_profile(void)
{
    return &dcu_profile;
}
