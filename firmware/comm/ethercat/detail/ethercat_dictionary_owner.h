#ifndef DCU_ETHERCAT_DICTIONARY_OWNER_H
#define DCU_ETHERCAT_DICTIONARY_OWNER_H

/* Only ethercat_callbacks.c selects the generated application's owner mode. */
#define _DIGITAL_IO_ 1
#include "../digital_io.h"
#undef _DIGITAL_IO_

#endif
