#ifndef DCU_MOTOR_RESULT_HPP
#define DCU_MOTOR_RESULT_HPP

#include <cstdint>

namespace dcu::motor {
enum ResultCode : uint16_t {
    RESULT_OK, RESULT_BUSY, RESULT_INVALID, RESULT_STATE,
    RESULT_TIMEOUT, RESULT_UNSUPPORTED, RESULT_MISMATCH, RESULT_IO
};
}
#endif
