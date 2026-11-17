#ifndef DCU_MOTOR_RESULT_HPP
#define DCU_MOTOR_RESULT_HPP

#include <cstdint>

namespace dcu::app::motor_application {
enum ResultCode : uint16_t {
    RESULT_OK, RESULT_BUSY, RESULT_INVALID, RESULT_STATE,
    RESULT_TIMEOUT, RESULT_UNSUPPORTED, RESULT_MISMATCH, RESULT_IO
};
enum Owner : uint8_t { OWNER_NONE, OWNER_PDO, OWNER_SDO };
}
#endif
