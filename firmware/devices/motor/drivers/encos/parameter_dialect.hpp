#ifndef DCU_PARAMETER_DIALECT_HPP
#define DCU_PARAMETER_DIALECT_HPP

#include <cstdint>

namespace dcu::motor {
enum class ParameterDialect : uint8_t { None, EncosLegacy };
}
#endif
