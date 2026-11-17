#include "encos_service.h"
#include <limits.h>
#include <math.h>
#include <string.h>

uint16_t encos_be16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0] << 8 | p[1]); }
uint32_t encos_be32(const uint8_t *p)
{
    return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3];
}
bool encos_float(const uint8_t *p, double scale, int32_t *value)
{
    uint32_t bits = encos_be32(p);
    float real;
    memcpy(&real, &bits, sizeof(real));
    double scaled = real * scale;
    if (!isfinite(scaled) || scaled < INT32_MIN || scaled > INT32_MAX) return false;
    *value = (int32_t)scaled;
    return true;
}
uint8_t encos_parameter_query(uint16_t parameter)
{
    static const uint8_t query[] = {0, 5, 0, 22, 23, 24, 25, 26, 27, 28, 31, 32, 33, 34, 35, 36, 38};
    return parameter < sizeof(query) ? query[parameter] :
        (parameter >= 0x101 && parameter <= 0x127 ? (uint8_t)(parameter - 0x100) : 0);
}
bool encos_query(uint8_t code, uint8_t group, dcu_can_frame_t *frame)
{
    if (!(code >= 1 && code <= 5) && !(code >= 22 && code <= 39)) return false;
    if (code == 36 && (group < 1 || group > 10)) return false;
    frame->dlc = 2;
    memset(frame->data, 0, sizeof(frame->data));
    frame->data[0] = 0xE0 | (code == 36 ? group : 0);
    frame->data[1] = code;
    return true;
}

bool encos_parameter(uint16_t id, const encos_parameter_values_t *values, dcu_can_frame_t *frame)
{
    static const uint8_t code[] = {0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    // Acceleration ACK table conflicts with its example; disabled pending confirmation.
    if (id < 2 || id > 15) return false;
    dcu_can_frame_t candidate = *frame;
    memset(candidate.data, 0, sizeof(candidate.data));
    candidate.data[0] = 0xC0;
    candidate.data[1] = code[id];
    candidate.dlc = 4;
    const auto *v = values->words;
    if (id == 2 || id == 14) {
        if (v[0] > (id == 2 ? 1 : 2)) return false;
        if (id == 2) candidate.data[0] = 0xC1;
        candidate.data[2] = static_cast<uint8_t>(v[0]);
        candidate.dlc = 3;
    } else if (id == 15) {
        if (values->group < 1 || values->group > 10 || v[0] > 4095 || v[1] > 4095 || v[2] > 4095 || v[3] > 4095)
            return false;
        candidate.data[0] |= values->group;
        const uint64_t bits = uint64_t(v[0]) << 36 | uint64_t(v[1]) << 24 | uint64_t(v[2]) << 12 | v[3];
        for (unsigned i = 0; i < 6; ++i)
            candidate.data[i + 2] = static_cast<uint8_t>(bits >> (40 - 8 * i));
        candidate.dlc = 8;
    } else {
        if ((id == 3 || id == 10) && !v[0]) return false;
        const unsigned count = (id >= 4 && id <= 9) || (id >= 11 && id <= 13) ? 2 : 1;
        for (unsigned i = 0; i < count; ++i) {
            candidate.data[2 + 2 * i] = v[i] >> 8;
            candidate.data[3 + 2 * i] = v[i];
        }
        candidate.dlc = 2 + 2 * count;
    }
    *frame = candidate;
    return true;
}
