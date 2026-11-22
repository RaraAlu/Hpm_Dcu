#include "encos_parameter_transactions.hpp"
#include "motor_result.hpp"
#include "can_transport.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>

#define CHECK(expr) do { if (!(expr)) { \
    std::fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); std::exit(1); \
} } while (0)

namespace service = dcu::motor::encos_parameters;
using namespace dcu::motor;
static unsigned responses, queries, completions, cancellations[4];
static unsigned last_axis;
static uint8_t last_code;
static uint16_t last_result;
static uint32_t last_sequence;
static bool last_internal, last_maintenance;
static service::QueryValue last_value{};
static dcu_can_frame_t last_response{};

extern "C" bool dcu_can_cancel_tx(unsigned bus, dcu_can_tx_id_t id)
{ CHECK(bus < 4 && id); ++cancellations[bus]; return true; }

static void simulate_transmitted(unsigned bus, uint32_t now)
{
    static dcu_can_tx_id_t serial;
    service::mark_submitted(bus, now, ++serial);
    service::finish_tx(bus, {serial, DCU_CAN_TX_COMPLETED, {}});
}
static void response(unsigned axis, bool maintenance, const dcu_can_frame_t &frame)
{
    ++responses; last_axis = axis; last_maintenance = maintenance; last_response = frame;
}
static void query(unsigned axis, uint8_t code, const service::QueryValue &value)
{
    ++queries; last_axis = axis; last_code = code; last_value = value;
}
static void complete(unsigned axis, bool maintenance, bool internal, uint32_t sequence, uint16_t result)
{
    ++completions; last_axis = axis; last_maintenance = maintenance;
    last_internal = internal; last_sequence = sequence; last_result = result;
}
static service::Request request(unsigned axis = 7)
{
    service::Request value{};
    value.device = axis;
    value.bus = 1;
    value.dialect = dcu::motor::ParameterDialect::EncosLegacy;
    value.can_id = 8;
    value.frame_format = 1;
    value.sequence = 9;
    value.parameter = 3;
    value.opcode = 2;
    value.write_valid = true;
    value.write.dlc = 4;
    value.write.data[0] = 0xC0;
    value.write.data[1] = 4;
    value.write.data[3] = 50;
    return value;
}
static dcu_can_frame_t reply(uint8_t code, uint8_t length = 6)
{
    dcu_can_frame_t frame{};
    frame.id = 8;
    frame.flags = DCU_CAN_FLAG_FD | DCU_CAN_FLAG_BRS;
    frame.dlc = length;
    frame.data[0] = 0xA0;
    frame.data[1] = code;
    return frame;
}
static void start(uint8_t code)
{
    CHECK(service::start_query(1, 7, code, 1, 1, dcu::motor::ParameterDialect::EncosLegacy, 8, 1));
    simulate_transmitted(1, 1);
}

int main(int argc, char **argv)
{
    CHECK(argc == 2);
    service::initialize({response, query, complete});
    if (!std::strcmp(argv[1], "tx_then_reply")) {
        const auto dialect = dcu::motor::ParameterDialect::EncosLegacy;
        CHECK(service::start_query(1, 7, 1, 1, 1, dialect, 8, 1));
        service::mark_submitted(1, 1, 11);
        auto frame = reply(1);
        frame.data[2] = 0x3F; frame.data[3] = 0x80;
        CHECK(!service::consume(1, 7, frame, 2).valid && !queries && !completions);
        service::finish_tx(0, {11, DCU_CAN_TX_COMPLETED, {}});
        service::finish_tx(1, {10, DCU_CAN_TX_COMPLETED, {}});
        CHECK(service::inspect(1).awaiting_tx && !service::consume(1, 7, frame, 2).valid);
        service::finish_tx(1, {11, DCU_CAN_TX_CANCELLED, {}});
        CHECK(!service::inspect(1).busy && last_result == RESULT_IO && completions == 1 && !queries);
        service::finish_tx(1, {11, DCU_CAN_TX_COMPLETED, {}});
        CHECK(!service::consume(1, 7, frame, 3).valid && completions == 1 && !queries);
        CHECK(service::start_query(1, 7, 1, 1, 4, dialect, 8, 1));
        service::mark_submitted(1, 4, 12);
        service::finish_tx(1, {12, DCU_CAN_TX_COMPLETED, {}});
        CHECK(service::inspect(1).busy && completions == 1 && !queries);
        auto wrong_id = frame;
        ++wrong_id.id;
        CHECK(!service::consume(1, 7, wrong_id, 5).valid && completions == 1 && !queries);
        CHECK(service::consume(1, 7, frame, 5).valid && completions == 2 && queries == 1);
        return 0;
    }
    if (!std::strcmp(argv[1], "request")) {
        auto value = request();
        auto outcome = service::commit(value, 1, true, false);
        CHECK(outcome.accepted && outcome.changed && outcome.result == RESULT_BUSY && !outcome.done_sequence);
        const auto status = service::inspect(1);
        CHECK(status.busy && !status.sent && status.device == 7 && status.can_id == 8);
        CHECK(status.dialect == value.dialect && status.frame_format == 1);
        value.can_id = 2; value.sequence = 88; value.write.data[3] = 99;
        CHECK(service::pending(1)->id == 8 && service::pending(1)->data[3] == 50);
        CHECK(service::pending(1)->flags == 24);
        CHECK(!service::commit(value, 1, true, false).accepted);
        simulate_transmitted(1, 2);
        dcu_can_frame_t ack{};
        ack.id = 8; ack.dlc = 3; ack.data[0] = 0x80; ack.data[1] = 4; ack.data[2] = 1;
        CHECK(service::consume(1, 7, ack, 3).acknowledged);
        CHECK(service::pending(1)->data[1] == 22 && service::pending(1)->id == 8);
        simulate_transmitted(1, 4);
        auto readback = reply(22, 4);
        readback.data[3] = 50;
        CHECK(service::consume(1, 7, readback, 5).valid);
        CHECK(!service::inspect(1).busy && completions == 1 && queries == 1 && responses == 2);
        CHECK(last_axis == 7 && last_sequence == 9 && !last_internal && !last_maintenance);
        CHECK(last_result == RESULT_OK && last_value.pair[0] == 50);
        return 0;
    }
    if (!std::strcmp(argv[1], "admission")) {
        auto value = request(28);
        CHECK(!service::commit(value, 1, true, false).accepted);
        value.device = 7;
        value.dialect = dcu::motor::ParameterDialect::None;
        CHECK(!service::commit(value, 1, true, false).accepted);
        value = request();
        CHECK(!service::commit(value, 1, true, true).accepted);
        CHECK(service::commit(value, 1, false, false).result == RESULT_STATE);
        value.done_sequence = value.sequence;
        CHECK(!service::commit(value, 1, true, false).changed);
        value.done_sequence = 0;
        value.frame_format = 2;
        CHECK(service::commit(value, 1, true, false).result == RESULT_INVALID);
        value.frame_format = 1;
        value.write_valid = false;
        CHECK(service::commit(value, 1, true, false).result == RESULT_UNSUPPORTED);
        value.write_valid = true;
        value.write.dlc = 10;
        CHECK(service::commit(value, 1, true, false).result == RESULT_UNSUPPORTED);
        value.opcode = 99;
        CHECK(service::commit(value, 1, true, false).result == RESULT_UNSUPPORTED);
        CHECK(!service::inspect(1).busy && !completions && !responses && !queries);
        CHECK(!service::start_query(1, 7, 1, 1, 1, value.dialect, 0, 0));
        CHECK(!service::start_query(1, 7, 1, 1, 1, value.dialect, 8, 2));
        return 0;
    }
    if (!std::strcmp(argv[1], "hold")) {
        CHECK(service::available(1, 0x90000000U)); // No phantom signed deadline after 24.8 days.
        auto value = request();
        value.maintenance = true; value.opcode = 3;
        CHECK(service::commit(value, UINT32_MAX - 101, true, false).accepted);
        service::mark_submitted(1, UINT32_MAX - 100, 7);
        service::cancel(1, RESULT_IO);
        CHECK(last_maintenance && last_result == RESULT_IO && cancellations[1] == 1);
        CHECK(!service::available(1, 398) && service::available(1, 399));
        CHECK(!service::available(1, 398)); // Querying future availability must not change state.
        service::check_timeout(1, 399);
        CHECK(service::available(1, UINT32_MAX - 100)); // Expired hold cannot recur next wrap.
        CHECK(service::available(1, 0x90000000U));
        return 0;
    }
    if (!std::strcmp(argv[1], "values")) {
        for (uint8_t code : {1, 2, 3, 4}) {
            start(code);
            auto frame = reply(code);
            frame.data[2] = 0x3F; frame.data[3] = 0x80;
            CHECK(service::consume(1, 7, frame, 2).valid);
            CHECK(last_code == code && last_value.real == 1.0F);
        }
        for (uint8_t code : {5, 22, 23, 24, 25, 26, 27, 28, 31, 32, 33, 34, 38}) {
            const bool pair = (code >= 23 && code <= 28) || (code >= 32 && code <= 34);
            start(code);
            auto frame = reply(code, pair ? 6 : 4);
            frame.data[2] = 0xFE; frame.data[3] = 0xDC;
            frame.data[4] = 0x12; frame.data[5] = 0x34;
            CHECK(service::consume(1, 7, frame, 2).valid);
            CHECK(last_code == code && last_value.pair[0] == 0xFEDC);
            if (pair) CHECK(last_value.pair[1] == 0x1234);
        }
        for (uint8_t code : {35, 37}) {
            start(code);
            auto frame = reply(code, 3); frame.data[2] = 1;
            CHECK(service::consume(1, 7, frame, 2).valid && last_value.choice == 1);
        }
        return 0;
    }
    if (!std::strcmp(argv[1], "identity")) {
        start(29);
        for (uint8_t part : {3, 1, 2}) {
            auto frame = reply(29, 7);
            frame.data[2] = part;
            frame.data[3] = part;
            CHECK(service::consume(1, 7, frame, 2).valid == (part == 2));
            CHECK(queries == (part == 2 ? 1U : 0U));
        }
        CHECK(last_value.uuid[0] == 0x01000000 && last_value.uuid[1] == 0x02000000 && last_value.uuid[2] == 0x03000000);
        start(30);
        auto frame = reply(30, 8);
        for (uint8_t i = 0; i < 6; ++i) frame.data[i + 2] = i + 1;
        CHECK(service::consume(1, 7, frame, 2).valid);
        for (uint8_t i = 0; i < 6; ++i) CHECK(last_value.version[i] == i + 1);
        return 0;
    }
    if (!std::strcmp(argv[1], "kt_groups")) {
        for (uint8_t group = 1; group <= 10; ++group) {
            CHECK(service::start_query(1, 7, 36, group, 1, dcu::motor::ParameterDialect::EncosLegacy, 8, 1));
            CHECK(service::pending(1)->data[0] == (0xE0 | group));
            simulate_transmitted(1, 1);
            auto frame = reply(36, 8);
            frame.data[0] = 0xEE;
            frame.data[1] = 0x20 | group;
            const uint8_t coefficients[] = {0x0F, 0xA1, 0x04, 0x10, 0xE1, 0x18};
            std::memcpy(frame.data + 2, coefficients, 6); // Manual section 9.2.15: 250,260,270,280.
            frame.dlc = 7;
            CHECK(!service::consume(1, 7, frame, 2).valid);
            frame.dlc = 8;
            frame.data[1] = group == 10 ? 1 : group + 1;
            CHECK(!service::consume(1, 7, frame, 2).valid);
            frame.data[1] = 0x20 | group;
            CHECK(service::consume(1, 7, frame, 2).valid);
            CHECK(last_value.kt.group == group && last_code == 36 && last_result == RESULT_OK);
            for (unsigned i = 0; i < 4; ++i) CHECK(last_value.kt.words[i] == 250 + 10 * i);
        }
        CHECK(queries == 10 && completions == 10);
        return 0;
    }
    if (!std::strcmp(argv[1], "kt_write")) {
        for (bool mismatch : {false, true}) {
            auto value = request();
            value.parameter = 15;
            value.group = 10;
            value.write.dlc = 8;
            value.write.data[0] = 0xCA;
            value.write.data[1] = 16;
            const uint8_t coefficients[] = {0x00, 0x0F, 0xFF, 0x12, 0x3A, 0xBC};
            std::memcpy(value.write.data + 2, coefficients, 6);
            CHECK(service::commit(value, 1, true, false).accepted);
            simulate_transmitted(1, 1);
            auto frame = reply(0x2A, 8);
            frame.data[0] = 0xEE;
            std::memcpy(frame.data + 2, coefficients, 6);
            const unsigned completed = completions, published = queries;
            CHECK(service::consume(1, 7, frame, 2).acknowledged);
            CHECK(completions == completed && queries == published && service::inspect(1).busy);
            CHECK(service::pending(1)->data[0] == 0xEA && service::pending(1)->data[1] == 36);
            CHECK(service::pending(1)->id == 8 && service::pending(1)->flags == 24);
            CHECK(!service::consume(1, 7, frame, 2).valid); // Readback has not been submitted.
            simulate_transmitted(1, 3);
            if (mismatch) frame.data[7] ^= 1;
            CHECK(service::consume(1, 7, frame, 4).valid);
            CHECK(completions == completed + 1 && queries == published + 1);
            CHECK(last_result == (mismatch ? RESULT_MISMATCH : RESULT_OK));
            CHECK(last_value.kt.words[0] == 0 && last_value.kt.words[1] == 4095 && last_value.kt.words[2] == 0x123);
            CHECK(last_value.kt.words[3] == (mismatch ? 0xABD : 0xABC));
        }
        auto value = request();
        value.parameter = 15;
        value.group = 0;
        CHECK(service::commit(value, 1, true, false).result == RESULT_UNSUPPORTED);
        CHECK(!service::inspect(1).busy);
        return 0;
    }
    if (!std::strcmp(argv[1], "encoder")) {
        for (uint8_t enabled : {0, 1}) {
            start(39);
            auto frame = reply(39, 7);
            frame.data[2] = 2;
            frame.data[3] = 0x80; frame.data[4] = 0;
            frame.data[5] = 0x7F; frame.data[6] = 0xFF;
            const unsigned published = queries;
            CHECK(!service::consume(1, 7, frame, 2).valid && queries == published);
            frame.data[2] = enabled;
            frame.dlc = 6;
            CHECK(!service::consume(1, 7, frame, 2).valid && queries == published);
            frame.dlc = 7;
            CHECK(service::consume(1, 7, frame, 2).valid);
            CHECK(last_value.encoder.calibration_enabled == bool(enabled));
            CHECK(last_value.encoder.raw_centidegrees == -32768 && last_value.encoder.calibrated_centidegrees == 32767);
            CHECK(last_result == RESULT_OK);
        }
        return 0;
    }
    if (!std::strcmp(argv[1], "invalid")) {
        start(3);
        auto frame = reply(3);
        frame.data[2] = 0x44; frame.data[3] = 0x7A; // 1000 A overflows the application current value.
        CHECK(!service::consume(1, 7, frame, 2).valid);
        CHECK(!queries && !completions && service::inspect(1).busy);
        frame.data[2] = 0x7F; frame.data[3] = 0xC0; // NaN.
        CHECK(!service::consume(1, 7, frame, 2).valid && !queries);
        frame = reply(3, 4);
        CHECK(!service::consume(1, 7, frame, 2).valid && !queries);
        service::check_timeout(1, 101);
        CHECK(completions == 1 && last_result == RESULT_TIMEOUT && last_internal);
        CHECK(!service::consume(1, 7, reply(3), 102).valid);
        CHECK(completions == 1 && !queries);
        return 0;
    }
    CHECK(false);
}
