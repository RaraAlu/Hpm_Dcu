#ifndef DCU_ENCOS_PARAMETER_TRANSACTIONS_HPP
#define DCU_ENCOS_PARAMETER_TRANSACTIONS_HPP

#include "can_transport.h"
#include "parameter_dialect.hpp"
#include <cstdint>

namespace dcu::motor::encos_parameters {

struct Status {
    bool busy, sent, internal;
    unsigned device;
    uint32_t can_id;
    ParameterDialect dialect;
    uint8_t frame_format;
    bool awaiting_tx;
    uint32_t started_ms;
};

struct Request {
    unsigned device{};
    ParameterDialect dialect{};
    uint16_t can_id{};
    uint8_t frame_format{}, group{};
    uint32_t sequence{}, done_sequence{};
    bool maintenance{};
    uint16_t parameter{}, opcode{};
    uint32_t argument0{}, argument1{};
    bool write_valid{};
    unsigned bus{};
    dcu_can_frame_t write{};
};

struct CommitOutcome {
    bool accepted{};
    bool changed{};
    uint16_t result{};
    uint32_t done_sequence{};
};

struct KtGroup {
    uint16_t words[4];
    uint8_t group;
};

struct EncoderAngles {
    int16_t raw_centidegrees, calibrated_centidegrees;
    bool calibration_enabled;
};

// Query code selects the populated value; these are CAN parameter values,
// not EtherCAT object offsets or writable storage references.
union QueryValue {
    float real;
    uint16_t pair[2];
    uint32_t uuid[3];
    uint8_t version[6];
    uint8_t choice;
    KtGroup kt;
    EncoderAngles encoder;
};

struct Observer {
    void (*response)(unsigned device, bool maintenance, const dcu_can_frame_t &frame){};
    void (*query)(unsigned device, uint8_t code, const QueryValue &value){};
    void (*complete)(unsigned device, bool maintenance, bool internal, uint32_t sequence, uint16_t result){};
};

// Initialize once before task work. Notifications run synchronously on the
// caller's task; observers must not reenter transaction operations.
void initialize(const Observer &observer);


enum class Field : uint8_t { None,
                             Position,
                             Velocity,
                             Current,
                             Power };

struct Observation {
    bool valid{};
    bool update_actual{};
    bool acknowledged{};
    Field field{};
    int32_t value{};
};


Status inspect(unsigned bus);


bool available(unsigned bus, uint32_t now_ms);


const dcu_can_frame_t *pending(unsigned bus);


void mark_submitted(unsigned bus, uint32_t now_ms, dcu_can_tx_id_t id);
void finish_tx(unsigned bus, const dcu_can_tx_event_t &event);


void check_timeout(unsigned bus, uint32_t now_ms);


void cancel(unsigned bus, uint16_t result);


void reset_session(uint16_t result);


void abort_internal(unsigned bus);


bool start_query(unsigned bus, unsigned device, uint8_t query, uint8_t group,
                 uint32_t now_ms, ParameterDialect dialect, uint16_t can_id, uint8_t format);


CommitOutcome commit(const Request &request, uint32_t now_ms, bool permitted, bool channel_active);


bool consume_maintenance(unsigned bus, const dcu_can_frame_t &frame);


Observation consume(unsigned bus, unsigned device, const dcu_can_frame_t &frame,
                    uint32_t now_ms);

}
#endif
