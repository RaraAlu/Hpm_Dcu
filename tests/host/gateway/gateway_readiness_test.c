#include "startup/gateway_session.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static const dcu_gateway_profile_t profile = {
    .profile_id = 1,
    .revision = 1,
    .digest = {[0] = 1, [31] = 42},
    .readiness_ttl_ms = 50,
    .activation_wait_s = 5,
};
static const uint64_t boot_id = UINT64_C(0x0102030405060708);
static uint64_t next_id = 100;
static unsigned calls;

static uint64_t issue_id(void)
{
    ++calls;
    return next_id;
}

static dcu_gateway_activation_t request(uint64_t now_us)
{
    dcu_gateway_readiness_t state = dcu_gateway_readiness(now_us);
    dcu_gateway_activation_t activation = {
        .profile_id = profile.profile_id,
        .revision = profile.revision,
        .boot_id = boot_id,
        .ready_generation = state.ready_generation,
        .request_id = 9,
        .client_tag = 11,
    };
    memcpy(activation.digest, profile.digest, sizeof(activation.digest));
    return activation;
}

static void check(dcu_gateway_activation_t activation, uint64_t now_us,
                  dcu_gateway_activation_code_t code, uint64_t session_id)
{
    dcu_gateway_activation_result_t result = dcu_gateway_activate(&activation, now_us);
    assert(result.code == code);
    assert(result.session_id == session_id);
    assert(result.request_id == activation.request_id);
    assert(result.state == dcu_gateway_readiness(now_us).state);
}

static void prepare(void)
{
    assert(dcu_gateway_boot_session(&profile, boot_id, issue_id, 100));
    assert(dcu_gateway_probe_verified(200, 201));
}

static void static_startup(void)
{
    dcu_gateway_profile_t fixed = profile;
    fixed.activation_wait_s = 0;
    assert(dcu_gateway_boot_static(&fixed, boot_id, 100));
    assert(!dcu_gateway_boot_session(&profile, boot_id, issue_id, 101));
    assert(!dcu_gateway_resume_static(101));
    assert(dcu_gateway_probe_verified(200, 201));
    dcu_gateway_activation_t valid = request(201);
    check(valid, 201, DCU_GATEWAY_INVALID, 0);
    assert(!dcu_gateway_start_run(100, 201));
    assert(dcu_gateway_resume_static(201));
    check(valid, 201, DCU_GATEWAY_INVALID, 0);
    assert(dcu_gateway_readiness(201).state == DCU_GATEWAY_RUNNING && calls == 0);
}

static void startup(void)
{
    assert(dcu_gateway_readiness(100).state == DCU_GATEWAY_BOOT);
    assert(dcu_gateway_boot_session(&profile, boot_id, issue_id, 100));
    assert(!dcu_gateway_boot_session(&profile, boot_id + 1, issue_id, 101));
    assert(!dcu_gateway_boot_static(&profile, boot_id, 101));
    dcu_gateway_activation_t early = request(200);
    check(early, 200, DCU_GATEWAY_NOT_READY, 0);
    assert(calls == 0);
    assert(dcu_gateway_readiness(200).feedback_age_us == UINT32_MAX);
    assert(!dcu_gateway_probe_verified(99, 201));
    assert(!dcu_gateway_probe_verified(203, 202));
    assert(dcu_gateway_probe_verified(202, 203));
    dcu_gateway_readiness_t ready = dcu_gateway_readiness(205);
    assert(ready.state == DCU_GATEWAY_READY && ready.session_id == 0);
    assert(ready.boot_id == boot_id && ready.ready_generation != 0);
    assert(ready.profile_id == profile.profile_id && ready.revision == profile.revision);
    assert(memcmp(ready.digest, profile.digest, sizeof(ready.digest)) == 0);
    assert(ready.feedback_age_us == 3);
    check(early, 205, DCU_GATEWAY_STALE_CONTEXT, 0);
    dcu_gateway_activation_t fresh = request(205);
    assert(!dcu_gateway_resume_static(205));
    check(fresh, 205, DCU_GATEWAY_ACCEPTED, 100);
    ready = dcu_gateway_readiness(205);
    assert(ready.state == DCU_GATEWAY_READY);
    assert(ready.active_request_id == 9 && ready.client_tag == 11);
    assert(ready.session_id == 100 && calls == 1);
    assert(!dcu_gateway_resume_static(205));
}

static void mismatch(void)
{
    prepare();
    dcu_gateway_activation_t good = request(201);
    dcu_gateway_activation_t bad = good;
    assert(dcu_gateway_activate(NULL, 201).code == DCU_GATEWAY_INVALID);
    bad.reserved = 1;
    check(bad, 201, DCU_GATEWAY_INVALID, 0);
    bad = good;
    bad.client_tag = 0;
    check(bad, 201, DCU_GATEWAY_INVALID, 0);
    bad = good;
    ++bad.profile_id;
    check(bad, 201, DCU_GATEWAY_CONFIG_MISMATCH, 0);
    bad = good;
    bad.digest[31] ^= 1;
    check(bad, 201, DCU_GATEWAY_CONFIG_MISMATCH, 0);
    bad = good;
    ++bad.revision;
    check(bad, 201, DCU_GATEWAY_CONFIG_MISMATCH, 0);
    bad = good;
    ++bad.boot_id;
    check(bad, 201, DCU_GATEWAY_STALE_CONTEXT, 0);
    bad = good;
    ++bad.ready_generation;
    check(bad, 201, DCU_GATEWAY_STALE_CONTEXT, 0);
    assert(calls == 0 && dcu_gateway_readiness(201).session_id == 0);
}

static void duplicate(void)
{
    prepare();
    dcu_gateway_activation_t good = request(201);
    check(good, 201, DCU_GATEWAY_ACCEPTED, 100);
    dcu_gateway_activation_t altered = good;
    ++altered.client_tag;
    check(altered, 205, DCU_GATEWAY_INVALID, 0);
    ++altered.request_id;
    check(altered, 206, DCU_GATEWAY_BUSY, 0);
    assert(dcu_gateway_probe_verified(299, 300));
    assert(dcu_gateway_readiness(300).ready_generation == good.ready_generation);
    check(good, 301, DCU_GATEWAY_ACCEPTED, 100);
    assert(calls == 1 && dcu_gateway_readiness(301).session_id == 100);
}

static void freshness(void)
{
    prepare();
    dcu_gateway_activation_t good = request(201);
    check(good, 201, DCU_GATEWAY_ACCEPTED, 100);
    assert(!dcu_gateway_probe_verified(200, 50199));
    assert(dcu_gateway_readiness(50199).state == DCU_GATEWAY_READY);
    dcu_gateway_readiness_t expired = dcu_gateway_readiness(50200);
    assert(expired.state == DCU_GATEWAY_HOLD && expired.reason == DCU_GATEWAY_REASON_FEEDBACK_TIMEOUT);
    assert(expired.ready_generation != good.ready_generation);
    assert(expired.session_id == 0 && expired.feedback_age_us == UINT32_MAX);
    check(good, 50200, DCU_GATEWAY_NOT_READY, 0);
}

static void recovery(void)
{
    prepare();
    dcu_gateway_activation_t first = request(201);
    check(first, 201, DCU_GATEWAY_ACCEPTED, 100);
    dcu_gateway_link_lost(DCU_GATEWAY_REASON_CAN_OFFLINE, 300);
    dcu_gateway_readiness_t lost = dcu_gateway_readiness(300);
    assert(lost.state == DCU_GATEWAY_HOLD && lost.reason == DCU_GATEWAY_REASON_CAN_OFFLINE);
    assert(lost.session_id == 0 && lost.active_request_id == 0 && lost.client_tag == 0);
    assert(!dcu_gateway_probe_verified(400, 401));
    assert(dcu_gateway_retry_preparation(500));
    assert(!dcu_gateway_probe_verified(499, 501));
    assert(dcu_gateway_probe_verified(501, 502));
    check(first, 502, DCU_GATEWAY_STALE_CONTEXT, 0);
    dcu_gateway_activation_t second = request(502);
    check(second, 502, DCU_GATEWAY_STALE_CONTEXT, 0);
    ++second.request_id;
    check(second, 502, DCU_GATEWAY_STALE_CONTEXT, 0);
    ++second.client_tag;
    next_id = 101;
    check(second, 502, DCU_GATEWAY_ACCEPTED, 101);
    assert(calls == 2);
}

static void deadline(void)
{
    prepare();
    dcu_gateway_activation_t first = request(201);
    check(first, 201, DCU_GATEWAY_ACCEPTED, 100);
    for (uint64_t now = 10201; now <= 4990201; now += 10000) {
        assert(dcu_gateway_probe_verified(now, now));
        check(first, now, DCU_GATEWAY_ACCEPTED, 100);
    }
    check(first, 5000200, DCU_GATEWAY_ACCEPTED, 100);
    check(first, 5000201, DCU_GATEWAY_NOT_READY, 0);
    dcu_gateway_readiness_t status = dcu_gateway_readiness(5000201);
    assert(status.state == DCU_GATEWAY_HOLD && status.reason == DCU_GATEWAY_REASON_PROTOCOL_MISMATCH);
    assert(calls == 1);
}

static void clock_regression(void)
{
    prepare();
    dcu_gateway_activation_t valid = request(201);
    check(valid, 201, DCU_GATEWAY_ACCEPTED, 100);
    dcu_gateway_readiness_t fault = dcu_gateway_readiness(200);
    assert(fault.state == DCU_GATEWAY_FAULT && fault.session_id == 0);
    assert(!dcu_gateway_retry_preparation(202));
    assert(!dcu_gateway_probe_verified(203, 204));
}

static void invalid_profile(void)
{
    dcu_gateway_profile_t invalid = profile;
    invalid.readiness_ttl_ms = 0;
    assert(!dcu_gateway_boot_session(&invalid, boot_id, issue_id, 100));
    dcu_gateway_readiness_t fault = dcu_gateway_readiness(101);
    assert(fault.state == DCU_GATEWAY_FAULT && fault.reason == DCU_GATEWAY_REASON_CONFIG_INVALID);
    assert(!dcu_gateway_boot_session(&profile, boot_id, issue_id, 102));
    assert(!dcu_gateway_probe_verified(103, 104));
}

static void invalid_session_boot(bool missing_issuer)
{
    dcu_gateway_profile_t diagnostic = profile;
    if (!missing_issuer) diagnostic.activation_wait_s = 0;
    assert(!dcu_gateway_boot_session(&diagnostic, boot_id, missing_issuer ? NULL : issue_id, 100));
    dcu_gateway_readiness_t fault = dcu_gateway_readiness(101);
    assert(fault.state == DCU_GATEWAY_FAULT && fault.reason == DCU_GATEWAY_REASON_CONFIG_INVALID);
    assert(!dcu_gateway_boot_static(&profile, boot_id, 102));
    assert(calls == 0);
}

static void exhausted(void)
{
    prepare();
    dcu_gateway_activation_t valid = request(201);
    next_id = 0;
    check(valid, 201, DCU_GATEWAY_NOT_READY, 0);
    dcu_gateway_readiness_t fault = dcu_gateway_readiness(201);
    assert(fault.state == DCU_GATEWAY_FAULT && fault.reason == DCU_GATEWAY_REASON_COUNTER_EXHAUSTED);
    assert(calls == 1);
}

static void reused_session(void)
{
    prepare();
    dcu_gateway_activation_t first = request(201);
    check(first, 201, DCU_GATEWAY_ACCEPTED, 100);
    dcu_gateway_link_lost(DCU_GATEWAY_REASON_CAN_OFFLINE, 210);
    assert(dcu_gateway_retry_preparation(220));
    assert(dcu_gateway_probe_verified(221, 222));
    dcu_gateway_activation_t second = request(222);
    ++second.request_id;
    ++second.client_tag;
    check(second, 222, DCU_GATEWAY_NOT_READY, 0);
    assert(dcu_gateway_readiness(222).reason == DCU_GATEWAY_REASON_COUNTER_EXHAUSTED);
}

static void near_clock_limit(void)
{
    const uint64_t base = UINT64_MAX - 80000;
    assert(dcu_gateway_boot_session(&profile, boot_id, issue_id, base));
    assert(!dcu_gateway_probe_verified(base, base + 50000));
    assert(dcu_gateway_probe_verified(base + 50001, base + 50001));
    check(request(base + 50001), base + 50001, DCU_GATEWAY_ACCEPTED, 100);
    assert(dcu_gateway_readiness(UINT64_MAX).state == DCU_GATEWAY_READY);
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    if (strcmp(argv[1], "startup") == 0) startup();
    else if (strcmp(argv[1], "static_startup") == 0) static_startup();
    else if (strcmp(argv[1], "missing_issuer") == 0) invalid_session_boot(true);
    else if (strcmp(argv[1], "missing_deadline") == 0) invalid_session_boot(false);
    else if (strcmp(argv[1], "mismatch") == 0) mismatch();
    else if (strcmp(argv[1], "duplicate") == 0) duplicate();
    else if (strcmp(argv[1], "freshness") == 0) freshness();
    else if (strcmp(argv[1], "recovery") == 0) recovery();
    else if (strcmp(argv[1], "deadline") == 0) deadline();
    else if (strcmp(argv[1], "clock_regression") == 0) clock_regression();
    else if (strcmp(argv[1], "invalid_profile") == 0) invalid_profile();
    else if (strcmp(argv[1], "exhausted") == 0) exhausted();
    else if (strcmp(argv[1], "reused_session") == 0) reused_session();
    else if (strcmp(argv[1], "near_clock_limit") == 0) near_clock_limit();
    else {
        fprintf(stderr, "Unknown case: %s\n", argv[1]);
        return 2;
    }
    return 0;
}
