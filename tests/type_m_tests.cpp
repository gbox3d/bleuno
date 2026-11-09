#include "type_m_protocol.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

static unsigned checks = 0;
#define CHECK(condition) do { ++checks; if (!(condition)) { \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); exit(1); \
} } while (0)

using namespace type_m;

static Reply command(Controller& controller, const char* text, uint32_t now) {
    controller.tick(now); // The safety task checks expiry before dispatch.
    const Reply reply = controller.execute(parse(text), now, now);
    CHECK(strlen(reply.text) <= kPacketLimit);
    return reply;
}

static void expect(Controller& controller, const char* text, uint32_t now, const char* reply) {
    CHECK(!strcmp(command(controller, text, now).text, reply));
}

static bool off(const Outputs& output) {
    return !output.sleepHigh && !output.motor[0] && !output.motor[1] &&
           !output.motor[2] && !output.motor[3] && !output.servo[0] && !output.servo[1];
}

static void arm(Controller& controller, uint32_t now) {
    controller.link(true);
    controller.subscription(true);
    expect(controller, "ready", now, "ok ready");
    CHECK(off(controller.outputs(now))); // No automatic servo centering or motor wake.
}

static void parserTests() {
    CHECK(parse("about").kind == Kind::About);
    CHECK(parse("ready\n").kind == Kind::Ready);
    CHECK(parse("ping\r\n").kind == Kind::Ping);
    CHECK(parse(" motor 2 -100 ").value == -100);
    CHECK(parse("servo 1 180").value == 180);
    CHECK(parse("servo 2 0").error == Error::None);
    CHECK(parse("motor 1 100").error == Error::None);
    const char* syntax[] = {"", " ", "READY", "motor", "motor 1", "motor 1 2 3", "ping x",
        "motor 1 1x", "motor 1 --1", "motor 1 +1", "motor\t1 2", "ping\nstop", "ping\r",
        "motor 1 1234567890123x", "abcdefghijklmnopqrstu", "ready\n\n"};
    for (const char* text : syntax) CHECK(parse(text).error == Error::Syntax);
    const char* range[] = {"motor 0 0", "motor 3 0", "motor -1 0", "motor 1 101",
        "motor 2 -101", "servo 1 -1", "servo 1 181", "servo 99 90", "motor 1 999999999999"};
    for (const char* text : range) CHECK(parse(text).error == Error::Range);
    const uint8_t nul[] = {'s', 't', 'o', 'p', 0, 'x'};
    CHECK(parse(nul, sizeof(nul)).error == Error::Syntax);
    CHECK(parse(nullptr, 0).error == Error::Syntax);
    for (unsigned byte = 0; byte < 256; ++byte) {
        const uint8_t raw[] = {static_cast<uint8_t>(byte)};
        CHECK(parse(raw, 1).error == Error::Syntax);
    }
}

static void sessionTests() {
    Controller state;
    CHECK(off(state.outputs(0)));
    expect(state, "about", 0, kAbout);
    expect(state, "motor 1 10", 0, "err not_ready");
    state.link(true);
    expect(state, "ready", 10, "err subscribe");
    state.subscription(true);
    expect(state, "motor 1 10", 10, "err not_ready");
    expect(state, "ping", 10, "err not_ready");
    expect(state, "ready", 10, "ok ready");
    CHECK(off(state.outputs(10)));
    expect(state, "motor 1 100", 20, "ok motor 1 100");
    expect(state, "servo 2 180", 20, "ok servo 2 180");
    CHECK(state.outputs(20).sleepHigh);
    CHECK(state.outputs(20 + kWakeMs).motor[0] == 255);
    CHECK(state.outputs(20 + kWakeMs).servo[1] == 1638);
    const uint32_t beforeStop = state.epoch;
    expect(state, "stop", 30, "ok stop");
    CHECK(state.ready && state.epoch != beforeStop);
    CHECK(off(state.outputs(30)));
    expect(state, "servo 1 0", 31, "ok servo 1 0");
    CHECK(state.outputs(31).servo[0] == 819);
    state.subscription(false);
    CHECK(!state.ready && off(state.outputs(32)));
    state.subscription(true);
    expect(state, "servo 1 0", 33, "err not_ready");
    expect(state, "ready", 34, "ok ready");
    CHECK(off(state.outputs(34)));
    expect(state, "motor 2 -50", 35, "ok motor 2 -50");
    state.outputs(35);
    CHECK(state.outputs(35 + kWakeMs).motor[3] == 128);
    const uint32_t oldConnectionEpoch = state.epoch;
    state.link(false);
    CHECK(!state.ready && !state.subscribed && off(state.outputs(40)));
    state.link(true);
    CHECK(state.epoch != oldConnectionEpoch && !state.ready && !state.subscribed);
    CHECK(off(state.outputs(41)));
}

static void watchdogTests() {
    Controller state;
    arm(state, 100);
    expect(state, "servo 1 90", 101, "ok servo 1 90");
    CHECK(state.outputs(101).servo[0] == 1229);
    CHECK(!state.tick(1600));
    CHECK(state.tick(1601));
    CHECK(!state.ready && off(state.outputs(1601)));
    CHECK(!state.tick(1602));
    expect(state, "ping", 1602, "err not_ready");
    expect(state, "ready", 1700, "ok ready");
    expect(state, "ping", 3000, "ok ping");
    CHECK(!state.tick(4499));
    CHECK(state.tick(4500));
    arm(state, 5000);
    expect(state, "motor 1 101", 5100, "err range");
    expect(state, "about", 5200, kAbout);
    CHECK(state.heartbeatAt == 5000); // Invalid and informational reads do not feed it.
    CHECK(state.tick(6500));
    arm(state, 7000);
    const Reply delayed = state.execute(parse("motor 1 100"), 7001, 8501);
    CHECK(!strcmp(delayed.text, "err not_ready"));
    CHECK(!state.ready && off(state.outputs(8501)));
    arm(state, UINT32_MAX - 100);
    CHECK(!state.tick(1398));
    CHECK(state.tick(1399)); // Unsigned millis wrap is safe.
    CHECK(off(state.outputs(1399)));
}

static void motorTests() {
    Controller state;
    arm(state, 0);
    expect(state, "motor 1 100", 1, "ok motor 1 100");
    Outputs out = state.outputs(1);
    CHECK(out.sleepHigh && !out.motor[0] && !out.motor[1]);
    CHECK(!state.outputs(1 + kWakeMs - 1).motor[0]);
    out = state.outputs(1 + kWakeMs);
    CHECK(out.motor[0] == 255 && !out.motor[1]);
    expect(state, "motor 1 -100", 10, "ok motor 1 -100");
    out = state.outputs(10);
    CHECK(out.sleepHigh && !out.motor[0] && !out.motor[1]);
    CHECK(!state.outputs(10 + kDirectionGapMs - 1).motor[1]);
    out = state.outputs(10 + kDirectionGapMs);
    CHECK(!out.motor[0] && out.motor[1] == 255);
    expect(state, "motor 2 25", 20, "ok motor 2 25");
    CHECK(state.outputs(20).motor[2] == 64);
    expect(state, "motor 1 0", 21, "ok motor 1 0");
    out = state.outputs(21);
    CHECK(out.sleepHigh && !out.motor[0] && !out.motor[1] && out.motor[2] == 64);
    expect(state, "motor 1 100", 21, "ok motor 1 100");
    CHECK(!state.outputs(21).motor[0]); // A brief zero does not bypass reversal delay.
    CHECK(state.outputs(21 + kDirectionGapMs).motor[0] == 255);
    expect(state, "stop", 30, "ok stop");
    CHECK(off(state.outputs(30)));
    expect(state, "motor 2 -100", 31, "ok motor 2 -100");
    CHECK(!state.outputs(31).motor[3]); // Sleep after STOP requires wake delay again.
    CHECK(state.outputs(31 + kWakeMs).motor[3] == 255);
    state.disarm();
    CHECK(off(state.outputs(40)));
}

static void sensorParserTests() {
    CHECK(parse("dht").kind == Kind::Dht && parse("dht").value == 0);
    CHECK(parse("dht 11").error == Error::None && parse("dht 11").value == 11);
    CHECK(parse("dht 22\r\n").error == Error::None && parse("dht 22\r\n").value == 22);
    const char* syntax[] = {"dht 11 22", "dht x", "dht +11", "dht 1.1", "dht\nready", "DHT", "dht 22x"};
    for (const char* text : syntax) CHECK(parse(text).error == Error::Syntax);
    const char* range[] = {"dht 0", "dht -11", "dht 12", "dht 21", "dht 9999999999999999"};
    for (const char* text : range) CHECK(parse(text).error == Error::Range);
}

static void sensorCacheTests() {
    DhtCache cache;
    DhtCache::ReadToken token;
    CHECK(cache.model == 11);
    CHECK(!strcmp(cache.reply(0).text, "dht 11 wait"));
    CHECK(!cache.beginRead(2499, token));
    CHECK(cache.beginRead(2500, token));
    CHECK(!cache.beginRead(5000, token)); // Only one physical read may be pending.
    CHECK(cache.complete(token, true, 253, 612, 2510));
    CHECK(!strcmp(cache.reply(2510).text, "dht 11 253 612"));
    CHECK(!strcmp(cache.reply(7509).text, "dht 11 253 612"));
    CHECK(!strcmp(cache.reply(7510).text, "dht 11 error")); // No stale numeric display.
    CHECK(!cache.beginRead(4999, token));
    CHECK(cache.beginRead(5000, token));
    CHECK(cache.complete(token, false, 253, 612, 5010));
    CHECK(!strcmp(cache.reply(5010).text, "dht 11 error"));
    CHECK(cache.temperature10 == 0 && cache.humidity10 == 0);
    CHECK(cache.beginRead(7500, token));
    const DhtCache::ReadToken oldToken = token;
    cache.select(22, 7501);
    CHECK(!strcmp(cache.reply(7501).text, "dht 22 wait"));
    CHECK(!cache.complete(oldToken, true, 260, 500, 7502));
    CHECK(!cache.beginRead(10000, token));
    CHECK(cache.beginRead(10001, token));
    CHECK(cache.complete(token, true, -400, 1000, 10010));
    CHECK(!strcmp(cache.reply(10010).text, "dht 22 -400 1000"));
    CHECK(strlen(cache.reply(10010).text) <= kPacketLimit);
    const uint32_t generation = cache.generation;
    cache.select(22, 10011);
    CHECK(cache.generation == generation); // Re-selecting an unchanged model is harmless.
    CHECK(!strcmp(cache.reply(10011).text, "dht 22 -400 1000"));
    CHECK(cache.beginRead(12501, token));
    CHECK(cache.complete(token, true, 800, 0, 12510));
    CHECK(!strcmp(cache.reply(12510).text, "dht 22 800 0"));
    const int invalid[][2] = {{-401, 500}, {801, 500}, {200, -1}, {200, 1001}};
    uint32_t at = 15001;
    for (const auto& sample : invalid) {
        CHECK(cache.beginRead(at, token));
        CHECK(cache.complete(token, true, sample[0], sample[1], at + 10));
        CHECK(!strcmp(cache.reply(at + 10).text, "dht 22 error"));
        at += kDhtIntervalMs;
    }
    cache.select(11, UINT32_MAX - 100);
    CHECK(!cache.beginRead(2398, token));
    CHECK(cache.beginRead(2399, token)); // millis wrap preserves the minimum interval.
    CHECK(cache.complete(token, true, 200, 500, 2400));
    CHECK(!strcmp(cache.reply(2400).text, "dht 11 200 500"));
    cache.select(22, 2500);
    CHECK(!strcmp(cache.reply(7499).text, "dht 22 wait"));
    CHECK(!strcmp(cache.reply(7500).text, "dht 22 error")); // A stalled first read cannot wait forever.
}

static void sensorSessionTests() {
    Controller state;
    expect(state, "dht", 0, "err not_ready");
    state.link(true);
    expect(state, "dht 22", 1, "err subscribe");
    state.subscription(true);
    expect(state, "dht 22", 2, "err not_ready");
    CHECK(state.dht.model == 11);
    expect(state, "ready", 3, "ok ready");
    expect(state, "dht", 4, "dht 11 wait");
    expect(state, "dht 22", 5, "ok dht 22");
    expect(state, "dht", 6, "dht 22 wait");
    CHECK(state.heartbeatAt == 3); // Sensor traffic does not keep actuator control armed.
    expect(state, "motor 1 100", 7, "ok motor 1 100");
    state.outputs(7);
    CHECK(state.outputs(7 + kWakeMs).motor[0] == 255);
    expect(state, "dht 12", 8, "err range");
    CHECK(state.dht.model == 22 && state.ready);
    CHECK(state.outputs(8 + kWakeMs).motor[0] == 255);
    expect(state, "dht", 1499, "dht 22 wait");
    CHECK(state.heartbeatAt == 7);
    CHECK(state.tick(1507));
    CHECK(off(state.outputs(1507)));
    expect(state, "dht", 1508, "err not_ready");
    expect(state, "ready", 2500, "ok ready");
    DhtCache::ReadToken token;
    CHECK(state.dht.beginRead(2505, token));
    CHECK(state.dht.complete(token, false, 0, 0, 2510));
    expect(state, "dht", 2511, "dht 22 error");
    CHECK(state.ready && state.heartbeatAt == 2500);
    expect(state, "motor 2 0", 2512, "ok motor 2 0");
    expect(state, "stop", 2513, "ok stop");
    CHECK(off(state.outputs(2513)));
}

int main() {
    parserTests();
    sessionTests();
    watchdogTests();
    motorTests();
    const unsigned baseChecks = checks;
    sensorParserTests();
    sensorCacheTests();
    sensorSessionTests();
    printf("type_m: %u checks passed (%u existing safety/parser + %u sensor)\n", checks, baseChecks, checks - baseChecks);
    return 0;
}
