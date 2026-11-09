#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

namespace type_m {

constexpr size_t kPacketLimit = 20;
constexpr uint32_t kHeartbeatMs = 1500;
// Two millis() ticks guarantee at least 1 ms despite clock quantization.
constexpr uint32_t kWakeMs = 2;
// Application-level all-low interval; this is not the DRV8833's internal dead time.
constexpr uint32_t kDirectionGapMs = 2;
constexpr char kAbout[] = "type_m/1 fw=1.1.0";
constexpr uint32_t kDhtIntervalMs = 2500;
constexpr uint32_t kDhtFreshnessMs = 5000;

enum class Kind : uint8_t { About, Ready, Ping, Motor, Servo, Stop, Dht, Invalid };
enum class Error : uint8_t { None, Syntax, Range };

struct Command {
    Kind kind = Kind::Invalid;
    Error error = Error::Syntax;
    uint8_t channel = 0;
    int16_t value = 0;
};

inline bool number(const char* token, int& result) {
    bool negative = *token == '-';
    if (negative) ++token;
    if (*token < '0' || *token > '9') return false;
    int value = 0;
    while (*token) {
        if (*token < '0' || *token > '9') return false;
        // Saturate rather than overflow, while continuing to validate the token.
        value = value > 10000 ? 10001 : value * 10 + (*token - '0');
        ++token;
    }
    result = negative ? -value : value;
    return true;
}

inline Command parse(const uint8_t* bytes, size_t length) {
    Command result;
    if (!bytes || !length || length > kPacketLimit) return result;
    // One optional LF or CRLF terminator; embedded newlines are rejected.
    if (bytes[length - 1] == '\n') {
        --length;
        if (length && bytes[length - 1] == '\r') --length;
    }
    if (!length) return result;
    char buffer[kPacketLimit + 1];
    for (size_t i = 0; i < length; ++i) {
        if (bytes[i] < 32 || bytes[i] > 126) return result;
        buffer[i] = static_cast<char>(bytes[i]);
    }
    buffer[length] = 0;
    char* tokens[4] = {};
    size_t count = 0;
    char* cursor = buffer;
    while (*cursor) {
        while (*cursor == ' ') ++cursor;
        if (!*cursor) break;
        if (count == 4) return result;
        tokens[count++] = cursor;
        while (*cursor && *cursor != ' ') ++cursor;
        if (*cursor) *cursor++ = 0;
    }
    if (!count) return result;
    if (count == 1) {
        if (!strcmp(tokens[0], "about")) result.kind = Kind::About;
        else if (!strcmp(tokens[0], "ready")) result.kind = Kind::Ready;
        else if (!strcmp(tokens[0], "ping")) result.kind = Kind::Ping;
        else if (!strcmp(tokens[0], "stop")) result.kind = Kind::Stop;
        else if (!strcmp(tokens[0], "dht")) result.kind = Kind::Dht;
        else return result;
        result.error = Error::None;
        return result;
    }
    if (count == 2 && !strcmp(tokens[0], "dht")) {
        int model;
        if (!number(tokens[1], model)) return result;
        if (model != 11 && model != 22) {
            result.error = Error::Range;
            return result;
        }
        result.kind = Kind::Dht;
        result.value = static_cast<int16_t>(model);
        result.error = Error::None;
        return result;
    }
    if (count != 3) return result;
    if (!strcmp(tokens[0], "motor")) result.kind = Kind::Motor;
    else if (!strcmp(tokens[0], "servo")) result.kind = Kind::Servo;
    else return result;
    int channel, value;
    if (!number(tokens[1], channel) || !number(tokens[2], value)) return result;
    if (channel < 1 || channel > 2 ||
        (result.kind == Kind::Motor && (value < -100 || value > 100)) ||
        (result.kind == Kind::Servo && (value < 0 || value > 180))) {
        result.error = Error::Range;
        return result;
    }
    result.channel = static_cast<uint8_t>(channel);
    result.value = static_cast<int16_t>(value);
    result.error = Error::None;
    return result;
}

inline Command parse(const char* text) {
    return parse(reinterpret_cast<const uint8_t*>(text), strlen(text));
}

struct Reply {
    char text[kPacketLimit + 1] = {};
    explicit Reply(const char* message = "") {
        snprintf(text, sizeof(text), "%s", message);
    }
};

// Pure cached sensor state. All calls are serialized by the runtime's short
// state mutex; the physical DHT read always happens outside that mutex.
class DhtCache {
public:
    enum class State : uint8_t { Waiting, Valid, Error };
    struct ReadToken {
        uint32_t generation = 0;
        uint8_t model = 11;
    };

    uint8_t model = 11;
    uint32_t generation = 0;
    State state = State::Waiting;
    int16_t temperature10 = 0;
    int16_t humidity10 = 0;

    void select(uint8_t selected, uint32_t now) {
        if (selected == model) return;
        model = selected;
        ++generation;
        state = State::Waiting;
        temperature10 = humidity10 = 0;
        selectedAt_ = lastAttemptAt_ = now;
        reading_ = false;
    }

    bool beginRead(uint32_t now, ReadToken& token) {
        if (reading_ || static_cast<uint32_t>(now - lastAttemptAt_) < kDhtIntervalMs) return false;
        lastAttemptAt_ = now;
        reading_ = true;
        token.generation = generation;
        token.model = model;
        return true;
    }

    bool complete(const ReadToken& token, bool valid, int temperature, int humidity, uint32_t now) {
        // A model switch invalidates even a result already being acquired.
        if (!reading_ || token.generation != generation || token.model != model) return false;
        reading_ = false;
        const bool inRange = temperature >= -400 && temperature <= 800 && humidity >= 0 && humidity <= 1000;
        state = valid && inRange ? State::Valid : State::Error;
        temperature10 = state == State::Valid ? temperature : 0;
        humidity10 = state == State::Valid ? humidity : 0;
        sampledAt_ = now;
        return true;
    }

    Reply reply(uint32_t now) const {
        Reply response;
        if (state == State::Valid && static_cast<uint32_t>(now - sampledAt_) < kDhtFreshnessMs) {
            snprintf(response.text, sizeof(response.text), "dht %u %d %d", model, temperature10, humidity10);
        } else {
            const bool waiting = state == State::Waiting &&
                static_cast<uint32_t>(now - selectedAt_) < kDhtFreshnessMs;
            snprintf(response.text, sizeof(response.text), "dht %u %s", model, waiting ? "wait" : "error");
        }
        return response;
    }

private:
    uint32_t selectedAt_ = 0;
    uint32_t lastAttemptAt_ = 0;
    uint32_t sampledAt_ = 0;
    bool reading_ = false;
};

struct Outputs {
    uint16_t motor[4] = {};
    uint16_t servo[2] = {};
    bool sleepHigh = false;
};

class Controller {
public:
    bool connected = false;
    bool subscribed = false;
    bool ready = false;
    uint32_t epoch = 0;
    uint32_t heartbeatAt = 0;
    int16_t motorTarget[2] = {};
    int16_t servoAngle[2] = {-1, -1};
    DhtCache dht;

    void link(bool active) {
        connected = active;
        subscribed = false;
        disarm();
    }

    void subscription(bool enabled) {
        subscribed = connected && enabled;
        if (!subscribed) disarm();
    }

    void disarm() {
        ready = false;
        ++epoch;
        clearOutputs();
    }

    // stop cancels queued pre-stop commands but keeps an existing READY session.
    void stop(uint32_t receivedAt) {
        ++epoch;
        clearOutputs();
        if (ready) heartbeatAt = receivedAt;
    }

    bool tick(uint32_t now) {
        if (ready && static_cast<uint32_t>(now - heartbeatAt) >= kHeartbeatMs) {
            disarm();
            return true;
        }
        return false;
    }

    Reply execute(const Command& command, uint32_t receivedAt, uint32_t now) {
        if (command.error != Error::None)
            return Reply(command.error == Error::Range ? "err range" : "err syntax");
        if (command.kind == Kind::About) return Reply(kAbout);
        if (command.kind == Kind::Stop) {
            stop(receivedAt);
            return Reply("ok stop");
        }
        if (!connected) return Reply("err not_ready");
        if (!subscribed) return Reply("err subscribe");
        if (static_cast<uint32_t>(now - receivedAt) >= kHeartbeatMs) {
            disarm();
            return Reply("err not_ready");
        }
        if (command.kind == Kind::Ready) {
            clearOutputs();
            ready = true;
            heartbeatAt = receivedAt;
            return Reply("ok ready");
        }
        if (!ready) return Reply("err not_ready");
        // Sensor polling/configuration does not feed the actuator heartbeat.
        if (command.kind == Kind::Dht) {
            if (!command.value) return dht.reply(now);
            dht.select(static_cast<uint8_t>(command.value), now);
            Reply response;
            snprintf(response.text, sizeof(response.text), "ok dht %d", command.value);
            return response;
        }
        heartbeatAt = receivedAt;
        if (command.kind == Kind::Ping) return Reply("ok ping");
        Reply response;
        if (command.kind == Kind::Motor) {
            motorTarget[command.channel - 1] = command.value;
            snprintf(response.text, sizeof(response.text), "ok motor %u %d", command.channel, command.value);
        } else if (command.kind == Kind::Servo) {
            servoAngle[command.channel - 1] = command.value;
            snprintf(response.text, sizeof(response.text), "ok servo %u %d", command.channel, command.value);
        } else {
            return Reply("err syntax");
        }
        return response;
    }

    Outputs outputs(uint32_t now, uint16_t servoMinUs = 1000, uint16_t servoMaxUs = 2000) {
        Outputs out;
        if (!ready || !connected || !subscribed) return out;
        for (unsigned i = 0; i < 2; ++i) {
            if (servoAngle[i] >= 0) {
                const uint32_t pulse = servoMinUs +
                    (static_cast<uint32_t>(servoAngle[i]) * (servoMaxUs - servoMinUs) + 90) / 180;
                out.servo[i] = static_cast<uint16_t>((pulse * 16384UL + 10000) / 20000);
            }
        }
        if (!motorTarget[0] && !motorTarget[1]) {
            driverAwake_ = false;
            motors_[0] = MotorState{};
            motors_[1] = MotorState{};
            return out;
        }
        out.sleepHigh = true;
        if (!driverAwake_) {
            driverAwake_ = true;
            wakeStarted_ = now;
        }
        if (static_cast<uint32_t>(now - wakeStarted_) < kWakeMs) return out;
        for (unsigned i = 0; i < 2; ++i) {
            MotorState& state = motors_[i];
            const int target = motorTarget[i];
            const int sign = (target > 0) - (target < 0);
            if (!sign) {
                if (state.driven) state.zeroStarted = now;
                state.driven = false;
                continue;
            }
            if (state.lastSign && sign != state.lastSign) {
                if (state.driven) {
                    state.driven = false;
                    state.zeroStarted = now;
                }
                if (static_cast<uint32_t>(now - state.zeroStarted) < kDirectionGapMs) continue;
            }
            state.lastSign = sign;
            state.driven = true;
            const unsigned magnitude = target > 0 ? target : -target;
            const unsigned duty = (magnitude * 255 + 50) / 100;
            out.motor[i * 2 + (sign < 0 ? 1 : 0)] = static_cast<uint16_t>(duty);
        }
        return out;
    }

private:
    struct MotorState {
        int lastSign = 0;
        bool driven = false;
        uint32_t zeroStarted = 0;
    } motors_[2];
    bool driverAwake_ = false;
    uint32_t wakeStarted_ = 0;

    void clearOutputs() {
        motorTarget[0] = motorTarget[1] = 0;
        servoAngle[0] = servoAngle[1] = -1;
        motors_[0] = MotorState{};
        motors_[1] = MotorState{};
        driverAwake_ = false;
    }
};

}  // namespace type_m
