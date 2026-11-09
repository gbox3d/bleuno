#if defined(TYPE_M_FIRMWARE)

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <DHT.h>
#include <math.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <esp_gatts_api.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include "type_m_protocol.hpp"

#ifndef TYPE_M_RGB_PIN
#define TYPE_M_RGB_PIN 8
#endif
#ifndef TYPE_M_SERVO_MIN_US
#define TYPE_M_SERVO_MIN_US 1000
#endif
#ifndef TYPE_M_SERVO_MAX_US
#define TYPE_M_SERVO_MAX_US 2000
#endif
static_assert(TYPE_M_SERVO_MIN_US >= 500 && TYPE_M_SERVO_MIN_US < TYPE_M_SERVO_MAX_US &&
              TYPE_M_SERVO_MAX_US <= 2500, "Invalid servo pulse range");
static_assert(TYPE_M_RGB_PIN != 0 && TYPE_M_RGB_PIN != 1 && TYPE_M_RGB_PIN != 3 && TYPE_M_RGB_PIN != 4 &&
              TYPE_M_RGB_PIN != 5 && TYPE_M_RGB_PIN != 6 && TYPE_M_RGB_PIN != 7 &&
              TYPE_M_RGB_PIN != 10 && TYPE_M_RGB_PIN != 18 && TYPE_M_RGB_PIN != 19 && TYPE_M_RGB_PIN != 20,
              "RGB pin conflicts with type_m outputs, BAT ADC, USB or DHT");

namespace {
constexpr char kServiceUuid[] = "c6f8b088-2af8-4388-8364-ca2a907bdeb8";
constexpr char kCharacteristicUuid[] = "f6aa83ca-de53-46b4-bdea-28a7cb57942e";
constexpr uint8_t kMotorPins[] = {1, 3, 4, 5};
constexpr uint8_t kServoPins[] = {6, 10};
constexpr uint8_t kSleepPin = 7;
constexpr uint8_t kDhtPin = 20; // type_m/v1: jumper J7 SEN1 to U1 pad 15 (silkscreen 20).
constexpr unsigned kQueueDepth = 12;

struct Request {
    type_m::Command command;
    uint32_t epoch;
    uint32_t receivedAt;
};
struct Notification {
    char text[type_m::kPacketLimit + 1];
    uint32_t epoch;
};

SemaphoreHandle_t stateMutex;
QueueHandle_t requests;
QueueHandle_t notifications;
type_m::Controller controller;
type_m::Outputs applied;
BLEServer* server;
BLECharacteristic* characteristic;
BLE2902* cccd;
esp_gatt_if_t activeInterface = ESP_GATT_IF_NONE;
esp_gatt_if_t registeredInterface = ESP_GATT_IF_NONE;
uint16_t activeConnection = 0;
bool restartAdvertising = false;
char lastResponse[type_m::kPacketLimit + 1] = "type_m/1 fw=1.1.0";
Adafruit_NeoPixel pixel(1, TYPE_M_RGB_PIN, NEO_GRB + NEO_KHZ800);

struct Guard {
    Guard() { xSemaphoreTake(stateMutex, portMAX_DELAY); }
    ~Guard() { xSemaphoreGive(stateMutex); }
};

// Only short GPIO/LEDC operations run with stateMutex held. USB, BLE sends and
// RGB rendering never hold it, so they cannot delay the dedicated safety task.
void applyOutputs(const type_m::Outputs& output) {
    if (!output.sleepHigh) digitalWrite(kSleepPin, LOW);
    for (unsigned i = 0; i < 4; ++i)
        if (applied.motor[i] != output.motor[i]) ledcWrite(i, output.motor[i]);
    for (unsigned i = 0; i < 2; ++i)
        if (applied.servo[i] != output.servo[i]) ledcWrite(i + 4, output.servo[i]);
    if (output.sleepHigh && !applied.sleepHigh) digitalWrite(kSleepPin, HIGH);
    applied = output;
}

void remember(const char* text) { snprintf(lastResponse, sizeof(lastResponse), "%s", text); }

void queueReply(const char* text, uint32_t epoch) {
    Notification response{};
    {
        Guard guard;
        if (epoch != controller.epoch) return;
        remember(text);
        if (!controller.connected || !controller.subscribed) return;
        snprintf(response.text, sizeof(response.text), "%s", text);
        response.epoch = epoch;
    }
    if (xQueueSend(notifications, &response, 0) != pdTRUE) {
        Guard guard;
        controller.disarm();
        applyOutputs({});
        remember("err overflow");
    }
}

void enqueueCommand(const uint8_t* bytes, size_t length, uint16_t connection, bool fragmented) {
    Request request{};
    request.command = fragmented ? type_m::Command{} : type_m::parse(bytes, length);
    request.receivedAt = millis();
    bool stopped = false;
    {
        Guard guard;
        if (!controller.connected || connection != activeConnection) return;
        if (request.command.error == type_m::Error::None && request.command.kind == type_m::Kind::Stop) {
            controller.tick(request.receivedAt);
            controller.stop(request.receivedAt);
            applyOutputs({});
            stopped = true;
        }
        request.epoch = controller.epoch;
    }
    // STOP is never blocked behind an RX backlog. Its new epoch invalidates it.
    if (stopped) {
        queueReply("ok stop", request.epoch);
    } else if (xQueueSend(requests, &request, 0) != pdTRUE) {
        uint32_t epoch;
        {
            Guard guard;
            controller.disarm();
            applyOutputs({});
            epoch = controller.epoch;
        }
        queueReply("err overflow", epoch);
    }
}

// Raw GATT events preserve the boundary and size of each write. The Arduino
// characteristic callback hides prepared-write fragments, so it is not used.
// A prepared/offset write is rejected and EXEC_WRITE never executes a command.
void gattsEvent(esp_gatts_cb_event_t event, esp_gatt_if_t interface,
                esp_ble_gatts_cb_param_t* parameter) {
    if (event == ESP_GATTS_REG_EVT && parameter->reg.status == ESP_GATT_OK) {
        registeredInterface = interface; // This firmware registers exactly one GATT server.
        return;
    }
    if (registeredInterface != ESP_GATT_IF_NONE && interface != ESP_GATT_IF_NONE &&
        interface != registeredInterface) return;
    if (event == ESP_GATTS_CONNECT_EVT) {
        bool reject = false;
        {
            Guard guard;
            // A broadcast/repeated event for the same link must not reset or reject it.
            if (controller.connected && parameter->connect.conn_id == activeConnection) return;
            reject = controller.connected;
            if (!reject) {
                activeConnection = parameter->connect.conn_id;
                activeInterface = registeredInterface;
                controller.link(true);
                applyOutputs({});
                restartAdvertising = false;
                remember(type_m::kAbout);
            }
        }
        if (reject) {
            server->disconnect(parameter->connect.conn_id);
        } else if (cccd) {
            cccd->setNotifications(false);
            cccd->setIndications(false);
        }
    } else if (event == ESP_GATTS_DISCONNECT_EVT) {
        Guard guard;
        if (controller.connected && parameter->disconnect.conn_id == activeConnection) {
            controller.link(false);
            applyOutputs({});
            restartAdvertising = true;
            remember(type_m::kAbout);
        }
    } else if (event == ESP_GATTS_WRITE_EVT) {
        const auto& write = parameter->write;
        if (characteristic && write.handle == characteristic->getHandle()) {
            enqueueCommand(write.value, write.len, write.conn_id, write.is_prep || write.offset != 0);
        } else if (cccd && write.handle == cccd->getHandle()) {
            const bool enabled = !write.is_prep && !write.offset && write.len == 2 &&
                                 write.value[0] == 1 && write.value[1] == 0;
            Guard guard;
            if (controller.connected && write.conn_id == activeConnection) {
                controller.subscription(enabled);
                if (!enabled) applyOutputs({});
            }
        }
    }
}

class ReadCallbacks : public BLECharacteristicCallbacks {
    void onRead(BLECharacteristic* value) override {
        char response[sizeof(lastResponse)];
        {
            Guard guard;
            memcpy(response, lastResponse, sizeof(response));
        }
        value->setValue(reinterpret_cast<uint8_t*>(response), strlen(response));
    }
};

void controlTask(void*) {
    for (;;) {
        bool timedOut;
        uint32_t epoch;
        {
            Guard guard;
            const uint32_t now = millis();
            timedOut = controller.tick(now);
            applyOutputs(controller.outputs(now, TYPE_M_SERVO_MIN_US, TYPE_M_SERVO_MAX_US));
            epoch = controller.epoch;
        }
        if (timedOut) queueReply("event timeout", epoch);
        for (unsigned count = 0; count < 4; ++count) {
            Request request;
            if (xQueueReceive(requests, &request, 0) != pdTRUE) break;
            type_m::Reply reply;
            {
                Guard guard;
                if (request.epoch != controller.epoch || !controller.connected) continue;
                const uint32_t now = millis();
                if (controller.tick(now)) reply = type_m::Reply("event timeout");
                else reply = controller.execute(request.command, request.receivedAt, now);
                applyOutputs(controller.outputs(now, TYPE_M_SERVO_MIN_US, TYPE_M_SERVO_MAX_US));
                epoch = controller.epoch;
            }
            queueReply(reply.text, epoch);
        }
        vTaskDelay(pdMS_TO_TICKS(1) ? pdMS_TO_TICKS(1) : 1);
    }
}

void sensorTask(void*) {
    DHT dht11(kDhtPin, DHT11);
    DHT dht22(kDhtPin, DHT22);
    uint32_t configuredGeneration = UINT32_MAX;
    for (;;) {
        type_m::DhtCache::ReadToken token;
        bool due;
        {
            Guard guard;
            due = controller.dht.beginRead(millis(), token);
        }
        if (due) {
            DHT& sensor = token.model == 11 ? dht11 : dht22;
            if (configuredGeneration != token.generation) {
                sensor.begin();
                configuredGeneration = token.generation;
            }
            // Adafruit briefly masks interrupts during pulse acquisition, even
            // on single-core C3. This is NOT a zero-latency/nonblocking driver.
            // Never hold stateMutex across it; a failed read is just cached error.
            const uint32_t readStarted = millis();
            const float temperature = sensor.readTemperature();
            // Skip humidity after failed or >=2 s temperature acquisition: the
            // driver's cache has expired and a second physical read could occur.
            const bool timely = static_cast<uint32_t>(millis() - readStarted) < 2000;
            const float humidity = timely && isfinite(temperature) ? sensor.readHumidity() : NAN;
            const uint32_t sampledAt = millis();
            const bool valid = isfinite(temperature) && isfinite(humidity) &&
                temperature >= -40.0f && temperature <= 80.0f && humidity >= 0.0f && humidity <= 100.0f;
            const int temperature10 = valid ? static_cast<int>(lroundf(temperature * 10.0f)) : 0;
            const int humidity10 = valid ? static_cast<int>(lroundf(humidity * 10.0f)) : 0;
            Guard guard;
            controller.dht.complete(token, valid, temperature10, humidity10, sampledAt);
        }
        vTaskDelay(pdMS_TO_TICKS(25) ? pdMS_TO_TICKS(25) : 1);
    }
}

void transmitOne() {
    Notification response;
    if (xQueueReceive(notifications, &response, 0) != pdTRUE) return;
    uint16_t connection;
    esp_gatt_if_t interface;
    {
        Guard guard;
        if (response.epoch != controller.epoch || !controller.connected || !controller.subscribed) return;
        connection = activeConnection;
        interface = activeInterface;
    }
    // Send an explicit immutable payload; do not race the characteristic's
    // shared value against incoming writes or its READ callback.
    const esp_err_t result = esp_ble_gatts_send_indicate(interface, connection,
        characteristic->getHandle(), strlen(response.text),
        reinterpret_cast<uint8_t*>(response.text), false);
    if (result != ESP_OK) {
        Guard guard;
        if (response.epoch == controller.epoch) {
            controller.disarm();
            applyOutputs({});
            remember("err transport");
        }
    }
}

void updateIndicator() {
    bool linked, ready;
    {
        Guard guard;
        linked = controller.connected;
        ready = controller.ready;
    }
    const uint32_t now = millis();
    const uint32_t color = ready ? pixel.Color(0, 0, 255) : linked ?
        ((now / 250) % 2 ? 0 : pixel.Color(0, 0, 255)) :
        ((now / 500) % 2 ? 0 : pixel.Color(255, 0, 0));
    static uint32_t previous = UINT32_MAX;
    if (color != previous) {
        pixel.setPixelColor(0, color);
        pixel.show();
        previous = color;
    }
}

void printStatus() {
    bool linked, subscribed, ready;
    uint32_t epoch;
    int motor[2], servo[2];
    type_m::Outputs output;
    {
        Guard guard;
        linked = controller.connected;
        subscribed = controller.subscribed;
        ready = controller.ready;
        epoch = controller.epoch;
        for (unsigned i = 0; i < 2; ++i) {
            motor[i] = controller.motorTarget[i];
            servo[i] = controller.servoAngle[i];
        }
        output = applied;
    }
    Serial.printf("status link=%u subscribed=%u ready=%u epoch=%lu motor=%d,%d pwm=%u,%u,%u,%u servo=%d,%d servoDuty=%u,%u sleep=%u\n",
        linked, subscribed, ready, static_cast<unsigned long>(epoch), motor[0], motor[1],
        output.motor[0], output.motor[1], output.motor[2], output.motor[3],
        servo[0], servo[1], output.servo[0], output.servo[1], output.sleepHigh);
}

void serialDiagnostics() {
    static char line[24];
    static size_t length = 0;
    static bool overflow = false;
    for (unsigned count = 0; count < 32 && Serial.available(); ++count) {
        const int value = Serial.read();
        if (value == '\r') continue;
        if (value == '\n') {
            line[length] = 0;
            if (!overflow && !strcmp(line, "about")) Serial.println(type_m::kAbout);
            else if (!overflow && !strcmp(line, "status")) printStatus();
            else Serial.println("err read_only");
            length = 0;
            overflow = false;
        } else if (value < 32 || value > 126 || length == sizeof(line) - 1) {
            overflow = true;
        } else if (!overflow) line[length++] = static_cast<char>(value);
    }
}

[[noreturn]] void fatal(const char* message) {
    digitalWrite(kSleepPin, LOW);
    Serial.println(message);
    for (;;) delay(1000);
}
}  // namespace

void setup() {
    // Assert nSLEEP before any USB/BLE/library initialization.
    pinMode(kSleepPin, OUTPUT);
    digitalWrite(kSleepPin, LOW);
    for (uint8_t pin : kMotorPins) { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }
    for (uint8_t pin : kServoPins) { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }
    Serial.begin(115200);
    for (unsigned i = 0; i < 6; ++i) {
        if (!ledcSetup(i, i < 4 ? 20000 : 50, i < 4 ? 8 : 14)) fatal("error ledc");
        ledcWrite(i, 0);
        ledcAttachPin(i < 4 ? kMotorPins[i] : kServoPins[i - 4], i);
    }
    stateMutex = xSemaphoreCreateMutex();
    requests = xQueueCreate(kQueueDepth, sizeof(Request));
    notifications = xQueueCreate(kQueueDepth, sizeof(Notification));
    if (!stateMutex || !requests || !notifications) fatal("error memory");
    if (xTaskCreate(controlTask, "type_m_safety", 4096, nullptr, 4, nullptr) != pdPASS)
        fatal("error task");
    if (xTaskCreate(sensorTask, "type_m_dht", 4096, nullptr, 1, nullptr) != pdPASS)
        fatal("error sensor task");
    pixel.begin();
    pixel.setBrightness(16);
    pixel.clear();
    pixel.show();
    const uint64_t mac = ESP.getEfuseMac();
    char name[24];
    snprintf(name, sizeof(name), "ESP32_BLE_M_%02X%02X",
        static_cast<unsigned>((mac >> 32) & 0xff), static_cast<unsigned>((mac >> 40) & 0xff));
    BLEDevice::init(name);
    BLEDevice::setCustomGattsHandler(gattsEvent);
    server = BLEDevice::createServer();
    BLEService* service = server->createService(kServiceUuid);
    characteristic = service->createCharacteristic(kCharacteristicUuid,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);
    characteristic->setCallbacks(new ReadCallbacks());
    characteristic->setValue(type_m::kAbout);
    cccd = new BLE2902();
    cccd->setNotifications(false);
    cccd->setIndications(false);
    characteristic->addDescriptor(cccd);
    service->start();
    BLEAdvertising* advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(kServiceUuid);
    advertising->setScanResponse(true);
    BLEDevice::startAdvertising();
    Serial.printf("%s name=%s servoOFF=-1\n", type_m::kAbout, name);
    printStatus();
}

void loop() {
    bool restart;
    {
        Guard guard;
        restart = restartAdvertising;
        restartAdvertising = false;
    }
    if (restart) BLEDevice::startAdvertising();
    transmitOne();
    updateIndicator();
    serialDiagnostics();
    delay(1);
}

#endif  // TYPE_M_FIRMWARE
