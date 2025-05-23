#include <Arduino.h>

// #include <WiFi.h>
#include <vector>

#include <TaskScheduler.h>
#include <ArduinoJson.h>

#include <DHT.h>

#include "config.hpp"
#include "etc.hpp"

//version string
String __version__ = "1.0.4";

Scheduler g_ts;
Config g_config;

std::vector<int> ledPins;

struct PwmChannelInfo {
    int pin;
    int channel;
};

std::vector<PwmChannelInfo> pwmChannels;


int8_t systemMode = 0;
int dht11Pin = -1;
uint32_t safeBackDelay;
uint32_t safeBackTimeCheck = 0;
Task task_SafeBackCheck(1000,TASK_FOREVER,NULL,&g_ts,false);

#define DHTTYPE DHT11
DHT *dht = nullptr;
float temperature = 0.0;
float humidity = 0.0;
Task task_ReadDHT(2000, TASK_FOREVER, NULL, &g_ts, false);

extern String parseCmd(String _strLine);
extern void ble_setup(String strDeviceName);
extern bool deviceConnected;


#if not defined(BUILTIN_LED)
    #if defined(WROVER_KIT)
    #define BUILTIN_LED 5
    #elif defined(SEED_XIAO_ESP32C3) | defined(BEETLE_C3)
    #define BUILTIN_LED D10
    #elif defined(D32Lite)
    #define BUILTIN_LED 22
    #elif defined(SEED_XIAO_ESP32C3)
    #define BUILTIN_LED D10
    #endif
#endif

// #if defined(BEETLE_C3)
// #elif defined(LOLIN_D32) | defined(LOLIN_D32_PRO) | defined(WROVER_KIT)
// #elif defined(SEED_XIAO_ESP32C3)
// #define BUILTIN_LED D10
// #elif defined(D32Lite)
// #define BUILTIN_LED 22
// #else
// #endif

// #if defined(WROVER_KIT) | defined(WROOM32)
// #define BUILTIN_LED 5
// #endif



Task task_LedBlink(500, TASK_FOREVER, []()
                   {
#if defined(BUILTIN_LED)
                       if (deviceConnected)
                       {
                           digitalWrite(BUILTIN_LED, LOW);
                           Serial.println("BUILTIN_LED : " + String(BUILTIN_LED) + " " + String(digitalRead(BUILTIN_LED)));
                       }
                       else
                       {
                           // Serial.println("BUILTIN_LED : " + String(BUILTIN_LED) + " " + String(digitalRead(BUILTIN_LED)));
                           digitalWrite(BUILTIN_LED, !digitalRead(BUILTIN_LED));
                       }
#endif
                   },
                   &g_ts, true);

void startBlink()
{
    task_LedBlink.enable();
}

void stopBlink()
{
    task_LedBlink.disable();
    if (deviceConnected)
    {
        digitalWrite(BUILTIN_LED, LOW);
        Serial.println("BUILTIN_LED : " + String(BUILTIN_LED) + " " + String(digitalRead(BUILTIN_LED)));
    }
    else
    {
        digitalWrite(BUILTIN_LED, HIGH);
    }
}


void ledControl(int pinIndex,uint8_t value)
{
    if (pinIndex == -1)
    {
        for (int i = 0; i < ledPins.size(); i++)
        {
            int pin = ledPins[i];
            digitalWrite(pin, value); // turn the LED on (HIGH is the voltage level)
        }
    }
    else
    {
        int pin = ledPins[pinIndex];
        digitalWrite(pin, value); // turn the LED on (HIGH is the voltage level)
    }
}


void ledOn(int pinIndex)
{
    if (pinIndex == -1)
    {
        for (int i = 0; i < ledPins.size(); i++)
        {
            int pin = ledPins[i];
            digitalWrite(pin, HIGH); // turn the LED on (HIGH is the voltage level)
        }
    }
    else
    {
        int pin = ledPins[pinIndex];
        digitalWrite(pin, HIGH); // turn the LED on (HIGH is the voltage level)
    }
}

void ledOff(int pinIndex)
{
    if (pinIndex == -1)
    {
        for (int i = 0; i < ledPins.size(); i++)
        {
            int pin = ledPins[i];
            digitalWrite(pin, LOW); // turn the LED on (HIGH is the voltage level)
        }
    }
    else
    {
        int pin = ledPins[pinIndex];
        digitalWrite(pin, LOW); // turn the LED on (HIGH is the voltage level)
    }
}

int pwmLed(int pinIndex, int dutyCycle)
{
    if(pinIndex < 0) {
        //all pwm pins set
        for (int i = 0; i < pwmChannels.size(); i++)
        {
            int channel = pwmChannels[i].channel;
            ledcWrite(channel, dutyCycle);
        }

        safeBackTimeCheck = millis(); // current time

        return 0;
    }

    if(pinIndex >= pwmChannels.size()) {
        return 2;
    }

    //8bit duty cycle
    if (dutyCycle < 0 || dutyCycle > 255)
    {
        return 1;
    }

    int channel = pwmChannels[pinIndex].channel;
    ledcWrite(channel, dutyCycle);
    return 0;
}


void clearAllControl()
{
    for (int i = 0; i < ledPins.size(); i++)
    {
        int pin = ledPins[i];
        digitalWrite(pin, LOW); // turn the LED on (HIGH is the voltage level)
    }

    for (int i = 0; i < pwmChannels.size(); i++)
    {
        int channel = pwmChannels[i].channel;
        ledcWrite(channel, 0);
    }
}

Task task_Cmd(100, TASK_FOREVER, []()
              {
    if (Serial.available() > 0)
    {
        String _strLine = Serial.readStringUntil('\n');
        _strLine.trim();
        
        // Serial.println(_strLine);

        Serial.println(parseCmd(_strLine));
    } }, &g_ts, true);


// the setup function runs once when you press reset or power the board
void setup()
{

    String strDeviceName = "ESP32_BLE" + String(getChipID().c_str());

// initialize digital pin BUILTIN_LED as an output.
#if defined(BUILTIN_LED)
    pinMode(BUILTIN_LED, OUTPUT);
    digitalWrite(BUILTIN_LED, HIGH); // turn the LED off by making the voltage LOW
#endif

    Serial.begin(115200);

    g_config.load();

    delay(250);

    // initialize digital pin BUILTIN_LED as an output.

    // LED pins
    {
        JsonDocument _doc_ledpins;
        g_config.getArray("ledpin", _doc_ledpins);

        JsonArray ledpin = _doc_ledpins.as<JsonArray>();

        int _index = 0;
        for (JsonVariant v : ledpin)
        {

            int pin = v.as<int>();

            pinMode(pin, OUTPUT);
            // digitalWrite(pin, HIGH); // turn the LED off by making the voltage LOW
            digitalWrite(pin, LOW); // turn the LED on (HIGH is the voltage level)
            ledPins.push_back(pin);

            Serial.printf("%2d led pin : %d\n", _index, pin);

            _index++;
        }
    }

    // PWM pins
    {
        JsonDocument _pins;
        g_config.getArray("pwmpin", _pins);

        JsonArray _pinArray = _pins.as<JsonArray>();
        int _index = 0;

        const int pwmFrequency = 5000; // PWM 주파수 (Hz)
        const int pwmResolution = 8;  // PWM 해상도 (비트)
        int pwmChannel = 0;           // 채널 시작 번호

        for (JsonVariant v : _pinArray)
        {
            int pin = v.as<int>();

            if (pwmChannel < 16) // ESP32는 최대 16개의 PWM 채널 지원
            {
                // PWM 채널 초기화
                ledcSetup(pwmChannel, pwmFrequency, pwmResolution);

                // PWM 핀과 채널 연결
                // ledcAttachPin(pin, pwmChannel);
                ledcAttachPin(pin, pwmChannel);
                pwmChannels.push_back({pin, pwmChannel});
                // pwmPins.push_back(pin);

                Serial.printf("%2d pwm pin : %d, channel: %d\n", _index, pin, pwmChannel);

                pwmChannel++; // 다음 채널로 증가
                _index++;
            }
            else
            {
                Serial.println("Maximum PWM channels exceeded (16 channels max)");
                break; // 더 이상 설정할 수 없음
            }
        }
    }

    // load system mode
    systemMode = g_config.get<int>("mode", 0);
    Serial.println("system mode : " + String(systemMode));

    // load dht11 sensor pin
    dht11Pin = g_config.get<int>("dht11pin", -1);

    Serial.println("dht11 pin : " + String(dht11Pin));

    if (dht11Pin != -1)
    {
        dht = new DHT(dht11Pin, DHTTYPE);
        dht->begin();

        task_ReadDHT.set(2000, TASK_FOREVER, []()
                         {
                             // Read temperature and humidity
                             float h = dht->readHumidity();
                             float t = dht->readTemperature();
                             // Check if any reads failed and exit early (to try again)
                             if (isnan(h) || isnan(t))
                             {
                                 // Serial.println("Failed to read from DHT sensor!");
                                 return;
                             }
                             // Store readings in global variables
                             temperature = t;
                             humidity = h;
                             // Serial.printf("Temperature: %.1f°C Humidity: %.1f%%\n", temperature, humidity);
                         });

        task_ReadDHT.enable(); // Enable the task
    }

    // safe back delay
    //제어중 일ㅓ시간 입력이 없으면 안전하게 모든 핀을 초기화
    safeBackDelay = g_config.get<uint32_t>("safebackdelay", 0);

    if(safeBackDelay > 0) {
        Serial.println("safe back delay : " + String(safeBackDelay));

        task_SafeBackCheck.set(safeBackDelay, TASK_FOREVER, []()
        {
            // Serial.printf("safe back delay : %d\n", millis() - safeBackTimeCheck);
            if(millis() - safeBackTimeCheck > (safeBackDelay/2)) {
                clearAllControl();
            }
        });
        task_SafeBackCheck.enable();

        // Task task_CheckTimeOut(500, TASK_FOREVER, []()
        // {
        //     Serial.printf("safe back delay : %d\n", millis() - safeBackTimeCheck);
        //     if(millis() - safeBackTimeCheck > (safeBackDelay/2)) {
        //         clearAllControl();
        //     }
        // }, &g_ts, true);
    }

    Serial.println(":-]");
    Serial.println("Serial connected");
    Serial.println("led built-in : " + String(BUILTIN_LED));

    Serial.printf("Free heap: %d\n", ESP.getFreeHeap());

    // BLE setup
    ble_setup(strDeviceName);

    g_ts.startNow();
}

// the loop function runs over and over again forever
void loop()
{
    g_ts.execute();
}