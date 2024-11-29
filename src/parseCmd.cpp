#include <Arduino.h>

#include "tonkey.hpp"

#include "config.hpp"

tonkey g_MainParser;

extern Config g_config;

extern void ledOn(int pinIndex);
extern void ledOff(int pinIndex);

extern void startBlink();
extern void stopBlink();

extern float temperature;
extern float humidity;


String parseCmd(String _strLine)
{

    JsonDocument _res_doc;

    g_MainParser.parse(_strLine);

    if (g_MainParser.getTokenCount() > 0)
    {
        String cmd = g_MainParser.getToken(0);
        // JsonDocument _res_doc;

        if (cmd == "about")
        {
            /* code */
            _res_doc["result"] = "ok";
            _res_doc["os"] = "cronos-v1";
            _res_doc["app"] = "BLEuno";
            _res_doc["version"] = "1.0.2";
            _res_doc["author"] = "gbox3d";
// esp8266 chip id
#ifdef ESP8266
            _res_doc["chipid"] = ESP.getChipId();
#elif ESP32
            _res_doc["chipid"] = ESP.getEfuseMac();
#endif
        }
        else if (cmd == "reboot")
        {
            ESP.restart();
        }
        else if (cmd == "config")
        {
            std::vector<String> tokens;
            for (int i = 0; i < g_MainParser.getTokenCount(); i++)
            {
                tokens.push_back(g_MainParser.getToken(i));
            }
            g_config.parseCmd(tokens, _res_doc);
        }
        else if (cmd == "on")
        {
            /* code */
            int tokenCount = g_MainParser.getTokenCount(); // 토큰 개수를 미리 변수에 저장
            if (tokenCount > 1)
            {
                for (int i = 1; i < tokenCount; i++) // tokenCount를 사용
                {
                    int pinIndex = g_MainParser.getToken(i).toInt();
                    ledOn(pinIndex);
                }

                _res_doc["result"] = "ok";
                _res_doc["ms"] = "led(s) on";
            }
            else
            {
                _res_doc["result"] = "err";
                _res_doc["ms"] = "need pin index";
            }
        }
        else if (cmd == "off")
        {
            int tokenCount = g_MainParser.getTokenCount(); // 동일하게 tokenCount를 미리 저장
            if (tokenCount > 1)
            {
                for (int i = 1; i < tokenCount; i++) // tokenCount를 사용
                {
                    int pinIndex = g_MainParser.getToken(i).toInt();
                    ledOff(pinIndex);
                }

                _res_doc["result"] = "ok";
                _res_doc["ms"] = "led(s) off";
            }
            else
            {
                _res_doc["result"] = "err";
                _res_doc["ms"] = "need pin index";
            }
        }
        else if (cmd == "dht11")
        {
            char buffer[50];
            snprintf(buffer, sizeof(buffer), "[%.1f,%.1f]", temperature, humidity);
            _res_doc["result"] = "ok";
            _res_doc["value"] = String(buffer);
        }
        else if (cmd == "blink")
        {
            /* code */
            startBlink();
            _res_doc["result"] = "ok";
            _res_doc["ms"] = "led blink";
        }
        else if (cmd == "stopblk")
        {
            /* code */
            stopBlink();
            _res_doc["result"] = "ok";
            _res_doc["ms"] = "led stop blink";
        }
        else
        {
            _res_doc["result"] = "fail";
            _res_doc["ms"] = "unknown command";
        }
    }
    else
    {
        _res_doc["result"] = "fail";
        _res_doc["ms"] = "need command";
    }

    return _res_doc.as<String>();
}