#ifndef ETC_HPP
#define ETC_HPP

#include <Arduino.h>

inline String getChipID() {
#if defined(ESP32)
    // ESP32의 경우
    uint64_t chipid = ESP.getEfuseMac();
    char chipid_str[13];
    sprintf(chipid_str, "%04X%08X", (uint16_t)(chipid >> 32), (uint32_t)chipid);
    return String(chipid_str);
    
#elif defined(ESP8266)
    // ESP8266의 경우
    uint32_t chipid = ESP.getChipId();
    char chipid_str[9];
    sprintf(chipid_str, "%08X", chipid);
    return String(chipid_str);

#elif defined(__AVR_ATmega2560__) || defined(__AVR_ATmega328P__)
    // ATmega2560 또는 ATmega328P의 경우 (Mega, Nano)
    // 고유 ID를 EEPROM에서 읽어오거나, fuse 값을 사용해 특정 ID 생성 가능
    // 예시로 EEPROM의 첫 번째 4바이트에서 값을 가져옴 (초기화 시 고유 ID를 저장했다고 가정)
    char chipid_str[9];
    for (int i = 0; i < 4; i++) {
        sprintf(chipid_str + i * 2, "%02X", EEPROM.read(i));  // EEPROM에서 고유 ID 읽기
    }
    return String(chipid_str);
    
#else
    return "UnknownChip";
#endif
}

#endif // ETC_HPP
