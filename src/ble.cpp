#include <Arduino.h>
//#include <TaskScheduler.h>
#include <tonkey.hpp>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <vector>

extern String parseCmd(String _strLine);
extern int systemMode;
extern std::vector<int> ledPins;

extern void startBlink();
extern void stopBlink();

extern void ledOn(int pinIndex);
extern void ledOff(int pinIndex);

//------------------------------------------------ ble start
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
bool deviceConnected = false;

// UUID for service and characteristic
#define SERVICE_UUID "c6f8b088-2af8-4388-8364-ca2a907bdeb8"
#define CHARACTERISTIC_UUID "f6aa83ca-de53-46b4-bdea-28a7cb57942e"

#define CHECK_CODE 241111

struct S_Ble_Header_Req_Packet_V1
{
  uint32_t checkCode;
  uint8_t cmd;
  uint8_t parm[3];
};

struct S_Ble_Header_Res_Packet_V1
{
  uint32_t checkCode;
  uint8_t cmd;
  uint8_t parm[3];
};

// 개행 문자를 기준으로 문자열을 나누는 함수
std::vector<String> splitString(const String &str, char delimiter) {
    std::vector<String> tokens;
    int startIndex = 0;
    int endIndex = 0;

    while ((endIndex = str.indexOf(delimiter, startIndex)) != -1) {
        tokens.push_back(str.substring(startIndex, endIndex));
        startIndex = endIndex + 1;
    }
    if (startIndex < str.length()) {
        tokens.push_back(str.substring(startIndex));
    }

    return tokens;
}

class MyCharateristicCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    Serial.println("Characteristic write event");

    
    if (systemMode == 0)
    {
      // ascii mode , text mode
      std::string value = pCharacteristic->getValue();
      

      if (value.length() > 0) {
            // 개행 문자를 기준으로 문자열을 분할
          std::vector<String> commands = splitString(String(value.c_str()), '\n');

          for (const auto &command : commands) {
              // 명령어가 비어있는 경우는 무시
              if (command.length() == 0) {
                  continue;
              }

              String response = parseCmd(command);
              Serial.println("Command:");
              Serial.println(command);
              Serial.println("Response:");
              Serial.println(response);

              // BLE를 통해 응답 전송
              pCharacteristic->setValue(response.c_str());
              pCharacteristic->notify();
          }
        }
    }
    else
    {
      //binary mode

      std::string value = pCharacteristic->getValue();

      if (value.length() >= sizeof(S_Ble_Header_Req_Packet_V1))
      {
        S_Ble_Header_Req_Packet_V1 *packet = (S_Ble_Header_Req_Packet_V1 *)value.data();

        if (packet->checkCode == CHECK_CODE)
        {
          switch (packet->cmd)
          {
          case 0x01:
          {
            S_Ble_Header_Res_Packet_V1 resPacket;
            resPacket.checkCode = CHECK_CODE;
            resPacket.cmd = 0x01;
            resPacket.parm[0] = packet->parm[0];

            Serial.println("Echoing first parameter:");
            Serial.println(packet->parm[0], HEX);

            pCharacteristic->setValue((uint8_t *)&resPacket, sizeof(resPacket));
          }

          break;
          case 0x02:
          {
            int nPinIndex = packet->parm[0];

            digitalWrite(ledPins[nPinIndex], HIGH);
          }

            // for (int i = 0; i < sizeof(ledPins) / sizeof(ledPins[0]); i++)
            // {
            //     digitalWrite(ledPins[i], HIGH);
            // }
            Serial.printf("LEDs ON: %d\n", packet->parm[0]);
            break;
          case 0x03:
          {
            int nPinIndex = packet->parm[0];

            digitalWrite(ledPins[nPinIndex], LOW);
          }
            Serial.printf("LEDs OFF: %d\n", packet->parm[0]);

            break;
          case 0x04:
          {
            S_Ble_Header_Res_Packet_V1 resPacket;
            resPacket.checkCode = CHECK_CODE;
            resPacket.cmd = 0x04;
            resPacket.parm[0] = 1;
            resPacket.parm[1] = 2;
            resPacket.parm[2] = 3;
            // notify the client of the new value
            pCharacteristic->setValue((uint8_t *)&resPacket, sizeof(resPacket));
            // delay(500);
            pCharacteristic->notify();
            break;
          }
          default:
            Serial.println("Unknown command");
          }
        }
        else
        {
          Serial.println("Invalid check code");
        }
      }
      else
      {
        Serial.println("Received value too short");
      }
    }
  }
};

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    // task_Blink.disable();
    deviceConnected = true;

    stopBlink();

// #if defined(LED_BUILTIN)
//     digitalWrite(  LED_BUILTIN, LOW);
// #endif

    

    Serial.println("Client connected");
    pServer->getAdvertising()->stop(); // 클라이언트가 연결되면 광고 중지
  };

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;

    // task_Blink.enable();
    startBlink();

    Serial.println("Client disconnected");
    pServer->getAdvertising()->start(); // 클라이언트가 연결 해제되면 광고 다시 시작
  }
  
  // void onMtuChanged(BLEServer* pServer,uint16_t mtu) {
  //   Serial.print("MTU size updated: ");
  //   Serial.println(mtu);
  // }
};

//------------------------------------------------ ble end

void ble_setup(String strDeviceName) 
{
    // // print device name and info
    /////////////////////////////////////////////////////////
    // ble setup ---------------------------------------------
    //  Create the BLE Device
    BLEDevice::init(strDeviceName.c_str());

    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create a BLE Characteristic
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE |
            BLECharacteristic::PROPERTY_NOTIFY |
            BLECharacteristic::PROPERTY_INDICATE);

    pCharacteristic->addDescriptor(new BLE2902()); // 알람 표시 기능활성화
    pCharacteristic->setCallbacks(new MyCharateristicCallbacks());

    // Start the service
    pService->start();

    // Start advertising
    pServer->getAdvertising()->start();
    // Serial.printf("Waiting a client connection to notify...%s \n", getChipID().c_str());
    
    Serial.println("Ble Ready , Device name: " + strDeviceName);
}