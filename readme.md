# BLEuno (ble led controller)

## Description
This is a simple BLE LED controller that can be used to control an LED over BLE. The controller is implemented using the ESP32 microcontroller and the Arduino framework. The controller can be used to turn the LED on and off, as well as to change the color of the LED.

devoce name start with `ESP32_BLE'  


## uuid

The UUIDs for the service and characteristic are defined in the `ble_led_controller.ino` file. The UUIDs are as follows:
```cpp
#define SERVICE_UUID "c6f8b088-2af8-4388-8364-ca2a907bdeb8"
#define CHARACTERISTIC_UUID "f6aa83ca-de53-46b4-bdea-28a7cb57942e"
```

## Commands

**setup led pins** : Setup the pins for the LED  

```txt
config setA ledpin [4,5,6,7] // espc3 -> d2,d3,d4,d5 
config setA ledpin [1,4,6,5] 
config setA ledpin [4,16,17,18] 
config setA ledpin [2,12,15,4,13,14,16,17,18] 
config setA ledpin [0,2,5,12,15,13,14,18,19] 
config setA ledpin [15,2,0,4,16,17,18,19] //worover kit
config save
reboot
```

**on** : Turn the LED on   
example:    
`on 1` : turn led index 1 on  
`on 3 6 9` : turn led index 3, 6, 9 on  
`on -1` : all leds on  

**off** : Turn the LED off  
example:  
`off 1` : turn led index 1 off  
`off 3 6 9` : turn led index 3, 6, 9 off 
`off -1` : all leds off  


dht11 pin setup
```txt

config set dht11pin 7

config set dht11pin 32
config save
reboot
```

