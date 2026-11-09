# type_m 모터·서보·DHT 펌웨어 가이드

## 목차

- [대상과 현재 확인 범위](#대상과-현재-확인-범위)
- [빌드와 업로드](#빌드와-업로드)
- [핀맵과 전원](#핀맵과-전원)
- [DHT11·DHT22 연결과 조회](#dht11dht22-연결과-조회)
- [상태 LED와 연결 준비](#상태-led와-연결-준비)
- [명령 프로토콜](#명령-프로토콜)
- [정지와 시간 제한](#정지와-시간-제한)
- [호스트 테스트](#호스트-테스트)
- [출력을 구동하지 않는 BLE 검사](#출력을-구동하지-않는-ble-검사)
- [앱과 수업 자료](#앱과-수업-자료)
- [문제 확인](#문제-확인)

## 대상과 현재 확인 범위

현재 결정은 **두 보드의 수업 배치를 보류하고 type_m 보드 완성을 우선하는 것**이다. 센서는 DHT11·DHT22로 정했고 GPIO20 연결과 지원을 추가한다. 기존 type_m 펌웨어·TypeMControl 검증 기록은 유지하며 수업 적용은 이후 정한다. type_s 전용 펌웨어는 아직 없고 기존 LED 4핀 설정 `ledpin=[4,3,1,0]`은 센서·배터리 분압 입력과 충돌하므로 그대로 적용하지 않는다.

`type_m`은 [type_m v1 마더보드](../hardware/type_m/v1/README.md)의 모터 2개·서보 2개를 제어하는 별도 펌웨어 환경이다. 프로토콜은 `type_m/1`이며 **1.1.0에 DHT11·DHT22를 추가하고 보드에 업로드했다.** 기존 LED/DHT11 펌웨어 `1.0.5_dev` 및 `Config` 명령과 구분한다. 1.1.0 검증과 기존 1.0.0 기록을 아래에서 나눠 표시한다.

하드웨어는 원본 v1을 보존하고 [v1.1 수정본](../hardware/type_m/v1.1/README.md)에 SEN1–GPIO20 연결을 추가했으며, 2026-11-01 전원부 수정으로 배터리 커넥터 J1의 극성 표시·이름(`BAT`)과 전원 스위치 J2의 직렬 연결(J1.1→VIN→J2.3, J2.2=VCC→부하)을 바로잡았다. [KiCad 프로젝트](../hardware/type_m/v1.1/kicad/type_m_v1_1.kicad_pro)의 DRC 오류·미배선·회로도/PCB 차이는 0, ERC는 오류 20·경고 26이며, 제작·실물 검증은 완료하지 않았다. 하드웨어 리비전 `v1.1`과 펌웨어 버전 `1.1.0`은 별개다.

| 확인 항목 | 1.1.0 현재 기록 |
|---|---|
| 호스트 검사 | 501 checks PASS: 기존 399 + 센서 102. `.local/type_m/unit-tests-1.1.0.log` |
| ESP32 전체 빌드 | SUCCESS. RAM 38,892/327,680 bytes, 앱 Flash 1,000,960/1,310,720 bytes. `.local/type_m/build-1.1.0.log` |
| 업로드 | COM11, base MAC `8c:d0:b2:a8:b3:6e`에 SUCCESS·Flash hash 검증. `.local/type_m/upload-b36e-1.1.0.log` |
| 보존 이미지 / SHA-256 | `.local/type_m/firmware-1.1.0.bin` / `1DF91B8E665700114642A46AD47B5B90213D0E1915350D1EAB4AE37B1AB828E8` |
| BLE 통합 검사 | 비구동 54 checks PASS. `.local/type_m/ble-smoke-b36e-1.1.0-retry.json` |
| DHT 오류 경로 | DHT11·DHT22 모두 `error`. 종류 변경 직후 `wait`, 실패 후 READY 유지, STOP·watchdog·구독 해제·실제 단절·재연결 확인 |
| Android 새 DHT 화면 | 단위 테스트 34개 PASS·Lint 문제 없음. IM_L101/API34 설치·검색·CCCD/about/ready·DHT11→22 선택·error 표시 및 READY 유지·STOP·Home 단절 확인. 프로세스 재시작 뒤 DHT22 선택 유지 |
| 실제 센서·부하 | 사용자 DHT22의 기존 v1 SENSOR1 연결 확인. v1 원본의 SEN1–MCU 배선은 누락돼 있고 실제 온습도 수치는 미검증. v1.1은 설계 수정 단계이며 모터·서보 부하·파형도 미검증 |

1.1.0 Android 실기에서 DHT22 선택 후 READY, STOP 후 모든 내부 출력 0, Home 배경 진입 후 실제 연결 해제와 출력 0을 확인했다. 시리얼 기록은 `.local/type_m/android-dht22-ready-1.1.0.txt`, `android-stop-1.1.0.txt`, `android-background-confirmed-1.1.0.txt`다. 이 확인은 실제 출력 파형 계측과 구분한다. 온습도는 `error`만 관찰했으며 정상 수치를 얻지는 못했다.

시험 중 첫 USB 장치 소실은 사용자가 직접 USB를 분리한 것으로 확인됐다. 다시 연결한 뒤 검사를 이어갔지만, 현재 마감 시점에는 USB 장치와 BLE 광고가 다시 관찰되지 않아 배선·전원 상태를 확인 중이다. 두 번째 소실의 원인은 확정하지 않았다.

사용자는 **type_m v1을 설계한 그대로 실장했다**고 확인했다. MCU 모듈은 **EstarDyn ESP32-C3 SuperMini Plus V2.0, 빨간 기판의 RGB LED 탑재 모델**이다. [판매 제품](https://ko.aliexpress.com/item/1005011842934236.html)의 사진과 [첨부 PDF](https://ae-pic-a1.aliexpress-media.com/kf/S7a771a2922204066a89753ebae6c070a1.pdf)에 차이가 있다. PDF의 GPIO0 red / GPIO8 green 단색 LED 설명은 이 펌웨어의 핀 근거로 채택하지 않았다. 기존 TENSTAR 코드·사용자의 RGB 확인을 근거로 GPIO8 NeoPixel을 설정했고, 업로드 뒤 사용자가 대기 빨강 점멸과 READY 파랑 고정을 확인했다. 준비 전 파랑 점멸은 소스·상태 기준이며 별도 실물 색상 확인은 하지 않았다. GPIO0은 배터리 ADC이므로 LED 시험 출력으로 바꾸지 않는다.

| 확인 항목 | 1.0.0 검증 당시 기록 |
|---|---|
| 칩 식별 | ESP32-C3 QFN32, revision 0.4 |
| Flash | XMC 4MB |
| 최초 식별 보드 | USB Serial/JTAG `COM14`, base MAC `8c:d0:b2:a8:54:d5` |
| 최초 보드 백업 | `.local/type_m/previous-firmware-8cd0b2a854d5-20261101.bin`, 4MB |
| 최초 백업 SHA-256 | `48C5576155C2FAFBD1419CAFF46CAA99EE8A7DA0627FEDF4A74920D661F4ADC3` |
| 현재 업로드 대상 | USB Serial/JTAG `COM11`, base MAC `8c:d0:b2:a8:b3:6e`; 최초 보드와 다른 MAC |
| 현재 보드 백업 | `.local/type_m/previous-firmware-8cd0b2a8b36e-20261101.bin`, 4MB 백업 완료 |
| 현재 백업 SHA-256 | `7C66D38A5FD54642DAA9F3293AD3DC2030592E104D85580B0D795571EA9F319C` |
| 호스트 파서·제어 상태 테스트 | 399 checks PASS, `.local/type_m/unit-tests.txt` |
| ESP32 전체 빌드 | PlatformIO `type_m` SUCCESS. RAM 38,860/327,680 bytes(11.9%), 앱 Flash 998,000/1,310,720 bytes(76.1%). `.local/type_m/build.log` |
| 업로드 | COM11/b3:6e에 SUCCESS, Flash 데이터 hash 검증 완료. `.local/type_m/upload-b36e.log` |
| 업로드 이미지 SHA-256 | 당시 `.pio/build/type_m/firmware.bin`: `BC681C8C53B118E480AFC1855BFF9096FBDDAEE88F9F13AF187ED6066AD69BDB`. 재빌드 후 현재 파일의 해시와 구분 |
| BLE 통합 검사 | 29 checks PASS, `.local/type_m/ble-smoke-b36e.json`. 실제 연결 해제 후 보드 상태와 재연결/새 ready 필요 확인 |
| 광고 / BLE 주소 | `ESP32_BLE_M_B36E` / `8C:D0:B2:A8:B3:70`. 이 보드에서는 base MAC보다 2 큰 주소로 관찰 |
| Android | 단위 테스트17개 PASS, Lint `No issues found`. IM_L101/API34 설치·권한·초기 화면, 검색→CCCD→about→ready→`제어 준비됨`, STOP·배경 진입·재연결 확인 |
| RGB 실물 표시 | 대기 빨강 점멸·READY 파랑 고정 사용자 확인. 준비 전 파랑 점멸은 소스·상태 기준 |
| type_m v1 실장 | 설계대로 실장 완료 — 사용자 확인 |
| 모터·서보 부하 구동 / 파형 계측 | 미실시. 0이 아닌 모터·유효한 서보 명령을 전송하지 않음 |

두 보드는 모두 C3 QFN32 rev0.4/XMC 4MB로 식별했다. `.local/`은 git에서 제외되는 로컬 보관 영역이다. MAC별 백업 파일을 공개 저장소에 포함하거나 삭제하지 않는다. base MAC과 BLE 검색 주소를 동일하다고 가정하지 않는다. 최종 증거 메타데이터는 `.local/type_m/verification.json`에 있다. 위 식별·백업·빌드 기록은 새 펌웨어의 실제 출력 확인과 별개다.

1.0.0 Android READY 상태에서 시리얼은 `link=1 subscribed=1 ready=1`, 모터 요청 `0,0`, PWM `0,0,0,0`, 서보 `-1,-1`, 서보 duty `0,0`, nSLEEP `0`을 보고했다. STOP의 `ok stop` 응답 뒤에도 READY는 유지됐고 같은 정지 상태를 확인했다(`.local/type_m/android-ready-stop.txt`). Home으로 배경에 보내면 STOP 응답 뒤 연결이 해제되고 `link=0 subscribed=0 ready=0`이 됐다(`android-background.txt`). 앱 복귀 때 제어 비활성과 `정지 확인 후 연결을 종료했습니다` 문구를 확인했다. 재연결하면 about/ready를 다시 수행해 READY가 되며 출력은 계속 0이었다(`android-reconnected.txt`). 이 로그는 모두 `.local/type_m/`에 있으며 **펌웨어 내부 적용 상태를 보여 주고 실제 핀 파형 측정을 대신하지 않는다.**

## 빌드와 업로드

엔트리포인트는 [src/type_m_main.cpp](../src/type_m_main.cpp), 명령 파서와 제어 상태는 [include/type_m_protocol.hpp](../include/type_m_protocol.hpp)다. `platformio.ini`의 `type_m` 환경은 `espressif32@6.12.0`과 Arduino-ESP32 2.0.17을 사용한다. 이 환경은 기존 LED/DHT11 엔트리포인트와 분리해 빌드한다.

보드 빌드 정의는 `esp32-c3-devkitm-1`, Flash는 4MB, USB Serial/JTAG CDC를 사용한다. 이것은 컴파일 설정이며 실물 모듈 모델명 EstarDyn을 바꾸는 뜻이 아니다. NeoPixel 의존성은 현재 `^1.12.2` 범위이며 1.0.0 빌드에서 해석된 버전은 1.15.5다. 1.1.0은 Adafruit DHT 1.4.6·Unified Sensor 1.1.15를 추가한다. 플랫폼 고정과 개별 라이브러리의 버전 범위를 구분한다.

저장소 루트에서 실행한다. 먼저 설치된 Python을 확인하며, Python 설치·자동 다운로드를 하지 않는다.

```powershell
uv python list --only-installed
uv run --no-python-downloads --python 3.12 --with platformio==6.2.0 python -m platformio run -e type_m
```

이 작업에서 사용 중인 PlatformIO 전용 Python도 이미 설치된 인터프리터다. 해당 환경을 그대로 재사용하는 명령은 다음과 같다.

```powershell
uv run --no-python-downloads --python 'C:\Users\user\.platformio\penv\Scripts\python.exe' -m platformio run -e type_m
```

업로드 전 현재 포트와 대상 보드를 확인하고 모터·서보 부하를 분리한다. 아래 `COMxx`를 확인한 포트로 바꾼다. USB 재연결이나 펌웨어 전환으로 포트가 바뀔 수 있으므로 `COM14`를 모든 PC의 고정값으로 사용하지 않는다.

```powershell
uv run --no-python-downloads --python 3.12 --with platformio==6.2.0 python -m platformio device list
uv run --no-python-downloads --python 3.12 --with platformio==6.2.0 python -m platformio run -e type_m --target upload --upload-port COMxx
uv run --no-python-downloads --python 3.12 --with platformio==6.2.0 python -m platformio device monitor --port COMxx --baud 115200
```

기존 PlatformIO Python을 쓸 때도 같은 `run -e type_m --target upload --upload-port COMxx` 인수를 사용한다. Serial Monitor는 업로드 도구와 동시에 포트를 점유하지 않도록 닫는다.

## 핀맵과 전원

| 기능 | GPIO | 연결과 확인 범위 |
|---|---:|---|
| 모터 1 IN1 / IN2 | 1 / 3 | D1 / D2 → DRV8833 모듈 pin7 / pin8 |
| 모터 2 IN3 / IN4 | 4 / 5 | D3 / D4 → DRV8833 모듈 pin11 / pin12 |
| nSLEEP | 7 | DRV8833 모듈 pin1 |
| Servo1 / Servo2 | 6 / 10 | J5 / J6의 신호 패드 2 |
| 배터리 ADC | 0 | CHKBAT, 220kΩ 두 개의 분압 중간점. 이번 프로토콜에 읽기 명령 없음 |
| RGB 데이터 | 기본 8 | NeoPixel. 대기 빨강 점멸·READY 파랑 고정 실물 확인 |
| DHT11 / DHT22 데이터 | 20 | 펌웨어 1.1.0 지원. v1.1 수정본에서 J7.2·R3.2·U1 pad15(실크20)를 SEN1으로 연결; 원본 v1에는 MCU 연결 누락 |

J5·J6은 **패드 1=VCC, 2=신호, 3=GND**다. VCC는 MCU의 `+5V` 표기 핀, 드라이버, 서보가 공유하며 마더보드에 별도 전압 조절 회로는 없다. 원본 v1은 배터리 +가 VCC에 직결되어 있고, v1.1은 배터리 + → `VIN` → 전원 스위치 J2(1=OFF/NC, 2=COM=VCC, 3=ON=VIN) → VCC다. USB만으로 모터·서보의 큰 전류를 공급하지 않는다. 판매자 매뉴얼은 모듈 5V 핀 입력을 5 V ±0.2 V 정전압으로 적고 Type-C와 5V 핀 동시 급전을 금지하므로, 배터리를 연결한 채 스위치 ON 상태로 USB를 꽂지 않는다. 외부 전압·공통 접지·커넥터 방향은 실물에서 확인한다. [전체 원본 핀맵](../hardware/type_m/v1/pinmap.md), [v1.1 전원 경로와 주의](../hardware/type_m/v1.1/README.md#전원-경로와-주의)

원본 v1의 `SEN1`은 MCU 미연결이며 `VIN`도 전원 헤더 한 핀에만 연결된다. v1.1에서는 SEN1 배선을 추가하고 U1 심볼 pin15·16 이름을 GPIO20·GPIO21로 바로잡아 기존 풋프린트 실크 20·21에 맞췄다. VIN 고립은 v1.1 전원부 수정에서 J1.1을 VIN으로 옮겨 해결했다. 원본 심볼의 GPIO22 표기는 실제 ESP32-C3 GPIO가 아니다. [Espressif GPIO 문서](https://docs.espressif.com/projects/esp-idf/en/stable/esp32c3/api-reference/peripherals/gpio.html)

현재 빌드는 USB Serial/JTAG CDC를 사용하며 USB 신호는 GPIO18/19다. GPIO20은 UART0 RX 겸용 일반 GPIO로, 이 구성에서는 DHT 데이터에 사용한다. GPIO20에 외부 UART0 RX를 동시에 연결·사용하지 않는다. [Espressif 핀 기능과 USB 설명](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32c3/schematic-checklist.html)

서보는 50Hz에서 각도 `0..180`을 기본 **1000..2000µs** 펄스로 변환한다. 90은 1500µs이며, 이 값은 실제 기구 각도를 보장하지 않는다. 서보의 허용 범위에 맞춰 전용 최소·최대 펄스 상수를 조정한다. 최초 연결·ready 때는 자동 중립 이동 없이 펄스가 꺼져 있다.

모터 4입력은 LEDC 채널 0–3의 20kHz·8bit, 서보 2출력은 채널 4–5의 50Hz·14bit를 사용한다. 6채널을 모두 배정하므로 임의로 다른 PWM 라이브러리의 채널 자동 할당을 혼용하지 않는다. RGB는 NeoPixel로 구동한다. [Espressif ESP32-C3 LEDC 문서](https://docs.espressif.com/projects/esp-idf/en/stable/esp32c3/api-reference/peripherals/ledc.html)

## DHT11·DHT22 연결과 조회

센서 하나를 J7에 연결하고 앱에서 DHT11 또는 DHT22를 선택한다. 펌웨어 부팅 기본은 DHT11이며 자동 판별하지 않는다. 앱은 선택한 종류를 저장하며 프로세스 재시작 후에도 DHT22 선택이 유지되는 것을 확인했다. 사용자는 기존 v1의 SENSOR1에 DHT22를 연결했지만 실제 온습도 측정에는 성공하지 못했다.

**v1.1 수정본은 센서 신호가 PCB 배선으로 GPIO20에 연결되는 설계다.** 아래 표는 이 수정본의 연결 기준이며 전체 전기 설계 검토·제작·실측 완료를 뜻하지 않는다. 이미 실장한 v1은 파일을 수정하거나 펌웨어를 업로드해도 누락된 물리 배선이 생기지 않는다. J7.2–U1 pad15 점퍼는 기존 v1을 임시 보수할 때의 방법일 뿐 정규 사용 절차가 아니다. 이번 작업은 점퍼 사용을 전제로 하지 않고 v1.1 회로도·PCB를 수정한다.

| 마더보드 | 센서 쪽 연결 |
|---|---|
| J7 패드1, 3.3V | 실제 센서 모듈의 VCC / + |
| J7 패드2, SEN1 | DATA / S. v1.1에서 R3.2·U1 패드15, 실크20(GPIO20)와 PCB 배선으로 연결 |
| J7 패드3, GND | GND / − |
| R3, 10kΩ | 3.3V–SEN1의 기존 풀업 유지 |

J7 패드 번호는 커넥터를 보는 좌우 방향이 아니다. 3핀 센서 모듈은 제품마다 배열이 다르므로 실제 VCC/DATA/GND 표기를 대조한다. 일반 4핀 DHT 소자는 VCC·DATA·NC·GND이며 NC는 연결하지 않는다. R3가 있으므로 풀업이 없는 소자도 보드 쪽 풀업을 이용한다. 모듈에 추가 풀업이 있는지는 별도로 확인한다. [Adafruit 연결 안내](https://learn.adafruit.com/dht/connecting-to-a-dhtxx-sensor)

이번 구성은 **3.3V와 짧은 케이블**을 기준으로 한다. Aosong AM2302는 3.3–5.5V 범위, 3.3V 배선 길이 1m 이하, 읽기 간격 최소 2초를 명시한다. 1m를 이 보드의 검증된 허용 길이로 해석하지 않는다. 제조사 대표 회로는 5.1kΩ를 제시하고 Adafruit 예제는 10kΩ를 사용하므로, 이번에는 원본 R3 10kΩ를 유지한 상태에서 실제 통신을 확인한다. [Aosong AM2302 매뉴얼 §3–5](https://www.aosong.com/uploadfiles/2025/04/20250417105409216.pdf), [Adafruit 연결 안내](https://learn.adafruit.com/dht/connecting-to-a-dhtxx-sensor)

일부 센서·모듈은 3.3V 동작이 충분하지 않을 수 있다. 사용하는 제품의 전원 조건과 전압을 확인한다. J7을 임의로 5V로 바꾸거나 모듈의 DATA 풀업을 5V에 연결하지 않는다. 전원 변경이 필요하면 ESP32의 3.3V 신호 조건을 포함해 별도로 검토한다. [Adafruit 전원 설명](https://learn.adafruit.com/dht/connecting-to-a-dhtxx-sensor)

새 버전은 **2500ms 주기**로 측정하고 `dht`에 최근 결과를 응답한다. `wait`는 아직 사용할 측정 결과가 없는 상태, `error`는 읽기 실패 또는 마지막 결과에서 5000ms 이상 지난 상태다. 정상 숫자를 받기 전에는 온습도를 0으로 간주하지 않는다. 종류 선택 후 결과가 준비될 때까지 기다리며 센서 오류만으로 BLE·모터 READY를 해제하지 않는다. 센서 선택·조회는 모터·서보 watchdog을 갱신하지 않으므로 500ms `ping`을 계속 유지한다.

Adafruit DHT 1.4.6 읽기는 낮은 우선순위 작업에서 상태 mutex 밖으로 분리하고 응답에는 캐시를 사용한다. 다만 단일 코어 ESP32-C3에서 라이브러리가 40비트를 읽는 동안 인터럽트를 차단하므로 **완전히 지연 없는 처리가 아니다.** 정상 수신에도 수ms, 불량 신호에서는 더 긴 지연이 생길 수 있다. 이 구간에도 하드웨어 PWM은 유지되지만 통신 콜백의 STOP과 안전 작업 실행은 늦어질 수 있다. 이 시간 특성은 실기 검증 대상이다. [Adafruit DHT 1.4.6 수신 구현](https://github.com/adafruit/DHT-sensor-library/blob/1.4.6/DHT.cpp#L251-L287)

## 상태 LED와 연결 준비

| 상태 | RGB 표시 | 제어 출력 |
|---|---|---|
| 연결 대기 | 빨강, 500ms 간격 점멸 | 모터 정지·서보 펄스 없음 |
| BLE 연결, 준비 전 | 파랑, 250ms 간격 점멸 | 모터 정지·서보 펄스 없음 |
| READY | 파랑 고정 | 유효한 모터·서보 명령 허용 |

앱은 검색 → GATT 연결 → 서비스·특성 확인 → CCCD 알림 구독 → `about` 모델 확인 → `ready`/`ok ready` 순서로 준비한다. BLE 링크가 생긴 사실만으로 READY가 되지 않는다.

| 항목 | 값 |
|---|---|
| 광고 이름 앞부분 | `ESP32_BLE_M_` |
| Service | `c6f8b088-2af8-4388-8364-ca2a907bdeb8` |
| Characteristic | `f6aa83ca-de53-46b4-bdea-28a7cb57942e` |
| CCCD | `00002902-0000-1000-8000-00805f9b34fb` |

Characteristic은 명령 write, 최근 응답 read, 응답·이벤트 notify에 사용한다. CCCD는 이 연결의 알림 수신 설정이다. 기존 펌웨어와 UUID가 같으므로 `about` 응답으로 `type_m/1`인지 확인한다. [Android BLE 데이터 교환 설명](https://developer.android.com/develop/connectivity/bluetooth/ble/transfer-ble-data)

## 명령 프로토콜

**한 번의 GATT write는 ASCII 명령 하나이며 전체 길이는 최대 20바이트**다. 앱은 개행 없이 보낸다. 펌웨어는 선택적으로 마지막 LF 또는 CRLF 하나를 허용하지만 그 바이트까지 20바이트에 포함하며, 중간 개행·탭·비ASCII·명령 묶음은 허용하지 않는다. 응답은 JSON이 아닌 ASCII다.

| 명령 | 범위·조건 | 정상 응답 |
|---|---|---|
| `about` | READY 전에도 가능 | 1.1.0: `type_m/1 fw=1.1.0`; 기존 버전은 `type_m/1 fw=1.0.0` |
| `ready` | 연결·CCCD 구독 후. 출력은 0에서 시작 | `ok ready` |
| `ping` | READY 이후 | `ok ping` |
| `motor N V` | N=1/2, V=-100..100 | 예: `ok motor 1 -100` |
| `servo N A` | N=1/2, A=0..180 | 예: `ok servo 1 90` |
| `stop` | READY 전에도 허용. 양 모터·양 서보 정지 | `ok stop` |
| `dht 11` / `dht 22` | 1.1.0 추가. CCCD 구독·READY 이후 센서 종류 선택 | `ok dht 11` / `ok dht 22` |
| `dht` | 1.1.0 추가. CCCD 구독·READY 이후 최근 온도·습도 조회 | `dht 22 253 614`, `dht 22 wait` 또는 `dht 22 error` |

DHT 숫자 응답은 `dht <11 또는 22> <온도°C×10 정수> <습도%RH×10 정수>`다. 예를 들어 `dht 22 253 614`는 25.3°C·61.4%RH, 음의 온도는 부호 있는 정수로 표현한다. 모델 선택의 `ok`는 설정 수락이며 센서 연결·측정 성공을 뜻하지 않는다. 기존 `dht11` 명령과 JSON 응답 형식은 사용하지 않는다.

모터 `V`는 부호로 방향, 절댓값으로 PWM 비율을 지정한다. 실제 속도 피드백이나 엔코더 제어가 아니다. `V=0`은 해당 모터 정지다. 서보는 명령을 받은 채널만 펄스를 내보낸다. 정상 응답은 명령 수락이며 실물 이동 성공 확인이 아니다.

| 오류·이벤트 | 뜻과 처리 |
|---|---|
| `err syntax` | 길이·문자·명령·인수 형식 오류 |
| `err range` | 모터/서보 번호 또는 값 범위 오류 |
| `err not_ready` | READY 이전 또는 시간 제한으로 해제됨 |
| `err subscribe` | 알림 미구독. notify 수신은 불가능하므로 진단 도구는 최근 응답 read로 확인 가능 |
| `err overflow` | 수신·응답 큐 용량 초과로 출력 정지·READY 해제. 연결 정리 후 재준비 |
| `err transport` | BLE 응답 송신 실패로 출력 정지·READY 해제. 최근 응답 read로 진단 가능 |
| `event timeout` | watchdog에 의한 출력 정지·READY 해제 |

`about`과 잘못된 명령은 watchdog을 갱신하지 않는다. READY 이후 유효한 `ping`·모터·서보·STOP 명령은 수신 시각을 기준으로 갱신한다. 기존 `config`, `on`, `off`, `pwm`, `dht11`, `blink`는 이 환경의 명령이 아니다.

USB Serial Monitor는 읽기 전용 진단이다. 개행으로 끝나는 `about`은 버전을, `status`는 연결·구독·READY·요청값·출력 duty·nSLEEP 상태를 출력한다. 다른 시리얼 명령에는 `err read_only`를 반환하며, 시리얼로 모터·서보를 구동하지 않는다. 상태 로그의 duty 역시 실제 파형 계측값은 아니다.

## 정지와 시간 제한

- **STOP:** 양 모터 출력 0, nSLEEP LOW, 양 서보 펄스 OFF. 대기 중이던 이전 제어 요청을 취소한다. 기존 READY 세션은 유지하므로 새로운 명시적 명령은 다시 받을 수 있다.
- **연결 해제·알림 구독 해제:** 즉시 제어 준비를 해제하고 출력을 끈다. 다시 구독했더라도 `ready` 절차를 다시 거친다.
- **Watchdog:** READY 상태에서 유효한 통신이 1500ms 동안 없으면 출력을 끄고 READY를 해제하며 `event timeout`을 알린다. 앱은 500ms heartbeat를 보내고 timeout 시 연결을 정리한 뒤 재연결한다.
- **앱 생명주기:** 배경 진입 시 STOP을 요청하고 연결을 해제한다. 모터 슬라이더에서 손을 떼면 해당 모터에 0을 보낸다.
- **방향 전환:** 반대 방향을 켜기 전에 해당 모터 양 입력을 LOW로 두는 구간을 둔다. 현재 방향 전환·드라이버 기동 대기 상수는 `millis()` 기준 2ms 차이를 기다려 시간값의 양자화를 고려해 최소 1ms를 확보한다. 이 값은 기계적인 완전 정지 시간을 보장하지 않는다.

DRV8833의 nSLEEP LOW는 두 H-bridge 출력을 비활성화하고, HIGH로 바꾼 뒤 최대 1ms의 기동 시간이 필요하다. 입력 `00`은 coast, `11`은 brake다. 드라이버 내부 dead time과 펌웨어의 방향 전환 간격은 다른 개념이다. STOP은 전원 차단·기계적 브레이크·서보 전원 차단이 아니다. [TI DRV8833 데이터시트 §6.5, §7.3–7.4](https://www.ti.com/lit/ds/symlink/drv8833.pdf)

## 호스트 테스트

[tests/type_m_tests.cpp](../tests/type_m_tests.cpp)는 PC에서 파서·제어 상태를 검사한다. 설치된 Python 3.12와 `uv`의 Zig 0.13.0 패키지로 **1.1.0의 501 checks를 통과**했다. 기존 399개에 센서 102개를 추가했으며 로그는 `.local/type_m/unit-tests-1.1.0.log`다. 1.0.0의 399개 기록 `.local/type_m/unit-tests.txt`도 보존한다.

```powershell
uv run --no-project --no-python-downloads --python 3.12 --with ziglang==0.13.0 python -m ziglang c++ -nostdlib++ -std=c++11 -Wall -Wextra -Werror -I include tests/type_m_tests.cpp -o .local/type_m/type_m_tests-1.1.0.exe
.\.local\type_m\type_m_tests-1.1.0.exe
```

검사 범위는 패킷·인수 경계, 구독/READY 조건, watchdog 만료와 시간값 wrap, 오래된 요청 거부, STOP 이후 epoch, 서보 초기 OFF, 모터 기동 대기·방향 전환 간격이다. 1.1.0은 DHT 종류·숫자 범위·캐시 만료·종류 전환 중 오래된 결과 거부·오류 시 READY 유지·센서 조회의 heartbeat 미연장을 추가로 검사한다. 이는 실제 DHT 신호·GPIO 파형·부하 동작을 대신하는 결과가 아니다.

## 출력을 구동하지 않는 BLE 검사

[tools/type_m_smoke.py](../tools/type_m_smoke.py)는 설치된 Python과 `uv`를 사용한다. `bleak==3.0.1`, `pyserial==3.5` 의존성은 스크립트 메타데이터에 명시되어 있으며 새 Python을 다운로드하지 않는다. BLE 어댑터가 있는 PC에서 검색으로 확인한 **대상 BLE 주소**와 현재 시리얼 포트를 입력한다. 다른 앱의 보드 연결과 Serial Monitor는 먼저 닫는다.

```powershell
uv python list --only-installed
uv run --no-python-downloads --python 3.12 tools/type_m_smoke.py --address '8C:D0:B2:A8:B3:70' --serial-port COM11 --output .local/type_m/ble-smoke-b36e-1.1.0-rerun.json
```

위 주소·포트는 **1.1.0의 54개 검사를 통과한 보드**의 값이다. 다른 보드에서는 다시 확인한다. 예시 출력 파일은 기존 증거를 덮어쓰지 않는 별도 재실행 경로다. 검사는 about, 미구독/준비 전 거부, 범위·문법 오류, DHT11/22 선택·wait/error·실패 후 READY 유지, 모터 0, STOP, watchdog, 구독 해제 및 재연결 후 재준비를 확인한다. `--serial-port`를 주면 OS 객체 종료만으로 단절됐다고 가정하지 않고 보드의 `link=0`과 READY·모터·서보·nSLEEP 내부 상태를 확인한다. **0이 아닌 모터 명령이나 유효한 서보 명령을 보내지 않는다.** 54개 통과는 실제 온습도 측정·모터·서보 파형·부하 구동 검증이 아니다. 실행 결과의 `passed`와 각 `checks`를 보존한다.

Windows에서는 BLE 클라이언트를 닫은 뒤 실제 링크 해제가 늦어질 수 있다. 1.0.0 최초 검사에서 고정 1초 대기는 이를 구분하지 못했으므로, 보드의 단절 상태를 기다리고 다시 검색하도록 보강한 뒤 29개 검사를 통과했다. 1.1.0 최초 검사는 COM 포트를 다시 연 직후 한 번의 status 응답을 받지 못해 실패했다. 읽기 전용 상태 요청을 재시도하도록 도구를 보완한 뒤 54개 검사를 통과했으며 첫 실패 로그도 보존했다. `--serial-port`를 생략한 검사는 보드 내부 단절 상태 확인을 포함하지 않는다.

watchdog 시험에서는 링크를 유지하며 heartbeat만 멈춘다. 앱을 배경으로 보내는 시험은 연결 해제에 의한 정지 검사이며 watchdog 단독 검사와 다르다.

USB의 읽기 전용 상태 조회는 [tools/type_m_status.py](../tools/type_m_status.py)를 사용한다. `about`·`status`만 요청하며 구동 명령이나 리셋 요청을 보내지 않는다. 현재 포트를 확인해 `COMxx`를 바꾼다.

```powershell
uv run --no-python-downloads --python 3.12 tools/type_m_status.py --port COMxx
```

## 앱과 수업 자료

- 독립 앱: `bleunoApp/TypeMControl/`을 Android Studio에서 연다. Kotlin·XML, compile SDK 36.1, 최소 Android API 24다. 앱 저장소 루트나 기존 `BleunoTest/`가 이 프로젝트의 루트가 아니다.
- 기존 실습 참고 자료: [교재 type_m 90분 실습](https://github.com/gbox3d/teaching_repo/blob/main/android_programming/bleuno/type_m_lab.md). 수업 배치는 보류하며 type_m 보드 완성 뒤 적용 여부를 정한다.
- SmartIO/Fake의 JSON·LED·센서 실습과 type_s 하드웨어 자료는 별도로 관리한다. 이 프로토콜을 기존 LED용 파서·Config 화면에 연결하지 않는다.

## 문제 확인

| 증상 | 확인 순서 |
|---|---|
| 찾을 수 없음 | 전원·BLE 권한·다른 앱 연결 여부·광고 이름 확인 |
| 파랑 점멸에서 진행 안 됨 | CCCD 쓰기 완료 → about 모델 응답 → ready 응답 순서 확인 |
| JSON 응답이 옴 | 기존 펌웨어 또는 다른 보드에 연결했는지 확인 |
| `event timeout` 반복 | heartbeat·명령 큐 지연·앱 생명주기 확인, 재연결 |
| RGB가 안 켜짐 / 색이 다름 | 실제 보드 리비전과 NeoPixel 데이터 핀·색 순서 확인. GPIO0을 시험 출력으로 쓰지 않음 |
| 서보가 최초 연결에서 움직이지 않음 | 자동 중립 이동이 없는 것이 정상. 유효 명령 후 펄스·전원 확인 |
| 정상 응답인데 부하가 안 움직임 | 부하 전원·공통 GND·모듈 및 커넥터 방향·실제 출력 측정. 명령 수락과 물리 구동을 구분 |
| `dht`가 문법 오류 | `about` 버전 확인. DHT는 1.1.0 추가 명령이며 1.0.0에는 없음 |
| DHT `wait` / `error` | 첫 측정 대기·11/22 종류 선택 → 센서 실제 핀 라벨·3.3V/GND → 보드 리비전 확인(v1은 SEN1–MCU 연결 누락) → v1.1의 J7.2–U1 pad15 배선·풀업·짧은 케이블 확인 |
