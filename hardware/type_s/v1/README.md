# BLEuno type_s v1

모델은 `type_s`, 하드웨어 버전은 `v1`입니다. 지정된 EasyEDA **ZeroFactorySensorBoard → Board_DHT → PCB1**을 KiCad `type_s_v1` 프로젝트로 관리합니다. 펌웨어 버전과 하드웨어 버전은 별개입니다.

2026-11-01에 KiCad 이전과 원본 대조를 완료했습니다. 편집할 파일은 [kicad/type_s_v1.kicad_pro](kicad/type_s_v1.kicad_pro)입니다. 원본의 부품·배선·연결을 보존했으며, 아래 검증 기록에 이전 과정의 보정과 남은 항목을 기록합니다.

## 구성

| 경로 | 내용 |
|---|---|
| [source/README.md](source/README.md) | EasyEDA 원본 `.epro2`, 가져오기용 `.epro`, 출처·시간·SHA-256 |
| [kicad/type_s_v1.kicad_pro](kicad/type_s_v1.kicad_pro) | KiCad 프로젝트 |
| [kicad/type_s_v1.kicad_sch](kicad/type_s_v1.kicad_sch) | 회로도 |
| [kicad/type_s_v1.kicad_pcb](kicad/type_s_v1.kicad_pcb) | PCB |
| `kicad/type_s_v1.kicad_sym`, `kicad/type_s_v1.pretty/` | 프로젝트 전용 심볼·풋프린트 라이브러리 |
| [pinmap.md](pinmap.md) | MCU·커넥터·저항·전원 핀맵 및 원본 검토 항목 |
| [validation/README.md](validation/README.md) | 원본 대조, ERC·DRC, 보정 내역과 재검증 명령 |

## 보드 용도와 원본 차이

이 보드는 ESP32-C3 모듈과 센서·디지털·아날로그 연결용 헤더를 사용하는 마더보드입니다. `DHT`, `CDS`, `DS18B20`은 원본 커넥터의 표기이며 해당 센서가 보드에 실장되었다는 뜻은 아닙니다.

- 센서 신호: `SENSOR1` GPIO3, `SENSOR2` GPIO4, `SENSOR3` GPIO5.
- 확장 신호: `A1` GPIO1, `D1` GPIO6, `D2` GPIO7, `D3` GPIO10.
- 배터리 측정: `BATCHK` GPIO0.
- **배터리 분압 저항 R1/R2:** 원본 회로도는 각각 `220kΩ`, PCB의 상속값·표기는 각각 `10kΩ`입니다. 이전 과정에서 임의로 동일하게 만들지 않습니다. 실제 부품값을 확인한 뒤 설계 변경으로 확정해야 합니다.

기존 교재의 `ledpin=[4,3,1,0]`, `dht11pin=7` 설정은 이 보드의 센서·분압·확장 신호 배치와 다릅니다. [핀맵](pinmap.md)의 전원·커넥터 순서를 먼저 확인하고 실습에 맞는 설정을 사용합니다. 센서 실물, 전압, 모듈 핀 표기, USB/외부 전원의 관계는 별도 실기 검증 대상입니다.

## 이전 검증

최초 가져오기 파일과 원본을 별도로 보존했습니다. 원본에는 물리 부품 15개·부품 패드 52개·독립 홀 패드 3개·배선 114개·연결 네트 그룹 12개가 있습니다. 외곽은 15개 선분, 내부 절개는 8개 선분입니다.

[원본 비교 결과](validation/import-comparison.json)의 `checked_comparisons_pass`는 `true`입니다. 회로도와 PCB를 KiCad에서 다시 열어 표시·라이브러리 연결도 확인했습니다.

- PCB DRC: **오류 0개, 실크 관련 경고 39개**, 미배선 0개.
- 회로도/PCB 대응 검사: **R1/R2의 원본 값 차이 2개**가 남습니다. 다른 가져오기 연결·빈 값 표현의 차이는 정리했습니다.
- 회로도 ERC: **오류 10개, 경고 26개**. 원본 핀 유형·미연결·전원 표현에 대한 검토 항목입니다.

원본에서 표시되던 실크 15개를 복원했고 GND 구리면은 KiCad에서 재계산했습니다. 상세 범위와 한계는 [검증 기록](validation/README.md)을 기준으로 판단합니다. 파일 이전은 회로 기능이나 제작 적합성, BLE·센서 동작의 실물 검증을 대신하지 않습니다.
