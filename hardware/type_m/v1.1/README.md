# BLEuno type_m v1.1

[원본 v1](../v1/README.md)을 그대로 보관하고 별도로 수정한 하드웨어 리비전입니다. 2026-11-01에 두 단계로 수정했습니다.

1. **센서 배선** — 원본에서 빠져 있던 SENSOR1 신호(SEN1) → C3 GPIO20 연결을 회로도와 PCB에 추가.
2. **전원부** — 배터리 커넥터 J1의 극성 표시와 이름을 바로잡고, 전원 스위치 J2가 실제로 배터리와 부하 사이의 직렬 경로가 되도록 네트와 배선을 수정.

v1.1은 아직 제작하지 않았으므로 두 수정을 같은 리비전으로 관리합니다. **이 파일 수정으로 이미 실장된 v1 보드의 배선이 바뀌지는 않습니다.** 제작·실물 통전·배터리 구동·모터/서보 부하 검증은 아직 하지 않았습니다.

**[KiCad 프로젝트 열기](kicad/type_m_v1_1.kicad_pro)** · **[전원부 수정 확인 화면](validation/power-fix-2026-11-01/review.html)** · **[센서 연결 확인 화면](validation/preview/review.html)** · [회로도 PDF](validation/preview/type_m_v1_1-schematic.pdf)

## 센서 배선 수정 (1단계)

- SEN1의 연결: **J7.2(SENSOR1 가운데 신호 핀) ↔ R3.2(기존 10 kΩ 풀업) ↔ U1.15(GPIO20)**.
- U1의 `15`는 모듈 심볼/풋프린트의 패드 번호입니다. 실제 C3 보드에 표시된 GPIO 번호는 **20**입니다.
- MCU 심볼의 잘못된 핀 이름을 바로잡았습니다: 패드 15 `GPIO21 → GPIO20`, 패드 16 `GPIO22 → GPIO21`. 패드 16은 연결하지 않습니다.
- 새 배선 6개(폭 0.254 mm)와 비아 1개(외경 0.6 mm / 드릴 0.3 mm)를 추가했습니다. PCB 실크에 `type_m v1.1`을 표시했습니다.
- SENSOR1의 기존 3.3 V 전원, GND, 10 kΩ 풀업과 부품 위치·외곽·기존 배선을 유지했습니다. DHT11/DHT22를 사용하며 앱에서 센서 종류를 선택합니다.

GPIO20은 UART0 RX와 일반 GPIO를 겸합니다. 현재 type_m 펌웨어의 USB HWCDC는 GPIO18/19를 사용하므로 GPIO20을 DHT 입력으로 사용할 수 있습니다. GPIO20에 외부 UART RX를 동시에 연결하는 구성은 지원하지 않습니다. [Espressif GPIO 문서](https://docs.espressif.com/projects/esp-idf/en/stable/esp32c3/api-reference/peripherals/gpio.html)

## 전원부 수정 (2단계)

[수정 전 검토](validation/power-audit-2026-11-01/design-review.md)에서 확인한 3건을 고쳤습니다. 수정 전후 그림과 수치는 [전원부 수정 확인 화면](validation/power-fix-2026-11-01/review.html)에 있습니다.

| 항목 | 수정 전 | 수정 후 |
|---|---|---|
| J1 극성 표시 | 실크 `BAT + -` 한 줄. 전면 기준 왼쪽 원형 패드 J1.2 = GND(−), 오른쪽 사각 패드 J1.1 = 배터리 + 이므로 좌우 순서가 반대 | 실크 `BAT`(원래 자리) + 각 패드 바로 위에 `−`(J1.2) / `+`(J1.1) 0.8 mm 표시. 패드 네트는 바꾸지 않음 |
| 전원 스위치 J2 | J1.1과 J2.2가 모두 VCC, J2.3의 `VIN`은 고립 패드 → J2로 전원을 끊을 수 없음 | J1.1(배터리 +) → **VIN** → J2.3. J2.2(공통) → **VCC** → 부하. J2.1은 OFF 위치로 의도적 미연결(회로도 NC 표시) |
| J1 이름 | 회로도·PCB 값 `MT1`(모터 출력 J3와 같은 이름) | `BAT` |

PCB 변경은 J1.1에서 내려오던 1.0 mm 배선 3개의 네트를 VIN으로 바꾸고, VCC 노드로 이어지던 대각 배선 1개를 삭제한 뒤, (172.385, 96.731)에서 J2.3 패드 중심 (172.385, 93.048)까지 1.0 mm F.Cu 배선 1개를 추가한 것입니다. 가장 가까운 다른 네트 패드(J2.2)와의 간격은 1.19 mm이고, 주변 GND 채움 동판과는 설계 규칙 간격 0.254 mm를 둡니다. 센서 배선·비아, 외곽, 나머지 부품과 배선은 바꾸지 않았고 동판은 KiCad에서 다시 채웠습니다. 새 `−`/`+` 표시의 글자 획은 패드 구리에서 각각 0.64 mm·0.33 mm, 보드 외곽선 중심에서 0.65 mm·0.47 mm(외곽 선폭 안쪽 기준 0.34 mm) 떨어져 있습니다. 이 프로젝트 규칙에는 실크–외곽 최소 간격이 없어 DRC가 검사하지 않는 값이며, 일반적인 제작사 최소값(0.15–0.2 mm)보다는 큽니다.

### J2 스위치 사용 규칙

| 패드 | 위치 (전면, 왼쪽부터) | 네트 | 역할 |
|---|---|---|---|
| J2.1 (사각) | 왼쪽, `OFF` 쪽 | 없음 (NC) | 슬라이드 스위치 OFF 위치 접점. 아무것도 연결하지 않음 → 배터리 분리 |
| J2.2 | 가운데 | **VCC** | 스위치 공통(COM). U1 5V 핀, DRV8833, 서보 2개, R1 분압으로 전원이 나감 |
| J2.3 | 오른쪽, `ON` 쪽 | **VIN** | 배터리 + 입력(J1.1). 스위치가 2–3을 이으면 ON |

- 실크 `PW OFF ON`은 왼쪽이 OFF, 오른쪽이 ON 방향입니다. 슬라이드 스위치 손잡이를 오른쪽(BAT 커넥터 쪽)으로 밀면 2–3이 이어져 ON입니다. 글자 `OFF`/`ON`이 패드 1/3 중심에 정확히 놓인 것은 아닙니다.
- 스위치 대신 **2–3 핀 쇼트 점퍼**를 꽂으면 항상 ON입니다.
- J2 접점에는 모터 드라이버·서보 2개·MCU의 전체 부하 전류가 흐릅니다. 2.54 mm 소형 슬라이드 스위치(SS12D00 계열)는 접점 정격이 0.3–0.5 A 수준이라 모터·서보 피크 전류에 부족할 수 있습니다. 부하 전류 예산을 확인하기 전에는 쇼트 점퍼로 운용하거나 피크 전류 이상 정격의 스위치를 선택합니다. 보드 배선 폭은 1.0 mm입니다.

### 전원 경로와 주의

- 배터리 + → J1.1 → VIN → (J2 ON) → VCC → U1.8(모듈 `5V` 핀), U2.9(DRV8833 VCC), J5.1/J6.1(서보 +), R1.1(CHKBAT 분압). 배터리 − → J1.2 → GND.
- CHKBAT(R1 220 kΩ / R2 220 kΩ)는 VCC 쪽에 그대로 두었습니다. 스위치 ON이면 GPIO0이 배터리 전압의 1/2을 읽고, OFF이면 분압 전류(약 9 µA)도 흐르지 않으며 꺼진 MCU의 GPIO0에 전압이 걸리지도 않습니다. 분압을 VIN 쪽으로 옮기면 OFF 상태에서 110 kΩ(R1∥R2)을 통해 VIN/2이 꺼진 핀에 걸리므로 옮기지 않았습니다.
- U1의 `5V` 핀은 VCC에 그대로 있습니다. 보관 중인 판매자 매뉴얼(`.local/type_m/estardyn-manual.pdf` 6장)은 5V 핀 입력을 **정전압 5 V ±0.2 V**로 적고 **Type-C와 5V 핀 동시 급전을 금지**합니다. 모듈 내부 회로는 KiCad에 없으므로 실물 동작을 단정하지 않지만, 아래는 설계상 사실입니다.
  - 스위치 OFF는 **배터리만** 분리합니다. USB가 꽂혀 있고 모듈이 USB 전원을 5V 핀으로 내보내는 구조라면 모터 드라이버·서보는 USB에서 계속 전원을 받습니다.
  - 모듈이 USB 5 V를 5V 핀으로 내보내는 구조라면, 스위치 ON 상태로 USB를 꽂을 때 USB 5 V와 배터리가 VCC에서 만납니다. 이 보드에는 충전·보호 회로가 없으므로, 모듈의 USB→5V 핀 경로를 확인하기 전에는 리튬 배터리를 연결한 채 ON + USB로 두지 않습니다. 이 상태의 CHKBAT는 배터리가 아니라 USB가 공급하는 VCC를 읽습니다.
  - 원본 v1은 배터리가 항상 VCC에 직결되어 있어 이 상태를 피할 수 없었습니다.
- 3.7 V 배터리 직결 적합성, 승압 필요 여부, 사용자 보고(USB 부팅 O / 배터리 부팅 X, 4.9 V 측정)의 원인은 이 수정에서 판단하지 않았습니다.

## 확인 방법

1. [회로도](kicad/type_m_v1_1.kicad_sch): J1 아래 값 `BAT`, J1.1의 `VIN` 라벨과 J2.3의 `VIN` 라벨, J2.1의 NC(×) 표시, J2.2의 VCC. U1 `GPIO20` 옆 `SEN1` 라벨과 J7 가운데 핀의 같은 라벨.
2. [PCB](kicad/type_m_v1_1.kicad_pcb): J1 패드 위 `−`/`+`. J2.3 패드에서 아래로 내려오는 VIN 배선이 J1.1에서 온 배선과 (172.385, 96.731)에서 만납니다. J2.2 패드의 기존 VCC 배선은 U1·R1·서보·드라이버로 이어집니다. SENSOR1 가운데 패드의 네트를 강조하면 U1의 GPIO20까지 연결됩니다.
3. [전원부 수정 확인 화면](validation/power-fix-2026-11-01/review.html)과 [센서 연결 확인 화면](validation/preview/review.html)의 그림을 실제 KiCad 화면과 비교합니다. 두 화면 모두 KiCad가 내보낸 SVG를 바탕으로 합니다.

## 검사 결과 (전원부 수정 후, 현재 파일 기준)

| 검사 | 결과 |
|---|---|
| KiCad 10.0.6 PCB DRC / 동판 재채움 / 회로도 일치 검사 | 오류 0, 기존 실크 글자 높이 경고 12, 미배선 0, 불일치 0 |
| 회로도 ERC | 오류 20, 경고 26. 센서 단계 21/27에서 J2.1 미연결 오류, VIN 고립 라벨 경고, J1.1–전원 심볼 핀 유형 경고가 사라지고 J1.1–J2.3 핀 유형 경고 1개가 추가됨 |
| 전원부 변경 독립 대조 ([verify_power_fix.py](validation/power-fix-2026-11-01/verify_power_fix.py)) | 34/34 PASS: 허용한 변경 외 차이 없음, 구리 접촉 그래프에서 J1.1은 J2.3에만 닿음, 서로 다른 네트의 구리 접촉 0, SEN1 배선·비아 유지, 기록 파일이 설계 파일보다 최신 |
| 원본 v1 보존 | 기록한 57개 파일의 SHA-256 불변 |

검사 자료(현재 상태): [DRC](validation/drc.json), [ERC](validation/erc.json), [netlist](validation/netlist.xml), [전원부 검증 결과](validation/power-fix-2026-11-01/power-fix-verification.json), [PCB 패치 기록](validation/power-fix-2026-11-01/pcb-power-patch.json), [회로도 패치 기록](validation/power-fix-2026-11-01/schematic-patch.json).

센서 단계(전원부 수정 전) 기록: [독립 변경 대조 25/25](validation/revision-comparison.json)와 그 입력이었던 회로도·PCB·netlist·ERC·DRC는 `validation/power-fix-2026-11-01/before-power-fix.*`로 보존했고 해시가 일치합니다. [verify_revision.py](validation/verify_revision.py)는 그 스냅샷을 기준으로 쓴 검사라 현재 파일에는 그대로 통과하지 않습니다. 현재 파일의 검사는 `verify_power_fix.py`입니다.

남은 ERC 오류 20은 no_connect 표시가 없는 미연결 핀 5(U1 GPIO2/8/9/21, U2 ULT), 전원 심볼 미구동 2(VCC, GND), 가져온 심볼의 Input 핀 유형에 따른 pin_not_driven 13이고, 경고 26은 모두 Unspecified 핀 유형 조합(pin_to_pin)입니다. 검사 통과를 위해 핀 유형이나 규칙을 완화하지 않았습니다. 제작 적합성 검토와 실물 검증은 별개입니다.

## 재검증

설치된 KiCad의 CLI와 Python을 사용합니다. Python을 새로 설치하지 않습니다.

```powershell
$kicad = "$env:LOCALAPPDATA\Programs\KiCad\10.0\bin"
& "$kicad\kicad-cli.exe" sch erc --format json --severity-all -o hardware/type_m/v1.1/validation/erc.json hardware/type_m/v1.1/kicad/type_m_v1_1.kicad_sch
& "$kicad\kicad-cli.exe" sch export netlist --format kicadxml -o hardware/type_m/v1.1/validation/netlist.xml hardware/type_m/v1.1/kicad/type_m_v1_1.kicad_sch
& "$kicad\kicad-cli.exe" pcb drc --format json --severity-all --schematic-parity -o hardware/type_m/v1.1/validation/drc.json hardware/type_m/v1.1/kicad/type_m_v1_1.kicad_pcb
& "$kicad\python.exe" -B hardware/type_m/v1.1/validation/power-fix-2026-11-01/verify_power_fix.py
```

위 명령은 `validation/`의 기록 파일을 현재 파일 기준으로 다시 씁니다. 기록을 덮어쓰지 않고 감사만 하려면 `kicad-cli`의 `-o`와 `verify_power_fix.py --output`(필요하면 `--erc/--drc/--netlist`)을 임시 폴더로 지정합니다.

`patch_schematic_sensor.py`, `patch_pcb_sensor.py`, `patch_schematic_power.py`, `patch_pcb_power.py`는 이미 적용된 수정 스크립트이며 다시 실행하지 않습니다. 전원부 스크립트 2개는 기준 해시가 다르거나 KiCad GUI가 프로젝트를 열고 있으면(`~*.lck`) 스스로 거부하고, 센서 스크립트 2개는 대상 상태로만 재적용을 거부합니다. `type_m_v1_1.kicad_prl`은 GUI 화면 설정 파일이라 기록·검증 대상이 아닙니다(`.gitignore`). KiCad GUI로 저장하기 전에는 외부에서 바꾼 파일을 다시 열어 최신 상태를 불러옵니다.
