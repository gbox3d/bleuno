# BLEuno 하드웨어

## 관리 구조

보드 회로도·PCB·프로젝트 전용 라이브러리는 펌웨어와 같은 저장소에서 관리합니다.
모델 아래에 보드 리비전을 둡니다. 현재 `type_m`은 원본 `v1`과 수정본 `v1.1`, `type_s`는 `v1`을 관리합니다.
KiCad 프로젝트명은 `<모델>_<리비전>`이며 `v1.1`은 `type_m_v1_1`로 표기합니다. 리비전별 폴더를 구분하고 펌웨어 버전과 독립적으로 관리합니다.

| 모델 / 리비전 | 원본과 용도 | 관리 경로 | 상태 |
|---|---|---|---|
| `type_m` / `v1` | MovingTarget / MotorConT1, 모터·서보 제어 | [type_m/v1](type_m/v1/README.md) · [핀맵](type_m/v1/pinmap.md) | KiCad 이전·원본 대조 완료, 회로·실물 검토 필요 |
| `type_m` / `v1.1` | v1의 SEN1–GPIO20 누락 배선·심볼 핀 이름 수정 + 전원부 수정(BAT 극성 표시, J2 직렬 스위치, J1 이름) | [type_m/v1.1](type_m/v1.1/README.md) · [KiCad 프로젝트](type_m/v1.1/kicad/type_m_v1_1.kicad_pro) | DRC 오류·미배선·정합 차이 0, ERC 오류 20/경고 26, 제작·실물 검증 남음 |
| `type_s` / `v1` | ZeroFactorySensorBoard / Board_DHT, 센서·입출력 커넥터 | [type_s/v1](type_s/v1/README.md) · [핀맵](type_s/v1/pinmap.md) | KiCad 이전·원본 대조 완료, 부품값·전기적/실물 검토 필요 |

### type_m v1

- [type_m/v1/README.md](type_m/v1/README.md): BLEuno type_m / v1 원본, 이전 결과와 남은 검토 항목.
- [type_m/v1/source/README.md](type_m/v1/source/README.md): 원본 `.epro2`, 가져오기용 `.epro`, 출처와 SHA-256.
- [type_m/v1/kicad/type_m_v1.kicad_pro](type_m/v1/kicad/type_m_v1.kicad_pro): 편집할 KiCad 프로젝트. 회로도·PCB와 프로젝트 전용 라이브러리를 함께 보관한다.
- [type_m/v1/validation/README.md](type_m/v1/validation/README.md): 원본 비교, ERC·DRC 결과, 설계 규칙 대응과 검증 명령.
- [type_m/v1/pinmap.md](type_m/v1/pinmap.md): 원본에서 확인한 부품·커넥터·GPIO 대응과 미연결 항목.

Git에는 원본 아카이브, KiCad 설계 파일, 프로젝트 전용 라이브러리와 검증 기록을 포함합니다.
개인 화면 설정, 잠금 파일, 자동 백업은 [`.gitignore`](.gitignore)로 제외합니다.

2026-10-31에 지정 EasyEDA 설계를 KiCad `10.0.6` 프로젝트로 이전했습니다.
부품 12개, 부품 패드 52개, 트랙 118개, 연결 네트 그룹 17개와 별도 홀 패드 2개의 원본 대조를 통과했습니다.
이전 완료는 제작 가능 또는 실물 동작 검증 완료를 뜻하지 않습니다. 원본의 SEN1·VIN 미연결과 ERC·DRC 검토 항목을 보존했습니다.

### type_m v1.1

[v1.1 수정본](type_m/v1.1/README.md)은 v1 원본을 보존한 별도 리비전입니다. 센서 단계에서 SEN1을 J7.2·R3.2·U1.15(GPIO20)로 연결하고 U1 심볼 pin15·16 이름을 GPIO20·GPIO21로 바로잡았으며, 2026-11-01 전원부 수정에서 배터리 커넥터 J1의 극성 표시(각 패드 위 `−`/`+`)와 이름(`MT1`→`BAT`)을 바로잡고 J1.1을 `VIN`으로 옮겨 전원 스위치 J2(1=OFF/NC, 2=COM=VCC, 3=ON=VIN)가 실제 직렬 경로가 되게 했습니다. [편집 프로젝트](type_m/v1.1/kicad/type_m_v1_1.kicad_pro)의 DRC 오류·미배선·회로도/PCB 차이는 0, ERC는 오류 20·경고 26입니다. 수정 전후 그림은 [전원부 수정 확인 화면](type_m/v1.1/validation/power-fix-2026-11-01/review.html)에 있습니다.

이미 실장한 v1의 물리 배선은 설계 파일 변경으로 고쳐지지 않습니다. J7.2–U1.15 점퍼는 기존 v1의 임시 보수 방법이며 정규 사용 절차가 아닙니다. v1.1은 이 연결을 PCB 배선으로 구현하는 수정본이고, 제작·실물 검증 완료를 뜻하지 않습니다. J2에 꽂는 스위치의 접점 정격과 USB·배터리 동시 연결 조건은 v1.1 안내의 "J2 스위치 사용 규칙"과 "전원 경로와 주의"를 따릅니다.
현재 펌웨어는 GPIO18/19의 USB CDC를 사용하고 GPIO20을 DHT 데이터에 사용합니다. GPIO20은 UART0 RX 겸용이므로 외부 UART0 RX와 동시에 사용하지 않습니다. [핀 기능 근거](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32c3/schematic-checklist.html)

### type_s v1

[type_s v1 안내](type_s/v1/README.md)에서 [원본](type_s/v1/source/README.md), KiCad 프로젝트와 [검증 기록](type_s/v1/validation/README.md)을 관리합니다.
KiCad 이전과 원본 대조를 마쳤으며, R1/R2의 원본 회로도·PCB 값 차이는 보존했습니다. 남은 ERC 오류·경고와 DRC 경고, 값 차이 검토는 검증 기록을 확인합니다.
원본은 모듈 1개·저항 5개·커넥터 9개로 구성됩니다. DHT·CDS·DS18B20은 헤더 표기이며 센서 실장·펌웨어 지원을 뜻하지 않습니다.
[핀맵](type_s/v1/pinmap.md)에 센서 신호 GPIO3/4/5, D1~D3 신호 GPIO6/7/10, BATCHK 분압과 원본 저항값 불일치를 기록했습니다. 제작·실물 동작은 검증하지 않았습니다.

## 교재와 앱의 역할

| 위치 | 역할 |
|---|---|
| 이 저장소 `src/`, `platformio.ini` | 보드 펌웨어 |
| 이 저장소 `hardware/type_m/v1/` | type_m 모델의 v1 하드웨어 |
| 이 저장소 `hardware/type_m/v1.1/` | SEN1–GPIO20 배선과 전원부(BAT 표시·J2 스위치·J1 이름)를 수정한 type_m 수정본 |
| 이 저장소 `hardware/type_s/v1/` | type_s 모델의 v1 하드웨어 |
| [teaching_repo/android_programming/bleuno](https://github.com/gbox3d/teaching_repo/tree/main/android_programming/bleuno) | 수업용 BLE 공통 코드와 프로토콜 설명 |
| [bleunoApp/BleunoTest](https://github.com/gbox3d/bleunoApp/tree/main/BleunoTest) | 보드 점검·강의자 시연 앱 |

기존 교재의 `ledpin=[4,3,1,0]`은 type_m v1의 GPIO0 분압 입력을 출력으로 사용하며,
`dht11pin=7`은 모터 드라이버 SLEEP과 충돌합니다. 원본 v1의 SEN1은 MCU 미연결이며 v1.1에서 GPIO20 배선을 추가합니다.
기존 LED·DHT11 실습을 그대로 적용하지 않고 [원본 핀맵](type_m/v1/pinmap.md)과 [v1.1 수정 내용](type_m/v1.1/README.md)을 구분해 확인합니다.

type_s v1에서도 기존 LED 설정은 센서·분압 입력과 충돌하며 `dht11pin=7`은 DHT 헤더가 아닌 D2에 해당합니다.
[type_s v1 핀맵](type_s/v1/pinmap.md)을 기준으로 외부 센서·공급 전압·모델별 설정을 검증한 뒤 실습에 연결합니다.
