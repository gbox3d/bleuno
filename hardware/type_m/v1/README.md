# BLEuno type_m v1

## 기준 원본과 현재 상태

2026-10-31에 지정 EasyEDA **MovingTarget → MotorConT1 → PCB1**과 연결 회로도를 KiCad `10.0.6`으로 이전했습니다.
편집은 [kicad/type_m_v1.kicad_pro](kicad/type_m_v1.kicad_pro)를 열어 시작합니다.
원본 파일 보존, 참조명·라이브러리·회로도/PCB 링크 정리 및 원본 데이터 대조를 완료했습니다.
전기적 미완성 항목과 검사 오류는 남아 있으므로 제작 승인이나 실물 검증 완료 상태로 해석하지 않습니다.

- [사용자가 지정한 EasyEDA 설계](https://pro.easyeda.com/editor#id=a6f9cc95756f415597ebfd40a028dfdd,tab=*230020836b7d42a2ae0bd4e6ece12948@a6f9cc95756f415597ebfd40a028dfdd)
- 프로젝트 ID: `a6f9cc95756f415597ebfd40a028dfdd`
- PCB ID: `230020836b7d42a2ae0bd4e6ece12948`
- 회로도 ID: `dd2cfae3a64d4b3db5dd5a512ac44bc2`
- 하드웨어 모델: `type_m`, 리비전: `v1`. KiCad 프로젝트명은 `type_m_v1`입니다. 펌웨어 `1.0.5_dev` 및 `about` 응답의 `cronos-v1`과 독립적입니다.
- 원본 `.epro2`와 호환 `.epro`의 출처·내보내기 시간·해시는 [source/README.md](source/README.md)에 기록했습니다. 웹 원본 설계는 수정하지 않았습니다.

## 파일 구성

| 위치 | 내용 |
|---|---|
| [source/](source/) | 현재 EasyEDA 형식 원본과 KiCad 가져오기용 아카이브 |
| [kicad/type_m_v1.kicad_pro](kicad/type_m_v1.kicad_pro) | 프로젝트와 원본에서 대응시킨 설계 규칙 설정 |
| [kicad/type_m_v1.kicad_sch](kicad/type_m_v1.kicad_sch) | 편집할 회로도 |
| [kicad/type_m_v1.kicad_pcb](kicad/type_m_v1.kicad_pcb) | 편집할 PCB |
| [kicad/type_m_v1.kicad_dru](kicad/type_m_v1.kicad_dru) | 객체 유형별 원본 설계 규칙 대응 |
| `kicad/type_m_v1.kicad_sym`, `kicad/type_m_v1.pretty/` | 프로젝트 전용 심볼·풋프린트. 라이브러리 표는 `${KIPRJMOD}` 상대 경로 사용 |
| [kicad/empty.kicad_wks](kicad/empty.kicad_wks) | 원본 도면틀 심볼을 유지하며 KiCad 기본 도면틀의 중복 표시를 막는 빈 worksheet |
| [pinmap.md](pinmap.md) | 원본 부품·커넥터·GPIO·전원 연결 및 교재와의 차이 |
| [validation/README.md](validation/README.md) | 검사 결과와 재검증 방법 |

## 이전 중 보정한 항목

- 원본의 중복 `220k`, 공백·줄바꿈이 있는 참조명을 `U1/U2`, `R1–R3`, `J1–J7`로 정리했습니다. 원본 이름은 숨김 속성에, PCB 실크 문구는 같은 위치·스타일의 사용자 텍스트에 보존했습니다.
- 물리 부품 12개의 심볼 UUID와 풋프린트 경로를 연결하고 값·풋프린트 ID·프로젝트 이름을 맞췄습니다. 별도 홀 패드는 `MP1/MP2`, 전원 심볼은 고유 `#PWR` 참조명, 도면틀은 `#FRAME1`로 관리합니다.
- importer가 원본 내부 절개를 Edge.Cuts의 사각형 zone으로 바꾼 문제를 원본 14개 꼭짓점 폐루프로 복원했습니다. 기존 외곽과 합쳐 Edge.Cuts 선분 28개를 대조했습니다.
- 외부 라이브러리에 비어 있던 도면틀 정의를 회로도 캐시와 동기화했습니다. 원본 도면틀 그래픽을 보존하고 누락된 `empty.kicad_wks`를 추가했습니다.
- [원본 설계 규칙](validation/rules-mapping.md)을 근거가 있는 범위에서 `.kicad_pro`와 `.kicad_dru`에 대응시켰습니다. 원본과 검사 대상 분류가 다른 항목은 문서에 남겼습니다.

기존 부품·패드·트랙의 기하와 핀 사이 연결은 유지했습니다. 미연결 핀 7개에는 KiCad 회로도가 생성한 개별 네트 이름만 부여했으며 다른 핀과 연결하지 않았습니다. 재현 스크립트와 변경 전 가져오기 파일은 `validation/`에 보존되어 있습니다.
기존 가져오기 동판 채움은 별도로 보존한 뒤 KiCad에서 재계산했습니다. 두 GND zone의 외곽·레이어·네트는 같지만 채워진 구리 내부 형상이 원본과 동일하다고 주장하지 않습니다. 세부 범위는 [검증 기록](validation/README.md)을 확인합니다.

## 검증 결과와 한계

[import-comparison.json](validation/import-comparison.json)의 `checked_comparisons_pass`는 `true`입니다.
부품 12개·부품 패드 52개·트랙 118개·비아 0개·연결 네트 그룹 17개, 별도 홀 패드 2개와 외곽/절개 선분 28개를 원본과 대조했습니다.
회로도 netlist와 PCB 패드 연결도 일치합니다. 구리 영역은 외곽·레이어·네트를 검사했으며 채워진 동판 전체의 제조 적합성까지 증명한 결과는 아닙니다.

원본 EasyEDA 검사는 물리 DRC 오류 0개와 회로도/PCB netlist 알림 1개를 반환했습니다.
KiCad ERC는 **48개: 오류 22개, 경고 26개**입니다. 연결되지 않은 핀, 구동되지 않은 입력/전원 핀, 핀 유형 조합과 고립 라벨을 포함합니다.
오류를 없애기 위해 전기적 핀 유형이나 연결을 바꾸지 않았습니다.
최종 KiCad DRC는 **오류 0개, 경고 12개**이며 모두 원래 실크 문구의 `text_height` 경고입니다.
미배선 연결은 0개, 회로도/PCB 일치 검사 차이는 0개입니다. 이는 원본 회로의 기능적 미연결이나 ERC 오류가 해결됐다는 뜻은 아닙니다.
회로도 GUI 재열기에서 파싱·라이브러리 경고가 없고 도면틀 중복 표시가 해소된 것을 확인했습니다.
PCB도 GUI에서 다시 열어 배선·중앙 절개·외곽·GND 채움을 확인했습니다. 표시된 패드 54개, 비아 0개, 트랙 118개와 미배선 0개가 검사 결과와 일치합니다.
검사 도구 간 판정 차이는 [검증 기록](validation/README.md)과 [규칙 대응](validation/rules-mapping.md)을 확인합니다.

## 다음 회로·실기 검토

1. **SEN1은 MCU에 연결되지 않았습니다.** J7 pin2와 R3만 연결됩니다. 센서 커넥터가 있다는 사실만으로 DHT11 기능이 완성된 것은 아닙니다.
2. **VIN은 J2 pin3에만 존재합니다.** 배터리 VCC는 부하에 직결되어 있으므로 J2를 전체 전원 차단 스위치로 설명하지 않습니다.
3. MCU 모듈의 원본 `GPIO22` 표기, 드라이버 모듈의 핀 이름과 실제 부품을 대조해야 합니다. 전압·극성·USB/외부 전원 관계도 실물 확인 대상입니다.
4. 기존 `ledpin=[4,3,1,0]`은 CHKBAT의 GPIO0을 출력으로 사용하고 `dht11pin=7`은 SLEEP과 충돌합니다. 기존 교재 설정을 그대로 저장하지 않습니다.
5. 기존 `pwm` 명령을 서보 각도 제어로 해석하지 않습니다. 모터 정지·활성화·서보 제어와 앱 화면은 별도 구현·검증이 필요합니다.

수업 정본은 `teaching_repo/android_programming`의 SmartIO와 BLE 공통 코드이며 `bleunoApp/BleunoTest`는 점검·시연용입니다.
이번 이전에서 펌웨어 빌드·업로드, Android 빌드, 실제 BLE 연결과 모터·서보·센서 구동은 검증하지 않았습니다.
