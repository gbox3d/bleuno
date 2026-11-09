# Dev Log

## 목차

- [Entries](#entries)

## Entries

### 2026-10-31 — 교재 연계 및 하드웨어 V1 관리 준비

- 사용자 요청: Android 교재 적용 방식 판단, 지정 EasyEDA 마더보드의 KiCad 이전, `hardware/` 관리 및 forai-scaffold 적용.
- 교재 SmartIO와 BLE 공통 코드를 수업 정본으로, `bleunoApp/BleunoTest`를 점검·시연용으로 결정했다.
- `hardware/README.md`, `hardware/v1/README.md`, KiCad 로컬 상태용 `.gitignore`를 추가하고 루트 README에서 연결했다.
- 스킬의 `init_forai.py`를 설치된 Python 3.12 + `uv run --no-python-downloads`로 실행해 5개 문서를 만들고 이번 작업 문맥을 기록했다.
- 초기 EasyEDA 인증 만료를 해결하고 지정 원본 내보내기와 KiCad 이전까지 이어서 완료했다. 아래 후속 기록을 현재 상태로 읽는다.
- 펌웨어 소스 변경, 실기 업로드, commit/push는 수행하지 않았다.

### 2026-10-31 — 원본 보존·KiCad 이전과 검증 완료

- 지정 `MovingTarget/MotorConT1/PCB1`의 `.epro2` 원본과 호환 `.epro`를 내보내 보존하고 출처·SHA-256을 기록했다. 웹 원본 설계는 변경하지 않았다.
- KiCad `10.0.6`으로 가져온 뒤 `bleuno_v1` 프로젝트명과 로컬 라이브러리를 정리했다. 부품 참조를 U1/U2, R1–R3, J1–J7로 바꾸고 원래 PCB 실크를 동일 위치·스타일로 보존했다.
- 심볼–풋프린트 UUID 경로·값·라이브러리를 맞추고, 독립 홀 패드 MP1/MP2와 가상 전원/도면틀 참조를 정리했다. 외부 라이브러리에 빠진 도면틀 그래픽을 회로도 캐시와 동기화했다.
- 잘못 가져온 사각형 Edge.Cuts zone을 원본의 14개 꼭짓점 내부 절개로 복원했다. 부품·패드·트랙·네트와 회로도 기하는 유지했다.
- 원본 규칙을 근거가 있는 범위에서 `.kicad_pro`/`.kicad_dru`에 옮기고 판정 차이와 미대응 항목을 기록했다. 오류 심각도·예외나 핀 전기 유형을 변경해 검사 결과를 숨기지 않았다.
- 원본 비교는 부품 12개·부품 패드 52개·트랙 118개·비아 0개·연결 그룹 17개, 독립 홀 패드 2개 및 외곽/절개 선분 28개에 대해 통과했다. 보정 스크립트의 반복 실행 동일성도 확인했다.
- 원본 DRC는 물리 0개·netlist 알림 1개를 반환했다. KiCad ERC 최종 48개(오류 22, 경고 26)와 DRC 보고서를 보존했다. 회로의 SEN1·VIN 미완성과 기존 교재 LED/DHT 핀 충돌을 핀맵·검증 문서에 명시했다.
- 메타데이터 추가 동기화 후 회로도/PCB 차이는 0개가 되었다. 재계산 전 PCB/DRC를 보존하고 KiCad에서 동판을 재계산했으며 최종 DRC 오류 0개·텍스트 높이 경고 12개, 미배선 0개를 확인했다. 원본 비교도 다시 통과했지만 구리 내부 채움 형상이 원본과 동일하다고 주장하지 않는다.
- 회로도 GUI 재열기로 파싱·라이브러리 경고가 없음을 확인하고 누락된 `empty.kicad_wks`를 추가해 기본 프레임 겹침을 해결했다. 원본 도면틀 심볼은 보존했다.
- PCB GUI 재열기로 배선·중앙 14선 절개·외곽·GND 채움과 패드 54개·비아 0개·트랙 118개·미배선 0개를 확인했다. GUI의 net 24개는 연결 그룹 17개와 미연결 핀 개별 net 7개를 포함한다. 두 SVG 미리보기도 최종 파일로 갱신했다.
- hardware 안내·검증 안내와 `_forAI` 5개를 현재 실제 파일 기준으로 갱신했다. KiCad 이전 완료와 제작·실기 미검증 상태를 구분했다. 펌웨어 업로드나 실제 부하 구동은 수행하지 않았다.

### 2026-10-31 — type_m 모델과 v1 리비전 명칭 확정

- 사용자 지정 모델명 `type_m`과 리비전 `v1`을 반영해 관리 경로를 `hardware/v1/`에서 `hardware/type_m/v1/`로, KiCad 프로젝트명을 `bleuno_v1`에서 `type_m_v1`으로 변경했다.
- 위 이전 작업의 당시 경로·프로젝트명은 이력으로 남긴다. 현재 편집 정본은 `hardware/type_m/v1/kicad/type_m_v1.kicad_pro`다.
- 루트 안내, 하드웨어 안내, `_forAI`의 현행 경로와 명칭을 정리하고 일회성 검증 폴더의 `.gitignore` 경로에도 `type_m` 모델 계층을 반영했다.

### 2026-11-01 — type_s v1 원본과 교재 연계 등록

- 추가 모델 `type_s` / `v1`의 EasyEDA `ZeroFactorySensorBoard / Board_DHT` 원본 `.epro2`와 가져오기용 `.epro`를 `hardware/type_s/v1/source/`에 보존하고 `type_s_v1` KiCad 이전·원본 대조를 완료했다.
- `hardware/type_s/v1/pinmap.md`에 15개 부품·52핀의 MCU/센서·입출력 헤더/전원 연결을 기록했다. 회로도·PCB의 패드별 네트는 일치하며, 분압 저항의 회로도 220kΩ / PCB 상속값 10kΩ 차이는 원본 확인 사항으로 남겼다.
- DHT·CDS·DS18B20이 센서 실장이 아닌 헤더 표기임을 명시하고 기존 LED·DHT 수업 설정의 충돌과 현재 센서 명령 지원 범위를 기록했다.
- 참조·라이브러리를 정리하고 원본 실크 표시를 복원했다. 최종 ERC·DRC와 원본 값 차이에 따른 R1/R2 parity 항목은 `hardware/type_s/v1/validation/README.md`에 기록했다. 이전 완료를 회로·BOM·제작·실물 검증 완료로 해석하지 않는다.
- 하드웨어 모델 표, 루트 안내, `_forAI` 5개와 교재/조교 안내에서 type_s 원본 핀맵을 연결했다. 기존 type_m 내부 파일·원본과 런타임 코드는 변경하지 않았다.
- 제작 출력·펌웨어 업로드·실물 BLE/센서 검증·commit/push는 수행하지 않았다.

### 2026-11-01 — type_m 실장 보드 식별·펌웨어와 수업 자료 준비

- 사용자는 type_m v1을 설계대로 실장했다고 확인했으며, 모듈을 RGB가 있는 EstarDyn ESP32-C3 SuperMini Plus V2.0으로 특정했다. 이 확인을 통전·파형 계측 또는 부하 구동 성공과 구분했다.
- USB Serial/JTAG COM14에서 ESP32-C3 QFN32 rev0.4, XMC Flash 4MB, base MAC `8c:d0:b2:a8:54:d5`를 확인했다. 기존 4MB 이미지를 `.local/type_m/previous-firmware-8cd0b2a854d5-20261101.bin`에 보관했다. SHA256은 `48C5576155C2FAFBD1419CAFF46CAA99EE8A7DA0627FEDF4A74920D661F4ADC3`다.
- 기존 LED 환경을 유지하고 별도 `type_m` 환경/버전 1.0.0, `src/type_m_main.cpp`·`include/type_m_protocol.hpp`, espressif32 6.12.0/Arduino-ESP32 2.0.17 기준으로 모터·서보 프로토콜 구현을 진행한다.
- ASCII 20바이트, CCCD/about/ready 준비 절차, 모터 ±100·서보 0..180→1000..2000µs, STOP·1500ms watchdog 및 500ms heartbeat를 문서화했다. RGB는 GPIO8 NeoPixel 초기 설정이며 판매 첨부 PDF의 GPIO0 red 설명은 채택하지 않았다.
- `docs/type_m.md`와 루트 진입 안내, 별도 교재 `type_m_lab.md` 90분 실습·정지 확인표를 작성했다. `bleunoApp/TypeMControl`은 기존 SmartIO/Fake와 별도 Kotlin·XML 앱으로 진행한다.
- 비구동 BLE 검사용 `tools/type_m_smoke.py`와 기존 백업 보관 영역을 인벤토리에 기록하고 사용자 요청 범위의 `_forAI` 5개를 갱신했다. 문서 제목 바로 아래 목차와 기존 하드웨어 이력은 유지했다.
- 이 항목 작성 시 새 펌웨어 빌드/업로드·BLE·RGB 색·모터·서보 실기 결과는 아직 완료로 기록하지 않는다. 결과가 확인되면 검증 범위를 구분해 후속 기록한다. commit/push는 수행하지 않았다.

### 2026-11-01 — type_m 호스트 검사와 Android 설치 확인

- `tests/type_m_tests.cpp`를 기존 Python 3.12 + uv + Zig 0.13.0으로 C++11/경고 오류 처리 옵션을 적용해 컴파일했다. 399 checks PASS, 종료 코드 0이며 결과·재현 명령은 `.local/type_m/unit-tests.txt`에 보존했다. Python 설치·자동 다운로드는 하지 않았다.
- 파서 경계·준비/구독·watchdog 만료/wrap·오래된 요청·STOP epoch·서보 초기 OFF·모터 기동 및 방향 전환 간격을 검사했다. 이는 실제 GPIO 파형·부하 구동·BLE 검증이 아니다.
- `tools/type_m_status.py`를 USB 읽기 전용 about/status 조회 도구로 추가했다. 새 Android 패키지를 IM_L101(API34)에 설치하고 권한을 허용했다. 앱 최종 동작·ESP32 전체 빌드/업로드·BLE 통합 시험은 진행 중이다.
- 현재 type_m의 NeoPixel 의존성은 범위 `^1.12.2`, 해석된 버전은 1.15.5임을 기록했다. 펌웨어 빌드 도중 코드를 바꾸지 않고 결과 확인 후 문서를 이어서 갱신한다.

### 2026-11-01 — type_m 전체 빌드와 업로드 대상 재확인

- PlatformIO `type_m` 빌드 SUCCESS를 확인했다. RAM38,860/327,680 bytes(11.9%), 앱 Flash998,000/1,310,720 bytes(76.1%)이며 `.local/type_m/build.log`에 보존했다.
- Android 단위 테스트17개 PASS를 확인하고 IM_L101/API34에 새 APK를 설치했다. 초기 제어 비활성과 하단 고정 STOP의 실제 화면을 확인했다. BLE 통합 결과는 아직 미완료다.
- 업로드 직전 대상이 최초 COM14/base MAC `8c:d0:b2:a8:54:d5`에서 COM11/`8c:d0:b2:a8:b3:6e`로 바뀌었음을 확인했다. 현재 칩도 C3 QFN32 rev0.4/XMC 4MB다. 현재 대상은 별도 파일 `previous-firmware-8cd0b2a8b36e-20261101.bin`로 백업 진행하며 최초 보드 파일·해시와 혼동하지 않는다.

### 2026-11-01 — COM11 업로드·BLE 29개 검사·Android READY 확인

- 현재 b3:6e 보드의 4MB 백업을 완료했다. SHA256 `7C66D38A5FD54642DAA9F3293AD3DC2030592E104D85580B0D795571EA9F319C`. 두 보드 백업은 각각 보존한다.
- COM11에 업로드 SUCCESS와 Flash 데이터 hash 검증을 확인했다(`.local/type_m/upload-b36e.log`). 이미지 SHA256 `BC681C8C53B118E480AFC1855BFF9096FBDDAEE88F9F13AF187ED6066AD69BDB`.
- 광고 `ESP32_BLE_M_B36E`, BLE 주소 `8C:D0:B2:A8:B3:70`을 확인했다. base MAC b3:6e와 BLE 주소가 다름을 문서화했다.
- 최초 재연결 검사는 Windows의 실제 링크 해제 지연을 고정1초대기로 판단해 실패했다. smoke에 pyserial3.5와 `--serial-port`를 추가하여 보드 link=0 및 내부 정지 상태를 확인한 뒤 재검색하도록 보강했다. 이후 `.local/type_m/ble-smoke-b36e.json`의29checks PASS, movement_requested=false를 확인했다.
- 사용자는 업로드 후 “빨간색으로 깜박입니다”라고 확인했다. 파란 표시 확인은 대기 중이다.
- Android IM_L101/API34에서 광고 발견·CCCD/about/ready·제어 준비됨을 확인했다. 보드 내부 모터/PWM/서보duty/nSLEEP은0, 서보각도-1로 확인됐다. 앱 STOP/배경·재연결 후속 검사는 진행 중이다. 물리 모터·서보 구동 명령이나 파형 계측은 수행하지 않았다.

### 2026-11-01 — Android 정지·배경 전환·재연결과 RGB 확인 완료

- 사용자가 “파란색으로 계속 켜져 있습니다”라고 확인했다. 앞선 “빨간색으로 깜박입니다”와 함께 GPIO8 RGB의 대기 빨강 점멸·READY 파랑 고정이 실물 확인됐다. 준비 전 파랑 점멸은 소스·상태 기준이며 별도 사용자 색상 관찰로 기록하지 않는다.
- Android STOP의 `ok stop` 뒤 READY 유지, Home 배경 진입 시 STOP 응답 후 실제 단절, 앱 복귀 시 제어 비활성·`정지 확인 후 연결을 종료했습니다` 표시, 재연결 about/ready 성공을 확인했다.
- 시리얼 로그 `.local/type_m/android-ready-stop.txt`는 ready=1/epoch20, `android-background.txt`는 link=0/subscribed=0/ready=0/epoch23, `android-reconnected.txt`는 link=1/subscribed=1/ready=1/epoch24다. 모두 motor/PWM/servoDuty/nSLEEP=0, servo=-1로 확인했다.
- 펌웨어 가이드·루트 안내·교재 90분 실습·조교 안내와 `_forAI` 현재 상태를 갱신했다. KiCad/원본 핀맵은 변경하지 않았다. 0이 아닌 모터 명령·유효한 서보 각도 명령, 실제 부하 운동과 핀 파형 계측은 수행하지 않았다. 과거 진행 중 항목은 당시 기록으로 유지한다.

### 2026-11-01 — type_m DHT11·DHT22 지원 1.1.0

- 사용자는 수업 배치를 뒤로 미루고 type_m 완성을 우선했으며 센서를 DHT로 확정했다. DS18B20은 추가하지 않았다. GPIO20 입력, J7 3.3V/DATA/GND 및 R3 10kΩ 풀업을 유지한다. 원본 SEN1은 MCU와 미연결이므로 J7.2→U1 pad15(실크20) 점퍼가 필요하다. 원본 KiCad/EasyEDA 파일은 변경하지 않았다.
- 펌웨어1.1.0은 DHT11/22 선택, 기본11, 2500ms 측정, 5000ms 캐시 만료, 모델 변경 중 이전 샘플 폐기와 센서 오류 시 READY 유지를 구현했다. DHT 조회/선택은 actuator heartbeat를 연장하지 않는다. Adafruit 드라이버의 IRQ 차단 지연 한계는 문서에 명시했다.
- 호스트501개(기존399+센서102) 검사와 PlatformIO 빌드 성공. RAM38892, 앱Flash1000960. COM11/base MAC b3:6e에 업로드·Flash hash 검증 성공. 이미지 SHA256 1DF91B8E665700114642A46AD47B5B90213D0E1915350D1EAB4AE37B1AB828E8.
- BLE54개 검사 통과: DHT 종류 선택/대기/실패, 오류 후 READY 유지, STOP/watchdog/구독 해제/실제 단절/재연결. 최초 USB CDC 재열기 직후 status 미수신으로 검사 실패했으나 읽기 전용 재요청을 넣고 통과했다. .local/type_m/*-1.1.0.* 및 ble-smoke-b36e-1.1.0-retry.json에 기존1.0 기록과 구분해 보존했다. 모터·서보 구동을 요청하지 않았다.
- Android TypeMControl1.1.0은 DHT 선택 저장·재적용, 2.5초 조회/5초 수치 만료, 구버전 센서 기능 제한을 추가했다. JVM34개 통과, lint 오류·경고0, IM_L101/API34에 설치했다. 기본DHT11→DHT22 선택, 센서error 표시 중 READY 유지, STOP 후 출력0, Home 후 실제 단절·출력0, 프로세스 재실행 후 DHT22 선택 유지·제어 비활성을 확인했다.
- 사용자는 DHT22를 SENSOR1에 연결했다고 확인했다. 점퍼 연결을 포함하는 답변을 받았지만 실제 DHT22 조회는 error여서 별도 SEN1→GPIO20 점퍼 유무를 다시 확인 중이다. 실제 온습도 수치 성공으로 기록하지 않는다. 검증 중 첫 USB/BLE 소실은 사용자가 직접 분리했다고 확인했다. 재연결 후 앱 검사를 이어갔으며, 이후 보드가 다시 USB/광고에서 사라져 배선 확인·재연결을 기다린다.

### 2026-11-01 — SEN1 누락 증거와 KiCad v1.1 센서 배선 수정

- 사용자는 점퍼를 의도하지 않았으며, 센서 미동작을 먼저 확인·설명하지 않은 점을 지적하고 원본 설계 증거를 요청했다. 이후 “새로 고쳐주세요 키캐드에서요”라고 요청했다. 앞선 항목의 “점퍼 필요/확인 중”은 당시 임시 대응 기록이며 현재 정상 사용 절차로 적용하지 않는다.
- 원본 v1의 SEN1은 J7.2와 R3.2에만 연결되고 MCU에 연결되지 않음을 확인했다. 원본 근거는 `hardware/type_m/v1/validation/sen1-evidence-2026-11-01/`에 보존했다. 현재 전원 검토는 사용자 지시에 따라 KiCad만 근거로 진행한다.
- 원본 v1을 보존하고 `hardware/type_m/v1.1/kicad/type_m_v1_1.*`를 만들었다. 회로도/외부 심볼의 U1 pin15 이름은 GPIO20, pin16은 GPIO21로 정정하고 U1.15에 SEN1 라벨을 추가했다. pad16은 미연결이다.
- PCB에 SEN1 트랙6개(폭0.254mm)와 비아1개(외경0.6mm/드릴0.3mm)를 추가해 J7.2–R3.2–U1.15를 연결했다. 실크 `type_m v1.1`, 회로도 도면틀 Version v1.1/Update2026-11-01을 반영했다. 기존 Board MotorConT1/Project MovingTarget 명칭은 유지했다.
- Native KiCad 동판 재채움·DRC 결과 오류0/기존 실크 높이 경고12, 미배선0, schematic parity0. ERC는 오류21/경고27이다. 기존 오류22/경고26에서 U1.15 미연결 오류가 없어지고 가져온 핀 유형 조합 경고1개가 추가됐다. 핀 유형이나 규칙을 완화하지 않았다.
- 독립 `verify_revision.py`에서25/25 PASS: v1의57개 기록 파일 해시, 기존118트랙·28외곽/절개·54패드, 의도하지 않은 네트 변경 없음과 SEN1 실제 구리 접촉을 확인했다. 원래 기하·규칙 보존과 기능적 회로 완성은 다른 판정이다.
- 사용자 요청으로 KiCad 회로도/PCB와 연결 강조 화면을 열었다. native SVG/PDF/PNG와 설명용 `validation/preview/review.html`을 보존했다. 전원부의 후속 발견 때문에 센서 수정본을 제작 정본으로 표시하지 않는다.
- 최종 센서 작업본 PCB SHA256 `1105FF3D34DB037440E3A9B3378BE9334527ECE302C66043DF2A05D6D55889DA`, SCH `574C1CA528CBA32E3AD642419D7F4B3A9AAD8A8A8FA6D7D08B7733D3F197ABEA`.

### 2026-11-01 — 배터리 전원부 검토와 미수정 오류 확인

- 사용자는 배터리 전원이 들어오지 않는 것 같다고 지적했다. USB로는 부팅되고 배터리로는 부팅되지 않는다고 확인했다. 배터리는 처음3.7V라고 설명한 뒤4.9V가 측정된다고 보고했다. 측정 위치/전원 모듈 여부는 확인되지 않았다.
- 승압/부하 사양 확인을 먼저 제안하거나 사용자에게 반복 측정을 요청하자, 사용자는 “키캐드만 보세요”, “시키지좀 마세요..보드 확인 부터 해요”라고 명시했다. 이후 추가 측정·연결 변경을 요구하지 않고 저장된 KiCad의 설계 오류를 먼저 검토했다. 다음 세션에서도 이 사용자 요청을 유지한다.
- `power-audit-2026-11-01/audit_power.py`는 네트 이름으로 객체를 합치지 않고 패드·트랙·실제 filled zone polygon의 구리 접촉을 검사했다. 별도 `independent_power_review.py`는 KiCad native 연결 계산과 exact endpoint 경로를 확인했다.
- **BAT+→C3 배선 누락은 확인되지 않았다.** J1.1(VCC)→U1.8은 실제 F.Cu8개 배선으로 이어지고 U2.9/J5.1/J6.1 등에도 연결된다. BAT− J1.2는 실제 GND 채움 영역을 통해 U1.7과 각 부하 GND에 연결된다. KiCad 설계 연결과 실물 통전 상태는 구분했다.
- **전원 스위치 설계 오류:** J1.1과 J2.2가 같은 VCC에 연결되어 배터리가 부하에 직결된다. J2.3의 VIN과 J2.1은 각각 다른 구리에 닿지 않은 단독 패드다. J2는 부하 전원을 차단하는 직렬 경로가 아니다.
- **BAT 극성 안내 오류:** KiCad 전면 기준 왼쪽 원형 J1.2는 GND(−), 오른쪽 사각 J1.1은 VCC(+)다. 실크 문자열은 `BAT + -`로 좌우 순서가 반대다. J1/문자 회전0°·미러false이며 native SVG stroke 좌표로도 확인했다. 단일 문자열이 패드 중심에 정렬된 표시는 아니므로 실제 사용자 역극성을 확정하지 않는다.
- **명칭 오류:** 배터리 J1의 회로도/PCB 값이 `MT1`이다. 실제 모터 출력 J3과 혼동할 수 있다.
- U1 내부 레귤레이터/USB 전원 회로와 실제 모듈 전원 핀 식별 실크는 KiCad에 없다. KiCad 내부의 pin8=+5V/VCC,7=GND,6=+3V3/3.3V 대응은 확인했지만 실물 모듈 내부 동작까지 입증하지 않았다. 3.7V 직결 실패·승압 필수·실제 역극성·부품 손상을 단정하지 않는다.
- 근거: `hardware/type_m/v1.1/validation/power-audit-2026-11-01/`의 `design-review.md`, `power-audit.json`, `independent-power-review.json`, `battery-polarity-review.png`. 확대 그림은 native PCB SVG의 표시 영역을 확대한 것으로 배선/인쇄를 수정한 그림이 아니다.
- 이 전원 검토에서는 설계 파일을 변경하지 않았다. BAT 극성 안내·J2 직렬 연결·J1 이름이 다음 수정 대상이다. 승압은 사양 확인 없이 추가하지 않는다. 원래 BATADC 분압이 전원 네트 변경 뒤에도 실제 배터리를 읽도록 검토해야 한다.

### 2026-11-01 — 다음 세션 인수인계용 _forAI 갱신

- 사용자 명시 요청: “작업할 내용들 forai 에 정리해서 남겨요 다른 세션에서 이어갈수잇게요.” forai-scaffold 지침에 따라 README/inventory/memo/plan/dev_log 5개를 갱신했다.
- 최신 우선순위·v1/v1.1 구분·센서 수정 완료·전원부 미수정3건·검증 파일 경로·불확정 사항·사용자 요청을 기록했다. plan의 점퍼 재확인과 v1 편집 정본 등 오래된 현행 지시를 교정하고 이력은 보존했다.
- 이번 인수인계 정리는 문서 작업이다. 펌웨어/앱·KiCad 설계 수정, 업로드, 구동 시험, 새 Python 설치/다운로드, commit/push는 수행하지 않았다. 기존 미커밋 작업과 로컬 Flash 백업을 보존한다.

### 2026-11-01 — type_m v1.1 전원부 수정 적용과 검증

- 사용자 요청: "키캐드관련한 스킬있다면 사용해서 type-m의 잘못된점 수정부탁합니다" + `/forai-scaffold` `/karpathy-coding`. KiCad 전용 스킬은 없어 KiCad 10.0.6 `kicad-cli`와 `pcbnew` Python으로 직접 수정했다. 사용자는 작업 중 KiCad GUI를 닫았다고 확인했다(lock 파일 제거·파일 해시 불변 확인 후 적용).
- 수정 전 회로도·PCB·netlist·ERC·DRC를 `validation/power-fix-2026-11-01/before-power-fix.*`로 보존했다(해시는 센서 단계 `revision-comparison.json`의 입력과 일치). 복사본 드라이런 2회 → 3인 독립 설계 리뷰(전기/레이아웃/파일 형식, 모두 조건부 승인) → 지적 반영(기준 해시·lock 가드, `BAT` 글자 원위치 유지, no_connect 삽입 순서, 프로젝트 파일 미기록) → 실제 적용 순서로 진행했다.
- 회로도(`patch_schematic_power.py`): J1 값 `MT1`→`BAT`, J1.1의 VCC 전원 심볼 #PWR012 제거, 그 자리에 `VIN` 전역 라벨, J2.1에 no_connect. PCB(`patch_pcb_power.py`, pcbnew): J1.1 네트 VCC→VIN, J1.1에서 내려오는 1.0 mm 배선 3개 VIN 재지정, VCC 노드로 가던 대각 배선 66ef6120 삭제, (172.385,96.731)→J2.3 1.0 mm F.Cu 배선 추가, J1 값 `BAT`, 실크 `BAT + -`→`BAT`(원위치), 패드 위 `−`/`+` 0.8 mm 추가, 동판 재채움. `.kicad_pro`·`.kicad_sym`·`.kicad_dru` 바이트 동일.
- 결과: netlist VIN={J1.1,J2.3}, VCC={J2.2,J5.1,J6.1,R1.1,U1.8,U2.9}; ERC 21/27→20/26(J2.1 미연결 오류·VIN 고립 라벨·J1.1–전원심볼 경고 해소, J1.1–J2.3 Unspecified 경고 1 추가); DRC 오류0·같은 경고12·미배선0·parity0; `verify_power_fix.py` 34/34 PASS(허용 외 변경 없음, 구리 접촉 그래프 J1.1→J2.3만, 다른 네트 접촉 0, SEN1 유지, v1 57개 파일 불변, 기록 파일 최신성). 규칙·핀 유형 완화 없음. 사후 독립 검증 3인(파일/문서/스크립트)의 지적을 반영해 깨진 링크, 과거형 표시 누락, 조건부 표현, 스크립트 가드 설명, 바이트코드 미생성, 보고서 최신성 검사를 보완했다.
- 미리보기 SVG/PDF/PNG를 재생성하고 `power-fix-2026-11-01/review.html`에 전후 확대 그림·J2 핀 역할·검증표·주의를 정리했다. 리뷰 지적에 따라 J2 접점 정격(소형 슬라이드 0.3–0.5 A, 부하 예산 확인 전 2–3 점퍼), USB·배터리 동시 연결(판매자 매뉴얼: 5V 핀 5 V±0.2 V, Type-C 동시 급전 금지; OFF는 배터리만 분리, ON+USB 금지), CHKBAT 유지 근거(110 kΩ 테브냉, OFF 무전류)를 문서화했다.
- 갱신 문서: `hardware/type_m/v1.1/README.md`(전면 재작성), `hardware/README.md`, `docs/type_m.md`, 루트 `readme.md`, `design-review.md` 적용 상태 주석, `_forAI` 5개. 리비전 표기는 제작 전이라 v1.1 유지(센서 스냅샷으로 분리 가능).
- 하지 않은 것: PCB 제작·실물 통전·배터리/모터/서보 검증, 승압 추가, 펌웨어 변경/업로드, ERC 잔여 항목의 핀 유형 수정, commit/push, 새 Python 설치. 배터리 부팅 실패 원인은 미확정이다.
