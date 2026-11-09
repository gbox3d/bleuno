# Memo

## 목차

- [재개 시 사용자 작업 원칙](#재개-시-사용자-작업-원칙)
- [제품 기준선](#제품-기준선)
- [기본 설정값](#기본-설정값)
- [type_m 펌웨어와 실장 보드](#type_m-펌웨어와-실장-보드)
- [하드웨어 이전과 검증](#하드웨어-이전과-검증)
- [전원 설계 검토와 미확정 사항](#전원-설계-검토와-미확정-사항)
- [type_s 원본과 적용 조건](#type_s-원본과-적용-조건)
- [런타임 구조 메모](#런타임-구조-메모)
- [동작 규칙](#동작-규칙)
- [반복 금지](#반복-금지)

## 재개 시 사용자 작업 원칙

- 2026-11-01 사용자 명시: **“KiCad만보세요”**, **“시키지좀마세요 보드확인부터”**. 먼저 저장된 KiCad 회로도·PCB와 감사 자료를 검토한다. 사용자에게 반복 측정·점퍼 작업·부하 사양 제공을 요구하는 방식으로 되돌아가지 않는다.
- 2026-11-01 v1.1 전원부 설계 오류 3건을 KiCad 파일에서 수정·검증했다(아래 "전원 설계 검토와 미확정 사항"). 3.7V라는 명목값만으로 승압 추가를 확정하거나 실물 역극성·부품 손상을 단정하지 않는다. 배터리 부팅 실패의 원인은 여전히 미확정이다.
- KiCad 파일을 스크립트로 수정할 때는 GUI(eeschema/pcbnew)를 먼저 닫는다. 열린 채 수정하면 GUI 저장이 변경을 덮어쓴다. 패치 스크립트는 `~*.lck`와 기준 해시를 확인하고 아니면 거부한다. GUI 재열기 전 `kicad-cli`로 ERC/DRC를 먼저 돌린다.
- 사용자는 점퍼 조작을 전제로 하지 않았다. 점퍼는 이미 제작된 원본 v1의 임시 보수 방법일 뿐 정규 사용 절차가 아니다. 센서 연결은 별도 v1.1 설계에서 수정했고, 파일 변경이 실장된 v1을 자동 수리하지는 않는다.
- 현행 검토 프로젝트는 `hardware/type_m/v1.1/kicad/type_m_v1_1.kicad_pro`다. 원본 `hardware/type_m/v1/`을 보존하고 EasyEDA 재가져오기·초기 정규화·센서 패치 스크립트를 반복 실행하지 않는다.
- Python은 이미 설치된 `uv`와 인터프리터만 재사용한다. 먼저 `uv python list --only-installed`, 실행 시 `--no-python-downloads` 또는 `UV_PYTHON_DOWNLOADS=never`. Python/가상환경/패키지 실행도 uv를 사용하며 새 Python을 설치하지 않는다.
- 이번 `_forAI` 갱신은 사용자의 “작업할 내용들 forai에 정리해서 남겨요 다른 세션에서 이어갈수있게” 요청으로 승인됐다. 이후 일반 코드 작업의 부수 효과로 `_forAI`를 자동 수정하거나 commit하지 않는다.

## 제품 기준선

- 기존 LED/DHT11 펌웨어 `1.0.5_dev`, 새 모터·서보·DHT `type_m` 펌웨어 `1.1.0`. Arduino/PlatformIO, ESP32 계열. 수업 배치는 보류하고 type_m 완성을 우선한다.
- 사용자 지정 하드웨어 모델명은 `type_m`이다. 원본 `v1`/`type_m_v1`은 보존하며, 센서 배선·전원부 수정본은 `hardware/type_m/v1.1/`의 `type_m_v1_1`이다. 하드웨어 `v1.1`과 펌웨어·앱 `1.1.0`은 다른 버전이다.
- 추가 모델은 `type_s` / `v1`이다. 관리 경로는 `hardware/type_s/v1/`, KiCad 프로젝트명은 `type_s_v1`로 구분한다. 기존 type_m 내부 설계·원본은 유지한다.
- BLEuno type_m / v1의 기준 원본: EasyEDA 프로젝트 `a6f9cc95756f415597ebfd40a028dfdd`, 지정 문서 `230020836b7d42a2ae0bd4e6ece12948`.
- KiCad `10.0.6`이 설치되어 있으며 새 설치는 필요 없다.

## 기본 설정값

- Service UUID: `c6f8b088-2af8-4388-8364-ca2a907bdeb8`.
- Characteristic UUID: `f6aa83ca-de53-46b4-bdea-28a7cb57942e`.
- 광고 이름 앞부분: `ESP32_BLE`.
- 교재 기존 LED 인덱스 `0~3`의 GPIO 예시는 `[4,3,1,0]`. V1 마더보드의 확정 핀맵이 아니다.
- 실제 `ledpin`, `pwmpin`, `dht11pin`은 보드의 저장 설정에 따른다.

## type_m 펌웨어와 실장 보드

- 센서는 DHT11·DHT22로 확정했다. 기본11, 앱 모델 선택. J7.1=3.3V/J7.2=SEN1/J7.3=GND, R3 10kΩ 유지. 원본 v1의 SEN1은 MCU 미연결이고, **v1.1 회로도·PCB에서 J7.2/R3.2/U1.15(GPIO20)를 연결**했다. U1.15는 모듈 패드 번호이며 GPIO 번호는20이다. 이미 실장된 v1의 물리 배선은 별개이고 사용자에게 점퍼를 필수 조작으로 안내하지 않는다.
- `dht 11|22` 설정과 `dht` 조회는 READY에서 사용하며 actuator heartbeat를 갱신하지 않는다. 수치는 0.1°C/0.1%RH 정수, 실패는 `dht <type> error`, 첫 대기는 `wait`. 2500ms 측정·5000ms 캐시 만료이며 오류만으로 BLE·모터 제어를 해제하지 않는다.
- Adafruit DHT1.4.6은 읽는 동안 인터럽트를 차단한다. 별도 낮은 우선순위 작업과 캐시를 사용해도 단일 코어 C3에서 STOP/안전 작업이 전혀 지연되지 않는다고 설명하지 않는다. DS18B20은 추가하지 않았다.

- 사용자는 type_m v1을 설계대로 실장했다고 확인했다. MCU는 EstarDyn ESP32-C3 SuperMini Plus V2.0 빨간 RGB 보드다. 원본 v1의 SEN1/VIN 미연결은 v1에 그대로 보존했고, v1.1에서 SEN1(1단계)과 전원부(2단계, 2026-11-01)를 수정했다. 남은 ERC 오류20/경고26은 미연결 핀 5·전원 심볼 미구동 2·핀 유형 13/26 항목이며, 실장 완료는 통전·계측 완료와 다르다.
- 최초 칩 식별: ESP32-C3 QFN32 rev0.4, XMC 4MB, USB Serial/JTAG COM14, base MAC `8c:d0:b2:a8:54:d5`. 현재 업로드 직전 대상은 COM11/base MAC `8c:d0:b2:a8:b3:6e`로 바뀌었고 동일한 칩/Flash 사양이다. BLE 주소는 검색으로 별도 확인한다.
- 기존 4MB 백업: `.local/type_m/previous-firmware-8cd0b2a854d5-20261101.bin`, SHA256 `48C5576155C2FAFBD1419CAFF46CAA99EE8A7DA0627FEDF4A74920D661F4ADC3`. `.local/`은 git 제외이며 백업을 삭제하지 않는다.
- 현재 COM11/b3:6e 백업 경로는 `.local/type_m/previous-firmware-8cd0b2a8b36e-20261101.bin`이다. 4MB 완료, SHA256 `7C66D38A5FD54642DAA9F3293AD3DC2030592E104D85580B0D795571EA9F319C`. 최초 보드의 해시를 재사용하지 않는다.
- [type_m 가이드](../docs/type_m.md)의 `type_m` 환경은 espressif32 6.12.0 / Arduino-ESP32 2.0.17을 고정하고 기존 LED 엔트리포인트와 분리한다. 업로드·BLE 통합과 Android READY는 확인했고 물리 모터·서보 출력 계측은 하지 않았다.
- M1=GPIO1/3, M2=GPIO4/5, nSLEEP=GPIO7, Servo1/2=GPIO6/10. GPIO0은 CHKBAT 입력으로 유지한다. 서보 논리 0..180은 1000..2000µs이며 최초 명령 전 펄스가 없고 실제 기구 각도 보장은 아니다.
- RGB 기본 NeoPixel GPIO8은 기존 TENSTAR 코드·사용자의 RGB 확인을 근거로 한다. [제품 첨부 PDF](https://ae-pic-a1.aliexpress-media.com/kf/S7a771a2922204066a89753ebae6c070a1.pdf)의 GPIO0 red/GPIO8 green 설명은 제품 사진과 달라 채택하지 않는다. 로컬 근거는 `.local/type_m/estardyn-manual.pdf`. 임의로 GPIO0을 LED 출력으로 바꾸지 않는다.
- 업로드 뒤 대기 빨강 점멸과 READY 파랑 고정은 사용자 확인을 받았다. 준비 전 파랑 점멸은 소스·상태 기준이며 사용자 색상 관찰 결과에 포함하지 않는다. 광고 `ESP32_BLE_M_B36E`, 실제 BLE 주소 `8C:D0:B2:A8:B3:70`은 base MAC b3:6e와 다르다.
- 광고 앞부분 `ESP32_BLE_M_`, 기존과 같은 service/characteristic UUID다. CCCD 구독 후 `about`의 `type_m/1 fw=1.1.0`을 확인하고 `ready`/`ok ready`를 거쳐야 제어한다. 앱은 1.0.x의 모터·서보 호환도 유지하되 DHT는 비활성화한다. 대기 빨강 500ms 점멸, 연결 파랑 250ms 점멸, READY 파랑 고정이다.
- 한 write당 ASCII 20바이트 이하, 응답은 JSON이 아니다. `motor 1/2 -100..100`, `servo 1/2 0..180`, `stop`, `ping`을 사용한다. 앱은 개행 없이 전송하며 펌웨어의 선택적 종단 LF/CRLF도 20바이트 제한에 포함한다.
- STOP은 양 모터와 양 서보 펄스를 끄고 기존 READY를 유지한다. 연결/구독 해제와 1500ms watchdog은 READY도 해제한다. 앱 heartbeat는 500ms이며 배경 진입 시 STOP 요청·연결 해제한다.
- `tools/type_m_smoke.py`는 about/구독/ready/거부/STOP/watchdog/재연결을 시험하며 실제 구동 명령을 내지 않는다. 성공을 물리 출력·RGB·Android 실기 성공으로 확대하지 않는다.
- 현행 **1.1.0은 호스트501/BLE54/Android34 PASS**, 전체 펌웨어 빌드·업로드/hash 검증과 앱 lint 문제없음이다. `.local/type_m/verification-1.1.0.json`, `unit-tests-1.1.0.log`, `ble-smoke-b36e-1.1.0-retry.json`, 앱 `TypeMControl/README.md`를 참조한다. 1.0.0의399/29/17을 현행 결과로 인용하지 않는다.
- 1.1.0 Android는 DHT22 선택/프로세스 재시작 후 저장, STOP 후 READY·출력0, Home 후 실제 단절·출력0을 확인했다. 사용자가 DHT22를 처음 서보 포트에 꽂았다가 SENSOR1으로 옮긴 뒤 재연결과 저장 DHT22 재적용까지 성공했다. **실제 온습도는 계속 error이며 성공 수치를 얻지 못했다.** v1.1 PCB 제작·실측, 모터·서보 부하/파형도 미검증이다.
- 이전 **1.0.0** BLE29checks는 `.local/type_m/ble-smoke-b36e.json`에서 PASS했다. Windows BLE 객체 종료 후 1초 대기만으로 실제 단절을 가정해 최초 시험이 실패했고, `--serial-port COM11`로 보드 link=0/정지상태를 기다린 뒤 재검색하도록 보강했다. 새 연결에서 ready가 다시 필요함을 확인했다.
- Android STOP은 `ok stop` 뒤 READY를 유지했다. Home 배경 진입은 STOP 응답 후 단절됐고 `link=0 subscribed=0 ready=0`이었다. 복귀 시 제어 비활성·정지 확인 문구를 보였으며 재연결 about/ready 뒤 다시 준비됐다. 각 단계에서 motor/PWM/servoDuty/nSLEEP은0, servo는-1이었다. `.local/type_m/android-*.txt`는 내부 상태 기록이며 출력 파형 계측이 아니다.
- IM_L101/API34에서 검색·CCCD/about/ready와 제어 준비됨을 확인했다. 해당 시점 내부 상태는 link/subscribed/ready=1, motor0,0/pwm0,0,0,0/servo-1,-1/servoDuty0,0/sleep0이다. 파형 측정이나 부하 구동 성공이 아니다.
- 이전 **1.0.0** 호스트399 checks는 `.local/type_m/unit-tests.txt`에 기록했다. 파서·제어 상태 검증이며 ESP32 빌드·BLE·파형 검증과 구분한다. `tools/type_m_status.py`는 about/status만 요청하는 USB 읽기 전용 도구다.
- 이전 **1.0.0** PlatformIO 빌드는 SUCCESS, RAM11.9%·앱 Flash76.1%다(`.local/type_m/build.log`). 당시 Android 단위17개와 초기 화면 기록도 현재 1.1.0 결과와 구분한다.
- type_m NeoPixel 의존성은 현재 `^1.12.2`이며 이번에 해석된 버전은 1.15.5다. espressif32 6.12.0 고정과 라이브러리 범위 지정은 다르다. 빌드 중 의존성을 변경하지 않는다.

## 하드웨어 이전과 검증

- 현재 편집·검토 정본은 [v1.1 프로젝트](../hardware/type_m/v1.1/kicad/type_m_v1_1.kicad_pro)다. 원본 [v1 프로젝트](../hardware/type_m/v1/kicad/type_m_v1.kicad_pro)는 비교 기준으로 보존한다. 원본 `.epro2`·`.epro` 해시는 `hardware/type_m/v1/source/README.md`에 있다.
- **v1.1 센서 수정(1단계):** SEN1=J7.2/R3.2/U1.15(GPIO20), 심볼 pin15 `GPIO21→GPIO20`·pin16 `GPIO22→GPIO21`, 새 트랙6개·비아1개. 이 단계에서는 기존 부품 위치·전원·외곽·배선을 유지했다(이후 2단계 전원부 수정으로 J1.1 네트와 VCC 배선 일부가 바뀜). 현재 USB HWCDC는 GPIO18/19이며 GPIO20은 DHT로 사용한다. GPIO20의 UART0 RX를 외부 UART와 동시에 사용하지 않는다.
- v1.1 센서 단계 [독립 변경 대조](../hardware/type_m/v1.1/validation/revision-comparison.json)는 **25/25 PASS**, 원본 v1의 기록한57개 파일 해시 불변. 당시 DRC 오류0/기존 실크 높이 경고12, 미배선0/parity0, ERC 오류21/경고27. 그 입력 파일(회로도·PCB·netlist·ERC·DRC)은 `validation/power-fix-2026-11-01/before-power-fix.*`로 보존했고 `verify_revision.py`는 그 스냅샷 기준이라 현재 파일에는 그대로 통과하지 않는다. 현재 파일의 검사는 `verify_power_fix.py`(34/34 PASS)다.
- [연결 확인 화면](../hardware/type_m/v1.1/validation/preview/review.html)은 네이티브 CAD SVG 위 설명용 표시다. 같은 폴더의 PCB SVG/PNG, 회로도 SVG/PNG/PDF가 있다. 실제 연결 판단은 저장 KiCad·netlist·연결 계산을 우선한다.
- 아래 원본 이전 수치와 SEN1/VIN 설명은 **v1** 기록이며 v1.1의 센서 수정과 구분한다.
- 원본 부품 12개·부품 패드 52개·트랙 118개·비아 0개·연결 그룹 17개와 독립 홀 패드 2개를 대조했다. 내부 절개의 importer 오류를 원본 14개 꼭짓점으로 복원했고 외곽/절개 선분 28개가 일치한다.
- 원본 EasyEDA 검사는 물리 DRC 0개·netlist 알림 1개였다. KiCad ERC는 48개(오류 22, 경고 26), 최종 DRC는 오류 0개·텍스트 높이 경고 12개이고 미배선/회로도·PCB 차이는 0개다. 검사 도구의 객체 분류와 규칙이 달라 오류 수만으로 우열이나 동일성을 판단하지 않는다.
- 동판은 원본 규칙을 대응시킨 KiCad에서 재계산했다. 두 zone의 외곽·레이어·네트는 같지만 내부 채움 형상까지 원본과 동일하다고 설명하지 않는다. 재계산 전 PCB/DRC와 원본은 보존되어 있다.
- `normalize_import.py`는 최초 이전 보정용이다. 후속 설계 편집에 자동 실행하지 않는다. 원본 규칙 대응과 보수적으로 남긴 Slot Region/Edge.Cuts 차이는 `validation/rules-mapping.md`를 먼저 읽는다.
- 원본 v1의 SEN1은 센서 커넥터와 풀업 저항에만 있고 MCU 미연결이다. VIN은 PW OFF ON 헤더의 한 핀에만 존재한다(v1.1 전원부 수정에서 J1.1을 VIN으로 옮겨 해소). 임의 연결로 원본 이전 비교 결과를 맞추지 않는다.
- 기존 `ledpin=[4,3,1,0]`은 GPIO0 CHKBAT 분압 입력을 출력으로 사용한다. `dht11pin=7`은 드라이버 SLEEP과 충돌한다. D1~D4의 GPIO `[1,3,4,5]`도 실기 검증 없이 모터 설정 정본으로 저장하지 않는다.
- 현재 실장 모듈 모델은 위 EstarDyn으로 확인됐다. 원본 심볼의 GPIO22 표기는 C3 실제 GPIO가 아니며 펌웨어에서 사용하지 않는다. 모듈 핀 위치·전압·극성·전원 관계·모터 정지·서보 동작의 실측은 남아 있다. 파일 이전 완료는 제작 가능 판정이 아니다.

## 전원 설계 검토와 미확정 사항

**2026-11-01 전원부 수정 적용 완료.** 수정 전 검토 자료는 [전원 검토](../hardware/type_m/v1.1/validation/power-audit-2026-11-01/design-review.md), [구리 접촉 감사](../hardware/type_m/v1.1/validation/power-audit-2026-11-01/power-audit.json), [독립 네이티브 검토](../hardware/type_m/v1.1/validation/power-audit-2026-11-01/independent-power-review.json)다. 수정 내용·전후 그림·검증은 [전원부 수정 확인 화면](../hardware/type_m/v1.1/validation/power-fix-2026-11-01/review.html)과 [power-fix-verification.json](../hardware/type_m/v1.1/validation/power-fix-2026-11-01/power-fix-verification.json)이다.

- 수정 내용: J1 실크 `BAT + -`→`BAT`(원위치) + 패드 위 `−`(J1.2 GND)/`+`(J1.1) 0.8 mm; J1 값 `MT1`→`BAT`; J1.1 네트 VCC→VIN, VIN 배선 3개 재지정·대각 VCC 배선(66ef6120) 삭제·J2.3까지 1.0 mm F.Cu 배선 추가; 회로도 #PWR012 제거·VIN 전역 라벨(91.44,40.64)·J2.1 no_connect. `.kicad_pro`/`.kicad_sym`/`.kicad_dru`는 바이트 동일.
- 결과: netlist VIN={J1.1,J2.3}, VCC={J2.2,J5.1,J6.1,R1.1,U1.8,U2.9}; ERC 21/27→20/26(J2.1 미연결 오류·VIN 고립 라벨·J1.1–전원심볼 경고 해소, J1.1–J2.3 Unspecified 경고 1 추가); DRC 오류0·같은 경고12·미배선0·parity0; 구리 접촉 그래프 J1.1→J2.3만, 다른 네트 접촉 0.
- **J2 사용 규칙:** 1=OFF/NC(사각), 2=COM=VCC, 3=ON=VIN. 손잡이를 BAT 쪽(오른쪽)으로 밀면 ON. `OFF`/`ON` 글자는 패드 중심에 정확히 맞지 않는다. 2–3 쇼트 점퍼=항상 ON. 소형 슬라이드 스위치(SS12D00 계열) 정격 0.3–0.5 A는 모터·서보 피크에 부족할 수 있다 → 부하 예산 확인 전 점퍼 운용.
- **CHKBAT:** R1/R2 220k 분압은 VCC 쪽 유지. ON이면 배터리/2, OFF이면 무전류·꺼진 GPIO0에 전압 없음. VIN 쪽으로 옮기면 OFF에서 110 kΩ(R1∥R2)로 VIN/2이 꺼진 핀에 걸리므로 옮기지 않았다.
- **모듈 전원 사실(판매자 매뉴얼 `.local/type_m/estardyn-manual.pdf` 6장):** 5V 핀 입력 5 V±0.2 V 정전압, Type-C와 5V 핀 동시 급전 금지, 3.3 V LDO 300 mA. 매뉴얼은 범용 SuperMini 문서라 실장 EstarDyn Plus와 세부가 다를 수 있다(LED 설명은 이전에 채택하지 않음). KiCad에는 모듈 내부 회로가 없으므로 USB→5V 핀 경로를 단정하지 않는다. 설계상: OFF는 배터리만 분리(USB 연결 시 부하는 모듈 경유로 계속 급전될 수 있음), 모듈이 USB 5 V를 5V 핀으로 내보내는 구조라면 ON+USB에서 USB 5 V와 배터리가 VCC에서 만남(충전·보호 회로 없음) → 확인 전 금지, 이때 CHKBAT는 USB 쪽 VCC를 읽음.

아래 표는 **수정 전** 상태의 기록이다.

| 저장 KiCad에서 확정한 내용(수정 전) | 해석 범위 |
|---|---|
| J1 BAT 전면 실크는 좌우 `+ −`, 실제 패드는 왼쪽 원형 pad2=GND(−), 오른쪽 사각 pad1=VCC(+) | 극성 안내가 좌우 반대여서 오연결을 유도할 수 있다. 이것이 실물 배터리를 역극성으로 연결했다는 증거는 아니다. [실크 확대](../hardware/type_m/v1.1/validation/power-audit-2026-11-01/battery-polarity-review.png) |
| J2.2=VCC, J2.3=VIN 단독 구리 패드(J2.1도 고립), 배터리와 부하는 VCC에 직결 | 스위치가 부하 전원과 직렬이 아니므로 현재 설계로 배터리 전원을 차단할 수 없다. VIN이라는 이름만 보고 입력 전원이 이어졌다고 판단하지 않는다. |
| 배터리 J1의 회로도/PCB Value는 `MT1` | BAT 역할과 다른 이름이다. 명칭 오류이며 그 자체가 전기 연결을 바꾸지는 않는다. |
| J1.1→U1.8(VCC)은 실제 PCB 트랙 끝점 연속, J1.2→U1.7(GND)은 트랙·동판 포함 연결 | **저장 PCB에서 BAT→C3 전원 단선 증거는 없다.** 네트 이름만 비교한 결론이 아니라 구리 접촉과 별도 KiCad 연결 계산 결과다. 실물 도통을 측정한 결과는 아니다. |
| U1.8=+5V/VCC, U1.7=GND, U1.6=+3V3/3.3V | 해당 모듈 전원 핀의 실크와 모듈 내부 전원 회로는 이 캐리어 KiCad 파일에 없다. 모듈에 레귤레이터가 없다는 뜻이 아니며 USB 역류 방지·허용 입력·3.7V 적합성을 파일만으로 확정하지 않는다. |

- 사용자 보고: **USB로는 부팅, 배터리로는 부팅 안 됨**. 처음 3.7V 배터리라고 설명했고 이후 4.9V가 측정된다고 했지만 측정 위치·전원 모듈 포함 여부는 미확정이다. 부족 전압·실제 역극성·부품 손상·승압 필요를 결론내리지 않는다.
- 위 3건(극성 표시, 직렬 스위치 경로, J1 BAT 명칭)은 2026-11-01에 수정 적용했다. 남은 것은 J2 부품 정격 결정, USB·배터리 동시 연결 정책, 실장 v1 보드 처리, 제작·실물 검증이다. 사용자에게 측정·점퍼·부하 사양 질문을 반복하지 않는다.
- DRC 오류0/미배선0은 잘못된 실크 의미나 기능적으로 잘못된 스위치 회로를 검출했다는 뜻이 아니다. 수정 전 전원 감사의 `all_checks_passed=true`도 설계가 정상이라는 뜻이 아니라 기록된 연결 판정들이 확인됐다는 뜻이다. `audit_power.py`/`independent_power_review.py`는 수정 전 상태를 전제로 한 검사(예: "J1.1→J2.3 경로 없음")라 현재 파일에서는 일부 판정이 반대로 나오며, 재실행하지 않고 기록으로 보존한다.

## type_s 원본과 적용 조건

- EasyEDA 원본은 `ZeroFactorySensorBoard / Board_DHT`다. 원본 `.epro2`와 가져오기용 `.epro`를 `hardware/type_s/v1/source/`에 보존하고 KiCad 이전·원본 대조를 마쳤다. 편집 정본은 `hardware/type_s/v1/kicad/type_s_v1.kicad_pro`이며 검사 결과는 [type_s 검증 기록](../hardware/type_s/v1/validation/README.md)을 확인한다.
- 실제 배치 부품은 ESP32-C3 Supermini 모듈 1개·저항 5개·헤더 9개다. DHT·CDS·DS18B20은 센서 부품이 아닌 커넥터 표기다. 라이브러리에만 존재하는 부품을 실장 부품으로 세지 않는다.
- [type_s 핀맵](../hardware/type_s/v1/pinmap.md): DHT/SENSOR1=GPIO3, CDS/SENSOR2=GPIO4, DS18B20/SENSOR3=GPIO5, A1=GPIO1, D1/D2/D3=GPIO6/7/10, BATCHK=GPIO0. 모듈 패드 번호는 칩 패키지 핀 번호가 아니다.
- BAT의 VCC는 On Off 헤더 1번, 헤더 2번은 `5V` 네트다. D1~D3 전원은 5V지만 신호는 MCU GPIO에 직접 연결된다. 네트 이름만으로 전압 조정·5V 신호 허용을 판단하지 않는다. 모듈 외 별도 레귤레이터·레벨 변환기는 배치되어 있지 않다.
- 분압은 전원 헤더 뒤 5V에서 BATCHK를 만든다. 원본 저항 `e3937/e3961`의 회로도 값은 220kΩ, PCB 상속값과 표기는 10kΩ/10K다. 비율은 같아도 임피던스가 다르므로 실제 BOM은 설계자 확인 대상이다.
- KiCad R1/R2의 회로도·PCB 값 차이를 보존하여 parity 검사에도 남긴다. 원본 비교 통과와 미배선 없음이 이 부품값 차이 또는 남은 ERC·DRC 검토를 해결했다는 뜻은 아니다.
- 기존 `ledpin=[4,3,1,0]`은 SENSOR2/SENSOR1/A1/BATCHK와 충돌한다. `dht11pin=7`은 D2다. DHT 헤더 GPIO3도 실물·설정 검증 전에는 수업 정본 설정으로 저장하지 않는다.
- 현재 펌웨어에는 CDS·DS18B20·BATCHK 읽기 명령이 없다. 별도 펌웨어 변경과 실기 검증이 필요하며 이번 이전에서 런타임 코드는 변경하지 않는다.

## 런타임 구조 메모

- 기존 LED/DHT11 환경은 GATT write의 문자열을 `\n`으로 분리하고 각 명령을 `parseCmd()`로 처리해 JSON 결과를 notify한다. type_m은 위의 단일 ASCII 패킷·응답 규약을 사용한다.
- 교재는 BLE 콜백을 `RealBleunoClient`로 감싸고 Fake 구현으로 보드 없이 실습한다.
- TENSTAR 연결 콜백은 `stopBlink()` 후 NeoPixel을 파랑으로 켠다. 상태 LED 색은 타깃·코드·실기로 확인해야 한다.

## 동작 규칙

- `on N`의 N은 GPIO가 아닌 설정 배열의 인덱스다. `-1`은 전체를 뜻한다.
- 기존 LED/DHT11 펌웨어는 자발적 버튼 입력 이벤트를 전송하지 않는다. 교재 Fake의 `event: input`은 확장용 가안이다. type_m의 `event timeout`은 별도 ASCII watchdog 이벤트다.
- `about`의 `os: cronos-v1`을 하드웨어 리비전 증거로 사용하지 않는다.
- EasyEDA는 지정 원본을 읽고 내보내는 용도다. 원본 설계를 수정하거나 V2 프로젝트를 변환할 필요가 없다.

## 반복 금지

- “KiCad만보세요”, “시키지좀마세요 보드확인부터”를 우선한다. 반복 측정·점퍼 연결·부하 사양 질문이나 승압 추가 확정으로 설계 검토를 대신하지 않는다.
- Python 설치/자동 다운로드 금지. `uv python list --only-installed`로 확인한 인터프리터와 `--no-python-downloads`를 사용한다.
- EasyEDA HTTP 200 응답 안에 `code: 401`이 있을 수 있다. 인증 응답 본문과 실제 프로젝트 열림을 함께 확인한다.
- EasyEDA 원본을 확보하지 못했을 때 추정 설계나 빈 KiCad 파일을 만들어 이전 완료로 표시하지 않는다.
- 교재 공통 클래스는 여러 주차에 복제되어 있다. 향후 코드 변경 시 주차별 복사본도 확인한다.
