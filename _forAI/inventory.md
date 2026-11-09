# Inventory

## 목차

- [Repository](#repository)
- [Top-level structure](#top-level-structure)
- [Entrypoints and key modules](#entrypoints-and-key-modules)
- [Build and validation commands](#build-and-validation-commands)
- [Tests](#tests)
- [Notes](#notes)

## Repository

- Name: `bleuno`
- Path: `C:\Users\user\Desktop\work\cronos\bleuno`
- Summary: 기존 LED·PWM·DHT11 환경과 `type_m` 모터·서보·DHT11/22 BLE 환경, BLEuno 하드웨어를 관리한다. 2026-11-01 v1.1 KiCad 전원부 수정(BAT 극성 표시·J2 직렬 스위치·J1 이름)을 적용·검증했고, 제작·실물 검증은 아직이다.
- Hardware models / revisions: `type_m` / 원본 `v1` 및 센서 배선+전원부 수정본 `v1.1`, `type_s` / `v1`. Model directories: `hardware/<model>/<revision>/`; v1.1 KiCad project: `type_m_v1_1`.

## Top-level structure

- `src/`: 펌웨어 구현.
- `include/tonkey.hpp`: 명령 토큰 파서.
- `include/type_m_protocol.hpp`: type_m ASCII 파서, READY/watchdog/정지·방향 전환·DHT 상태.
- `docs/type_m.md`: type_m 빌드·핀맵·명령·정지 동작 및 실제 확인 범위.
- `tools/type_m_smoke.py`: 비구동 BLE 통합 검사. 정상 서보 또는 0이 아닌 모터 명령을 보내지 않는다.
- `tools/type_m_status.py`: USB 시리얼의 읽기 전용 about/status 조회. 구동·리셋 명령을 보내지 않는다.
- `tests/type_m_tests.cpp`: 호스트 C++ 파서·제어 상태 테스트. 기존 PlatformIO 기본 폴더 `test/`와 구분한다.
- `.local/type_m/`: git 제외 로컬 백업·식별·BLE 검사 기록. 보존 대상이며 공개 소스에 포함하지 않는다.
- `platformio.ini`: ESP32 보드별 빌드 환경.
- `readme.md`, `docs/type_m.md`, `hardware/README.md`: 사용자용 안내. 원본 v1과 센서 배선·전원부를 수정한 v1.1, 펌웨어 1.1.0을 구분한다. 점퍼는 v1 임시 보수 방법이며 정규 사용 절차가 아니다.
- `hardware/type_m/v1/README.md`: 지정 마더보드 원본, KiCad 이전 결과와 남은 검토 항목.
- `hardware/type_m/v1/source/`: 원본 `.epro2`와 가져오기용 `.epro`, 출처·해시 기록.
- `hardware/type_m/v1/kicad/type_m_v1.kicad_pro`: KiCad 10 프로젝트. 동일 이름의 회로도·PCB·심볼 라이브러리·풋프린트 폴더와 `.kicad_dru`를 함께 관리한다.
- `hardware/type_m/v1/pinmap.md`: 원본의 MCU·드라이버·커넥터·GPIO·전원 연결과 미완성 항목.
- `hardware/type_m/v1/validation/`: 원본/이전 비교, netlist, ERC·DRC, 원본 설계 규칙 대응 및 재현 스크립트.
- [hardware/type_m/v1.1/README.md](../hardware/type_m/v1.1/README.md): SEN1–GPIO20 수정(1단계)과 전원부 수정(2단계) 내역, J2 스위치 사용 규칙, 전원 경로와 주의, 검사 결과, 재검증 명령.
- [hardware/type_m/v1.1/kicad/type_m_v1_1.kicad_pro](../hardware/type_m/v1.1/kicad/type_m_v1_1.kicad_pro): 현재 작업 KiCad 프로젝트. 같은 이름의 `.kicad_sch`·`.kicad_pcb`, 전용 라이브러리·규칙을 함께 유지한다. 원본 v1은 불변으로 보존한다.
- `hardware/type_m/v1.1/validation/`: 현재 상태의 `drc.json`, `erc.json`, `netlist.xml`(전원부 수정 후); 센서 단계 기록 `revision-comparison.json`(25/25 PASS), `v1-input-hashes.json`, `pcb-patch.json`, `verify_revision.py`(센서 단계 스냅샷 기준).
- `hardware/type_m/v1.1/validation/power-fix-2026-11-01/`: 전원부 수정 적용 스크립트 `patch_schematic_power.py`·`patch_pcb_power.py`(적용 완료, 기준 해시·lock 검사 후 거부), 검증 `verify_power_fix.py`와 결과 `power-fix-verification.json`(34/34), 패치 기록 `schematic-patch.json`·`pcb-power-patch.json`, 수정 전 스냅샷 `before-power-fix.kicad_sch/.kicad_pcb`·`before-power-fix-{netlist.xml,erc.json,drc.json}`, 전후 확대 그림 `before/after-pcb-zoom.png`·`before/after-sch-zoom.png`, 설명 화면 `review.html`.
- [validation/preview/review.html](../hardware/type_m/v1.1/validation/preview/review.html): 네이티브 CAD 출력 위에 설명용 센서 배선을 강조한 화면. 같은 폴더의 PCB SVG/PNG·회로도 SVG/PNG/PDF 및 `pcb-preview.html`·`schematic-preview.html`을 함께 보관한다. 설명용 강조선 자체를 실제 구리 연결 증거로 삼지 않는다.
- [validation/power-audit-2026-11-01/design-review.md](../hardware/type_m/v1.1/validation/power-audit-2026-11-01/design-review.md): 전원부 **수정 전** 검토 기록. 같은 폴더 `power-audit.json`, `independent-power-review.json`, `battery-polarity-review.png`, `audit_power.py`, `independent_power_review.py`는 수정 전 상태 기준이며 재실행하지 않는다. 수정 적용은 `power-fix-2026-11-01/`에 있다.
- `hardware/type_s/v1/README.md`: ZeroFactorySensorBoard / Board_DHT의 이전 현황과 확인 사항.
- `hardware/type_s/v1/source/`: 원본 `.epro2`와 가져오기용 `.epro`, 출처·해시 기록.
- `hardware/type_s/v1/kicad/type_s_v1.kicad_pro`: 이전·원본 대조를 마친 type_s v1 KiCad 프로젝트. 회로도·PCB·전용 라이브러리를 함께 관리한다.
- `hardware/type_s/v1/pinmap.md`: MCU·센서/입출력 헤더·전원 연결과 원본 분압 저항값 불일치.
- `hardware/type_s/v1/validation/`: 원본 인벤토리와 이전 비교·검사 기록.
- `test/`, `lib/`: PlatformIO 기본 안내 파일만 존재.
- `_forAI/`: 작업 인수인계 문서.

## Entrypoints and key modules

- `src/main.cpp`: 버전, 초기화, LED/PWM 제어, 센서 및 스케줄러.
- `src/type_m_main.cpp`: 별도 type_m BLE·NeoPixel·모터·서보·DHT 엔트리포인트. 현행 `type_m/1 fw=1.1.0`; 기존 1.0.0 증거와 구분한다.
- `src/ble.cpp`: BLE 광고·연결·GATT 읽기/쓰기/알림과 명령 분리.
- `src/parseCmd.cpp`: `about`, `config`, `on`, `off`, `pwm`, `dht11` 등 텍스트 명령.
- `src/config.cpp`, `src/config.hpp`: 저장 설정과 `config` 명령.

## Build and validation commands

- 아래는 재현 참고 명령이다. 현재 사용자 지시는 **KiCad 파일부터 확인**하는 것이며 재개하면서 보드 측정·점퍼·부하 사양 요청, 펌웨어 재업로드를 먼저 반복하지 않는다. 우선 전원 검토 문서·저장된 회로도/PCB·독립 검토 JSON을 읽는다.
- 먼저 `uv python list --only-installed`로 인터프리터를 확인한다. 새 type_m 빌드: `uv run --no-python-downloads --python 3.12 --with platformio==6.2.0 python -m platformio run -e type_m`.
- `type_m`은 `espressif32@6.12.0`, Arduino-ESP32 2.0.17에 고정한다. 기존 타깃을 시험하려면 같은 명령의 `-e`만 변경하며 type_m에 기존 Config 설정을 넣지 않는다.
- 이 PC의 기존 PlatformIO Python 재사용: `uv run --no-python-downloads --python C:\Users\user\.platformio\penv\Scripts\python.exe -m platformio run -e type_m`.
- 업로드는 `run -e type_m --target upload --upload-port COMxx`. 최초 식별 포트는 COM14/MAC 끝자리54:d5, 현재 대상은 COM11/b3:6e다. 포트와 MAC을 함께 재확인하고 백업을 구분한다.
- BLE 비구동 검사: `uv run --no-python-downloads --python 3.12 tools/type_m_smoke.py --address 8C:D0:B2:A8:B3:70 --serial-port COM11 --output .local/type_m/ble-smoke-b36e-1.1.0-rerun.json`. 기존 성공/실패 증거를 덮어쓰지 않는다. 다른 보드에서는 주소·포트를 재확인한다. 시리얼 옵션으로 실제 단절과 내부 정지 상태를 확인한다.
- 시리얼 상태 조회: `uv run --no-python-downloads --python 3.12 tools/type_m_status.py --port COMxx`.
- 호스트 검사 빌드: `uv run --no-project --no-python-downloads --python 3.12 --with ziglang==0.13.0 python -m ziglang c++ -nostdlib++ -std=c++11 -Wall -Wextra -Werror -I include tests/type_m_tests.cpp -o .local/type_m/type_m_tests-1.1.0-rerun.exe`; 이후 해당 exe를 실행한다. 과거 버전의 실행 파일·로그를 덮어쓰지 않는다.
- 기존 `tenstar_esp32c3`는 Supermini 관련 LED 환경이며 현재 실장 type_m용 타깃으로 대신 사용하지 않는다. 기존 LED·DHT11 설정에는 V1 회로와 충돌이 있다.
- 현재 KiCad에서는 `hardware/type_m/v1.1/kicad/type_m_v1_1.kicad_pro`를 연다. `v1/kicad/type_m_v1.kicad_pro`는 원본 비교용이며 가져오기를 반복하지 않는다.
- v1.1 현재 파일 검증: `& "$env:LOCALAPPDATA\Programs\KiCad\10.0\bin\python.exe" -B hardware/type_m/v1.1/validation/power-fix-2026-11-01/verify_power_fix.py` (수정 전 스냅샷 대비 허용 변경만 있는지, 구리 접촉, DRC/ERC 델타). `verify_revision.py`는 센서 단계 스냅샷 기준이라 현재 파일에는 통과하지 않는다. `patch_schematic_sensor.py`·`patch_pcb_sensor.py`·`patch_schematic_power.py`·`patch_pcb_power.py`는 적용 완료 수정 스크립트이며 재실행하지 않는다(전원부 스크립트는 기준 해시·`~*.lck` 검사로, 센서 스크립트는 대상 상태로 재적용을 거부). `verify_power_fix.py`와 `kicad-cli`는 `--output`/`-o`를 임시 폴더로 주면 기록 파일을 덮어쓰지 않는다.
- `kicad-cli sch erc --format json --severity-all`, `kicad-cli sch export netlist --format kicadxml`, `kicad-cli pcb drc --format json --severity-all --schematic-parity`를 `hardware/type_m/v1.1/validation/{erc.json,netlist.xml,drc.json}`로 출력한다(`hardware/type_m/v1.1/README.md` 재검증 절). KiCad GUI가 열려 있으면 파일을 수정하지 않는다. 미리보기는 `kicad-cli pcb export svg --layers F.Cu,B.Cu,F.SilkS,F.Mask,Edge.Cuts --page-size-mode 2 --exclude-drawing-sheet --mode-single`, `kicad-cli sch export svg/pdf --exclude-drawing-sheet`로 만들고 PNG는 Chrome headless 스크린샷이다.
- type_s의 이전·검증 상태와 해당 모델의 명령은 `hardware/type_s/v1/validation/README.md`를 확인한다. type_m 검증 수치를 type_s에 적용하지 않는다.
- Python 실행은 기존 인터프리터를 `uv run --no-python-downloads --python <설치된 인터프리터>`로 재사용한다.

## Tests

### 현행 펌웨어·앱 1.1.0

- 호스트 **501 checks PASS**(기존 399 + 센서 102), 전체 ESP32 빌드·COM11 업로드/hash 검증, BLE 비구동 **54 checks PASS**, Android 단위 **34개 PASS**·lint `No issues found`. 현재 요약은 `.local/type_m/verification-1.1.0.json`이다.
- 증거: `.local/type_m/unit-tests-1.1.0.log`, `build-1.1.0.log`, `upload-b36e-1.1.0.log`, `ble-smoke-b36e-1.1.0-retry.json`. 첫 BLE 실행 실패 기록도 보존한다. 펌웨어 SHA-256 `1DF91B8E665700114642A46AD47B5B90213D0E1915350D1EAB4AE37B1AB828E8`.
- Android 1.1.0은 설치·검색·CCCD/about/ready·DHT11/22 선택·센서 error/빈 수치·STOP·Home 단절·연결 소실 시 제어 비활성·프로세스 재시작 후 DHT22 저장을 확인했다. 사용자가 센서를 서보 포트에서 SENSOR1으로 옮긴 후 재연결과 저장 모델 재적용까지 성공했으나 실제 온습도는 계속 `error`였다. 마지막 재연결을 미검증으로 적은 과거 기록보다 `verification-1.1.0.json`을 우선한다.
- 앱: `../bleunoApp/TypeMControl/`의 assembleDebug/testDebugUnitTest/lintDebug 통과. APK 925,331바이트, SHA-256 `849F7FA06C1647E7223136F75AB8CCCFA0A0D759E4C9323419B050635839DB31`. 실물 센서 측정과 모터·서보 부하/파형은 미검증이다.

### 이전 펌웨어·앱 1.0.0 — 보존 기록

아래 399/29/17 및 해당 빌드·이미지 기록은 **1.0.0 당시 결과**다. 현행 1.1.0 검증 수치로 인용하지 않는다.

- 기존 `test/README`는 PlatformIO 기본 안내다. 새 `tests/type_m_tests.cpp`는 399 checks PASS, 종료 코드 0을 확인했다. 로그 `.local/type_m/unit-tests.txt`에 컴파일·실행 명령을 보존했다. 파서 경계·구독/준비·watchdog/wrap·오래된 요청·STOP epoch·서보 초기 OFF·모터 기동/방향 간격을 검사하며 무선·물리 출력 검증은 아니다.
- PlatformIO type_m 빌드 SUCCESS: RAM38,860/327,680(11.9%), 앱 Flash998,000/1,310,720(76.1%). 로그 `.local/type_m/build.log`. 실제 모듈 Flash 용량 4MB와 앱 파티션 한도를 구분한다.
- Android 단위 테스트17개 PASS·Lint 문제없음. IM_L101/API34 설치·권한·초기 화면 및 검색→CCCD/about/ready→제어 준비됨, STOP 응답·배경 진입 후 연결 해제·재연결 준비를 확인했다. `.local/type_m/android-ready-stop.txt`, `android-background.txt`, `android-reconnected.txt`에 각 단계의 시리얼 정지 상태를 보존했다.
- COM11/b3:6e 업로드 SUCCESS·Flash hash 검증 완료(`.local/type_m/upload-b36e.log`). 이미지 SHA256 `BC681C8C53B118E480AFC1855BFF9096FBDDAEE88F9F13AF187ED6066AD69BDB`.
- BLE29checks PASS(`.local/type_m/ble-smoke-b36e.json`), movement_requested=false. subscribe/ready/범위·문법 거부/timeout/실제 disconnect와 내부 출력0/재연결새ready를 확인했다. 모터·서보 유효 구동 명령을 보내지 않았고 파형 계측은 하지 않았다.
- `.local/type_m/verification.json`에 최종 보드·이미지·백업 해시, 시험 결과, 사용자 RGB 확인과 미계측 범위를 기록했다.

### 하드웨어 리비전별 검사

- **type_m v1.1 센서 단계:** 독립 변경 대조 **25/25 PASS**, 원본 v1의 기록한 57개 파일 SHA-256 불변. SEN1=J7.2/R3.2/U1.15(GPIO20), 심볼 pin15/16 이름 수정, 트랙 6개·비아 1개 추가. 당시 DRC 오류0/경고12, 미배선0/parity0, ERC 오류21/경고27.
- **type_m v1.1 전원부 수정(2026-11-01):** `verify_power_fix.py` **34/34 PASS**. netlist VIN={J1.1,J2.3}, VCC={J2.2,J5.1,J6.1,R1.1,U1.8,U2.9}; ERC 오류20/경고26; DRC 오류0/기존 글자 높이 경고12, 미배선0/parity0; 구리 접촉 그래프 J1.1→J2.3만, 서로 다른 네트 접촉 0; SEN1 배선·비아 유지; `.kicad_pro`·`.kicad_sym`·`.kicad_dru` 바이트 동일. pin 전기 형식·규칙을 완화하지 않았다. 제작·실측은 미완료다.
- **전원 검토(수정 전 기록):** `power-audit.json`과 `independent-power-review.json`은 수정 전 BAT+ J1.1→U1.8(VCC), BAT− J1.2→U1.7(GND) 구리 연결과 J2 VIN 고립·극성 실크 불일치·J1 `MT1` 오표기를 확인한 기록이다. 세 문제는 수정했으며 두 스크립트는 수정 전 전제를 담고 있어 재실행하지 않는다.
- 아래 type_m 이전/22·26 ERC 수치는 **원본 v1** 기록이며 위 센서 수정본 v1.1 결과와 구분한다.
- 2026-10-31 type_m 원본 보존·KiCad 이전·참조/링크/절개 보정 후 비교 결과 `checked_comparisons_pass=true`를 확인했다. 12개 부품, 부품 패드 52개, 독립 홀 패드 2개, 트랙 118개, 비아 0개, 연결 그룹 17개, 외곽/절개 선분 28개가 비교 대상이다.
- ERC는 오류 22개·경고 26개다. 동판 재계산 후 최종 DRC는 오류 0개·텍스트 높이 경고 12개, 미배선/회로도·PCB 차이는 0개다. 결과는 `hardware/type_m/v1/validation/drc.json`, 판정 차이는 `rules-mapping.md`를 확인한다.
- 원본과 같은 zone 외곽·레이어·네트를 유지하되 채워진 구리 내부는 KiCad로 재계산했다. 재계산 전 PCB와 DRC도 `validation/`에 보존한다.
- 회로도·PCB GUI 재열기로 파싱·라이브러리 경고 없이 표시됨을 확인했다. PCB 표시의 net 24개는 비교한 연결 그룹 17개와 미연결 핀 개별 net 7개를 합친 값이다.
- type_m v1 실장은 설계대로 완료했다는 사용자 확인을 받았다. 최초 COM14/54:d5와 현재 COM11/b3:6e의 기존 Flash4MB를 각각 백업했다. 현재 백업 SHA256 `7C66D38A5FD54642DAA9F3293AD3DC2030592E104D85580B0D795571EA9F319C`. 두 칩 모두 C3 QFN32 rev0.4/4MB XMC다. RGB 대기 빨강 점멸·READY 파랑 고정은 사용자 확인, 준비 전 파랑 점멸은 소스·상태 기준이다. 모터·서보의 실제 부하 동작·파형은 미검증이다.
- 2026-11-01 type_s의 15개 부품·52개 부품 패드·114개 트랙·비아 0개·12개 연결 그룹·독립 홀 패드 3개 등의 원본 대조를 통과했다. 회로도와 PCB의 52핀 네트는 일치하지만 분압 저항 2개의 회로도 값 220kΩ와 PCB 상속값 10kΩ는 다르며 이 차이는 KiCad에도 보존했다. 최종 ERC·DRC와 원본 값 차이에 따른 parity 결과는 `hardware/type_s/v1/validation/README.md`를 확인한다.

## Notes

- `hardware/type_m/v1/source`, `kicad`, `validation`에는 실제 원본·설계·검사 파일이 있다. KiCad 이전 완료를 제작 승인이나 실물 동작 완료로 표시하지 않는다.
- 다음 세션은 [memo.md](memo.md)의 사용자 작업 원칙·전원 검토 확정/미확정 구분부터 읽는다. v1.1 센서 수정으로 현재 실장된 v1의 물리 배선이나 전원부가 자동으로 고쳐진 것은 아니다.
- 교재 저장소와 앱 저장소는 별도 유지하며 이 저장소에 복제하지 않는다.
