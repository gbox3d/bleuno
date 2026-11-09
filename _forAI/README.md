# _forAI Guide

## 목차

- [한 줄 요약](#한-줄-요약)
- [읽는 순서](#읽는-순서)
- [문서 역할](#문서-역할)
- [다음 세션 시작점](#다음-세션-시작점)
- [현재 스냅샷](#현재-스냅샷)
- [유지 규칙](#유지-규칙)

## 한 줄 요약

`bleuno`의 펌웨어·하드웨어 작업을 이어받기 위한 문서다. **2026-11-01 현재 type_m v1.1의 전원부 KiCad 수정(BAT 극성 표시·J2 직렬 스위치·J1 이름)을 적용·검증했고**, 다음은 제작 전 남은 결정(스위치 부품 정격, USB·배터리 동시 연결 정책, 실장 v1 보드 처리)과 실물 검증이다. 수업 구성은 보류했다.

## 읽는 순서

1. `README.md`
2. `inventory.md`
3. `memo.md`
4. `dev_log.md`의 최신 항목
5. `plan.md`

## 문서 역할

- `inventory.md`: 실제 파일·진입점·빌드/검증 명령.
- `plan.md`: 앞으로 할 작업과 우선순위.
- `memo.md`: 핀맵·전원 연결·프로토콜·사용자 요청·판정 한계.
- `dev_log.md`: 날짜별 작업 이력. 과거 진행 중 상태를 현재 지시로 적용하지 않는다.

## 다음 세션 시작점

1. [plan.md](plan.md)의 남은 작업을 읽는다.
2. [v1.1 안내](../hardware/type_m/v1.1/README.md)의 "전원부 수정", "J2 스위치 사용 규칙", "전원 경로와 주의"와 [전원부 수정 확인 화면](../hardware/type_m/v1.1/validation/power-fix-2026-11-01/review.html)을 읽는다. 수정 전 검토 기록은 [design-review.md](../hardware/type_m/v1.1/validation/power-audit-2026-11-01/design-review.md)다.
3. 작업용 [type_m_v1_1.kicad_pro](../hardware/type_m/v1.1/kicad/type_m_v1_1.kicad_pro)를 연다. v1은 보존 원본이다. 파일을 스크립트로 바꿀 때는 KiCad GUI를 먼저 닫는다(패치 스크립트는 `~*.lck`가 있으면 거부한다).
4. **사용자에게 측정·점퍼·부하 사양 확인을 반복해서 시키지 말고, KiCad 파일을 먼저 검토한다.** 사용자 표현은 “키캐드만 보세요”, “시키지좀 마세요..보드 확인 부터 해요”였다.

현재 v1.1은 **센서 배선 + 전원부 수정본**이다(제작 전이라 같은 리비전으로 관리). 과거 로그의 “점퍼 확인 중”, “v1 편집 정본”, “전원부 미수정”, 펌웨어 1.0.0 검사 결과를 최신 상태로 적용하지 않는다.

## 현재 스냅샷

- 저장소: `C:\Users\user\Desktop\work\cronos\bleuno`. Android 앱: `C:\Users\user\Desktop\work\cronos\bleunoApp\TypeMControl`. 교재: `C:\Users\user\Desktop\work\teaching_repo\android_programming`.
- 하드웨어: `type_m/v1`은 보존 원본, `type_m/v1.1`은 현재 수정 작업본, `type_s/v1`은 별도 이전본이다. 하드웨어 리비전과 펌웨어 버전은 별개다.
- v1.1 완료: SEN1을 J7.2–R3.2–U1.15(GPIO20)에 연결했다. 새 트랙 6개·비아 1개, U1 pin15/16 이름 GPIO20/GPIO21 정정, PCB 실크 `type_m v1.1`, 도면 버전/수정일 갱신. 원본 v1의 기록된 57개 파일은 해시가 유지됐다.
- v1.1 센서 단계 검사(전원부 수정 전): 독립 변경 대조 25/25 PASS, ERC 오류21·경고27. 그 입력 파일은 `validation/power-fix-2026-11-01/before-power-fix.*`로 보존했다.
- **전원부 수정 완료(2026-11-01):** J1 실크 `BAT + -` → `BAT` + 각 패드 위 `−`(J1.2 GND)/`+`(J1.1), J1 값 `MT1`→`BAT`, J1.1 네트 VCC→VIN, J2.3(VIN)으로 1.0 mm 배선 추가·대각 VCC 배선 1개 삭제, 회로도 #PWR012 제거·VIN 라벨·J2.1 NC. 결과 DRC 오류0·기존 경고12·미배선0·불일치0, ERC 오류20·경고26, `verify_power_fix.py` 34/34 PASS. 적용 스크립트와 전후 그림은 `validation/power-fix-2026-11-01/`에 있다. 설계 파일 수정이며 실장된 v1 보드는 바뀌지 않는다.
- J2 규칙: 1=OFF/NC(사각), 2=COM=VCC, 3=ON=VIN. 소형 슬라이드 스위치(0.3–0.5 A)는 모터·서보 피크에 부족할 수 있어 부하 예산 확인 전에는 2–3 쇼트 점퍼 운용. 판매자 매뉴얼은 모듈 5V 핀 입력 5 V±0.2 V·Type-C와 동시 급전 금지를 명시하므로 배터리 연결+스위치 ON 상태로 USB를 꽂지 않는다. CHKBAT 분압은 VCC 쪽 유지.
- BAT→C3 실제 구리 연결은 수정 전후 모두 확인됐다(수정 후 J1.1→J2.3→(스위치)→J2.2→U1.8). “배터리–C3 배선 누락” 또는 “사용자가 실제 역극성으로 연결했다”는 결론은 내리지 않았다.
- 사용자 보고: USB로 부팅되지만 배터리로 부팅되지 않음. 배터리는 처음 3.7V라고 설명했고 이후 4.9V가 측정된다고 말했다. 측정 지점/전원 모듈 포함 여부는 미확정이다. 전압 부족·승압 필요·부품 손상을 단정하지 않는다. KiCad에는 SuperMini Plus 내부 전원 회로가 없다.
- 실장 MCU: EstarDyn ESP32-C3 SuperMini Plus V2.0 빨간 RGB 보드. 사용자 실장은 v1이며 v1.1 파일 수정이 실물 배선을 바꾸지는 않는다. 실제 DHT22 수치·모터/서보 부하·파형은 미검증이다.
- 펌웨어: 기존 `src/main.cpp` 1.0.5_dev, 별도 `src/type_m_main.cpp` / `include/type_m_protocol.hpp`는 **1.1.0**, DHT11/22·모터·서보·RGB 및 STOP/watchdog 지원. DHT GPIO20은 UART0 RX 겸용이지만 현재 USB HWCDC의 GPIO18/19와 겹치지 않는다.
- 최신 펌웨어/앱 검증 기록: 호스트501·비구동 BLE54·Android JVM34 PASS, Android lint 오류·경고0. 업로드/hash와 앱 DHT22 선택 저장·오류 표시·READY 유지·STOP·배경 단절을 확인했다. 이번 문서 정리에서 다시 실행한 결과는 아니다.
- 마지막 업로드 대상 COM11/base MAC `8c:d0:b2:a8:b3:6e`, 광고 `ESP32_BLE_M_B36E`, BLE 주소 `8C:D0:B2:A8:B3:70`. 현재 연결 상태/포트는 미확정이다. 두 보드의 기존 4MB 백업은 `.local/type_m/`에 보존한다.
- 앱은 `bleunoApp/TypeMControl`에 있다. 기존 점검 앱 `BleunoTest`, 기존 교재 SmartIO와 구분한다. 수업용 완성본 배치·단계별 학습 자료와 type_s 작업은 보류했다.
- type_s의 이전 결과와 남은 ERC·DRC·R1/R2 값 차이는 [type_s 검증 기록](../hardware/type_s/v1/validation/README.md)에 있다.

## 유지 규칙

- 계획은 `plan.md`, 참고 정보는 `memo.md`, 이력은 `dev_log.md`에 기록한다.
- 실제 구조·명령 변경은 사용자 요청 범위에서 `inventory.md`에 반영한다.
- 모든 문서에는 제목 바로 아래에 `## 목차`를 둔다.
- `_forAI/`는 사용자 명시 요청이 있을 때 갱신한다. 이번 요청은 다음 세션 인수인계 정리다.
- 사용자 동의 없이 commit/push하지 않는다. 현재 작업 트리에는 기존 코드·문서·하드웨어의 미커밋 변경과 untracked 파일이 많으므로 일괄 초기화하거나 삭제하지 않는다.
- Python은 기존 설치본을 uv로 실행한다. 새 Python 설치·자동 다운로드는 금지한다.
