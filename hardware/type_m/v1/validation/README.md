# BLEuno type_m v1 이전 검증 기록

## 확인된 결과

2026-10-31 기준으로 원본 보존과 KiCad 프로젝트 이전을 완료했습니다.
[import-comparison.json](import-comparison.json)의 `checked_comparisons_pass: true`는 아래 비교 항목이 통과했다는 뜻입니다.
이는 ERC·DRC 무오류, 회로 동작 또는 제작 가능 판정이 아닙니다.

| 비교 대상 | 확인 결과 |
|---|---|
| 물리 부품 | 12개 대응 |
| 부품 패드 | 52개 위치·치수·홀·네트 대응 |
| 독립 홀 패드 | 2개, `MP1/MP2` |
| 트랙 / 비아 | 118개 / 0개 |
| 연결 네트 그룹 | 17개, 원본 PCB ↔ KiCad PCB ↔ 회로도 netlist 비교 통과 |
| 외곽과 내부 절개 | Edge.Cuts 선분 28개 대응 |
| 구리 영역 | GND 영역 2개의 외곽·레이어·네트 비교 |

핀의 미연결 상태도 원본대로 보존합니다. 원본부터 존재한 SEN1의 MCU 미연결과 VIN의 단일 핀 연결은 [핀맵](../pinmap.md)에 설명되어 있습니다.
동판의 채워진 모든 도형, 실제 부품 핀아웃, 전원 허용범위와 제조 적합성은 이 비교만으로 보증하지 않습니다.

## 검사 결과 읽기

- **원본 EasyEDA:** [easyeda-baseline.json](easyeda-baseline.json)에 물리 DRC 오류 0개와 회로도/PCB netlist 알림 1개를 보존했습니다. 독립적인 파일 대조에서는 12개 물리 부품의 52개 패드 연결이 일치했습니다. 원본 알림을 지우거나 원본 설계를 수정하지 않았습니다.
- **KiCad ERC:** [erc.json](erc.json), 48개 — 오류 22개와 경고 26개입니다.
- **KiCad DRC:** [drc.json](drc.json), **오류 0개·경고 12개**. 모두 원래 실크 문구의 `text_height` 경고입니다. 최소 높이 0.8 mm에 대해 일부 원본 문구가 0.686 mm입니다. 기존 실크 외형을 보존하고 경고를 남겼습니다.
- **미배선 연결 / 회로도·PCB 일치 검사 차이:** 각각 **0개 / 0개**입니다. 미배선 연결 0개는 현재 netlist를 모두 배선했다는 뜻이며, 설계 의도상 빠진 SEN1 등의 연결까지 완성됐다는 뜻이 아닙니다.

| ERC 유형 | 심각도 | 개수 |
|---|---|---:|
| `pin_not_connected` | 오류 | 7 |
| `pin_not_driven` | 오류 | 13 |
| `power_pin_not_driven` | 오류 | 2 |
| `pin_to_pin` | 경고 | 25 |
| `isolated_pin_label` | 경고 | 1 |

핀 유형·미연결·전원 구동 표현은 원본 심볼과 실제 회로를 검토한 뒤 결정할 항목입니다.
검사를 통과시키려는 목적으로 입력 핀을 수동형으로 바꾸거나, 임의 배선·PWR_FLAG·오류 제외를 추가하지 않았습니다.
원본 DRC와 KiCad DRC는 객체 분류와 검사 규칙이 완전히 같지 않습니다. 특히 Slot Region과 Edge.Cuts의 차이는 [rules-mapping.md](rules-mapping.md)를 읽고 판단합니다.

## 동판 재계산과 마지막 보정

원본에서 가져온 동판 채움 결과는 대응시킨 KiCad 규칙으로 검사했을 때 간격 위반을 발생시켰습니다.
[before-refill.kicad_pcb](before-refill.kicad_pcb)와 해당 단계의 [parity-drc.json](parity-drc.json)을 보존한 뒤 KiCad CLI의 `--refill-zones --save-board`로 두 GND zone을 재계산했습니다. [drc-before-refill.json](drc-before-refill.json)은 메타데이터 동기화 전 검사이므로 회로도/PCB 차이 13개도 포함합니다.
이전 전의 원본 아카이브와 최초 가져오기 ZIP도 그대로 보존되어 있습니다.

재계산 후 원본 대조를 다시 실행해 `checked_comparisons_pass: true`를 확인했습니다.
부품·패드·트랙·외곽·절개·네트와 zone 외곽·레이어·네트는 유지되지만 **채워진 구리 내부 형상은 KiCad가 재계산한 결과이며 원본과 동일하다고 주장하지 않습니다.**
동판 재계산 후 최종 DRC는 위의 텍스트 높이 경고 12개만 남았습니다.

추가 메타데이터 동기화로 회로도/PCB 일치 검사 차이를 0개로 맞췄습니다. 변경 내역은 [parity-metadata-report.json](parity-metadata-report.json)에 있습니다.
GUI에서 회로도를 다시 열어 파싱·라이브러리 경고 없이 표시되는 것을 확인했습니다. 프로젝트가 참조하던 `empty.kicad_wks`를 추가해 기본 프레임 중복 표시를 없애고 원본 도면틀 심볼을 유지했습니다.
PCB도 GUI 재열기에서 파싱·라이브러리 경고 없이 배선·중앙 14선 절개·외곽·GND 채움이 표시되는 것을 확인했습니다.
GUI 집계는 패드 54개·비아 0개·트랙 118개·미배선 0개입니다. 전체 net 24개에는 비교한 연결 그룹 17개와 미연결 핀의 개별 net 7개가 포함되어 있으므로 17개라는 비교 수치와 구분합니다.
두 SVG 미리보기는 최종 저장 상태에서 다시 출력했습니다. 이 GUI 확인은 실제 제작·실장 보드의 동작 검증이 아닙니다.

## 파일 안내

하드웨어 모델명은 `type_m`, 버전은 `v1`이며 현재 프로젝트는 `type_m_v1`입니다. [이름 변경 기록](rename-report.json)에 변경 전후 해시를 보존했습니다. 최초 가져오기·중간 검사 보고서와 `before-refill.kicad_pcb`의 `bleuno_v1` 표기는 당시 증거로 유지하고, 현재 netlist·ERC·DRC·원본 비교 및 미리보기는 새 이름으로 다시 생성했습니다.

| 파일 | 역할 |
|---|---|
| [source-inventory.json](source-inventory.json) | 원본 부품·패드·배선·형상·회로도 목록 |
| [easyeda-baseline.json](easyeda-baseline.json) | 웹 원본의 설계 규칙과 DRC 응답 |
| [initial-kicad-import.zip](initial-kicad-import.zip) | 정리하기 전 KiCad 가져오기 결과 |
| `initial-netlist.xml`, `initial-erc.json`, `initial-drc.json`, `initial-import-comparison.json` | 초기 가져오기 단계 검사. 현재 상태 판정에는 사용하지 않음 |
| [normalize_import.py](normalize_import.py) | 참조·UUID 링크·라이브러리·내부 절개 보정의 재현 스크립트 |
| [normalization-report.json](normalization-report.json) | 원본→KiCad 참조 대응과 기하/패드 연결 불변 검사 |
| [sync_parity_metadata.py](sync_parity_metadata.py), [parity-metadata-report.json](parity-metadata-report.json) | 마지막 회로도/PCB 메타데이터 동기화와 변경 증거 |
| [rules-mapping.md](rules-mapping.md), [rules-mapping.json](rules-mapping.json) | 적용한 원본 설계 규칙과 표현 한계, 설정 변경 증거 |
| [verify_migration.py](verify_migration.py) | 원본·KiCad PCB·회로도 netlist를 읽어 비교하는 스크립트 |
| [netlist.xml](netlist.xml), [import-comparison.json](import-comparison.json) | 최종 연결과 원본 대조 결과 |
| [erc.json](erc.json), [drc.json](drc.json) | 최종 ERC·DRC와 회로도/PCB 일치 검사 |
| [drc-before-refill.json](drc-before-refill.json) | 원본 규칙 대응 후 동판 재계산 전의 검사 결과. 최종 DRC와 구별 |
| [before-refill.kicad_pcb](before-refill.kicad_pcb) | 동판을 재계산하기 직전의 PCB 보존본 |
| [preview/type_m_v1.svg](preview/type_m_v1.svg), [preview/type_m_v1-pcb.svg](preview/type_m_v1-pcb.svg) | 저장된 설계의 SVG 미리보기 |

원본 `.epro`와 `.epro2`의 해시·출처는 [source/README.md](../source/README.md)에 있습니다.
참조명 변경 전후에도 패드·트랙·네트와 회로도 기하가 같은지 검사했습니다. 내부 절개 복원만 의도한 형상 보정으로 분리했습니다.
`normalize_import.py`는 두 번 실행해 주요 프로젝트/라이브러리 파일이 바이트 단위로 동일함을 확인했으며, 도면틀 라이브러리 동기화 함수도 별도로 멱등성을 확인했습니다.
이 스크립트는 최초 가져오기 결과를 정리하는 용도입니다. 후속 설계 편집에 자동으로 실행하지 않습니다.

## 재검증 명령

아래 PowerShell 명령은 `hardware/type_m/v1`에서 실행합니다. 파일 검증용 명령이며 실제 보드에는 업로드하지 않습니다.
설치된 KiCad `10.0` 경로를 재사용하고 Python을 설치하거나 다운로드하지 않습니다.

```powershell
$kicadBin = Join-Path $env:LOCALAPPDATA 'Programs\KiCad\10.0\bin'
$kicadCli = Join-Path $kicadBin 'kicad-cli.exe'
$kicadPython = Join-Path $kicadBin 'python.exe'
uv python list --only-installed
& $kicadCli sch export netlist --format kicadxml -o validation/netlist.xml kicad/type_m_v1.kicad_sch
& $kicadCli sch erc --format json -o validation/erc.json kicad/type_m_v1.kicad_sch
& $kicadCli pcb drc --format json --schematic-parity -o validation/drc.json kicad/type_m_v1.kicad_pcb
uv run --no-project --no-python-downloads --python $kicadPython python validation/verify_migration.py --pcb kicad/type_m_v1.kicad_pcb --netlist validation/netlist.xml --output validation/import-comparison.json
```

검사 전 편집 내용을 저장하고 파일이 검사 도중 바뀌지 않도록 합니다.
`verify_migration.py`는 입력 파일 해시를 보고서에 기록하고 검사 중 입력 변경을 확인합니다.
검사 보고서를 갱신할 때는 사용한 프로젝트와 규칙 파일도 함께 보존합니다.
