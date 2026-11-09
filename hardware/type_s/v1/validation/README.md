# type_s v1 이전 검증

원본은 [source/](../source/README.md)에 보존한 `ZeroFactorySensorBoard / Board_DHT`입니다. 2026-11-01에 `type_s_v1` 프로젝트 이전·원본 대조·저장 파일 재열기 검증을 완료했습니다.

## 원본 기준

| 항목 | 개수 |
|---|---:|
| 물리 부품 | 15 |
| 부품 패드 | 52 |
| 독립 홀 패드 | 3 |
| 트랙 / 비아 | 114 / 0 |
| 연결 네트 그룹 | 12 |
| 외곽 / 내부 절개 선분 | 15 / 8 |
| GND 구리 영역 | 2 |

[source-inventory.json](source-inventory.json)은 `.epcb` 내부의 커스텀 FOOTPRINT와 PCB 문서를 분리해 수집합니다. MCU 인스턴스의 수정된 패드 형상을 기준으로 삼으며, 별도 기본 라이브러리로 바꾸지 않습니다.

원본 웹 DRC는 물리 오류 0개, 회로도/PCB netlist 불일치 알림 1개입니다. [easyeda-baseline.json](easyeda-baseline.json)에 응답과 원본 설계 규칙을 보존했습니다. 분압 R1/R2의 회로도·PCB 저항값 차이는 [핀맵](../pinmap.md)에 기록했습니다.

## 가져오기와 보정 기록

- [initial-kicad-import.zip](initial-kicad-import.zip): 보정 전 KiCad 가져오기 결과.
- [initial-import-comparison.json](initial-import-comparison.json): 초기 부품·패드·배선·외곽 비교. 내부 8변 절개가 Edge.Cuts zone으로 잘못 가져와진 차이와 중복 참조명을 기록합니다.
- [initial-netlist.xml](initial-netlist.xml), [initial-erc.json](initial-erc.json), [initial-drc.json](initial-drc.json): 초기 검사 자료로 현재 상태 판정과 구별합니다.
- [verify_migration.py](verify_migration.py): 원본·PCB·회로도 netlist를 읽어 비교하는 스크립트.
- [rules-mapping.md](rules-mapping.md): 이 보드의 원본 규칙을 KiCad에 대응시키는 범위와 한계.

다음 보정을 적용했습니다.

- [normalize_import.py](normalize_import.py), [normalization-report.json](normalization-report.json): 중복 참조를 U1, R1–R5, J1–J9로 정리하고 UUID·라이브러리 링크와 원본 8변 절개를 복원했습니다. 독립 홀은 MP1–MP3로 구분합니다. 원래 실크 문구 15개는 같은 위치·스타일로 보존했습니다.
- [sync_parity_metadata.py](sync_parity_metadata.py), [parity-metadata-report.json](parity-metadata-report.json): 미연결 핀 6개에 각각 하나의 핀만 속하는 KiCad 네트 이름을 부여했습니다. 다른 핀·트랙·zone과 접속시키지 않았으며 기존 12개 연결 그룹은 그대로입니다.
- [repair_mcu_footprint_id.py](repair_mcu_footprint_id.py), [mcu-footprint-id-report.json](mcu-footprint-id-report.json): MCU 풋프린트 이름의 콜론 이스케이프 불일치를 `ESP32C3_SUPERMINI_CUSTOM`으로 정리했습니다. 라이브러리와 회로도·PCB 링크를 맞추고 실제 16개 패드의 번호·기하를 보존했습니다.
- [migrate_rules.py](migrate_rules.py), [rules-mapping.json](rules-mapping.json): type_s 원본에서 추출한 값으로 프로젝트 제약과 객체 유형별 규칙을 대응시켰습니다. DRC 심각도와 오류 제외 목록은 변경하지 않았습니다.
- [empty.kicad_wks](../kicad/empty.kicad_wks): 원본 도면틀을 유지하면서 KiCad 기본 도면틀의 중복 표시를 막습니다.
- [normalize_empty_connector_values.py](normalize_empty_connector_values.py), [empty-value-report.json](empty-value-report.json): J1–J9의 원본 빈 값을 KiCad netlist의 빈 값 표현 `~`와 맞추고 해당 PCB Value 필드를 숨겼습니다. 원본 빈 값, 보이는 실크, 연결과 동판은 보존했습니다.

정규화는 단계별 재로딩·기하/연결 비교 및 반복 실행 무변경을 확인했습니다. 전체 정규화 스크립트는 최초 가져오기 정리용으로, 후속 설계 편집에 자동 실행하지 않습니다.

## 최종 원본 비교와 PCB 검사

[import-comparison.json](import-comparison.json)의 `checked_comparisons_pass`는 `true`입니다. 위 표의 부품·패드·트랙·외곽·절개·zone 외곽/레이어/네트 및 회로도 netlist 연결을 대조했습니다. 미연결 핀도 원본대로 보존합니다. KiCad 네트 18개는 원본 연결 그룹 12개와 개별 미연결 핀용 네트 6개로 구성됩니다.

[drc.json](drc.json)의 물리 검사는 **오류 0개·경고 39개**, 미배선 연결은 **0개**입니다.

| 물리 DRC 유형 | 심각도 | 개수 |
|---|---|---:|
| `text_height` | 경고 | 15 |
| `silk_overlap` | 경고 | 10 |
| `silk_over_copper` | 경고 | 13 |
| `silk_edge_clearance` | 경고 | 1 |

별도 회로도/PCB 대응 검사에는 `footprint_symbol_mismatch` 경고 **2개**가 있습니다. R1/R2의 원본 회로도 값 `220kΩ`와 PCB 값 `10kΩ`의 차이를 의도적으로 보존한 결과입니다. 따라서 물리 경고와 대응 검사 경고를 합치면 41개이며, 값 차이를 해결했다고 해석하지 않습니다.

### 실크 표시와 구리면

최초 KiCad 가져오기는 부품 Reference 15개를 숨긴 상태였습니다. 그러나 원본 EasyEDA API에서 15개 Designator 모두 `valueVisible=true`, Top Silkscreen임을 확인했습니다. [easyeda-designator-visibility.json](easyeda-designator-visibility.json)에 직접 읽은 값과 위치·회전을 보존했습니다. 현재 사용자 텍스트는 원래 문구·위치·폰트·정렬로 이 표시를 복원한 것입니다. 표시된 실크에 대한 KiCad 검사 범위와 기준 때문에 초기 가져오기보다 경고 수가 늘었습니다. 경고를 없애기 위해 원본 문구를 숨기거나 이동하지 않았습니다.

[before-refill.kicad_pcb](before-refill.kicad_pcb)를 보존한 뒤 KiCad의 `--refill-zones --save-board`로 GND 영역 두 개를 재계산했습니다. 원본 부품·배선·외곽·절개와 zone 외곽·레이어·네트는 그대로지만, **채워진 구리 내부 형상은 KiCad 재계산 결과이므로 원본과 동일하다고 주장하지 않습니다.** 재계산 후 원본 비교를 다시 통과했습니다.

PCB GUI 재열기에서 파싱·라이브러리 오류 없이 55패드·114트랙·0비아·미배선 0개를 확인하고, 외곽·8변 절개·구리면·실크 표시를 검토했습니다. [회로도 SVG](preview/type_s_v1.svg)와 [PCB SVG](preview/type_s_v1-pcb.svg)도 저장했습니다. 실물·제조 출력 검증은 별도입니다.

## 회로도 검사

[erc.json](erc.json)은 **오류 10개·경고 26개**, 합계 36개입니다. 라이브러리 연결 오류는 해소되었습니다.

| ERC 유형 | 심각도 | 개수 |
|---|---|---:|
| `pin_not_connected` | 오류 | 6 |
| `pin_not_driven` | 오류 | 2 |
| `power_pin_not_driven` | 오류 | 2 |
| `pin_to_pin` | 경고 | 26 |

원본 핀 유형·미연결·전원 구동 표현은 실제 회로와 모듈을 확인한 뒤 결정할 항목입니다. 검사 오류를 줄이기 위한 임의 연결, 저항값 일치, 핀 유형 변경 또는 오류 제외는 수행하지 않습니다. 회로도 GUI 재열기에서 파싱·라이브러리 오류 없이 표시되고 도면틀이 중복되지 않는 것을 확인했습니다.

## 재검증 명령

`hardware/type_s/v1`에서 실행합니다. 기존 KiCad Python을 `uv`로 재사용하며 Python 다운로드를 금지합니다.

```powershell
$kicadBin = Join-Path $env:LOCALAPPDATA 'Programs\KiCad\10.0\bin'
$kicadCli = Join-Path $kicadBin 'kicad-cli.exe'
$kicadPython = Join-Path $kicadBin 'python.exe'
uv python list --only-installed
& $kicadCli sch export netlist --format kicadxml -o validation/netlist.xml kicad/type_s_v1.kicad_sch
& $kicadCli sch erc --format json --severity-all -o validation/erc.json kicad/type_s_v1.kicad_sch
& $kicadCli pcb drc --format json --severity-all --schematic-parity -o validation/drc.json kicad/type_s_v1.kicad_pcb
uv run --no-python-downloads --python $kicadPython python validation/verify_migration.py --pcb kicad/type_s_v1.kicad_pcb --netlist validation/netlist.xml --output validation/import-comparison.json
```

검사 전에 편집 내용을 저장합니다. 비교 스크립트는 입력 파일 해시와 검사 중 입력 변경 여부를 확인합니다. 연결·기하 비교는 실물 부품의 핀아웃, 전압, 조립·제작 적합성 또는 펌웨어 동작을 보증하지 않습니다.
