# type_s v1 설계 규칙 이전

근거는 이 보드에서 2026-11-01에 읽어 저장한 [easyeda-baseline.json](easyeda-baseline.json)이다. `migrate_rules.py`는 type_s의 해당 파일만 읽으며 type_m 규칙 파일은 읽지 않는다. 구성 이름은 `JLCPCB Capability(Two Layers Board)`이다. 아래 수치는 원본 설계에 저장된 값이며 현재 제조 공정의 보증값을 뜻하지 않는다.

활성 Safe Spacing 및 Track 기본 선택은 `copperThickness1oz`이다. 12개 net 모두 같은 기본 규칙과 `Copper_copperRegion`/`Plane_innerPlane` 선택을 사용한다. 스크립트는 이 선택과 해당 간격표를 확인하고, 묶어서 대응할 값이 서로 다르면 중단한다.

## 파일과 재현

- [migrate_rules.py](migrate_rules.py): 기본 실행은 `rules-proposal.json`과 `proposed-type_s_v1.kicad_dru`만 생성한다. `--apply`는 최종 `kicad/type_s_v1.kicad_pro`와 `.kicad_dru`에 아래 값을 적용하고 `rules-mapping.json`에 전후 값·입력 해시를 기록한다.
- 적용 시 PCB·회로도 파일 해시, 기존 DRC 심각도와 제외 목록이 유지되는지 검사한다. 기하와 연결을 수정하지 않는다.
- 사용자 규칙은 보드 전체의 객체 유형만 사용한다. 참조명·net·좌표·위반 ID에 따른 예외는 없다.

KiCad의 전역 최소 제약은 사용자 규칙으로 더 낮게 재정의할 수 없으므로 대응값 중 최솟값을 전역에 두고 객체별 규칙을 추가한다. 구체적인 규칙을 파일 뒤에 두는 것은 역순 규칙 평가에 따른 것이다. [KiCad 10 사용자 규칙 설명](https://docs.kicad.org/10.0/en/pcbnew/pcbnew.html#custom-design-rules)

## 원본에서 추출한 대응값

단위는 mm이다. 값 `0.29999939999999997`도 반올림한 문자열로 치환하지 않고 원본 JSON 수치를 전달한다. 내부 정수 길이 단위로 표현되지 않는 소수 부분은 KiCad가 해석한다.

| 원본 대상 | KiCad 대응 | 값 |
|---|---|---:|
| Track–Track | Track 쌍 `clearance` | 0.10199878 |
| Track–TH Pad, TH Pad–TH Pad | 일반 `clearance`, Default netclass | 0.15200122 |
| Copper/Plane Zone–구리 | Zone 쌍 `clearance` | 0.254 |
| Track/TH Pad–Board Outline | 일반 `edge_clearance` | 0.29999939999999997 |
| Zone–Board Outline | Zone `edge_clearance` | 0.254 |
| Track/TH Pad–Hole | 일반 `hole_clearance` | 0.17526 |
| Zone–Hole | Zone `hole_clearance` | 0.254 |
| Hole–Hole | `hole_to_hole`, 전역 최소 | 0.29999939999999997 |
| Track 폭 최소/기본/최대 | `track_width` | 0.127 / 0.254 / 2.54 |

전역 최소 설정은 copper clearance 0.10199878, copper edge clearance 0.254, hole clearance 0.17526, hole-to-hole 0.29999939999999997, track width 0.127이다. Default netclass의 clearance는 0.15200122, track width는 0.254이다. 이 값은 source `Track`, `TH Pad`, `Copper/Plane Zone`, `Hole`, `Board Outline` 항목에서 추출한다.

현재 검증 대상은 앞면 부품 15개/패드 52개, 독립 패드 3개, 직선 트랙 114개, GND 구리 영역 2개이다. importer가 만든 잘못된 Edge.Cuts Zone을 원본 내부 절개 8변의 폐루프로 복원한 상태를 전제로 한다. 외곽은 15변이다.

## 동일하게 대응하지 못한 범위

원본의 **Slot Region–Track/TH Pad** 값은 0.15200122로, **Board Outline–Track/TH Pad** 값 0.29999939999999997과 다르다. KiCad의 `edge_clearance`는 외곽과 내부 Edge.Cuts, 타원형 가공 홀을 함께 검사한다. 이번 이전에서는 내부 절개별 group/class나 위치 예외를 추가하지 않으므로 내부 절개에도 보수적인 Board Outline 기준이 적용될 수 있다. 이 때문에 남는 edge 오류는 원본 기하 손상과 구별해야 한다. [KiCad 공식 제약 정의](https://docs.kicad.org/10.0/en/pcbnew/pcbnew.html#constraint-clauses)

원본 Slot Region 값, 적용된 KiCad 규칙명·값 및 실제 간격을 함께 확인한다. 잔여 오류를 없애려고 절개·트랙·패드를 이동하거나 수치를 낮추지 않는다. 원본 Hole 분류와 KiCad의 drilled-hole/edge 검사 대상도 완전히 같다고 주장하지 않는다.

다음 항목은 baseline에 보존하되 이번 간격·트랙 폭 매핑에 새로 적용하지 않는다.

- 비활성 `copperThickness2oz` 및 이 보드에 없는 via/차동쌍 설정.
- Net Length Range의 0 값과 대상 그룹이 불명확한 길이 tolerance.
- 현재 독립 구리 객체가 없는 Fill Region/Teardrop, Graphic/Text/Image, test point별 분류. 향후 추가하면 일반 간격만으로 원본 규칙 전체를 재현했다고 볼 수 없다.
- 솔더/페이스트 마스크 확장, thermal/spoke/direct 연결 모드. 제조 기하에 영향을 주므로 숫자·모드 코드를 추측해 다시 적용하지 않는다.
- 원본과 직접 대응하지 않는 KiCad courtyard, 실크·텍스트, annular 등 기존 검사. 기존 심각도·제외 목록을 유지한다.

원본 live DRC는 물리 오류 0건과 회로도–PCB netlist 불일치 1건을 보고했다. 이것이 KiCad DRC 0건을 보증하지는 않는다. 최종 DRC와 [원본 비교 검증기](verify_migration.py)의 결과를 함께 읽어야 한다. 해당 검증기는 부품·패드·배선·외곽·절개·zone 경계와 12개 연결 그룹을 비교하고, 재계산된 채움 구리 형상 자체 및 실크는 별도 확인 대상으로 남긴다.
