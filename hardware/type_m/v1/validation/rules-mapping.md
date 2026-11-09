# BLEuno type_m v1 설계 규칙 이전 범위

2026-10-31에 저장한 [EasyEDA 검사 기준](easyeda-baseline.json)의 `configuration.config`를 기준으로 작성했다. 원본 구성은 `JLCPCB Capability(Two Layers Board)`이며, 활성 기본 간격·트랙 규칙은 `copperThickness1oz`이다. 아래 값은 이 설계에 저장된 값이며 제조사의 현재 공정 보증값으로 재해석하지 않는다.

목적은 원본 설계 규칙의 보존이다. KiCad 기본 간격 0.2 mm와 모서리 간격 0.5 mm를 그대로 사용하면 원본과 다른 기준으로 판정되므로, 원본에서 대응 가능한 항목을 가져왔다. 오류 개수를 줄이기 위한 참조명·net·좌표·위반 ID별 예외는 넣지 않았다. 오류 심각도와 `drc_exclusions`도 변경하지 않았다.

## 적용한 파일과 근거

- [type_m_v1.kicad_pro](../kicad/type_m_v1.kicad_pro): 전역 최솟값 및 Default netclass 기본값.
- [type_m_v1.kicad_dru](../kicad/type_m_v1.kicad_dru): 보드 전체에 적용되는 객체 유형별 규칙.
- [rules-mapping.json](rules-mapping.json): 원본 해시, 설정 전후 값, 규칙 파일 해시, PCB 파일 변경 없음 확인.

KiCad 전역 최소 제약은 더 구체적인 규칙으로 낮출 수 없으므로 가장 작은 대응값을 전역에 두고 유형별 규칙을 추가했다. 조건은 `A.Type`/`B.Type`을 사용하며, 더 구체적인 규칙은 뒤에 둔다. 이는 KiCad의 역순 평가 및 A/B 쌍 검사 방식에 따른 것이다. [KiCad 10 공식 제약·사용자 규칙 설명](https://docs.kicad.org/10.0/en/pcbnew/pcbnew.html#custom-design-rules)

## 직접 대응한 수치

단위는 mm이다. 표의 0.2999994는 원본 JSON에 `0.29999939999999997`로 저장되어 있으며, 실제 설정 파일에는 원본 JSON 수치를 그대로 전달했다. KiCad 내부 정수 길이 단위로 표현할 수 없는 소수 부분은 KiCad가 해석한다.

| 원본 항목 | KiCad 적용 | 값 |
|---|---|---:|
| Track ↔ Track | `clearance`, 두 객체가 Track | 0.10199878 |
| Track ↔ Pad, Pad ↔ Pad 등 일반 구리 객체 | 일반 `clearance` 및 Default netclass | 0.15200122 |
| Copper/Plane Zone ↔ 다른 구리 객체 | Zone을 포함하는 쌍의 `clearance` | 0.254 |
| 일반 구리 ↔ Board Outline | 일반 `edge_clearance` | 0.2999994 |
| Copper/Plane Zone ↔ Board Outline | Zone의 `edge_clearance` | 0.254 |
| 일반 구리 ↔ Hole | 일반 `hole_clearance` | 0.17526 |
| Copper/Plane Zone ↔ Hole | Zone을 포함하는 쌍의 `hole_clearance` | 0.254 |
| Hole ↔ Hole | `hole_to_hole` 및 전역 최소 | 0.2999994 |
| Track 최소 / 기본 / 최대 폭 | `track_width` | 0.127 / 0.254 / 2.54 |

전역 최소는 구리 간격 0.10199878, 구리–모서리 0.254, 구리–홀 0.17526, 홀–홀 0.2999994, 트랙 폭 0.127이다. Default netclass의 간격은 0.15200122, 트랙 폭은 0.254이다. 기타 KiCad 설정을 새 원본값이라고 주장하지 않는다.

현재 물리 객체는 트랙 118개, 부품 패드 52개, 독립 패드 2개 및 GND 구리 영역 2개이다. 원본에서 구리 영역으로 가져온 Zone만 남도록 내부 절개를 별도 `Edge.Cuts`로 복원한 상태를 전제로 한다.

## 원본과 같지 않은 판정 범위

**Slot Region과 Board Outline의 구분은 완전히 옮기지 못했다.** 원본 간격표는 Slot Region–Track/Pad를 0.15200122로, Board Outline–Track/Pad를 0.2999994로 구분한다. 원본 내부 절개는 KiCad에서 정상적인 `Edge.Cuts` 폐루프로 복원되며, KiCad의 `edge_clearance`는 보드 외곽뿐 아니라 내부 `Edge.Cuts` 및 타원형 가공 홀도 대상으로 삼는다. [KiCad 공식 edge_clearance 정의](https://docs.kicad.org/10.0/en/pcbnew/pcbnew.html#constraint-clauses)

이번 이전에서는 원본 유형을 보존하기 위한 별도 group/class나 영역별 예외를 만들지 않았다. 따라서 내부 절개나 타원홀과 일반 구리 사이에도 보수적인 0.2999994 기준이 적용될 수 있다. 이 영역에서 남는 모서리 간격 오류는 원본 기하가 바뀌었다는 증거와 구별해야 한다. 원본 Slot Region 기준(0.15200122), KiCad가 선택한 기준(0.2999994), 실제 간격을 함께 읽어 판정한다. Zone과 모서리의 간격은 0.254이다.

또한 원본의 Hole, Slot Region, Board Outline 분류와 KiCad의 `hole_clearance`·`hole_to_hole`·`edge_clearance` 검사 대상은 완전히 일치하지 않는다. 원본 DRC 0건과 KiCad DRC 0건이 서로를 보증하지 않는다. 잔여 오류를 없애려고 원본 절개·트랙·패드를 이동하거나 규칙을 낮추지 않는다. 최종 잔여 오류 수와 항목은 최종 DRC 보고서에서 확인한다.

## 이번에 새로 적용하지 않은 항목

- **비활성 2 oz 규칙:** 활성 기본값이 아니므로 적용하지 않았다.
- **Via Size·Blind/Buried Via·차동쌍:** 현재 보드에는 비아와 차동쌍이 없다. 해당 기능을 추가할 때 원본 설정과 KiCad 표현을 별도로 검토한다. 원본 수치는 baseline JSON에 보존되어 있다.
- **Net Length Range·Tolerance:** `0` 범위값과 대상 그룹의 의미를 KiCad의 길이 제한으로 추측하지 않았다.
- **Fill Region/Teardrop·구리 Graphic/Text/Image·test point 세부 분류:** 현재 검사 대상에 해당하는 독립 구리 객체가 없거나 importer가 원본 유형을 보존하지 않는다. 추후 추가하면 일반 clearance만으로 원본 규칙을 모두 재현했다고 볼 수 없다.
- **Slot Region의 별도 간격:** 위 한계대로 보수적인 Edge.Cuts 판정을 유지한다.
- **Solder/Paste Mask·열 연결·spoke·직결 모드:** 기하 또는 제조 출력에 영향을 주는 설정이다. 이번 .pro/.dru 간격 이전에서 숫자·모드 코드를 추측해 재적용하지 않았다. 기존 가져오기 결과와 원본을 별도로 확인해야 한다.
- **기타 KiCad 기본 검사:** 원본에 일대일 대응 근거가 없는 courtyard, 실크, 텍스트, 최소 annular 등은 원본 규칙으로 간주하지 않는다. 기존 심각도와 제외 상태는 그대로 보존했다.

## 확인 방법

원본 규칙 JSON, 이 문서, 설정 전후 기록을 함께 보관한다. KiCad가 사용자 규칙을 읽은 최종 DRC JSON에서 적용 규칙명과 실제 간격을 확인한다. PCB 파일의 내용은 이 규칙 이전에서 변경되지 않았으며, 좌표·패드·트랙·외곽·절개 및 net 연결의 보존은 [verify_migration.py](verify_migration.py)로 별도 확인한다. 규칙 수정 후 DRC 결과가 남더라도 이 문서의 판정 차이와 원본의 기존 회로도–PCB 불일치를 먼저 구별한다.
