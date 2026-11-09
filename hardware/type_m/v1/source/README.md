# BLEuno type_m v1 기준 원본

사용자가 지정한 EasyEDA **MovingTarget → MotorConT1 → PCB1**과 연결 회로도를 보존한 스냅샷입니다.
원본 웹 설계는 변경하지 않았습니다.

| 항목 | 값 |
|---|---|
| 프로젝트 ID | `a6f9cc95756f415597ebfd40a028dfdd` |
| PCB ID | `230020836b7d42a2ae0bd4e6ece12948` |
| 회로도 ID | `dd2cfae3a64d4b3db5dd5a512ac44bc2` |
| 페이지 ID | `06fa99e526b74b38a2f6d5d9c6b834aa` |
| 내보낸 편집기 | EasyEDA Pro `3.2.209` |
| 가져오기 도구 | KiCad `10.0.6` |

## 보관 파일

- [`MovingTarget_MotorConT1_2026-10-31.epro`](MovingTarget_MotorConT1_2026-10-31.epro): KiCad 가져오기용 호환 내보내기. 69,035바이트. 2026-10-31 18:15:56 KST.
- [`MovingTarget_MotorConT1_2026-10-31_native.epro2`](MovingTarget_MotorConT1_2026-10-31_native.epro2): EasyEDA 현재 형식의 원본 보존용. 234,690바이트. 2026-10-31 18:24:03 KST.

```text
SHA-256
AD53D548A11A0D6652DB9E8F0933E21C97D479A1E3235C9C981F93750B959BDA  MovingTarget_MotorConT1_2026-10-31.epro
B3CEB33C0E6DD1BCDCBFDF5A93C2447477274820E0BC2EFB70D7159924909487  MovingTarget_MotorConT1_2026-10-31_native.epro2
```

두 파일은 같은 지정 프로젝트를 `SYS_FileManager.getProjectFile()`의 `epro`, `epro2` 옵션으로 내보낸 것입니다.
`.epro` 내부 설계는 V2 JSON 배열 형식이며, 이것이 현재 웹 프로젝트 자체가 V2 읽기 전용이라는 뜻은 아닙니다.
원본 웹 프로젝트에는 `main`, `start` 브랜치가 존재했습니다.

## 원본 검증 기록

- [원본 부품·패드·네트·형상 목록](../validation/source-inventory.json)
- [원본 EasyEDA 설계 규칙과 DRC](../validation/easyeda-baseline.json)
- [KiCad 최초 가져오기 결과 보존](../validation/initial-kicad-import.zip)

원본 EasyEDA PCB DRC는 물리 오류 0개와 회로도/PCB netlist 불일치 알림 1개를 반환했습니다.
별도의 파일 대조에서는 회로도와 PCB의 물리부품 12개·패드 52개 연결을 확인했습니다.
이 결과가 원본의 센서/전원 연결 미완성까지 해결됐다는 의미는 아닙니다. [V1 핀맵과 검토 항목](../pinmap.md)을 함께 읽습니다.

원본을 다시 내보낼 때는 이 파일을 덮어쓰기보다 날짜가 다른 아카이브와 SHA-256을 추가해 출처를 남깁니다.
