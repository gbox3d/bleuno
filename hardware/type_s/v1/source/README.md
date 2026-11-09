# BLEuno type_s v1 기준 원본

사용자가 지정한 EasyEDA **ZeroFactorySensorBoard → Board_DHT → PCB1**과 연결 회로도의 보존본입니다. 원본 웹 설계는 수정하지 않았습니다.

- [지정한 원본 설계](https://pro.easyeda.com/editor#id=cdcbe964907a4283a8597f5fe1b00431,tab=*287e5461706743eeb18d8d86160a742f@cdcbe964907a4283a8597f5fe1b00431)
- 프로젝트: `cdcbe964907a4283a8597f5fe1b00431`
- PCB: `287e5461706743eeb18d8d86160a742f`
- 회로도: `170357fab581471f8a718f854fc9f31b`
- 회로도 페이지: `1056de0b4f7d41fa936977e612ff4c55`
- 내보낸 편집기: EasyEDA Pro `3.2.209`
- 가져오기 도구: 기존 설치된 KiCad `10.0.6`

## 보관 파일

| 파일 | 용도 | 크기 | 내보낸 시각(KST) |
|---|---|---:|---|
| [ZeroFactorySensorBoard_Board_DHT_2026-11-01.epro](ZeroFactorySensorBoard_Board_DHT_2026-11-01.epro) | KiCad 가져오기용 호환 아카이브 | 92,362바이트 | 2026-11-01 11:50:56 |
| [ZeroFactorySensorBoard_Board_DHT_2026-11-01_native.epro2](ZeroFactorySensorBoard_Board_DHT_2026-11-01_native.epro2) | EasyEDA 현재 형식 원본 | 199,647바이트 | 2026-11-01 11:51:07 |

```text
SHA-256
0681b9a902b06c018cb0079b439ae88aa3eb3200a29b7b0be6b08c4da0565d90  ZeroFactorySensorBoard_Board_DHT_2026-11-01.epro
7f84921f94b5624b4a0009df522c4bf95ae684696929fc61ccc624c5ab2f9416  ZeroFactorySensorBoard_Board_DHT_2026-11-01_native.epro2
```

같은 지정 프로젝트를 `SYS_FileManager.getProjectFile()`의 `epro`, `epro2` 옵션으로 내보냈습니다. 호환 `.epro` 내부 JSON 배열 형식은 웹 원본이 V2 읽기 전용이라는 의미가 아닙니다. 웹 원본에서 `start`, `main` 브랜치를 확인했습니다.

## 원본 검증과 주의할 차이

- [원본 부품·패드·배선·형상 목록](../validation/source-inventory.json)
- [웹 원본 설계 규칙과 DRC 응답](../validation/easyeda-baseline.json)
- [최초 KiCad 가져오기 파일](../validation/initial-kicad-import.zip)
- [부품·커넥터 핀맵](../pinmap.md)

원본 PCB DRC는 물리 오류 0개와 회로도/PCB netlist 불일치 알림 1개를 반환했습니다. 원본 알림을 지우거나 회로를 임의로 일치시키지 않았습니다.

PCB 문서 안에 MCU의 커스텀 FOOTPRINT 문서가 함께 들어 있습니다. 별도 기본 풋프린트보다 이 인스턴스에 연결된 형상을 기준으로 비교합니다. 배터리 분압 저항 두 개는 회로도 값과 PCB의 기본값·표기가 다르므로 핀맵의 검토 항목을 함께 확인합니다.

새 원본을 확보할 때에는 날짜가 다른 아카이브와 해시를 추가해 이 보존본을 유지합니다.
