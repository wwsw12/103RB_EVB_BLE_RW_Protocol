# 03. STM32CubeMX 및 MCU 주변장치 설정

> 근거: [main.c](../Core/Src/main.c) (`SystemClock_Config`, `MX_*_Init`), [stm32f1xx_hal_msp.c](../Core/Src/stm32f1xx_hal_msp.c), [.ioc](../88-2.EVB_DEMO2_RW.ioc)
> 다루는 범위: **RCC, ADC1 + DMA, TIM2 / TIM3 / TIM4, SPI2, I2C1, USART1 / USART2, GPIO / EXTI, NVIC**

## 0. 초기화 순서

```
HAL_Init()             → SysTick 1 ms, NVIC 그룹
SystemClock_Config()   → HSI/2 × 16 = 64 MHz, ADC 클럭 = PCLK2 ÷ 8 = 8 MHz
MX_GPIO_Init()         → CLCD·LED·CS 출력, 스위치 입력, 엔코더 EXTI
MX_DMA_Init()          → DMA1 클럭, DMA1_Channel1 인터럽트
MX_ADC1_Init()         → 3채널 스캔 + 연속 변환
MX_USART1/2_UART_Init()
MX_I2C1_Init()         → PB8/PB9 remap
MX_TIM3/TIM2/TIM4_Init()
MX_SPI2_Init()
```

`MX_DMA_Init()`이 `MX_ADC1_Init()`보다 먼저 호출되어야 ADC의 MSP 초기화에서 DMA 채널을 연결할 수 있습니다 (CubeMX가 생성한 순서 그대로).

## 1. 클럭

| 항목 | 값 | 비고 |
|---|---|---|
| 소스 | HSI 8 MHz → ÷2 → PLL ×16 | HSE 없이 낼 수 있는 최대값 64 MHz |
| SYSCLK / HCLK | 64 MHz | |
| APB1 (PCLK1) | 32 MHz (÷2) | APB1 최대 36 MHz 제한 → TIM2/3/4 클럭은 ×2 되어 64 MHz |
| APB2 (PCLK2) | 64 MHz | |
| ADC 클럭 | 8 MHz (PCLK2 ÷ 8) | ADC 최대 14 MHz 이하 |

## 2. ADC1 + DMA

| 항목 | 값 | 의미 |
|---|---|---|
| 채널 / 순서 | IN0 (가변저항) → IN1 (조도) → IN4 (TC1047) | `ADC_value[0..2]`에 이 순서로 저장 |
| 모드 | Scan + **Continuous** | 3채널을 계속 반복 변환 |
| 샘플링 시간 | 239.5 사이클 | 가장 긴 값 → 소스 임피던스가 커도 안정적 |
| 트리거 | 소프트웨어 시작 1회 | |
| DMA | Channel1, Peripheral → Memory, **Circular**, Half-word | 배열 끝까지 채우면 처음부터 다시 덮어씀 |

**변환 시간 계산**

| 단계 | 값 |
|---|---|
| 1채널 | (239.5 + 12.5) 사이클 ÷ 8 MHz = **31.5 µs** (T<sub>CONV</sub> = 샘플링 시간 + 12.5 사이클, RM0008 p.225) |
| 3채널 1회 | 31.5 × 3 = **94.5 µs** → 초당 약 10,600회 갱신 |

- **DMA를 쓰는 이유:** CPU가 변환 완료를 기다리거나 결과를 옮길 필요 없이, 메인 루프는 `ADC_value[]`에서 최신 값만 읽으면 됩니다.
- **Circular 모드:** 한 번 시작하면 멈추지 않고 같은 배열을 계속 덮어씁니다 (RM0008 p.278 Circular mode).
- 시작 전에 `HAL_ADCEx_Calibration_Start()`로 ADC 오프셋을 보정합니다.
- 참고: HAL은 DMA 반/완료 인터럽트를 켜므로, 94.5 µs마다 인터럽트가 두 번 들어옵니다. 개선점은 [05 문서](05_troubleshooting.md)에 정리했습니다.

## 3. 타이머

| 타이머 | 용도 | Prescaler | Period | 계산 | 결과 |
|---|---|---|---|---|---|
| **TIM2** | 1 ms 기준 시간 (인터럽트) | 63 | 999 | 64 MHz ÷ 64 = 1 MHz, 1000카운트 | **1 kHz → 1 ms** |
| **TIM3** | RGB LED PWM 3채널 | 63 | 255 | 1 MHz ÷ (2 × 255), center-aligned | **약 1.96 kHz**, 듀티 0~255 |
| **TIM4** | 서보 PWM (CH2) | 63 | 19999 | 1 MHz ÷ 20000 | **50 Hz (20 ms)**, 1 count = 1 µs |

- **TIM3 Center-aligned 모드:** 카운터가 0 → 255 → 0으로 올라갔다 내려오므로 주기가 2배(510 µs)가 됩니다 (RM0008 p.303). Period를 255로 둔 이유는 앱이 보내는 RGB 값(0~255, 1바이트)을 그대로 비교값(CCR)에 넣기 위해서입니다.
- **TIM4 1 count = 1 µs:** 서보 펄스 폭(µs)을 그대로 CCR 값으로 쓸 수 있습니다. 예: CCR = 1500 → 1.5 ms 펄스 = 중앙.
- **TIM4 극성 LOW:** 서보 신호가 트랜지스터 Q2를 거치며 반전되므로, 출력 극성을 `TIM_OCPOLARITY_LOW`로 설정해 보정했습니다 ([02 문서](02_hardware.md) 3-4절).
- **TIM3 remap:** TIM3 CH1~3을 PC6/PC7/PC8로 옮기는 Full remap (`__HAL_AFIO_REMAP_TIM3_ENABLE`). EVB 회로도의 RGB LED 연결 핀과 맞추기 위해서입니다.

## 4. SPI2 (DAC7512)

| 항목 | 값 | 이유 |
|---|---|---|
| 모드 | Master, 2-line, 8-bit, MSB first | DAC7512는 16비트를 상위 비트부터 받음 → 8비트 2번 전송 |
| CPOL / CPHA | Low / 2nd edge → **Mode 1** | DAC7512는 SCLK 하강 에지에서 데이터를 읽음 (DAC7512 데이터시트 p.2) |
| 속도 | PCLK1 32 MHz ÷ 32 = **1 MHz** | DAC7512 최대 30 MHz보다 충분히 낮음 |
| NSS | 소프트웨어 (PB12 GPIO) | 16비트 전송 동안 SYNC를 Low로 유지 |
| 핀 | PB13 SCK, PB15 MOSI (MISO 없음) | DAC는 쓰기만 함 |

## 5. I2C1 (HDC1080)

| 항목 | 값 | 이유 |
|---|---|---|
| 속도 | **400 kHz** (Fast mode), Duty 2 | HDC1080 최대 400 kHz (데이터시트 p.6) |
| 핀 | PB8 SCL / PB9 SDA, **remap** (`__HAL_AFIO_REMAP_I2C1_ENABLE`) | EVB 회로도에서 I2C1이 PB8/PB9에 연결됨 |
| 주소 모드 | 7-bit | HDC1080 주소 0x40 |

## 6. USART

| 항목 | USART1 | USART2 |
|---|---|---|
| 용도 | HM-10 블루투스 (앱 통신) | PC 디버그 로그 (`printf`) |
| 속도 | **9600 bps**, 8N1 | 115200 bps, 8N1 |
| 핀 | PA9 TX / PA10 RX | PA2 TX / PA3 RX |
| 수신 | **인터럽트** 1바이트씩 (`HAL_UART_Receive_IT`) | 콜백은 있으나 수신 시작 코드 없음 |
| 송신 | Blocking (`HAL_UART_Transmit`, timeout 50 ms) | Blocking (`printf`) |

**9600 bps 전송 시간:** 1바이트 = 10비트 ÷ 9600 = 약 1.04 ms. R 패킷(7~10바이트)은 약 7.3~10.4 ms 동안 CPU가 전송을 기다립니다.

## 7. GPIO / EXTI

| 핀 | 설정 | 이유 |
|---|---|---|
| PC2~PC5, PC10, PC11 | Output PP, Low speed | CLCD (느린 신호) |
| PA6, PA7, PA11, PA12 | Output PP, 초기 Low | LED 꺼진 상태로 시작 |
| PB12 | Output PP | DAC7512 SYNC (CS) |
| PB1, PB2, PB4, PB5 | Input, No pull | 택트 스위치 (EVB에 220 kΩ 외부 풀업) |
| PC12, PA15 | **EXTI 상승 에지**, No pull | 엔코더 A / B |
| PA8 | **EXTI 하강 에지**, No pull | 엔코더 스위치 (누를 때 Low) |
| SWJ | **JTAG 비활성화, SWD만 사용** (`__HAL_AFIO_REMAP_SWJ_NOJTAG`) | PA15·PB4를 GPIO로 쓰기 위해 |

## 8. NVIC (인터럽트 우선순위)

| 인터럽트 | 선점 우선순위 | 용도 |
|---|---|---|
| DMA1_Channel1 | 0 | ADC 결과 전송 |
| TIM2 | 0 | 1 ms 기준 시간 |
| USART1 / USART2 | 0 | 패킷 수신 |
| EXTI9_5 / EXTI15_10 | 0 | 엔코더 |
| SysTick | 15 (최하위) | `HAL_GetTick`, `HAL_Delay` |

- 모든 주변장치 인터럽트가 같은 우선순위(0)라서 서로 선점하지 않고, 들어온 순서대로 처리됩니다. 각 콜백이 짧아서 현재는 문제가 없지만, 우선순위를 나누지 않은 점은 개선 항목으로 정리했습니다 ([05 문서](05_troubleshooting.md)).
