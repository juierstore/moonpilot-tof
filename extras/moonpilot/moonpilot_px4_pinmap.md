# MoonPilot 飞控主控引脚 ↔ PX4 端口对照表

> **一句话结论**：这颗 STM32H743VIH6（TFBGA100）的球位分配与 **MatekH743** 几乎逐脚相同，PX4 板级配置可以拿 `boards/matek/h743` 当底板改；但有 **9 处差异必须改掉**（第 8 节），否则会出现「PWM 少两路、USB 和 SPI 抢脚、LED 反相、console 灌到 VTX」这类问题。

## 0. 数据来源与校验

| 项 | 内容 |
|---|---|
| 网表 | `moonpilot网表.tel`（PADS 格式；133 个网络，主控 U1 用了 92 个球） |
| 球位→引脚 | ST **DS12110 Rev 11** Table 9「Pin/ball definition」的 TFBGA100 列 |
| PX4 参考设计 | `PX4-Autopilot/boards/matek/h743`（同 MCU、同封装、无 IO 协处理器） |
| 复用功能校验 | 外设类网络 42 条全部与数据手册复用功能表对齐（42/42） |
| 未使用球 | E2、F3 = NC；F7 = PDR_ON 在网表中无任何连接 |

## 1. 功能块总览

| 功能 | 路数 | PX4 侧对应 | 备注 |
|---|---|---|---|
| PWM 输出 | 8 | `PWM_MAIN_FUNC1..8`（无 IO 板 → 前缀 PWM_MAIN） | TIM3(2) + TIM5(4) + TIM4(2) 三组 |
| UART | 7 | `/dev/ttyS0` … `/dev/ttyS6` | USART1/2/3、UART4/6/7/8 |
| SPI | 4 | `SPI1..SPI4` → 驱动 `-b 1..4` | 2 片 IMU + OSD + 1 路外扩 |
| I2C | 2 | bus 1 外部 / bus 2 内部 | 外罗盘 + 板载 IST8310 / DPS310 |
| CAN | 1 | CAN1 → UAVCAN | TJA1051 + silent 脚 |
| ADC | 6 | `ADC1_CH(10,11,18,7,4,8)` | 双电池电压/电流 + 空速 + RSSI |
| SD | 1 | `/dev/mmcsd0`（SDMMC1） | 无卡检测 |
| USB | 1 | `/dev/ttyACM0`（OTG FS） | 无 VBUS 检测脚 |

## 2. 主表：主控引脚 → PX4 端口

`引脚` = STM32 端口名（高亮复用功能）；`球` = TFBGA100 焊球号，与网表里的 `U1.xx` 一一对应；`PX4 端口` 是运行时看到的名字，`PX4 配置` 是板级代码要写的位置。

### 电机输出

| 网表网络 | 引脚 | 球 | 复用功能 | 板上连接 | PX4 端口 | PX4 配置项 | 备注 |
|---|---|---|---|---|---|---|---|
| MOTOR1 | PB0 | J4 | TIM3_CH3 | U17.1（S1） | PWM_MAIN_FUNC1 | timer_config: TIM3/CH3 |  |
| MOTOR2 | PB1 | K4 | TIM3_CH4 | U17.2（S2） | PWM_MAIN_FUNC2 | timer_config: TIM3/CH4 |  |
| MOTOR3 | PA0 | G2 | TIM5_CH1 | U17.3（S3） | PWM_MAIN_FUNC3 | timer_config: TIM5/CH1 |  |
| MOTOR4 | PA1 | H2 | TIM5_CH2 | U17.4（S4） | PWM_MAIN_FUNC4 | timer_config: TIM5/CH2 |  |
| MOTOR5 | PA2 | J2 | TIM5_CH3 | U18.1（S5） | PWM_MAIN_FUNC5 | timer_config: TIM5/CH3 |  |
| MOTOR6 | PA3 | K2 | TIM5_CH4 | U18.2（S6） | PWM_MAIN_FUNC6 | timer_config: TIM5/CH4 |  |
| MOTOR7 | PD12 | K10 | TIM4_CH1 | U18.3（S7） | PWM_MAIN_FUNC7 | timer_config: TIM4/CH1 |  |
| MOTOR8 | PD13 | J10 | TIM4_CH2 | U18.4（S8） | PWM_MAIN_FUNC8 | timer_config: TIM4/CH2 |  |

### 串口 UART

| 网表网络 | 引脚 | 球 | 复用功能 | 板上连接 | PX4 端口 | PX4 配置项 | 备注 |
|---|---|---|---|---|---|---|---|
| UART1_TX | PA9 | C9 | USART1_TX | U13.2（VTX 控制脚） | /dev/ttyS0 | USART1 = console | ⚠ 启动日志会灌到 VTX 控制脚，建议 console 改 UART3 |
| UART1_RX | PA10 | D10 | USART1_RX | 未引出 | /dev/ttyS0 | USART1 = console |  |
| UART2_TX | PD5 | B6 | USART2_TX | 未引出 | /dev/ttyS1 | CONFIG_BOARD_SERIAL_* |  |
| UART2_RX | PD6 | C6 | USART2_RX | U11.6 | /dev/ttyS1 | CONFIG_BOARD_SERIAL_* | 仅 RX 引出，可作 RX-only 设备 |
| UART3_TX | PD8 | K9 | USART3_TX | 未引出 | /dev/ttyS2 | — | 建议改作 console（不对外） |
| UART3_RX | PD9 | J9 | USART3_RX | 未引出 | /dev/ttyS2 | — | 同上 |
| UART4_TX | PB9 | A4 | UART4_TX | U11.3 | /dev/ttyS3 | CONFIG_BOARD_SERIAL_GPS1="/dev/ttyS3" | 建议 GPS1 |
| UART4_RX | PB8 | B4 | UART4_RX | U11.4 | /dev/ttyS3 | 同上 |  |
| UART6_TX | PC6 | F10 | USART6_TX | U15.1 | /dev/ttyS4 | RC_SERIAL_PORT "/dev/ttyS4" | RC 口（也可反向输出） |
| UART6_RX | PC7 | E10 | USART6_RX / TIM3_CH2 | U15.2 | /dev/ttyS4 | RC_SERIAL_PORT "/dev/ttyS4" | ⚠ 网表名 RX_PPM&&UART6_RX；PPM 会与 MOTOR1/2 抢 TIM3，用串口 RC |
| UART7_TX | PE8 | J5 | UART7_TX | 未引出 | /dev/ttyS5 | — |  |
| UART7_RX | PE7 | H5 | UART7_RX | U18.5（ESC2 遥测） | /dev/ttyS5 | dshot telemetry -d /dev/ttyS5 |  |
| UART8_TX | PE1 | C4 | UART8_TX | 未引出 | /dev/ttyS6 | — |  |
| UART8_RX | PE0 | D4 | UART8_RX | U17.5（ESC1 遥测） | /dev/ttyS6 | dshot telemetry -d /dev/ttyS6 |  |

### SPI

| 网表网络 | 引脚 | 球 | 复用功能 | 板上连接 | PX4 端口 | PX4 配置项 | 备注 |
|---|---|---|---|---|---|---|---|
| SPI1_SCLK | PA5 | H3 | SPI1_SCK | U2.13（ICM-42688 #1） | SPI1 = PX4 SPI bus 1 | GPIO_SPI1_SCK_1 |  |
| SPI1_MISO | PA6 | J3 | SPI1_MISO | U2.1 | SPI1 bus 1 | GPIO_SPI1_MISO_1 |  |
| SPI1_MOSI | PD7 | D6 | SPI1_MOSI | U2.14 | SPI1 bus 1 | GPIO_SPI1_MOSI_3 |  |
| GYRO1_CS | PC15 | B1 | GPIO（SPI1 CS） | U2.12 | SPI1 CS | initSPIDevice(ICM42688P, CS{PC15}, DRDY{PB2}) |  |
| GYRO1_EXTI | PB2 | G5 | GPIO / EXTI | U2.4（INT1） | SPI1 DRDY | 同上 |  |
| SPI2_SCLK | PB13 | J8 | SPI2_SCK | U5.10（AT7456E OSD） | SPI2 = PX4 SPI bus 2 | GPIO_SPI2_SCK_1 |  |
| SPI2_MISO | PB14 | H10 | SPI2_MISO | U5.11 | SPI2 bus 2 | GPIO_SPI2_MISO_1 |  |
| SPI2_MOSI | PB15 | G10 | SPI2_MOSI | U5.9 | SPI2 bus 2 | GPIO_SPI2_MOSI_1 |  |
| AT7456E_CS | PB12 | K8 | GPIO（SPI2 CS） | U5.8（经 R8） | SPI2 CS | initSPIDevice(ATXXXX, CS{PB12}) |  |
| SPI3_SCLK | PB3 | A7 | SPI3_SCK | U22.3 | SPI3 = PX4 SPI bus 3（外部） | initSPIBusExternal(SPI3, ...) |  |
| SPI3_MISO | PB4 | A6 | SPI3_MISO | U22.4 | SPI3 bus 3 | board.h: GPIO_SPI3_MISO_* |  |
| SPI3_MOSI | PB5 | C5 | SPI3_MOSI | U22.5 | SPI3 bus 3 | board.h: GPIO_SPI3_MOSI_* |  |
| SPI3_CS1 | PD4 | D7 | GPIO（SPI3 CS1） | U22.6（经 R51） | SPI3 CS1 | initSPIConfigExternal(CS{PD4}) |  |
| SPI3_CS2 | PE2 | A3 | GPIO（SPI3 CS2） | 仅 R50 上拉到 3V3 | SPI3 CS2 | initSPIConfigExternal(CS{PE2}) | ⚠ 无对外焊盘，只能留给板载第 2 片 SPI 器件 |
| SPI4_SCLK | PE12 | J6 | SPI4_SCK | U3.13（ICM-42688 #2） | SPI4 = PX4 SPI bus 4 | GPIO_SPI4_SCK_* |  |
| SPI4_MISO | PE13 | K6 | SPI4_MISO | U3.1 | SPI4 bus 4 | GPIO_SPI4_MISO_* |  |
| SPI4_MOSI | PE14 | G7 | SPI4_MOSI | U3.14 | SPI4 bus 4 | GPIO_SPI4_MOSI_* |  |
| GYRO2_CS | PE11 | H6 | GPIO（SPI4 CS） | U3.12 | SPI4 CS | initSPIDevice(ICM42688P, CS{PE11}, DRDY{PE15}) |  |
| GYRO2_EXTI | PE15 | H7 | GPIO / EXTI | U3.4（INT1） | SPI4 DRDY | 同上 |  |
| GYRO1_CLKIN | PD14 | H8 | TIM4_CH3 / GPIO | U2.9（ICM-42688 pin9 CLKIN） | 无（见第 9 节第 6 条） | 板级暂不可启用 | ⚠ 驱动有 ConfigureCLKIN()，但 STM32 的 initSPIDevice() 不传 custom1 |
| GYRO2_CLKIN | PD15 | G8 | TIM4_CH4 / GPIO | U3.9（ICM-42688 pin9 CLKIN） | 无（同上） | 板级暂不可启用 | ⚠ 同上 |

### I2C

| 网表网络 | 引脚 | 球 | 复用功能 | 板上连接 | PX4 端口 | PX4 配置项 | 备注 |
|---|---|---|---|---|---|---|---|
| I2C1_SCL | PB6 | B5 | I2C1_SCL | U16.4（外部罗盘口） | I2C bus 1（外部） | initI2CBusExternal(1) |  |
| I2C1_SDA | PB7 | A5 | I2C1_SDA | U16.3 | I2C bus 1 | 同上 |  |
| I2C2_SCL | PB10 | J7 | I2C2_SCL | U4.1 IST8310 / U6.4 DPS310 | I2C bus 2（内部） | initI2CBusInternal(2) |  |
| I2C2_SDA | PB11 | K7 | I2C2_SDA | U4.16 / U6.3 | I2C bus 2 | 同上 |  |

### CAN

| 网表网络 | 引脚 | 球 | 复用功能 | 板上连接 | PX4 端口 | PX4 配置项 | 备注 |
|---|---|---|---|---|---|---|---|
| CAN1_TX | PD1 | E8 | FDCAN1_TX | U20.1（TJA1051） | CAN1 → UAVCAN | CONFIG_BOARD_UAVCAN_INTERFACES=1 |  |
| CAN1_RX | PD0 | D8 | FDCAN1_RX | U20.4 | CAN1 | 同上 |  |
| CAN1_SILENT | PD3 | C7 | GPIO | U20.8（S 脚） | GPIO_CAN1_SILENT_S0 | board_config.h | 低电平=正常模式 |

### SD 卡

| 网表网络 | 引脚 | 球 | 复用功能 | 板上连接 | PX4 端口 | PX4 配置项 | 备注 |
|---|---|---|---|---|---|---|---|
| SDIO_CK | PC12 | C8 | SDMMC1_CK | CARD1.5 | /dev/mmcsd0 | CONFIG_STM32H7_SDMMC1=y |  |
| SDIO_CMD | PD2 | B7 | SDMMC1_CMD | CARD1.3 | /dev/mmcsd0 | SDIO_SLOTNO 0 |  |
| SDIO_D0 | PC8 | F9 | SDMMC1_D0 | CARD1.7 | /dev/mmcsd0 |  |  |
| SDIO_D1 | PC9 | E9 | SDMMC1_D1 | CARD1.8 | /dev/mmcsd0 |  |  |
| SDIO_D2 | PC10 | B9 | SDMMC1_D2 | CARD1.1 | /dev/mmcsd0 |  |  |
| SDIO_D3 | PC11 | B8 | SDMMC1_D3 | CARD1.2 | /dev/mmcsd0 |  | ⚠ 无卡检测脚，勿定义 GPIO_SDMMC1_NCD |

### USB

| 网表网络 | 引脚 | 球 | 复用功能 | 板上连接 | PX4 端口 | PX4 配置项 | 备注 |
|---|---|---|---|---|---|---|---|
| USB_OTG_FS_DM | PA11 | C10 | OTG_FS_DM | D4.3 → USB1（Type-C） | /dev/ttyACM0 | CONFIG_STM32H7_OTGFS=y |  |
| USB_OTG_FS_DP | PA12 | B10 | OTG_FS_DP | D4.1 → USB1 | /dev/ttyACM0 | 同上 | ⚠ 无 VBUS 检测脚，勿定义 GPIO_OTGFS_VBUS |

### 调试/启动/时钟

| 网表网络 | 引脚 | 球 | 复用功能 | 板上连接 | PX4 端口 | PX4 配置项 | 备注 |
|---|---|---|---|---|---|---|---|
| DEBUG_JTCK-SWCLK | PA14 | A9 | SWCLK | U12.2（调试口） | — | SWD 调试/下载 |  |
| DEBUG_JTMS-SWDIO | PA13 | A10 | SWDIO | U12.1 | — | SWD |  |
| BOOT0 | BOOT0 | D5 | BOOT0 | R1 + SW1（按键） | — | bootloader 进入 |  |
| NRST | NRST | E1 | NRST | C12.2 | — | 复位 |  |
| OSC_IN | PH0 | C1 | OSC_IN | X1.1（8 MHz 晶振） | — | STM32_BOARD_XTAL 8000000 |  |
| OSC_OUT | PH1 | D1 | OSC_OUT | X1.3 | — | 同上 |  |

### GPIO / 电源控制

| 网表网络 | 引脚 | 球 | 复用功能 | 板上连接 | PX4 端口 | PX4 配置项 | 备注 |
|---|---|---|---|---|---|---|---|
| LED0 | PE3 | B3 | GPIO | R2 → LED1（白） | GPIO_nLED_* | board_config.h | ⚠ 高电平点亮 → BOARD_LED_ON 1 |
| LED1 | PE4 | C3 | GPIO | R3 → LED2（蓝） | GPIO_nLED_* | board_config.h | ⚠ 同上 |
| LED_STRIP | PA8 | D9 | TIM1_CH1 | 未引出 | 可选 WS2812 | led_pwm_timers | PA8，本板未走线 |
| BEEPER | PA15 | A8 | GPIO | R52 → Q1 → 蜂鸣器 | GPIO_TONE_ALARM_GPIO | board_config.h | 高电平导通 |
| PINIO1 | PD10 | H9 | GPIO | 未引出 | 自定义 GPIO | systemcmds/gpio | PX4 无 PINIO 概念 |
| PINIO2 | PD11 | G9 | GPIO | 未引出 | 自定义 GPIO | — |  |
| PINIO3 | PE6 | E3 | GPIO | 未引出 | 自定义 GPIO | — |  |
| VCC_10V_EN | PE5 | D3 | GPIO | R14 → U8 EN（10V 输出） | 自定义 GPIO | — | PX4 无标准 VTX 电源参数 |

### ADC

| 网表网络 | 引脚 | 球 | 复用功能 | 板上连接 | PX4 端口 | PX4 配置项 | 备注 |
|---|---|---|---|---|---|---|---|
| ADC_VBAT1 | PC0 | F1 | ADC123_INP10 | R22(10k)/R25(1k) ← VBAT | ADC1_IN10 → BAT1 电压 | ADC_BATTERY_VOLTAGE_CHANNEL | V_DIV = 11.0 |
| ADC_CURR1 | PC1 | F2 | ADC123_INP11 | U17.6（ESC1 电流输出） | ADC1_IN11 → BAT1 电流 | ADC_BATTERY_CURRENT_CHANNEL | A_PER_V 取决于电调 |
| ADC_VBAT2 | PA4 | G3 | ADC12_INP18 | R23(20k)/R26(1k) ← VBAT2_PAD | ADC1_IN18 → BAT2 电压 | ADC_BATTERY2_VOLTAGE_CHANNEL | V_DIV = 21.0 |
| ADC_CURR2 | PA7 | K3 | ADC12_INP7 | U18.6（ESC2 电流输出） | ADC1_IN7 → BAT2 电流 | ADC_BATTERY2_CURRENT_CHANNEL | A_PER_V 取决于电调 |
| ADC_AIRS | PC4 | G4 | ADC12_INP4 | R24(10k)/R27(10k) ← AIRS_PAD | ADC1_IN4 → 空速 | ADC_AIRSPEED_IN_CHANNEL | 分压比 2.0 |
| ADC_RSSI | PC5 | H4 | ADC12_INP8 | 未引出 | ADC1_IN8 → RSSI | ADC_RSSI_IN_CHANNEL | 未走线，可留作调试 |

### 电源与地

| 信号 | 球位 | 网络 | 说明 |
|---|---|---|---|
| VDD | D2 / F5 / K1 | VCC_3V3_MCU | 数字 3.3V |
| VDD33USB | F6 | VCC_3V3_MCU | USB PHY 3.3V |
| VDDA | H1 | VCC_3V3_MCU | 模拟 3.3V（与数字同源） |
| VDDLDO | F4 | VCC_3V3_MCU | LDO 旁路（接 3.3V = 外部供电） |
| VSS | C2 / E4 / E5 / E6 / J1 | GND | 数字地 |
| VSSA | G1 | GND | 模拟地 |
| VCAP | E7 / F8 | C10 / C11 2.2µF → GND | 内核去耦 |
| PDR_ON | F7 | （网表中无网络） | ⚠ 需硬件确认，见第 9 节 |

## 3. PWM / 电机输出

| 输出通道 | 网表 | 引脚 | 球 | 定时器 | 通道 | 板上连接 |
|---|---|---|---|---|---|---|
| 1 | MOTOR1 | PB0 | J4 | TIM3 | CH3 | U17.1（S1） |
| 2 | MOTOR2 | PB1 | K4 | TIM3 | CH4 | U17.2（S2） |
| 3 | MOTOR3 | PA0 | G2 | TIM5 | CH1 | U17.3（S3） |
| 4 | MOTOR4 | PA1 | H2 | TIM5 | CH2 | U17.4（S4） |
| 5 | MOTOR5 | PA2 | J2 | TIM5 | CH3 | U18.1（S5） |
| 6 | MOTOR6 | PA3 | K2 | TIM5 | CH4 | U18.2（S6） |
| 7 | MOTOR7 | PD12 | K10 | TIM4 | CH1 | U18.3（S7） |
| 8 | MOTOR8 | PD13 | J10 | TIM4 | CH2 | U18.4（S8） |

`src/timer_config.cpp` 直接按这个顺序写（数组下标即通道号 1~8）：

```cpp
constexpr io_timers_t io_timers[MAX_IO_TIMERS] = {
	initIOTimer(Timer::Timer3, DMA{DMA::Index1, DMA::Stream2, DMA::Channel5}),
	initIOTimer(Timer::Timer5, DMA{DMA::Index1, DMA::Stream0, DMA::Channel6}),
	initIOTimer(Timer::Timer4, DMA{DMA::Index1, DMA::Stream6, DMA::Channel2}),
};

constexpr timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
	initIOTimerChannel(io_timers, {Timer::Timer3, Timer::Channel3}, {GPIO::PortB, GPIO::Pin0}),
	initIOTimerChannel(io_timers, {Timer::Timer3, Timer::Channel4}, {GPIO::PortB, GPIO::Pin1}),
	initIOTimerChannel(io_timers, {Timer::Timer5, Timer::Channel1}, {GPIO::PortA, GPIO::Pin0}),
	initIOTimerChannel(io_timers, {Timer::Timer5, Timer::Channel2}, {GPIO::PortA, GPIO::Pin1}),
	initIOTimerChannel(io_timers, {Timer::Timer5, Timer::Channel3}, {GPIO::PortA, GPIO::Pin2}),
	initIOTimerChannel(io_timers, {Timer::Timer5, Timer::Channel4}, {GPIO::PortA, GPIO::Pin3}),
	initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel1}, {GPIO::PortD, GPIO::Pin12}),
	initIOTimerChannel(io_timers, {Timer::Timer4, Timer::Channel2}, {GPIO::PortD, GPIO::Pin13}),
};
```

- `DIRECT_PWM_OUTPUT_CHANNELS 8`、`BOARD_NUM_IO_TIMERS 3`；**不要**带 TIM15 组（本板 PE5/PE6 是 GPIO）。
- 两个四合一电调口：U17 = 通道 1~4（S1~S4），U18 = 通道 5~8（S5~S8），各带一路遥测 RX + 电流采样。
- 无 IO 协处理器 → 参数是 `PWM_MAIN_*`（`generate_params.py` 在 `board_with_io=false` 时的分支）。通道功能用 `PWM_MAIN_FUNC1..8`；输出协议/频率按定时器组设：`PWM_MAIN_TIM0`（TIM3）、`PWM_MAIN_TIM1`（TIM5）、`PWM_MAIN_TIM2`（TIM4），PWM / OneShot / DShot / BDShot 都在这一组参数里选（已核对 `parameters.json` 的实际参数名）。

## 4. 串口：UART → /dev/ttySx → PX4 角色

| 外设 | 引脚 | NuttX 设备 | 建议 PX4 角色 | 板上连接 |
|---|---|---|---|---|
| USART1 | PA9 / PA10 | ttyS0 | console（调试） | U13.2 仅 TX → VTX 控制脚 |
| USART2 | PD5 / PD6 | ttyS1 | TEL / 自定义 | U11.6 仅 RX |
| USART3 | PD8 / PD9 | ttyS2 | 空闲 | 未引出 |
| UART4 | PB9 / PB8 | ttyS3 | GPS1（推荐） | U11.3 / U11.4 |
| USART6 | PC6 / PC7 | ttyS4 | RC 输入（SBUS/CRSF/DSM） | U15.1 / U15.2 |
| UART7 | PE8 / PE7 | ttyS5 | DShot 遥测 / TEL | U18.5 仅 RX |
| UART8 | PE1 / PE0 | ttyS6 | DShot 遥测 / TEL | U17.5 仅 RX |

编号规则（`arch/arm/src/stm32h7/stm32_serial.c`）：**console UART 固定占用 `/dev/ttyS0` 并从后续编号里剔除**，其余按 UART 序号 1→8 递增，从 `ttyS1` 开始。上表是「USART1 作 console、7 个 UART 全使能」的结果（与 MatekH743 相同）。**换 console 会让整列编号平移**，第 9 节第 3 条给了改法。

```c
// default.px4board
CONFIG_BOARD_SERIAL_GPS1="/dev/ttyS3"   // UART4 → U11 六芯口
CONFIG_BOARD_SERIAL_TEL1="/dev/ttyS5"   // UART7 RX → ESC2 遥测
CONFIG_BOARD_SERIAL_TEL2="/dev/ttyS6"   // UART8 RX → ESC1 遥测
// src/board_config.h
#define RC_SERIAL_PORT          "/dev/ttyS4"   // USART6 → U15 接收机口
#define BOARD_SUPPORTS_RC_SERIAL_PORT_OUTPUT
#define HRT_TIMER               2               // HRT 用 TIM2（与 Matek 相同）
#define HRT_TIMER_CHANNEL       1
```

## 5. SPI 总线与片选

| 总线 | SCK / MISO / MOSI | PX4 | 器件 | CS / DRDY |
|---|---|---|---|---|
| SPI1 | PA5 / PA6 / PD7 | bus 1 | 片内 ICM-42688-P #1 | CS=PC15，DRDY=PB2 |
| SPI2 | PB13 / PB14 / PB15 | bus 2 | AT7456E OSD | CS=PB12 |
| SPI3 | PB3 / PB4 / PB5 | bus 3（外部） | 外扩 SPI（U22 六芯口） | CS1=PD4，CS2=PE2（无焊盘） |
| SPI4 | PE12 / PE13 / PE14 | bus 4 | 片内 ICM-42688-P #2 | CS=PE11，DRDY=PE15 |

```cpp
constexpr px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {
	initSPIBus(SPI::Bus::SPI1, {
		initSPIDevice(DRV_IMU_DEVTYPE_ICM42688P, SPI::CS{GPIO::PortC, GPIO::Pin15},
		              SPI::DRDY{GPIO::PortB, GPIO::Pin2}),
	}),
	initSPIBus(SPI::Bus::SPI2, {
		initSPIDevice(DRV_OSD_DEVTYPE_ATXXXX, SPI::CS{GPIO::PortB, GPIO::Pin12}),
	}),
	initSPIBusExternal(SPI::Bus::SPI3, {
		initSPIConfigExternal(SPI::CS{GPIO::PortD, GPIO::Pin4}),
		initSPIConfigExternal(SPI::CS{GPIO::PortE, GPIO::Pin2}),
	}),
	initSPIBus(SPI::Bus::SPI4, {
		initSPIDevice(DRV_IMU_DEVTYPE_ICM42688P, SPI::CS{GPIO::PortE, GPIO::Pin11},
		              SPI::DRDY{GPIO::PortE, GPIO::Pin15}),
	}),
};
```

## 6. I2C

| 总线 | 引脚 | PX4 | 器件 | 备注 |
|---|---|---|---|---|
| I2C1 | PB6(SCL) / PB7(SDA) | `initI2CBusExternal(1)` | U16 外部罗盘口 | 上拉 R31/R32 4.7k |
| I2C2 | PB10(SCL) / PB11(SDA) | `initI2CBusInternal(2)` | IST8310(U4)、DPS310(U6) | 上拉 R9/R10 4.7k |

驱动用 `-X`（外部）/ `-I`（内部）区分总线，与 Matek 一致：

```sh
dps310 -I start -a 118     # 板载 DPS310（I2C2，地址 0x76）
ist8310 -I start -a 14     # 板载 IST8310（I2C2，地址 0x0E = 14）
```

## 7. ADC 通道

| ADC 通道 | 引脚 | 球 | 用途 | PX4 宏 | 分压 / 来源 |
|---|---|---|---|---|---|
| ADC1_IN10 | PC0 | F1 | 电池 1 电压 | ADC_BATTERY_VOLTAGE_CHANNEL | R22/R25 = 11:1 |
| ADC1_IN11 | PC1 | F2 | 电池 1 电流 | ADC_BATTERY_CURRENT_CHANNEL | U17.6 |
| ADC1_IN18 | PA4 | G3 | 电池 2 电压 | ADC_BATTERY2_VOLTAGE_CHANNEL | R23/R26 = 21:1 |
| ADC1_IN7 | PA7 | K3 | 电池 2 电流 | ADC_BATTERY2_CURRENT_CHANNEL | U18.6 |
| ADC1_IN4 | PC4 | G4 | 空速 | ADC_AIRSPEED_IN_CHANNEL | AIRS_PAD，2:1 |
| ADC1_IN8 | PC5 | H4 | RSSI | ADC_RSSI_IN_CHANNEL | 未走线 |

```c
#define PX4_ADC_GPIO  \
	/* PC0 */  GPIO_ADC123_INP10, \
	/* PC1 */  GPIO_ADC123_INP11, \
	/* PA4 */  GPIO_ADC12_INP18, \
	/* PA7 */  GPIO_ADC12_INP7, \
	/* PC4 */  GPIO_ADC12_INP4, \
	/* PC5 */  GPIO_ADC12_INP8

#define ADC_BATTERY_VOLTAGE_CHANNEL   ADC1_CH(10)   // PC0
#define ADC_BATTERY_CURRENT_CHANNEL   ADC1_CH(11)   // PC1
#define ADC_BATTERY2_VOLTAGE_CHANNEL  ADC1_CH(18)   // PA4
#define ADC_BATTERY2_CURRENT_CHANNEL  ADC1_CH(7)    // PA7
#define ADC_AIRSPEED_IN_CHANNEL       ADC1_CH(4)    // PC4
#define ADC_RSSI_IN_CHANNEL           ADC1_CH(8)    // PC5

#define BOARD_BATTERY1_V_DIV   (11.0f)   // R22 10k / R25 1k
#define BOARD_BATTERY2_V_DIV   (21.0f)   // R23 20k / R26 1k
```

这段与 MatekH743 的 ADC 段**完全一致**（球位、通道号逐条相同），可以直接照抄；`BOARD_BATTERY1_A_PER_V` / `BATTERY2_A_PER_V` 要按所用四合一电调的电流输出比例填（Matek 参考值是 40.0 A/V），两块电调若型号相同可填同一值，实测校准更准。

## 8. 与 MatekH743 的差异（必须改的 9 处）

| # | 项目 | MatekH743 | MoonPilot | 处理 |
|---|---|---|---|---|
| 1 | PWM 数量 | 12 路（TIM3/5/4/15） | 8 路（TIM3/5/4） | DIRECT_PWM_OUTPUT_CHANNELS=8、BOARD_NUM_IO_TIMERS=3，删掉 TIM15 组 |
| 2 | PD14 / PD15 | PWM9/10（TIM4_CH3/4） | 接 ICM-42688 pin9 CLKIN（U2.9 / U3.9） | 不能作 PWM；CLKIN 默认不启用（见 9.6），按 GPIO/输入处理 |
| 3 | SPI4 器件 | ICM42605(PC13) + ICM20602(PE11) | 仅 ICM-42688（PE11）+ DRDY PE15 | spi.cpp 只留一个 ICM42688P 设备；PC13 本板悬空 |
| 4 | VBUS 检测 | PE2 = GPIO_OTGFS_VBUS | 无 VBUS 检测脚；PE2 是 SPI3_CS2 | 删掉 GPIO_OTGFS_VBUS 与 bootloader 的 BOARD_VBUS |
| 5 | SD 卡检测 | 无 NCD | 无 NCD | 一致，勿定义 GPIO_SDMMC1_NCD |
| 6 | LED 极性 | 低电平点亮（阳极→5V） | 高电平点亮（阴极→GND） | BOARD_LED_ON=1，GPIO 初值 GPIO_OUTPUT_CLEAR |
| 7 | RC 输入 | ttyS4；无 PPM | ttyS4；PC7 挂着 RX_PPM 网络 | PPM 需 TIM3 捕获，与 MOTOR1/2 抢定时器 → 用串口 RC |
| 8 | UART1 | console，PA9 只做日志 | console，但 PA9 接 U13.2（VTX 控制） | 建议 console 移到 UART3（未引出） |
| 9 | UART/SPI/CAN/ADC 引脚 | — | 与 Matek 逐球一致 | board.h 的 GPIO_UARTx_*、GPIO_CAN1_*、ADC 通道可整段照抄 |

## 9. 待硬件确认 / 风险

1. **PDR_ON（F7）在网表中没有任何网络**。它决定 STM32H743 内部 POR 是否工作，通常必须接 VDD（或由外部复位芯片驱动）。若真悬空，可能出现上电不启动或随机不启动，**建议优先确认**。（E2、F3 是 NC，无影响。）
2. **PPM 输入不可用**：PC7 的网表名是 `RX_PPM&&UART6_RX`，但 PX4 的 PPM 捕获跑在 HRT 定时器上（`HRT_PPM_CHANNEL`，STM32 侧实现见 `stm32_common/hrt/hrt.c`），而本板 HRT 必须是 TIM2。把 PPM 放在 TIM3_CH2 就要 `HRT_TIMER=3`，直接顶掉 MOTOR1/2 的 PWM。→ 用 USART6 的 SBUS/CRSF 串口 RC。
3. **UART1 = console 与 VTX 控制脚冲突**：USART1_TX 只引到 U13.2。作 console 会把 boot 日志灌进 VTX。两个选择：**(a)** 把 console 改到 UART3（PD8/PD9 未引出，最干净，此时编号变为 ttyS0=UART3、ttyS1=UART1、ttyS2=UART2、ttyS3=UART4、ttyS4=UART6、ttyS5=UART7、ttyS6=UART8）；**(b)** 保留 UART1 作 console，硬件上断开 U13.2。
4. **USB 无 VBUS 检测**：VBUS 经 D2 直接并到 5V 轨，没有检测分压进 MCU。板级不要定义 `GPIO_OTGFS_VBUS`（那是 Matek 的 PE2，本板 PE2 = SPI3_CS2），bootloader 的 `hw_config.h` 里 `BOARD_VBUS` 也要去掉，否则 USB 会把 SPI3 片选抢走。
5. **SD 无卡检测**：`CARD1.9/10` 直接接地，没有 NCD，`HAVE_NCD` 不能定义（与 Matek 一致）。
6. **GYRO1/2 CLKIN 接在 PD14/PD15**：这里要说清楚——ICM42688P 驱动**有** CLKIN 支持（`ConfigureCLKIN()` 会把芯片 pin9 设成 CLKIN 输入并开 RTC_MODE），但只在 `I2CSPIDriverConfig.custom1 != 0`（外部时钟频率）时才启用；而 STM32 的板级 `initSPIDevice()` 只有 `(devid, CS, DRDY)` 三个参数、传不进 custom1，整个 PX4 树里也没有任何板子这么用。再加上 CLKIN 的时钟还得 MCU 侧自己发（PD14/PD15 = TIM4_CH3/CH4，可配 32.768 kHz 输出）。→ **默认按普通 SPI IMU 用，pin9 悬空即可（芯片内部时钟）**；要用 CLKIN 属于要改驱动接口的定制工作，且这两脚就彻底不能做 PWM 了。
7. **SPI3_CS2(PE2) 只有上拉、没有对外焊盘**；U22 六芯口上的 CS 是 `SPI3_CS1(PD4)`。
8. **LED 高电平点亮**（阴极接地），与 Matek「阳极→5V、低电平点亮」相反：`BOARD_LED_ON 1` / `BOARD_LED_OFF 0`，GPIO 初值用 `GPIO_OUTPUT_CLEAR`。白灯挂在 PE3（LED0 网络），蓝灯挂在 PE4（LED1 网络）。
9. **8 路 PWM 分属 3 个定时器**：PX4 要求同组同频，所以 TIM3 组（通道 1、2）、TIM5 组（通道 3~6）、TIM4 组（通道 7、8）可以各自设不同频率（`PWM_MAIN_TIM0/1/2`）；DShot 需要 DMA，用第 3 节的三条映射即可。

## 10. 落地成 PX4 板级目标时的文件清单

把 `boards/matek/h743` 复制成 `boards/<vendor>/moonpilot-h743/`，改动点：

| 文件 | 改动 |
|---|---|
| `default.px4board` | `CONFIG_BOARD_SERIAL_*` 按第 4 节；ADC / PWM_OUT / DSHOT / ICM42688P / DPS310 / UAVCAN 等保留 |
| `bootloader.px4board`、`firmware.prototype` | `BOARD_TYPE`、板名、Flash 2 MB（H743VI） |
| `nuttx-config/nsh/defconfig` | UART / SPI / I2C / ADC / SDMMC / OTGFS 使能集，与 Matek 相同即可 |
| `nuttx-config/include/board.h` | UART / SPI / CAN 的 AF 选择（**可整段照抄 Matek**）、`STM32_BOARD_XTAL 8000000` |
| `nuttx-config/include/board_dma_map.h` | SPI1 / SPI4 的 DMA 映射（可照抄） |
| `src/timer_config.cpp` | 第 3 节的三组 8 通道版本（比 Matek 少 TIM15 组） |
| `src/spi.cpp` | 第 5 节（SPI4 只留 ICM42688P） |
| `src/i2c.cpp` | 与 Matek 相同（1 外 2 内） |
| `src/board_config.h` | ADC 段照抄；删 `GPIO_OTGFS_VBUS`；LED 极性反转；8 路 PWM；CAN silent=PD3；tone=PA15 |
| `src/hw_config.h` | 删 `BOARD_VBUS`；改 `BOARD_TYPE`、`APP_LOAD_ADDRESS`、Flash 大小 |
| `src/init.c` / `led.c` / `sdio.c` / `usb.c` | 基本照抄（`init.c` 里 PWM 通道数改 8） |
| `init/rc.board_sensors` | `icm42688p -R 12 -b 1 -s`（SPI1）、`icm42688p -R 12 -b 4 -s`（SPI4）、`dps310 -I start -a 118`、`ist8310 -I start -a 14`、`atxxxx start -s` |
| `init/rc.board_extras` | `atxxxx start -s`；`dshot telemetry -d /dev/ttyS5`（ESC2）/ `-d /dev/ttyS6`（ESC1） |

> 本表只做引脚映射与板级配置设计，**没有编译任何固件**（按你的要求）。要落地成 target 时再动代码。
