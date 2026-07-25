# STM32 3D 打印耗材烘干控制器

这是一个基于 STM32F103C8T6 的 3D 打印耗材烘干控制器固件。系统读取箱内温湿度，监测风扇运行状态，通过 XY-MOS 控制 12V PTC 加热器，并使用旋转编码器和 SSD1306 OLED 完成参数设置与状态显示。

## 功能

- AHT20 读取温度与湿度，BMP280 提供第二路温度保护。
- SSD1306 实时显示温湿度、风扇状态、电机状态、运行时间和加热状态。
- SSD1306 初始化为180度旋转显示，以适配当前反装的屏幕。
- 依次设置温度阈值和湿度阈值，两项均可单独设为 `OFF`。
- 温度、湿度均为 `OFF` 时进入只吹风模式，PTC 始终关闭。
- PB8 通过 TIM4_CH3 输出高电平有效的 10Hz 控制信号，驱动 XY-MOS。
- PA0 通过 TIM2_CH1 输出 20kHz PWM，PA2 保持低电平，驱动 DRV8870 正转；电机只在实际加热烘干时运行。
- 风扇停止、传感器失联或过温时立即关闭 PTC。
- W25Q Flash 保存温湿度目标及阈值启用状态。
- 传感器和 OLED 支持运行期间重新探测。

## 主要硬件

| 硬件 | 用途 |
|---|---|
| STM32F103C8T6 | 主控制器 |
| 12V 100W PTC | 加热器 |
| XY-MOS | PTC 功率开关，高电平有效 |
| Nidec D04R-12TH 26B | 12V 循环风扇及锁转检测 |
| AHT20 + BMP280 | 温湿度检测及冗余温度保护 |
| SSD1306 128x64 | I2C 状态显示 |
| 旋转编码器 | 参数设置和任务控制 |
| W25Qxx | 设置掉电保存 |
| DRV8870DDAR + JGB37-520 | 耗材料盘旋转驱动，20kHz 开环 PWM 调速 |

完整引脚分配、接线要求、操作流程和安全控制逻辑见 [CONTROL_SYSTEM.md](CONTROL_SYSTEM.md)。

100W 版本的电源预算、PCB 器件选型、电机驱动、外置传感器接口及安全链设计见 [docs/HARDWARE_DESIGN_BOM.md](docs/HARDWARE_DESIGN_BOM.md)。

## 控制规则

- 温度阈值启用时，温度达到目标后关闭 PTC，下降 1°C 后允许重新加热。
- 温度阈值关闭时，仍保留 70°C 安全控温上限和 75°C 锁存过温保护。
- 湿度阈值启用时，达到目标湿度后暂停加热，高于目标 2%RH 后允许恢复。
- 风扇信号在任务启动后有 6 秒建立时间；停止状态持续 500ms 会终止任务并关闭 PTC。
- 风扇恢复运行 200ms 后自动清除 `FAN ERR`，但任务不会自动重新启动。
- 传感器失联时每 2 秒尝试恢复 I2C2，故障期间禁止加热。
- 电机与PTC加热状态同步：`HEATING` 时正转，等待、保持、只吹风、停止或故障状态下停转。

## 编码器操作

1. 空闲主页面短按，进入温度设置。
2. 旋转选择 `OFF` 或 30~70°C，短按进入湿度设置。
3. 旋转选择 `OFF` 或 10~80%RH，短按进入启动确认。
4. 旋转选择 `START` 或 `SAVE ONLY`，短按确认。
5. 运行中长按 1 秒停止任务；设置页面长按返回上一步。

## 构建

需要 CMake 3.30 或更高版本，以及 Arm GNU Toolchain，其中 `arm-none-eabi-gcc`、`arm-none-eabi-g++` 和 `arm-none-eabi-objcopy` 应位于 `PATH` 中。

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

构建完成后会生成：

- `build/demo_led.elf`
- `build/demo_led.hex`
- `build/demo_led.bin`

## 烧录

使用 ST-Link 和仓库内的 OpenOCD 配置：

```bash
openocd -f stm32f103c8_blue_pill.cfg \
  -c "program build/demo_led.elf verify reset exit"
```

## 目录

```text
Core/                         应用代码、设备驱动和启动文件
Drivers/                      STM32 HAL 与 CMSIS
demo_led.ioc                  STM32CubeMX 配置
STM32F103C8TX_FLASH.ld        链接脚本
CMakeLists.txt                CMake 交叉编译配置
stm32f103c8_blue_pill.cfg     OpenOCD/ST-Link 配置
CONTROL_SYSTEM.md             接线、控制和操作说明
```

## 安全提示

本项目控制 12V / 100W 大电流 PTC。PTC、风扇和电机电流不得经过开发板，功率回路应使用合适线径、保险丝和星形接地。首次调试时先断开 PTC 和电机，确认 PB8、PA0、PA2 在空闲和故障状态为低电平，再分别连接功率负载。
