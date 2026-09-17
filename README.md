# STM32 3D 打印耗材烘干控制器

这是一个基于 STM32F103C8T6 的 3D 打印耗材烘干控制器固件。系统读取箱内温湿度，监测风扇运行状态，通过 XY-MOS 控制 12V PTC 加热器，并使用旋转编码器和 SSD1306 OLED 完成参数设置与状态显示。

## 功能

- AHT20 与 BMP280 按 70% / 30% 融合为一个温度，用于 PID、LCD 和网关；湿度来自 AHT20，两路原始温度保留独立超温保护。
- SSD1306 实时显示温湿度、风扇状态、电机状态、运行时间和加热状态。
- SSD1306 初始化为180度旋转显示，以适配当前反装的屏幕。
- 依次设置温度阈值和湿度阈值，两项均可单独设为 `OFF`。
- 旋钮菜单 `ROOM TEMP CAL` 手动输入当前室温（−10～50°C，1°C 步进），按 22°C 测量基准补偿保温前馈。
- 温度、湿度均为 `OFF` 时进入只吹风模式，PTC 始终关闭。
- PB8 通过 TIM4_CH3 输出高电平有效的 10Hz 控制信号，驱动 XY-MOS。
- 板载 PC13 加热指示灯按实际加热占空比闪烁：占空比越高闪烁越快，稳态低占空比时闪烁间隔自动变长，占空比为 0 时熄灭。
- PA0 通过 TIM2_CH1 输出 20kHz PWM，PA2 保持低电平，驱动 DRV8870 正转；电机只在实际加热烘干时运行。
- 风扇停止、传感器失联或过温时立即关闭 PTC。
- W25Q Flash 保存温湿度目标、阈值启用状态及手动室温；旧记录默认室温 22°C。
- 传感器和 OLED 支持运行期间重新探测。
- 每秒通过 PB13/PB14/PB15 的 SPI2 半双工接口发送 64 字节 CRC 状态帧，并在同一周期接收 64 字节控制命令。
- ESP32-S 接入局域网后提供实时曲线、历史数据、CSV 下载以及启停和温湿度目标设置，电脑或手机无需安装客户端。
- PB15 与 ESP32 GPIO23 分时双向传输，四根线即可完成监控与控制；PA9、PA10 和 ESP32 GPIO17 均不使用。
- 远程启动采用 5 秒租约。ESP32 在远程任务运行时每秒续租，链路或 ESP32 失联后 STM32 自动停止该任务。

ESP32-S 接线、固件构建和网页使用步骤见 [ESP32 局域网监控与控制网关](esp32_gateway/README.md)。

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
| ESP32-S | 局域网监控、曲线、历史记录和远程控制网关 |

完整引脚分配、接线要求、操作流程和安全控制逻辑见 [CONTROL_SYSTEM.md](CONTROL_SYSTEM.md)。

100W 版本的电源预算、PCB 器件选型、电机驱动、外置传感器接口及安全链设计见 [docs/HARDWARE_DESIGN_BOM.md](docs/HARDWARE_DESIGN_BOM.md)。

温度融合、前馈和分离 PID 的设计依据与验证边界见 [温控算法报告](docs/TEMPERATURE_CONTROL_ALGORITHM_REPORT.md)。实机升温和温控试验报告保存在 `outputs/` 下；仓库只保留可读报告和必要图表，原始采集文件与编译产物默认留在本地。

## 控制规则

- 温度阈值启用时，温度达到目标后关闭 PTC，下降 1°C 后允许重新加热。
- 温度阈值关闭时，仍保留 70°C 安全控温上限和 75°C 锁存过温保护。
- 湿度阈值启用时，达到目标湿度后暂停加热，高于目标 2%RH 后允许恢复。
- 风扇信号在任务启动后有 6 秒建立时间；停止状态持续 500ms 会终止任务并关闭 PTC。
- 风扇恢复运行 200ms 后自动清除 `FAN ERR`，但任务不会自动重新启动。
- 传感器失联时每 2 秒尝试恢复 I2C2，故障期间禁止加热。
- 电机与PTC加热状态同步：`HEATING` 时正转，等待、保持、只吹风、停止或故障状态下停转。
- 网页或 HTTP API 发出的远程启动不会绕过上述风扇、传感器和过温联锁；条件不满足时 STM32 拒绝启动并返回原因。
- 远程控制的任务只有在持续收到 ESP32 心跳时保持运行；5 秒租约到期后 PTC 和电机关闭，任务停止。

## 上位机监控与控制

ESP32 网页每秒更新温湿度曲线，同时显示任务、加热器、电机、风扇、输出、目标值、故障和最近一次命令结果。网页可执行启动、停止、温湿度目标设置和故障清除。HTTP 客户端可通过 `GET /api/status` 和 `GET/POST /api/control` 接入后续自动化 PID 调整测试。

控制请求可以选择是否把目标写入 W25Q Flash。自动化调参时建议保持 `persist=false`，只在需要断电保存最终参数时写入，避免高频试验造成不必要的 Flash 擦写。完整接线、请求格式和返回状态见 [ESP32 局域网监控与控制网关](esp32_gateway/README.md)。

## 编码器操作

1. 空闲主页面短按，进入温度设置。
2. 旋转选择 `OFF` 或 30~70°C，短按进入湿度设置。
3. 旋转选择 `OFF` 或 10~80%RH，短按进入 `ROOM TEMP CAL` 室温校准。
4. 旋转选择当前箱外室温（−10～50°C，默认 22°C），短按进入启动确认。
5. 旋转选择 `START` 或 `SAVE ONLY`，短按应用并保存全部设置。
6. 运行中长按 1 秒停止任务；设置页面长按返回上一步，温度页面长按取消本次编辑。

室温是手动输入值，不会自动跟随环境变化，也不改变箱内传感器示值。22°C 时保留原有前馈曲线；其他室温按 `(目标温度−当前室温)/(目标温度−22)` 缩放原前馈，限幅 0～85%。这是温差比例估算，PID 继续修正误差；满功率升温速率表保持原值。未连接可用 W25Q 时设置仅在本次通电期间生效。

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

## 验证

根目录测试不需要连接硬件，覆盖温度融合、协议序列化、网页渲染和远程控制安全规则：

```bash
python3 tests/test_temperature_fusion.py
python3 tests/test_fusion_serialization.py
node tests/test_fusion_web.js
sh esp32_gateway/tests/run_host_tests.sh
g++ -std=c++17 -ICore/Inc tests/separation_pid_test.cpp -o /tmp/separation_pid_test
/tmp/separation_pid_test
```

修改 `esp32_gateway/main/index.html` 后，先运行 `python3 esp32_gateway/tools/embed_web.py`，再构建网关固件。STM32 固件可用上面的 CMake 命令交叉编译；构建会同时生成 ELF、HEX 和 BIN 文件。

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
esp32_gateway/                ESP32-S SPI 半双工监控、控制、Wi-Fi 与网页网关
demo_led.ioc                  STM32CubeMX 配置
STM32F103C8TX_FLASH.ld        链接脚本
CMakeLists.txt                CMake 交叉编译配置
stm32f103c8_blue_pill.cfg     OpenOCD/ST-Link 配置
CONTROL_SYSTEM.md             接线、控制和操作说明
docs/                         引脚、电源、器件和温控算法文档
tests/                        主机侧协议、融合和控制测试
tools/                        实验数据分析工具
```

## 安全提示

本项目控制 12V / 100W 大电流 PTC。PTC、风扇和电机电流不得经过开发板，功率回路应使用合适线径、保险丝和星形接地。首次调试时先断开 PTC 和电机，确认 PB8、PA0、PA2 在空闲和故障状态为低电平，再分别连接功率负载。远程控制仅进入同一套安全状态机，不替代独立保险丝、温控器或其他硬件断热措施。
