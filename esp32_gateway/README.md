# ESP32-S 局域网监控与控制网关

本工程适用于经典 ESP32-S、ESP32-WROOM-32 和 ESP32 DevKit 类开发板，基于 ESP-IDF。ESP32 每秒接收 STM32F103 的温湿度、当前手动室温和控制状态，在局域网网页中显示实时曲线、历史和故障；网页或 HTTP API 也可设置温湿度目标、保温前馈室温、启动、停止和清除故障。

固件默认开启演示热点：SSID 为 `Temperature-Gateway`，密码为 `temperature`，网页地址为 [http://192.168.4.1](http://192.168.4.1)。即使配置了局域网 Wi-Fi，演示热点仍保持开启；局域网连接只作为额外访问路径。这样刷写后无需先配置家庭 Wi-Fi 即可现场演示。

STM32 始终负责最终安全控制。ESP32 只提交期望状态，不直接驱动 PTC、电机或风扇，也不能绕过风扇、传感器和过温联锁。

## 接线

| 方向 | STM32F103 | ESP32 | 用途 |
|---|---|---|---|
| STM32 → ESP32 | PB13 | GPIO18 | SPI2 SCK，由 STM32 输出 |
| STM32 → ESP32 | PB14 | GPIO27 | 软件控制的 CS，低电平表示一个事务 |
| 双向、分时 | PB15 | GPIO23 | 半双工 SDIO：状态上行、命令下行 |
| — | GND | GND | 共地 |

只需要表中四根连接线，不再需要独立 MISO 或 UART 线。PB14 在本协议中作为普通 GPIO 片选输出，PA9、PA10 和 ESP32 GPIO17 均不连接。两端均为 3.3 V 逻辑电平。ESP32 建议通过自己的 USB/5V 输入供电，不要从余量不明的 STM32 3.3 V 电源取电。

PB15 与 GPIO23 建议串联 330 Ω～1 kΩ 电阻，用于限制复位或方向切换异常时的争用电流。现有直连线可以先使用，但首次验证应保持任务停止并断开 PTC、电机等功率负载，确认状态与停机设置可以双向传输后再做运行测试。

SPI 参数为 mode 1、MSB first、约 281.25 kHz；CS 空闲为高，每个事务期间保持低。STM32 启动时先让 CS 保持高、PB15 保持高阻约 250 ms，再用最多两个只读时隙自动同步。之后每秒先进行一次固定 64 字节的 STM32→ESP32 状态事务；STM32 随后释放 PB15，约 20 ms 后进行一次固定 64 字节的 ESP32→STM32 命令事务，再留约 1 ms 换向时间。没有命令时 ESP32 返回 `N*48FA\r\n`，其余字节补零；STM32 只有校验到这个 NOP 或有效命令后才允许重新驱动 PB15。

经典 ESP32 的 ESP-IDF 没有公开支持三线 SPI 从机 SIO 模式。本工程使用 GPIO matrix workaround：GPIO23 同时路由到从机 MOSI 输入和 MISO 输出，接收期间将输出从引脚矩阵断开，只在 64 字节下行事务期间接入并打开输出。这个配置依赖上述固定时序，只用于本工程。两种事务均限制为经典 ESP32 硬件 FIFO 可处理的 64 字节，并关闭 SPI 从机 DMA。

## 状态上行协议

STM32 每秒发送一帧固定 64 字节的二进制状态。多字节整数均为小端：

| 字节 | 内容 |
|---:|---|
| 0–1 | 魔数 `TG` |
| 2 | 协议版本 `3`；ESP32 同时兼容旧版 `2` |
| 3 | 有效字段区长度 `40` |
| 4–7 | STM32 毫秒计数，`uint32` |
| 8–11 | 融合温度，`int32`，单位 0.01°C |
| 12 | 手动室温编码值 `ambient_temperature_c + 10`，范围 0～60，对应 −10～50°C |
| 13–15 | 保留，必须为零 |
| 16–19 | 湿度，`uint32`，单位 0.001%RH |
| 20 | 传感器有效标志，`0` 或 `1` |
| 21 | `state_bits` |
| 22–23 | `output_permille`，`uint16` |
| 24–27 | 温度目标、湿度目标、故障、`ack_result` |
| 28–31 | `ack_session`，`uint32` |
| 32–35 | `ack_sequence`，`uint32` |
| 36–39 | 命令接收错误计数，`uint32` |
| 40–61 | 保留，必须为零 |
| 62–63 | 前 62 字节的 CRC16-CCITT-FALSE，小端 |

传感器读取失败时 `valid=0`，运行状态、目标值、故障和命令确认仍会发送；ESP32 将无效的温度和湿度呈现为不可用。

`state_bits` 定义如下：

| 位 | 含义 |
|---:|---|
| 0 | 任务正在运行 |
| 1 | PTC 输出已开启 |
| 2 | 电机已开启 |
| 3 | 温度目标已启用 |
| 4 | 湿度目标已启用 |
| 5 | 风扇状态为运行 |
| 6 | 风扇状态已经建立并有效 |
| 7 | 当前任务由 ESP32 远程控制 |

`output_permille` 为加热 PWM 占空比，积分分离 PID 控制取 `0`～`1000`，加热载波频率为 10 Hz。`fault` 的值为 `0=无故障`、`1=风扇`、`2=传感器`、`3=过温`。`ack_result` 的常用值为：

| 值 | 名称 | 含义 |
|---:|---|---|
| 0 | `none` | 尚无确认 |
| 1 | `applied` | 命令已应用 |
| 2 | `bad_range` | 目标超出范围 |
| 3 | `stale` | 命令序号或时间戳过期 |
| 4 | `fault_active` | 存在锁存故障 |
| 5 | `fan` | 风扇条件不满足 |
| 6 | `sensor` | 传感器条件不满足 |
| 7 | `overtemperature` | 过温 |
| 8 | `clear_rejected` | 故障原因仍存在，不能清除 |
| 9 | `lease_expired` | 远程运行租约已到期并停机 |
| 10 | `storage_failed` | 参数已应用，但持久化写入失败 |
| 11 | `persist_requires_stop` | 请求同时要求运行和写 Flash；须先停机保存 |

状态上行事务固定为 64 字节。ESP32 严格检查魔数、版本、长度、字段范围、保留区和 CRC；任一检查失败时丢弃整帧。固定 64 字节使接收可直接使用经典 ESP32 的 SPI 硬件 FIFO，避开较长半双工事务的 DMA 限制。

## 控制下行与失联保护

ESP32 在 GPIO23 的 64 字节下行时隙中发送带 CRC16-CCITT-FALSE 的一行命令：

```text
C,session,sequence,seen_tick,run,temp_en,temp,hum_en,hum,ambient,clear,persist*CRC16\r\n
```

温度目标必须为 30～70°C，湿度目标必须为 10～80%RH，室温必须为 −10～50°C；即使对应目标关闭，请求中也要提供范围内的数值。`session` 和 `sequence` 用于去重，相同命令重发不会重复写 Flash 或重复切换状态。命令不足 64 字节时使用 NUL 填满；没有命令时发送带 CRC16 的 NOP。ESP32 在等待确认时于每个一秒下行时隙重发。旧版 11 字段命令没有 `ambient` 字段，STM32 按 22°C 兼容处理。

远程启动需要最近 2.5 秒内收到扩展状态帧。STM32 只接受带有近期 `seen_tick` 的启动命令，并再次检查故障、传感器、风扇和过温条件。启动成功后，ESP32 只在收到新的 STM32 状态后才每秒重发同一命令作为心跳；STM32 的远程运行租约为 5 秒，任一方向的链路失效后不再续租，超时立即关闭 PTC 和电机并停止任务。STM32 重启时 ESP32 会取消旧的远程启动，必须由上位机重新发起。CRC16、会话号、序号、状态新鲜度和全部本地安全联锁都保留，四线传输没有降低这些检查。

网页或 API 请求会在下一个下行时隙送达 STM32，命令确认再随下一次上行状态返回；从提交到界面显示最终确认通常约 1～2 秒。这种时延适合秒级自动化调参，不适合作为毫秒级闭环控制总线。

停止和设置请求不会因为上行状态暂时失联而被 ESP32 阻塞。故障清除仍需先排除实际原因；清除故障本身不会绕过后续启动检查。

## 网页

网页不依赖 CDN 或互联网，包含：

- 融合温度、湿度和两条实时曲线；
- 任务、PTC、电机、风扇、实际输出、目标值和远程归属状态；
- 启动、停止、应用温湿度目标和清除故障；
- 最近命令的等待、应用或拒绝结果；
- 数据失效、超过 2.5 秒未更新和连接中断提示；
- ESP32 内存历史、扩展状态 CSV 导出、帧计数和解析错误统计；
- 局域网和常开演示热点设置。

历史保存在 RAM，ESP32 重启后会清空。自动化 PID 调整期间应取消“掉电保存”，即使用 `persist=false`；只在确定最终参数需要断电保存时使用 `persist=true`，避免高频试验造成不必要的 W25Q Flash 擦写。同步写 Flash 期间安全循环无法维持实时响应，因此固件只允许在 `running=false` 时使用 `persist=true`；需要保存并启动时应先提交停止/保存命令，收到确认后再单独启动。

## HTTP 接口

| 地址 | 方法 | 说明 |
|---|---|---|
| `/` | GET | 实时监控与控制仪表板 |
| `/settings` | GET | 同一仪表板的网络设置入口 |
| `/ws` | GET | WebSocket 实时样本流 |
| `/api/status` | GET | 当前测量、实际控制状态、统计和网络状态 |
| `/api/control` | GET | 最近请求、链路、确认结果和 STM32 实际状态 |
| `/api/control` | POST | 提交完整的期望控制状态 |
| `/api/pid` | GET | 最近一次 PID 参数请求及 STM32 确认状态 |
| `/api/pid` | POST | 直接提交 Kp、Ki、Kd 参数，运行时立即生效 |
| `/api/history?limit=300` | GET | 最近历史 JSON，最多返回 900 条 |
| `/api/history.csv` | GET | 下载 ESP32 当前保留的全部历史和控制状态 |
| `/api/settings` | GET/POST | 读取非敏感网络配置状态，或保存配置并重启 |

`POST /api/control` 必须使用 `Content-Type: application/json`。下面的请求会保持停机，同时应用目标值，适合先验证控制链路：

```sh
curl -X POST "http://<ESP32-IP>/api/control" \
  -H "Content-Type: application/json" \
  -d '{
    "running": false,
    "temperature_enabled": true,
    "target_temperature_c": 55,
    "humidity_enabled": true,
    "target_humidity_pct": 20,
    "ambient_temperature_c": 22,
    "clear_fault": false,
    "persist": false
  }'
```

前六个字段必须存在；`clear_fault` 和 `persist` 可省略，省略时为 `false`。服务器拒绝未知字段、重复字段、错误类型、越界整数，以及 `running=true` 与 `persist=true` 的组合。请求被接收时返回 HTTP 202 和 `session`、`sequence`，随后通过 `GET /api/control` 读取 `pending`、`applied`、`result_name` 与 `actual`。只有确认接线和所有安全条件后，才把 `running` 改为 `true`。

网页首页的“PID 参数”区域也可直接输入并提交参数。请求格式如下：

```sh
curl -X POST "http://<ESP32-IP>/api/pid" \
  -H "Content-Type: application/json" \
  -d '{"kp":160,"ki":2,"kd":180}'
```

参数范围和步进分别为 Kp `0–500`、`0.1`，Ki `0–50`、`0.01`，Kd `0–1000`、`0.1`。参数通过受 CRC 保护的 SPI 命令发送到 STM32，网页会显示待确认或已应用状态；当前参数为运行时设置，STM32 重启后恢复固件默认值。

该 HTTP 服务不做用户身份认证，应只接入受信任的本地网络。

## 编译、烧录与网络配置

使用仓库中的 PlatformIO 配置：

```sh
cd esp32_gateway
pio run
pio run --target upload
pio device monitor
```

也可以使用 ESP-IDF 5.1 或更新版本：

```sh
cd esp32_gateway
. "$IDF_PATH/export.sh"
idf.py set-target esp32
idf.py menuconfig
idf.py build
idf.py -p /dev/cu.usbserial-XXXX flash monitor
```

在 `menuconfig → Temperature gateway configuration` 中设置可选的 2.4 GHz 网络、常开演示热点和内存历史点数。刷写默认配置后，直接连接 `Temperature-Gateway`，访问 `http://192.168.4.1` 即可打开网页；也可在网页“网络设置”中保存局域网配置，ESP32 会把设置保存到 NVS 并重启。本文不记录现场网络凭据。

## 不依赖 ESP-IDF 的协议测试

电脑上有 C 编译器时可直接运行：

```sh
sh tests/run_host_tests.sh
```

测试覆盖 64 字节二进制状态帧、字段与数值边界、保留区、CRC16、64 字节命令槽以及控制命令编码。

## 单一温度与协议迁移

融合温度 = 0.7 × AHT20 + 0.3 × BMP280，按 0.01°C 四舍五入（负数半值向远离零方向舍入）。PID、LCD、网页共用该值；原始读数仅用于内部传感器校验及独立超温保护。任一路采样失败则融合无效并沿用故障停热策略。

HTTP/WebSocket/历史 JSON 输出 `temperature_c` 和 `ambient_temperature_c`，失效的传感器温湿度为 `null`；CSV 同名列失效为空。旧的 `aht_c` / `bmp_c` 字段已移除。STM32 当前发送 v3 状态帧，ESP32 同时接受 v2 和 v3；v2 帧的室温按 22°C 处理。升级现场设备时建议配套更新两端固件：旧 STM32 的 v2 状态也能被新 ESP32 读取，但只有更新 STM32 后网页才能显示实际保存的室温值。已有历史文件不迁移；分析脚本识别旧格式并保留原 max 反馈含义。PID 参数及采样周期未改变，原 PID 实测指标不代表融合后的性能，需要另行验证。

## 融合温度验证

在仓库根目录运行：

```sh
python3 tests/test_temperature_fusion.py
python3 tests/test_fusion_serialization.py
node tests/test_fusion_web.js
sh esp32_gateway/tests/run_host_tests.sh
```

修改网页后，在编译固件前运行 `python3 esp32_gateway/tools/embed_web.py`，同步生成嵌入固件的页面。网页测试会检查嵌入内容与 HTML 完全一致。
