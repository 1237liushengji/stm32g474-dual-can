# 7.CAN 双机CAN通信工程（STM32G474VET6 × 2 + SN65HVD230 × 2）

基于 `1.LED` 例程框架搭建的双板CAN总线通信工程。两块 STM32G474VET6
开发板通过 CAN 收发器接入同一条 CAN 总线，以经典CAN
（500 kbit/s）进行命令/应答式双向通信，板载 LED 作为通信状态指示。

驱动调用模式与位时序结构参照 **ST官方 STM32CubeG4 示例**
[FDCAN_Classic_Frame_Networking](https://github.com/STMicroelectronics/STM32CubeG4/tree/master/Projects/STM32G474E-EVAL/Examples/FDCAN/FDCAN_Classic_Frame_Networking)
（本项目 HAL 库版本 V1.2.3，与该示例 API 完全一致）；SN65HVD230 模块接线和
终端电阻配置参考了 [nopnop2002/Arduino-STM32-CAN](https://github.com/nopnop2002/Arduino-STM32-CAN)
的实测电路。

---

## 1. 硬件组成与接线

### 1.1 硬件清单

| 数量 | 器件 | 说明 |
|:---:|------|------|
| 2 | STM32G474VET6 开发板 | 丝印/引脚图：CAN_RX->PB12、CAN_TX->PB13；两板烧录不同固件 |
| 2 | SN65HVD230 CAN 收发器模块 | 3.3V 供电，与 MCU 电平直连（板载CAN收发电路的板子可不接） |
| 4 | 杜邦线（信号） | PB12/PB13 ↔ 收发器 |
| 2 | 双绞线/杜邦线（总线） | CANH↔CANH、CANL↔CANL |
| 2 | ST-Link 调试器（或1个分两次烧录） | SWD 烧录 |

### 1.2 接线方案

**方案A：使用两块外接 SN65HVD230 模块**

MCU 引脚与收发器**同名直连（不交叉）**——SN65HVD230 的 TXD 是"发送数据输入"，
接 MCU 的发送脚；RXD 是"接收数据输出"，接 MCU 的接收脚：

| 板A / 板B（两板接法相同） | SN65HVD230 模块 | 说明 |
|------|------|------|
| **PB13**（FDCAN2_TX，AF9） | CTX / TXD | MCU发送 → 收发器发送输入 |
| **PB12**（FDCAN2_RX，AF9） | CRX / RXD | 收发器接收输出 → MCU接收 |
| 3.3V | VCC / 3V3 | 模块供电 |
| GND | GND | **两板、两模块必须共地** |
| — | CANH | 板A模块CANH ↔ 板B模块CANH |
| — | CANL | 板A模块CANL ↔ 板B模块CANL |

**方案B：使用板载CAN收发电路**（板卡资料注明"使用跳线帽连接对应的引脚即可"）

按丝印用跳线帽连接 PB12↔CAN_RX、PB13↔CAN_TX，随后将两板的
CANH↔CANH、CANL↔CANL、GND↔GND 对接即可，无需外接模块。

```
   板A (STM32G474)                板B (STM32G474)
  ┌───────────┐                  ┌───────────┐
  │ PB13 (TX) ├─→TXD ┌──────┐ CANH┌──────┐ TXD←─┤ PB13 (TX) │
  │ PB12 (RX) │←─RXD │SN65  ├────┤SN65  │ RXD──→│ PB12 (RX) │
  │ 3V3      ├─→VCC │HVD230│ CANL│HVD230│ VCC←──┤ 3V3       │
  │ GND      ├──→GND│ (A)  ├────┤ (B)  │ GND←──┤ GND       │
  └───────────┘      └──┬───┘120Ω└──┬───┘      └───────────┘
                      (模块板载终端电阻，典型120Ω)
```

### 1.3 终端电阻（关键！）

* 常见 SN65HVD230 模块在 CANH–CANL 间**焊有 120Ω 板载电阻**（部分模块带 J1
  跳线选择）。两个节点各提供一个 120Ω，恰好构成总线两端终端，**无需外加电阻**。
* **上电前用万用表测量两模块 CANH–CANL 之间电阻：正常约 60Ω**。
* 若实测约 120Ω：说明只有一个终端（某模块无板载电阻），需在缺失端补一个 120Ω；
  若接近 0Ω：CANH/CANL 短路；若无穷大：断线。

### 1.4 引脚选择依据

PB12/PB13 为 STM32G474 **FDCAN2** 的 AF9 复用引脚（数据手册 Table 13：
PB12=FDCAN2_RX、PB13=FDCAN2_TX），与本板丝印/引脚图标注一致
（CAN_RX->PB12、CAN_TX->PB13）。经核对板卡引脚图，无外设冲突：

| 外设 | 占用引脚 | 备注 |
|------|------|------|
| LED（蓝/绿） | **PE0 / PE1** | 本工程用LED1=PE0（蓝）作状态指示 |
| 按键 | PA0 / PA2 / PA3 | WKUP/K0/K1 |
| W25Q128（SPI1） | PA4~PA7 | CS/SCK/MISO/MOSI |
| 485 / WiFi | PB10 / PB11 | 与CAN互不影响 |
| 24C02（I2C1） | PA15 / PB9 | |
| LCD（SPI） | PD11 / PB3 / PB5 / PD12 / PD13 | |
| **FDCAN2（本工程）** | **PB12 / PB13** | 丝印CAN_RX/CAN_TX |

---

## 2. 通信协议

总线：经典CAN，500 kbit/s，标准数据帧，ID 精确匹配过滤。

### 2.1 帧定义

**命令帧（板A → 板B）** ID = `0x321`，DLC = 8：

| 字节 | 含义 |
|------|------|
| data[0] | 序号 seq，每帧自增1（回绕），用于核对连续性 |
| data[1..2] | 板A开机秒数（小端16位） |
| data[3..7] | 保留 0x00 |

**应答帧（板B → 板A）** ID = `0x322`，DLC = 8：

| 字节 | 含义 |
|------|------|
| data[0] | 回显触发本次应答的命令帧 seq |
| data[1] | 板B当前LED状态（0灭 / 1亮） |
| data[2..3] | 板B累计收到命令帧总数（小端16位） |
| data[4..7] | 保留 0x00 |

### 2.2 运行行为

| 状态 | 板A（主节点） | 板B（从节点） |
|------|------|------|
| 正常 | 每1000ms发1帧命令；收到应答后LED1与板B同步翻转 | 收到命令即翻转LED1并立即应答 |
| 两板LED | **同频同相反射闪烁（周期2s）** ← 正常通信的直观判据 | 同左 |
| 链路断开 | 超过1500ms无应答 → LED1以100ms快闪报警 | 超过1500ms无命令 → LED1快闪 |
| 恢复 | 链路恢复后自动回到同步闪烁（bus-off自动恢复：Stop→Start） | 同左 |

**LED指示语义总表（排错时对照）**：

| LED | 状态 | 含义 |
|------|------|------|
| LED1（蓝） | 周期2s同步闪烁 | 通信正常（两板节奏一致） |
| LED1（蓝） | 100ms快闪 | 链路超时：本板收不到对端报文（曾收到过或发送过） |
| LED1（蓝） | 常灭 | 板B等待状态：从未收到命令帧 |
| LED2（绿） | 250ms闪烁（错误事件后5秒内） | 本板检测到总线错误（错误被动/bus-off）：发送的帧无人应答，通常是**对端收发器不在总线上**或接线/终端问题 |
| LED2（绿） | 常灭 | 本板未检测到总线错误 |

---

## 3. 时钟与位时序（全部参数可核算）

系统时钟（沿用LED例程，未改动）：

```
HSE 8MHz → PLL (M=2, N=75, R=2) → SYSCLK = HCLK = PCLK1 = PCLK2 = 150MHz
```

FDCAN内核时钟 = **PCLK1 = 150MHz**（在 `HAL_FDCAN_MspInit` 中选择，
与ST官方示例做法一致）。

位时序（500 kbit/s，采样点80%）：

| 参数 | 值 | 计算 |
|------|------|------|
| NominalPrescaler | 30 | tq = 150MHz / 30 = **200ns** |
| 同步段 | 1 tq | 固定 |
| NominalTimeSeg1 | 7 tq | 传播段+相位段1 |
| NominalTimeSeg2 | 2 tq | |
| NominalSyncJumpWidth | 2 | |
| **位时间** | **10 tq = 2µs** | **波特率 = 500 kbit/s** |
| 采样点 | (1+7)/10 = **80%** | 与ST官方示例相同的时序结构 |

---

## 4. 软件架构

```
7.CAN/G474/
├── Core/
│   ├── Inc/stm32g4xx_hal_conf.h   [改] 启用 HAL_FDCAN_MODULE_ENABLED
│   └── Src/
│       ├── main.c                 [改] 应用层：节点角色任务/链路监视/LED指示
│       ├── stm32g4xx_it.c         [改] 新增 FDCAN2_IT0_IRQHandler
│       └── stm32g4xx_hal_msp.c    [改] 新增 HAL_FDCAN_MspInit/MspDeInit
│                                    （内核时钟选择PCLK1、PB12/PB13 AF9、NVIC）
├── Drivers/User/
│   ├── Inc/can.h                  [新] 配置宏、协议定义、驱动API、载荷编解码
│   ├── Inc/bsp_uart.h             [新] 控制台串口驱动接口（USART1 PA9/PA10）
│   ├── Inc/console.h              [新] 命令行控制台接口
│   ├── Src/can.c                  [新] FDCAN2驱动：初始化/发送/接收信箱/
│   │                                 统计/bus-off自动恢复/运行时波特率切换/混杂模式
│   ├── Src/bsp_uart.c             [新] 串口驱动：单字节中断接收+128字节环形缓冲
│   └── Src/console.c              [新] 命令行：help/version/stats/bitrate/sniff/send
├── MDK-ARM/G474.uvprojx           [改] 编译清单加入 can.c、bsp_uart.c、console.c、
│                                       stm32g4xx_hal_fdcan.c、stm32g4xx_hal_uart.c
└── ../tools/can_console.py        [新] PC上位机（迷你CAN分析仪，Python+pyserial）
```

驱动设计要点：

1. **发送**：Tx FIFO（硬件深度3），发送前查询 `HAL_FDCAN_GetTxFifoFreeLevel`，
   FIFO满时返回 `HAL_BUSY` 并计数，不阻塞主循环；硬件自动重发（AutoRetransmission）。
2. **接收**：RX FIFO0 + FDCAN2中断线0；回调中循环排空FIFO写入单槽信箱，
   主循环 `CAN_PollRx()` 关中断短临界区取走，无帧丢失竞争。
3. **过滤**：标准ID掩码0x7FF精确匹配对端ID，非匹配帧全部硬件拒收。
4. **错误处理**：bus-off/错误被动中断使能，bus-off后自动 Stop→Start 恢复并
   重挂通知（M_CAN 硬件在bus-off时自动置INIT位）；全部HAL返回值检查并计数。
5. **统计**：`CAN_Stats`（发送/接收/发送失败/接收覆盖/bus-off/协议错误计数），
   供调试器实时观察。

---

## 5. 编译与烧录（两板固件不同！）

1. Keil MDK 打开 `7.CAN/G474/MDK-ARM/G474.uvprojx`。
2. **编译板A固件**：确认 `Drivers/User/Inc/can.h` 中
   `#define CAN_NODE_ROLE  CAN_NODE_A` → F7 编译 → ST-Link烧录到**板A**。
3. **编译板B固件**：改为 `#define CAN_NODE_ROLE  CAN_NODE_B` → 重新编译
   （会自动重编 can.c/main.c）→ 烧录到**板B**。
4. 两板断电，按第1节接线，先测 CANH–CANL ≈ 60Ω，再上电。

> 注意：两板固件仅节点角色不同（过滤ID、任务逻辑），改一处宏后完整重编译，
> 不要只烧录不重编。

---

## 6. 验证方法

### 6.1 现象验证

| 步骤 | 预期现象 |
|------|------|
| 两板上电 | 1s内两板LED1开始**同步**翻转（周期2s，一亮一灭节奏一致） |
| 拔掉CANH（或CANL）连线 | 两板LED在约1.5s后转为100ms快闪（链路故障指示） |
| 重新接回总线 | 两板自动恢复同步翻转（链路监视+bus-off恢复生效） |
| 只给板A上电 | 板A LED快闪（2.5s内无应答） |

**单板自测试（强烈建议在排错时首先执行）**：

1. `can.h`中置 `CAN_DEBUG_SELFTEST = 1`，确认 `CAN_NODE_ROLE = CAN_NODE_A`，编译烧录到**任意一块板**；
2. 该板FDCAN进入外部回环模式：自己发、自己收，单板即可走完"命令→应答→LED同步"全流程；
3. **单板LED1若能2s周期同步闪烁 → 固件100%正常，问题必然在硬件侧**（收发器/接线/终端电阻/共地）；
4. 排查完硬件后，**务必把 `CAN_DEBUG_SELFTEST` 改回 0** 并重烧两板。

**双板现象→故障定位矩阵**：

| 现象组合（正常模式） | 结论 |
|------|------|
| A：LED1快闪＋LED2闪烁；B：LED1灭＋LED2灭 | A在发但无人应答 → 查对端收发器是否在总线上、CANH/CANL通路、终端电阻、共地 |
| A：LED1快闪＋LED2常灭；B：同左 | A的帧根本没上总线 → 查A端收发器供电、跳线帽、PB13(TX)→TXD接线 |
| A：LED1同步闪；B：LED1灭 | 诡异状态（A不应在无应答时同步）→ 用自测试复验固件 |

### 6.2 调试器验证（Keil Debug → Watch窗口）

| 变量 | 位置 | 正常值 |
|------|------|------|
| `s_stats.txCount` | can.c | 板A每秒+1（板B同频） |
| `s_stats.rxCount` | can.c | 与对端 txCount 一致（拔线期间不变） |
| `s_stats.txErrors` | can.c | 0（持续为0） |
| `s_stats.rxOverruns` | can.c | 0 |
| `s_stats.busOffCount` | can.c | 0（接线正确时不会进入bus-off） |
| `s_stats.errPassiveCount` | can.c | 0（持续增长=本板发帧无应答/总线错误） |
| `s_seq` | main.c（板A） | 0~255循环递增 |
| `s_rxCmdCount` | main.c（板B） | 每秒+1 |
| `hfdcan2.Instance->RXF0S` | — | F0FL（FIFO0填充水平）0~1 |

---

## 7. 故障排查

| 现象 | 排查顺序 |
|------|------|
| 两板LED都快闪，永远不同步 | ①先做单板自测试确认固件正常；②测CANH–CANL电阻是否≈60Ω；③检查两板是否**共地**；④PB12/PB13是否接反（TX/RX**同名直连**不交叉）；⑤模块供电3.3V；⑥若板载收发器，确认跳线帽已接通PB12/PB13 |
| 板载收发器和外接模块同时接 | **只允许一条通路**：用外接模块时拔掉全部跳线帽；用板载收发器时不接外接模块，两个收发器挂在同一引脚会互相打架 |
| 用板载收发器、两板不通 | 板载收发电路很多不带120Ω终端电阻：在两板CANH–CANL间各外并一个120Ω再试（或直接改用带板载电阻的SN65HVD230模块） |
| 一板LED同步、另一板快闪 | 快闪板收不到对端：查该板PB12（RX）→收发器RXD通路及对端CANH/CANL |
| 曾正常、后进入快闪且不自恢复 | 总线短路或持续干扰：查 `s_stats.busOffCount` 是否增长，检查双绞/走线 |
| 烧录后无任何LED动作 | 固件未含正确角色宏：确认两板分别用对应 `CAN_NODE_ROLE` 编译烧录 |
| 快闪频率异常（≈10Hz） | 这就是故障指示（100ms周期），按第1节逐项查硬件 |

---

## 8. 串口控制台与PC上位机（v2.0新增）

### 8.1 连接

板卡USB转串口接PC（板内固件默认按 **USART1 PA9=TX / PA10=RX** 配置，
如果你的板卡USB转串口接的不是这两个脚，改 `Drivers/User/Inc/bsp_uart.h`
顶部的引脚宏后重编译）。串口参数：**115200 8N1**。任意串口助手或下面的
Python上位机均可。

### 8.2 节点命令

| 命令 | 功能 | 示例 |
|------|------|------|
| `help` | 命令列表 | `help` |
| `version` | 固件版本/节点角色/编译时间/当前波特率 | `version` |
| `stats` | 通信统计（tx/rx/错误计数）与运行时间 | `stats` |
| `bitrate <kbps>` | **运行时切换**总线波特率 125/250/500/1000 | `bitrate 250` |
| `sniff <on\|off>` | 总线监视模式：接收所有ID并按行打印 | `sniff on` |
| `send <id> [b..]` | 手动发送一帧（十六进制） | `send 123 AA 55` |

sniff输出格式（可供上位机解析）：`RX <毫秒> <ID> <DLC> <数据...>`
例如 `RX 12345 321 8 01 02 03 04 05 06 07 08`

**注意**：`bitrate`切换的是位时序（内核时钟150MHz下统一10tq/位、采样点80%），
**两块板必须切换到同一速率**，否则立即失联（现象：双方LED转快闪）。

### 8.3 PC上位机（迷你CAN分析仪）

```bash
pip install pyserial
cd tools
python can_console.py --list      # 列出串口
python can_console.py -p COM5     # 连接（COM号换成实际值）
```

功能：透传命令、彩色解析sniff报文行、按ID统计帧数、自动CSV记录
（`can_log_*.csv`，含主机毫秒时间戳+节点毫秒时间戳，可直接用Excel分析）、
快捷发帧 `/send 321 11 22`。

### 8.4 演示建议（面试场景）

1. `sniff on` 后观察命令帧0x321/应答帧0x322交替出现，`stats`核对计数；
2. `bitrate 1000`两板同步切换——LED同步节奏不变（通信未断）；
3. `/send 456 DE AD BE EF`手动注入一帧，对端sniff立刻显示；
4. 拔线演示bus-off自动恢复，`stats`里busOff/errPas计数增长可讲错误状态机。

## 9. 参考资料

* ST官方示例（本工程驱动模式来源）：
  [STM32CubeG4 FDCAN_Classic_Frame_Networking](https://github.com/STMicroelectronics/STM32CubeG4/tree/master/Projects/STM32G474E-EVAL/Examples/FDCAN/FDCAN_Classic_Frame_Networking)
* SN65HVD230 接线实测参考：
  [nopnop2002/Arduino-STM32-CAN](https://github.com/nopnop2002/Arduino-STM32-CAN)（含G474开发板+SN65HVD230接线图与终端电阻说明）
* SN65HVD230 数据手册：TI官网 SLVS407
* STM32G4 参考手册 RM0440（FDCAN章）、数据手册（引脚AF映射表 Table 13：PB12=FDCAN2_RX、PB13=FDCAN2_TX，AF9）
