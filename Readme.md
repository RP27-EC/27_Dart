# RP27_Dart：2027 镖架电控（C 板）

- 版本：固件版本宏 0.1.0 / 参数表 v1；文档对应 V0.6，详见 [CHANGELOG](CHANGELOG.md)。
- 来源：复制自 C 板模板 `code\RP27_Cboard`，模板本身没有改动。功能逻辑参考 2026 基线 `code\Dart_2026-main`。
- 状态：**已静态回读，纯编译通过，host_check 通过（只能说明逻辑符合预期）。尚未烧录，没有任何实机验证。** 所有标"待标定"的参数，都必须人工确认后才能上机。

## 文档索引

| 文档 | 内容 | 读者 |
|---|---|---|
| [上机调试流程](docs/上机调试流程.md) | **镖架造好后从首次上电到稳定发射的分步流程（从这里开始）** | 电控 |
| [遥控器手册](docs/遥控器手册.md) | 遥控映射（唯一来源）、指示灯与蜂鸣器、操作步骤 | 操作手 |
| [状态机说明](docs/状态机说明.md) | 模式、储能规则、单发与卸力状态表、释放许可、中止恢复、安全行为 | 电控 |
| [标定与测量](docs/标定与测量.md) | 待标定参数总表、零点与回转位、行程测量步骤和记录表 | 电控、机械 |
| [PID 调试指南](docs/PID调试指南.md) | 控制结构、调参接口 `g_dart_tune`、调参步骤 | 电控 |
| [调试与调参](docs/调试与调参.md) | Ozone / MemRW3 配置、现成 JSON 配置、枚举对照 | 电控 |
| [host_check/README.md](host_check/README.md) | 上位机纯逻辑检查的运行方法 | 电控 |
| [CHANGELOG](CHANGELOG.md) | 修改记录，以及标定、调参记录 | 所有人 |
| `docs/archive/` | 历史评估清单（已补处理状态，只作追溯） | — |

## Quick Start

需要：Keil MDK 5、ARMCC 5.06 update 7（Target `My_C`，芯片 STM32F407IGHx）。只有重新生成代码时才需要 CubeMX 6.14 / FW_F4 1.28.1。

```powershell
& 'G:\Keil 5\Core\UV4\UV4.exe' -b 'G:\STM32_ALL\RM\code\RP27_Dart\MDK-ARM\My_C.uvprojx' -t My_C -j0 -o build.log
```

UV4 有警告时也会返回退出码 1，判断是否成功要看日志末尾的 `0 Error(s)`。

> [!warning]
> `.ioc` 和 `Core/Src/tim.c`、`usart.c` 是手工同步修改的（TIM1 增开 CH2～CH4，TIM4_CH3 改到 PD14，USART1/6 改为 115200）。第一次用 CubeMX 重新生成后，要逐项比对，确认一致。

## 1. C 板资源分配

| 资源 | 引脚 / 参数 | 用途 | 状态 |
|---|---|---|---|
| CAN1 1Mbps | PD0 RX / PD1 TX | 0x201 上膛 L 3508、0x202 上膛 R 3508、0x203 变力 3508；控制帧 0x200 | 已实现 |
| CAN2 1Mbps | PB5 RX / PB6 TX | 0x204 Yaw 2006（电调 ID 保持 4） | 已实现。是否改回 CAN1（同 26）待定 |
| TIM1 50Hz，1µs/计数 | PE9 / PE11 / PE13 / PE14 | CH1 撒放器；CH2 剪叉 A；CH3 剪叉 B；CH4 储弹限位 | 已实现，脉宽待标定 |
| TIM8_CH1 | PC6 | 备用 PWM | 未配置 |
| TIM4_CH3 | **PD14**，2 kHz（PSC 83 / ARR 499） | 板载蜂鸣器提示音 | 已实现。模板原配 PB8（可能是 26 的外接），已改为 PD14，ioc 和 `tim.c` 同步修改；用户确认 PD14 为板载蜂鸣器（2026-11-04） |
| USART3 | PC11 RX，100000，偶校验 | DBUS 遥控 | 已实现 |
| USART1 | PA9 / PB7，115200 | 2.4G（测试时）或串口屏 | 二者编译期互斥：`DART_LINK24G_ENABLE=1` 时给 2.4G，串口屏桩不接收 |
| USART6 | PG14 / PG9，115200 | 裁判系统 | 桩 |
| USB OTG FS | PA11 / PA12 CDC | 小电脑 / 视觉 | 桩 |
| SPI2 | PB13 / 14 / 15，CS PB12 | TF 卡 | 未配置。⚠ 26 工程中 PB12 是微动开关 |
| GPIO | PF1 / PF0、PI6 / PI7 | 两路 ADS1232 测力 | 未配置 |
| 2.4G | DL20 点对点，USART1（板上丝印 UART2，待对照手册确认） | 测试时同步测试基地（工程 `code\RP27_DartBase`） | 已实现，未实机联调。只在测试时使用，比赛拆除；`DART_TEST_MODE=0` 时编译期禁止启用。协议见 `link24g_frame.h`，调试方法见下方 “2.4G 测试基地” |
| IWDG | — | 看门狗 | `DART_IWDG_ENABLE=0` |

## 2. 代码结构

```
Application/
  ConfigLayer/   dart_config.h（全部开关、映射和待标定参数）、config_uart.c（串口回调路由）
  DriverLayer/   drv_pwm（TIM1 四路），以及模板自带的 drv_can / drv_uart / drv_tick
  HardwareLayer/ RM_motor（模板）
  DeviceLayer/   dart_motor（电机表、CAN 发送）、servo（舵机，按时间推定到位）、buzzer（板载蜂鸣器提示音）、loadcell* / tf_log*、rc_sensor
  ProtocolLayer/ can_protocol（反馈分发）、judge* / vision* / screen*、link24g_protocol（2.4G 主机）+ link24g_frame.h（与基地共用的帧格式）
  ModuleLayer/   mech_axis（通用位置轴）、loader / tension / yaw / trigger / feeder
  ControlLayer/  rc_cmd（遥控快照）、fault（故障）、fire_gate（释放许可）、dart_fsm（模式和单发流程）、dart_tune（PID 调参接口，仅测试模式）、dart_app（初始化和周期入口）
  TaskLayer/     四个常驻任务，只是外壳，实际调用 dart_app
（带 * 的是桩：接口已定，功能未实现）
host_check/      上位机纯逻辑检查，不进 Keil 工程
docs/            文档，见上方索引
```

- 依赖只能从上往下：Task → Control → Module → Device/Protocol → Driver/HAL。
- 机构请求只由 `dart_fsm` 发出，电机和舵机输出只在 ControlTask 中写入。
- 中断里只做反馈解析。

**初始化顺序**（main.c USER CODE 2）：`MX_USB_DEVICE_Init` → `Dart_Motor_Init`（必须在 CAN 中断开启之前）→ `DRIVER_Init`（串口 DMA、PWM、CAN 过滤器）→ `DEVICE_Init`（遥控、舵机初始脉宽、桩）→ `Dart_App_Init`

**任务**（全部常驻，不动态创建）

| 任务 | 优先级（CubeMX） | 周期 | 内容 |
|---|---|---|---|
| ControlTask | Realtime | 1 ms | 遥控快照 → 状态机 → 三轴、撒放器、供弹 → CAN1/CAN2 0x200 → PWM |
| MonitorTask | Realtime | 1 ms | 遥控和电机心跳、在线类故障、喂狗（宏控制） |
| CommunityTask | High | 10 ms | 2.4G 收发与重发；裁判、视觉、串口屏、测力、日志轮询（这些仍是桩） |
| LedTask | AboveNormal | 50 ms | 指示灯 |

## 3. 待确认与后续工作

以下状态按用户 2026-11-04 的答复更新。

- [ ] 机械 / 实机：
  - 卸力接住：用户判断可以接住，但打开撒放器的一瞬间电流会明显增大，需要实机记录电流峰值（见 [状态机说明](docs/状态机说明.md) 第 4 节）。
  - 变力最小预紧位置：需要实机测量。
  - 新平 Yaw 的堵转方向：用户后续实机试验。
  - 供弹动作已明确：限位舵机放行，镖靠重力落到剪叉上，两个剪叉舵机再把镖放下。舵机尚未采购，脉宽和保持时间待到货后标定。
- [ ] 硬件：
  - Yaw 放 CAN2（当前代码）还是 CAN1（同 26），用户后续决定。
  - 舵机型号和供电（尚未采购）；PB12 的用途。
  - ~~2.4G 占用哪一路串口~~：已定 USART1，与串口屏分时复用（2026-11-05）。接线：DL20 TXD→PB7、RXD→PA9。
  - ~~PD14 是否为板载蜂鸣器~~：用户已确认。
- [ ] 裁判：2027 官方协议已发布，暂时用不到。接入时实现 `judge_protocol`，并把 `DART_TEST_MODE` 置 0（`DART_TEST_MODE=0` 且裁判未启用时，编译直接报错）。USART1/6 回调已传入实际收到的长度，USART6 已补上 DMA 计数重装；拆包、粘包由协议层处理。
- [ ] 测力：ADS1232 驱动、标定，以及 `Tension_Force_Trim` 力闭环（暂缓）
- [ ] TF：环形缓冲、写卡任务、FatFs；把累计打击次数持久化，目前只记在 RAM（暂缓）
- [ ] 四发：AUTO 模式（裁判驱动）和 30 s 四连发的时序（暂缓）
- [ ] **每发参数示教与 Flash 保存：与串口屏一起实现**（用户要求，2026-11-03）。已确定的约束：
  - 在手动模式下点动到满意位置后，保存为当前发的预设；遥控上用哪个操作待定。
  - 存内部 Flash 扇区 11（`drv_flash`，0x080E0000）。参数结构带版本号和 CRC，读到无效数据时回退到默认值。
  - 擦除时 CPU 会停住 1～2 s，**只能在安全位、电机输出为零时擦写**，失败最多重试 3 次。
  - 修改从下一发开始生效，不改动已经进入待发的这一发。
  - 屏幕和遥控使用同一套参数接口。
- [ ] 用 CubeMX 重新生成并比对；如需调整任务优先级，改 ioc

## 2.4G 测试基地（DART_LINK24G_ENABLE）

镖架通过 DL20 无线串口控制测试基地（基地工程见 `code\RP27_DartBase\Readme.md`）。状态：**已编译，host_check 通过，未实机联调**。

- **出镖同步**：`Fsm_Try_Fire` 发出释放命令时调用 `Link24g_Notify_Shot()`，只入队，不阻塞控制周期。计时起点是“释放命令时刻”，不是裁判的出镖检测时刻，两者的差值需要实测记录。
- **靶型和命令**：调试时用 Ozone 或 MemRW 改写 `g_link24g_ctrl`，写入 `req` 后自动下发，执行完 `req` 清零：

| `req` | 作用 | 用到的字段 |
|---|---|---|
| 1 SET_MODE | 设置靶型：0 固定 / 1 随机固定 / 2 随机移动 / 3 末端移动 | `mode`、`fixed_mm`（仅固定靶） |
| 2 ZERO | 基地堵转找零，完成后回到 0 mm | — |
| 3 GOTO | 移到指定位置（低速，不计时） | `goto_mm`（±280） |
| 4 RELOCATE | 随机固定靶换位（模拟开闸或命中后） | — |
| 5 STOP | 停止，并取消还没开始的换位 | — |
| 6 SYNC_TEST | 不发射，模拟一次出镖同步 | — |

- `g_link24g_ctrl.sync_enable=0`：发射时不触发基地。
- **观察**：`g_link24g_base.online`（500 ms 内收到过状态），`st.state/pos_mm/target_mm/last_move_ms/start_lag_ms/flags`。`flags` bit1 = 上次换位在 600 ms 内静止，bit2 = 规划时间已超出时限，bit3 = 同步到达时已晚于开始时刻。
- **可靠性**：命令带会话号和序号，未回执时每 60 ms 重发，最多 15 次，基地按序号去重；STOP 会清空队列。回执只表示基地收到了命令，不表示已经到位。

## Git 仓库

- 工程根目录就是仓库根目录：`code/RP27_Dart`，默认分支 `main`。
- 远程 `origin`：https://github.com/RP27-EC/27_Dart.git
- `.gitignore` 排除 Keil 编译输出、本机 IDE 配置、调试会话、上位机检查程序和临时日志；保留 `.uvprojx`、`.ioc`、启动汇编、源码和 CMSIS 依赖库。
- 原先只在 `MDK-ARM` 下的 Git 元数据，已移到 `G:\STM32_ALL\RM\code\RP27_Dart_MDK_Git备份_20261103`，保留了旧提交记录。

在工程根目录查看待提交内容：

```powershell
git status --short
git remote -v
```
