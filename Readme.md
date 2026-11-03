# RP27_Dart：2027 镖架电控（C 板）

- 版本：固件 0.1.0 / 参数表 v1，日期 2026-11-02
- 来源：从 `code\RP27_Cboard` 模板复制而来，模板本身没有改动。功能逻辑参考 `镖架电控\02_深大开源\Dart_2026-main`
- 状态：**已静态回读，已做一次纯编译（0 错误）。尚未烧录，也没有任何实机验证。** 所有标"待标定"的参数都必须人工确认后才能上机

## Quick Start

需要：Keil MDK 5，编译器 ARMCC 5.06 update 7（Target `My_C`，芯片 STM32F407IGHx），CubeMX 6.14 / FW_F4 1.28.1 仅在重新生成代码时需要。

```powershell
& 'G:\Keil 5\Core\UV4\UV4.exe' -b 'G:\STM32_ALL\RM\code\RP27_Dart\MDK-ARM\My_C.uvprojx' -t My_C -j0 -o build.log
```

UV4 有警告时也会返回退出码 1。判断是否成功要看日志末尾的 `0 Error(s)`。

> 警告：`.ioc` 和 `Core/Src/tim.c`、`usart.c` 是手工同步修改的（TIM1 增开 CH2～CH4，USART1/6 改为 115200）。第一次用 CubeMX 重新生成后，要逐项比对这两个文件，确认一致。

## 1. C 板资源分配

| 资源 | 引脚 / 参数 | 用途 | 状态 |
|---|---|---|---|
| CAN1 1Mbps | PD0 RX / PD1 TX | 0x201 上膛 L 3508、0x202 上膛 R 3508、0x203 变力 3508；控制帧 0x200 | 已实现 |
| CAN2 1Mbps | PB5 RX / PB6 TX | 0x204 Yaw 2006（电调 ID 保持 4） | Yaw 已实现，需要改接线 |
| TIM1 50Hz，1µs/计数 | PE9 / PE11 / PE13 / PE14 | CH1 撒放器；CH2 剪叉 A；CH3 剪叉 B；CH4 储弹限位 | 已实现，脉宽待标定 |
| TIM8_CH1 | PC6 | 备用 PWM | 未配置 |
| TIM4_CH3 | **PD14**，2 kHz（PSC 83 / ARR 499） | 板载蜂鸣器提示音 | 已实现；模板原配 PB8（可能是 26 的外接），已改为 PD14，ioc 与 `tim.c` 同步修改；需对照原理图确认 |
| USART3 | PC11 RX，100000 偶校验 | DBUS 遥控 | 已实现 |
| USART1 | PA9 / PB7，115200 | 串口屏 | 桩 |
| USART6 | PG14 / PG9，115200 | 裁判系统 | 桩 |
| USB OTG FS | PA11 / PA12 CDC | 小电脑 / 视觉 | 桩 |
| SPI2 | PB13 / 14 / 15，CS PB12 | TF 卡 | 未配置。⚠ 26 工程中 PB12 是微动开关 |
| GPIO | PF1 / PF0、PI6 / PI7 | 两路 ADS1232 测力 | 未配置 |
| 2.4G | 待定 | 测试时同步基地 / 靶台 | 只有接口。C 板的两路串口已占满 |
| IWDG | — | 看门狗 | `DART_IWDG_ENABLE=0` |

## 2. 代码结构

```
Application/
  ConfigLayer/   dart_config.h（全部开关、映射和待标定参数）、config_uart.c（串口回调路由）
  DriverLayer/   drv_pwm（TIM1 四路），以及模板自带的 drv_can / drv_uart / drv_tick
  HardwareLayer/ RM_motor（模板）
  DeviceLayer/   dart_motor（电机表、CAN 发送）、servo（舵机，按时间推定到位）、buzzer（板载蜂鸣器提示音）、loadcell* / tf_log*、rc_sensor
  ProtocolLayer/ can_protocol（反馈分发）、judge* / vision* / screen* / link24g*
  ModuleLayer/   mech_axis（通用位置轴）、loader / tension / yaw / trigger / feeder
  ControlLayer/  rc_cmd（遥控快照）、fault（故障）、fire_gate（释放许可）、dart_fsm（模式和单发流程）、dart_tune（PID 调参接口，仅测试模式）、dart_app（初始化和周期入口）
  TaskLayer/     四个常驻任务，只是外壳，实际调用 dart_app
（带 * 的是桩：接口已定，功能未实现）
host_check/      上位机纯逻辑检查，不进 Keil 工程；运行方法见 host_check/README.md
```

配套文档：[遥控器手册](遥控器手册.md)、[零点与中位调整说明](零点与中位调整说明.md)、[行程测量方案](行程测量方案.md)、[PID 调试指南](PID调试指南.md)、[调试工具配置](调试工具配置.md)。


依赖只能从上往下：Task → Control → Module → Device/Protocol → Driver/HAL。
- 机构请求只由 `dart_fsm` 发出，电机和舵机输出只在 ControlTask 中写入
- 中断里只做反馈解析

### 初始化顺序（main.c USER CODE 2）

`MX_USB_DEVICE_Init` → `Dart_Motor_Init`（必须在 CAN 中断开启之前）→ `DRIVER_Init`（串口 DMA、PWM、CAN 过滤器）→ `DEVICE_Init`（遥控、舵机初始脉宽、桩）→ `Dart_App_Init`

### 任务（全部常驻，不动态创建）

| 任务 | 优先级（CubeMX） | 周期 | 内容 |
|---|---|---|---|
| ControlTask | Realtime | 1 ms | 遥控快照 → 状态机 → 三轴、撒放器、供弹 → CAN1/CAN2 0x200 → PWM |
| MonitorTask | Realtime | 1 ms | 遥控和电机心跳、在线类故障、喂狗（宏控制） |
| CommunityTask | High | 10 ms | 裁判、视觉、串口屏、2.4G、测力、日志轮询（目前都是桩） |
| LedTask | AboveNormal | 50 ms | 指示灯 |

## 3. 遥控操作表

操作手速查版见 [遥控器手册.md](遥控器手册.md)。

映射与 2026 国赛一致：S1 上 = 手动，S1 中 = 半自动；S1 下暂时作为安全位，接入裁判后再决定全自动放在哪。S1 是左侧拨杆，以实物为准。拨轮上拨或下拨各触发一次，必须回中后才能再次触发，长按不会连发。

| S1 | 模式 | S2 | 摇杆 | 拨轮上 | 拨轮下 |
|---|---|---|---|---|---|
| 下 | SAFE | — | — | — | — |
| 上 | MANUAL | 上 | 右竖：上膛点动 | 撒放器切换；可能已储能时，3 s 内拨两次执行**人工卸力**（不会直接打开） | 三轴同时开始堵转找零，完成后自动回 home；找零中再拨一次 = 取消（同 26） |
| 上 | MANUAL | 中 | 右横：Yaw 点动；左竖：变力点动 | 单独执行当前发序的供弹 | 发序 +1（1→4 循环） |
| 上 | MANUAL | 下 | — | 供弹舵机全部回初始位 | Yaw 和变力转到当前发序的预设位置 |
| 中 | SEMI_AUTO | 上 | — | IDLE 时：开始储能（第 1 步）；ARMED 时：释放（第 2 步）；ABORTED 时：恢复 | 中止 |
| 中 | SEMI_AUTO | 中 | — | IDLE 时：连续上膛（每发打完自动准备下一发，释放仍要人工上拨） | 中止 |
| 中 | SEMI_AUTO | 下 | — | ABORTED 且中止于供弹步骤时：确认已人工处理储弹，跳过供弹继续 | 中止 |

人工零点函数 `Mech_Axis_Set_Zero_Here` 保留在代码里，但目前不映射到遥控：三轴都用堵转找零。

- **上电或遥控重新上线后，S1 必须先拨到下位一次**，否则一直停在 SAFE。
- **自动找零**（`DART_AUTO_ZERO_ENABLE=1`）：上电后**第一次把 S1 拨离安全位**时，对还没找零的轴自动堵转找零，每次上电只做一次。电机离线导致零点丢失后，不会自动重找，需要人工找零。可以用手动模式 S2 上的拨轮下拨取消，或拨回安全位取消。
- **防呆**：默认 `DART_FOOLPROOF_TWO_STEP`。改成 `DUAL_INPUT` 后，ARMED 状态下释放还需要右竖摇杆推满（≥600）。
- **储能状态 energy**（软件推定，不是传感器）：
  - 上电为 UNKNOWN。单发启动前，必须先在 MANUAL 模式下执行一次人工卸力（找零后，3 s 内上拨两次）。
  - 以下操作无法证明未储能，一律置为 UNKNOWN：人工锁止撒放器；撒放器未释放时点动上膛；在 LOCK 步骤中止。
  - 撒放器已打开、且上膛在 UP 时，滑台不可能储能，自动推定为 RELEASED。
  - 状态为 UNKNOWN 或 STORED 时，不能启动新一发，也不能直接打开撒放器，只能恢复原流程或执行人工卸力。

**人工卸力流程（2a 反向接力）**：

| 步骤 | 动作 | 完成条件 |
|---|---|---|
| UNLOAD_TENSION | 仅当 `UNLOAD_REDUCE_TENSION=1`：变力到 `TENSION_POS_UNLOAD` | 变力就绪 |
| UNLOAD_CATCH | 上膛快速到 MID（接触点之前） | 到 MID 稳定 |
| UNLOAD_CATCH_SLOW | 上膛慢速到 `LOADER_POS_CATCH`，重新顶住已锁止的滑台 | 到接住位稳定 |
| UNLOAD_OPEN | 撒放器释放，拉簧力由电机承担 | 已释放，并再等 `UNLOAD_OPEN_HOLD_MS` |
| UNLOAD_RETURN | 上膛以 `LOADER_SPEED_RETREAT` 送回 UP | 到 UP 稳定，储能状态改为 RELEASED |

- 前置条件：上膛已找零、L/R 两台电机在线、上膛无故障。不满足就拒绝，原因记入 `last_unload_deny`。
- 每一步有 10 s 超时。拨轮下拨或离开 MANUAL 都会中止卸力，上膛原地保持。

## 4. 单发状态转移表

| 步骤 | 进入时动作 | 完成依据 | 下一步 |
|---|---|---|---|
| IDLE | — | 拨轮上拨，并通过启动检查 | LOAD_PREP |
| LOAD_PREP | 撒放器释放，上膛回 UP | 撒放器已释放，且上膛在 UP 到位稳定 | LOAD_MID（若 `FEED_BEFORE_LOAD=1` 则先到 FEED_EARLY） |
| LOAD_MID | 上膛快速移到 MID | 到位稳定 | LOAD_DOWN |
| LOAD_DOWN | 上膛慢速移到 DOWN | 到位稳定 | LOCK |
| LOCK | 撒放器锁止 | 等待 300 ms，此时 energy 标记为 STORED | LOAD_RETURN |
| LOAD_RETURN | 上膛回 UP | 到位稳定 | FEED |
| FEED | 执行当前发序的供弹步骤表（第 1 发为 0 步） | 步骤表执行完 | AIM_TENSION |
| AIM_TENSION | Yaw 和变力转到当前发序的预设 | 两者都到位稳定，且力就绪（桩） | ARMED |
| ARMED | — | 再次拨轮上拨，并通过 `Fire_Gate_Check` | FIRING（被拒时留在 ARMED，并记录原因） |
| FIRING | 撒放器释放，发数 +1 | 撒放器已释放，且已等待 800 ms | RESET |
| RESET | 撒放器锁止 | 撒放器已锁止，发序 +1 | IDLE |

- **启动检查**：三轴都已找零；energy 不是 UNKNOWN 也不是 STORED；发序 ≤4；无故障；电机全部在线；供弹不在执行中；本发需要供弹时 `DART_FEEDER_CALIBRATED` 必须为 1（未标定时只能打不需要供弹的第 1 发，供弹只能在 MANUAL 中单测）。
- **释放许可**（每次释放都重新检查）：模式、遥控、故障、电机、找零、撒放器锁止、上膛在 UP、Yaw 就绪、变力和力就绪、供弹空闲、裁判许可、双控件确认。拒绝原因见 `fire_gate.h` 中的 `fire_deny_e`。
  - **保持带**：Yaw、变力、上膛三项只要求"曾经到位"，并且当前仍在一个比到位判定更宽的范围内（`*_FIRE_HOLD_ERR/SPEED`，约为到位带的 3 倍），避免镖架轻微抖动导致误拒。AIM_TENSION 步骤仍然用严格的到位判断。其余硬条件不放宽。
  - 被拒后不会自动重试，需要重新拨一次；被拒时蜂鸣器给出 DENY 提示音。

**中止与恢复**
- 以下情况都会中止流程，进入 ABORTED：拨轮下拨、离开 SEMI_AUTO、关控、轴故障、步骤兜底超时（10 s）。
- 中止后三轴原地保持，撒放器不动作。
- 恢复需要再次上拨。各状态的恢复入口：ARMED 恢复到 AIM_TENSION 重新确认；FIRING 恢复到 RESET；其余从中止时所在的步骤重新开始。
- 在 MANUAL 模式下做任何操作，都会丢弃这次中止的现场；丢弃现场不会把储能状态改成"已释放"。
- 在供弹步骤中止后，**不会重做供弹序列**（可能重复放镖）：S2 上位恢复会被拒绝；人工确认储弹后，S2 下位上拨跳过供弹继续。

**安全行为**
- SAFE 模式或遥控离线：舵机保持当前脉宽。Yaw 和变力立即停止输出；上膛按下一条处理。
- **受控回退（停止类别 1）**：进入 SAFE 时，如果上膛已找零、无故障、在线、不在 UP，且撒放器未锁止（滑台只靠电机拉着），上膛先以 `LOADER_SPEED_RETREAT` 送回 UP，再清零输出。回退时长由轴按距离自动计算超时（理论时间 × 1.5 + 1 s），超时会记故障并停止。回退之后，中止现场的恢复入口改为 LOAD_MID。
- **上膛单侧离线（策略 A）**：只有 R 离线时，L 单独原地保持（零点仍然有效），状态机中止流程；拨回 SAFE 时立即以 `LOADER_SPEED_RETREAT_SINGLE` 单侧回退；其他模式下保持超过 `LOADER_SINGLE_HOLD_MAX_MS` 也会自动回退。L 离线时 L 是位置来源，无法闭环，两侧都停止输出。
- **运动超时**：所有轴的运动超时都按"距离 ÷ 限速 × 1.5 + 1 s"计算，配置里的 `*_MOVE_TIMEOUT_MS` 只是下限。
- **保持卡死**：点动或保持时，如果目标与实际位置相差超过 `*_HOLD_STALL_ERR`，并且满足堵转条件，就判为堵转故障；点动目标最多超前 `*_JOG_LEAD_MAX`。
- **离线故障优先**：离线可以覆盖已有的其他故障，确保停止输出。
- ⚠ 上膛故障、离线或未找零时无法回退，滑台会回弹，只能靠机械缓冲。
- 撒放器已锁止时进入 SAFE：锁扣是机械自锁，电机直接清零，是安全的。
- 轴运动超时或堵转：上膛保持当前位置（`hold_on_fault`），变力和 Yaw 停止输出。拨回 SAFE 才能确认并清除这类故障。
- 电机离线：对应的轴零点作废，必须重新找零。

**蜂鸣器提示音**（`buzzer.c`，非阻塞，高优先级可以打断低优先级）

| 提示音 | 声音 | 触发 |
|---|---|---|
| ZERO_DONE | 短响 1 声 | 三轴全部找零完成 |
| ARMED | 短响 2 声 | 进入待发 |
| DENY | 一长一短 | 启动、恢复、释放、卸力被拒 |
| FAULT | 短响 3 声 | 出现新故障 |

**指示灯**
- 蓝：遥控在线。
- 绿：电机全部在线；找零过程中闪烁。
- 红：常亮表示有故障；快闪表示 ARMED 或已储能；慢闪表示 ABORTED。

## 5. 待标定参数（全部在 dart_config.h）

| 类别 | 宏 | 当前值的来源 |
|---|---|---|
| 方向 | `LOADER/TENSION/YAW_DIR_SIGN`、`LOADER_R_MIRROR_SIGN`、`*_ZERO_SPEED` 的符号、`RC_WHEEL_UP_SIGN` | 26 源码 / 模板，需实测 |
| 上膛 | `LOADER_POS_UP/MID/DOWN/MAX`、速度、到位阈值 | 26 源码 |
| 变力 | `TENSION_POS_HOME/MAX`、`TENSION_SHOT_POS_TABLE` | 26 源码 / 占位 |
| Yaw | `YAW_COUNT_PER_DEG`（只按 2006 的 36:1 算，未计外部传动）、`YAW_DEG_MIN/MAX/HOME`、`YAW_SHOT_DEG_TABLE` | 机械设计值 0～18°，基地 15.8° |
| 舵机 | `TRIGGER_PULSE_*`（26 的 500/880）、`FEEDER_*_PULSE_*`（全部 1500 占位）、`FEEDER_STEP_HOLD_MS` | 型号未定 |
| PID | `*_POS_KP`、`*_SPD_KP/KI/IMAX` | 26 源码 |
| 找零/堵转 | `*_ZERO_SPEED`、`*_ZERO_SPD_KP/KI/IMAX`、`*_STALL_OUT_MIN`、`*_STALL_SPEED_MAX`、`STALL_HOLD_MS` | 仿照 26：变力 -1500 rpm、2000/10；Yaw 物理 +2500 rpm、1000/50；上膛 1000/10；独立速度环 kp1 ki0.1；持续 100 ms |
| 供弹标定 | `DART_FEEDER_CALIBRATED` | 0：步骤表为占位值 |
| 回退与卸力 | `LOADER_SPEED_RETREAT`（800 rpm）、`LOADER_SPEED_RETREAT_SINGLE`（400 rpm）、`LOADER_SINGLE_HOLD_MAX_MS`（5000）、`LOADER_POS_CATCH`（= DOWN）、`UNLOAD_OPEN_HOLD_MS`（300）、`UNLOAD_REDUCE_TENSION`（0）、`TENSION_POS_UNLOAD`（占位） | 全部是占位值；最小预紧位置标定前，`UNLOAD_REDUCE_TENSION` 保持 0 |
| 保持与点动 | `*_HOLD_STALL_ERR`（上膛 / 变力 20000，Yaw 300）、`*_JOG_LEAD_MAX`（上膛 / 变力 50000，Yaw 800） | 占位值，待确认 |
| 行程 | `LOADER_POS_UP/MID/DOWN/CATCH/MAX`、`TENSION_POS_MAX`、`YAW_DEG_MIN/MAX` | 按[行程测量方案](行程测量方案.md)实测；软限位可以在 RAM 中临时放宽 |

## 6. 待确认与后续工作

- [ ] 机械：卸力时拨弹板能否可靠接住滑台；变力最小预紧位置；新平 Yaw 的堵转方向是否仍与 26 一致；供弹动作顺序（`DART_FEED_BEFORE_LOAD`）；三舵机供弹步骤与脉宽
- [ ] 硬件：Yaw 改接 CAN2；舵机型号和供电；PB12 的用途；PD14 是否为板载蜂鸣器；2.4G 链路
- [ ] 裁判：拿到 2027 协议后实现 `judge_protocol`，并把 `DART_TEST_MODE` 置 0（`DART_TEST_MODE=0` 且裁判未启用时编译直接报错）。USART1/6 回调现在传入实际收到的长度，USART6 已补上 DMA 计数重装；拆包、粘包由协议层处理
- [ ] 测力：ADS1232 驱动、标定，以及 `Tension_Force_Trim` 力闭环
- [ ] TF：环形缓冲、写卡任务、FatFs；把累计打击次数持久化（目前只记在 RAM）
- [ ] 四发：AUTO 模式（裁判驱动）和 30 s 四连发的时序（暂缓）
- [ ] **每发参数示教与 Flash 保存：与串口屏一起实现**（用户要求，2026-11-03）。已确定的约束：
  - 在手动模式下点动到满意位置后保存为当前发的预设；遥控上用哪个操作待定。
  - 存内部 Flash 扇区 11（`drv_flash`，0x080E0000）。参数结构带版本号和 CRC，读到无效数据时回退到默认值。
  - 擦除时 CPU 会停住 1～2 s，**只能在安全位、电机输出为零时擦写**，失败最多重试 3 次。
  - 修改从下一发开始生效，不改动已经进入待发的这一发。
  - 屏幕和遥控使用同一套参数接口。
- [ ] CubeMX 重新生成并比对；如需调整任务优先级，改 ioc

## 7. 修改记录

- 2026-11-02 V0.1：首版框架和单发流程。相对模板的改动：
  - TIM1 增开 CH2～CH4，USART1/6 改为 115200
  - PID 补上 D 项
  - 移出 HT/DM/KT/imu_xrobot 等未用文件，删除 `DeviceLayer/motor.c/h`
  - 重写 `can_protocol.c`、`config_uart.c`（改为 UTF-8 编码）、`device.c/h` 和四个任务
- 2026-11-02 V0.2：按《当前优化与修改清单》修改：
  - P0-01 储能状态保守失效；P1-03 供弹中止不重做
  - P0-03 每轴找零方式和堵转参数分开（三轴仍为堵转，参数仿照 26）；找零可取消；Yaw 方向符号改为 -1，使物理找零方向与 26 一致
  - P0-04 测试旁路编译保护 + 供弹标定开关
  - P1-01 编码器跨圈改为 8192，并加显式首帧标志（`RM_motor.c/h`）
  - P1-04 USART1/6 传入实际长度，USART6 重装 DMA 计数
  - P1-06 日志计数加临界区
  - 删除双丝杆供弹的全部预留
- 2026-11-03 V0.3：
  - 遥控映射向 2026 看齐：S1 上手动、中半自动、下安全；手动 S2 中位的拨轮上下对调；手动 S2 下位改为"供弹舵机回初始位 / 转到预设位置"
  - 新增 SAFE 受控回退和人工卸力流程；可能储能时不再允许直接打开撒放器
  - 新增 `Feeder_Request_Home`
  - 新增 `host_check/` 上位机纯逻辑检查（32 项）
  - 重写遥控器手册
- 2026-11-03 V0.4：
  - 释放许可中的 Yaw、变力、上膛改用保持带
  - 板载蜂鸣器改到 PD14，新增提示音模块 `buzzer.c`（找零完成、待发、被拒、故障）
  - 首次离开安全位时自动找零
  - host_check 增加到 37 项
  - 参数示教与 Flash 保存记入待办，与串口屏一起实现
- 2026-11-03 V0.5：
  - 找零后的回转量改为可在线调整：轴对象新增 RAM 字段 `home_pos`，新增 `Mech_Axis_Request_Home()`
  - 新增 `TENSION_ZERO_BACKOFF`；`YAW_DEG_HOME` 改为 `YAW_DEG_CENTER`，即找零后回到的中位，待确认
  - 新增 PID 调参接口 `dart_tune.c/.h`（`g_dart_tune`，只在测试模式和手动模式下生效），以及轴的调参速度模式 `MECH_AXIS_TUNE_SPEED`
  - host_check 增加到 42 项
  - 新增零点、PID、调试工具三份文档，重写遥控器手册
- 2026-11-03 V0.6（按《当前问题简表》修复）：
  - 运动超时按距离计算，修正 SAFE 回退和卸力的时间预算
  - 卸力接住改为两段（快速到 MID，再慢速到 `LOADER_POS_CATCH`）；轴运动步骤由轴自己的超时负责
  - 保持状态的卡死判定和点动超前限制
  - 离线故障可以覆盖其他故障；上膛单侧离线按策略 A 处理（单侧保持、低速回退）
  - 蜂鸣器请求加临界区
  - 软限位改为可在线修改的 RAM 字段 `pos_min/pos_max`
  - 新增行程测量方案
  - host_check 增加到 52 + 13 项（新增 `check_axis.c`）


## Git 仓库

- 工程根目录即仓库根目录：`code/RP27_Dart`，默认分支 `main`。
- 远程 `origin`：https://github.com/RP27-EC/27_Dart.git 。
- `.gitignore` 排除 Keil 编译输出、本机 IDE 配置、调试会话、上位机检查程序和临时日志；保留 `.uvprojx`、`.ioc`、启动汇编、源码及 CMSIS 依赖库。
- 原先仅位于 `MDK-ARM` 的 Git 元数据已移到 `G:\STM32_ALL\RM\code\RP27_Dart_MDK_Git备份_20261103`，保留旧提交记录。
- 本次只建立远程关联和忽略规则，尚未提交或推送。

在工程根目录查看待提交内容：

```powershell
git status --short
git remote -v
```
