# host_check：上位机纯逻辑检查

- 用途：在 PC 上检查 `Application/ControlLayer/dart_fsm.c` 的状态机判定。覆盖内容包括模式映射、SAFE 受控回退、人工卸力、自动找零、提示音和 PID 调参接口。
- 边界：
  - 机构、设备、协议都由 `fakes.c` 中的假实现替代；
  - 固件源码不为检查做任何改动，本目录也不加入 `.uvprojx`；
  - 结果只说明"逻辑符合预期"，**不代表功能已验证或实机通过**。
- 依赖：本机已有的 gcc（`C:\ClionToolchains\mingw64\bin\gcc.exe`），不需要安装任何东西。

## 运行

在 `code\RP27_Dart` 目录下执行：

```powershell
gcc -std=c99 -Wall -I host_check/stubs -I Application/ConfigLayer -I Application/ControlLayer -I Application/ModuleLayer -I Application/DeviceLayer -I Application/ProtocolLayer host_check/check_fsm.c host_check/fakes.c Application/ControlLayer/dart_fsm.c Application/ControlLayer/dart_tune.c -o host_check/check_fsm.exe
```

```powershell
.\host_check\check_fsm.exe
```

通用轴（mech_axis.c）的检查：超时按距离计算、RAM 软限位、点动超前、保持卡死、离线覆盖故障。

```powershell
gcc -std=c99 -Wall -I host_check/stubs -I Application/ConfigLayer -I Application/ModuleLayer host_check/check_axis.c Application/ModuleLayer/mech_axis.c -o host_check/check_axis.exe
```

```powershell
.\host_check\check_axis.exe
```

2.4G 帧协议（`link24g_frame.h`）的检查：CRC、往返编解码、拆包粘包、重同步、错误帧丢弃。

```powershell
gcc -std=c99 -Wall -I Application/ProtocolLayer host_check/check_link24g.c -o host_check/check_link24g.exe
```

```powershell
.\host_check\check_link24g.exe
```

退出码为 0 表示全部通过。生成的 exe 是临时文件，用完后删除。

## 维护

- `dart_fsm.c` 新增了对其他模块的调用时，要在 `fakes.c` 中补上对应的假实现。
- `stubs/` 中的头文件只用于隔离 HAL 和 CAN 依赖；字段要和固件中的同名结构保持一致。
