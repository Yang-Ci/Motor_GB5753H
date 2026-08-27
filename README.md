# GB5753H 样机 · DM 兼容 CAN 电机测试工具

这是一个面向 Linux 的 PyQt5 上位机，用于测试手头这台 HUA YI Dynamics `GB5753H`。

## 当前样机结论

厂家已经明确回复：

> 手头样机兼容 DM 协议，华翼 CANopen 通信手册不适用于该样机，直接使用 DM 标准控制帧。

因此当前样机按以下参数工作：

| 项目 | 当前值 |
|---|---|
| CAN 帧 | 11 位标准帧、经典 CAN、8 字节数据 |
| CAN 波特率 | 1 Mbps |
| 默认电机 ID | 1 |
| 控制协议 | DM 标准控制帧 |
| USB-CAN | DM Tools USB2CAN REV V3.0，CDC 串口 `/dev/ttyACM0` |
| 主机串口 | 921600 baud |

CANopen FD 参数不适用于当前样机。软件仍保留一个只读参考页；厂家对象字典不包含在公开仓库中，如需使用可在项目根目录自行放置 `华翼关节模组对象字典V11(只读).xlsx`。

## 已实现

- 照片中 DM USB2CAN 串口桥的 30 字节发送封装和 16 字节接收封装。
- DM MIT、位置-速度、速度模式控制帧。
- 使能、失能、清错、设置零点特殊帧。
- DM 反馈帧解析和原始 CAN 报文监视。
- 单次发送与可配置周期发送。
- 离线虚拟电机，便于不接实机检查界面和报文。
- 运动指令安全解锁、使能二次确认、断开前失能等保护。
- 使能后锁定当前运动帧类型；必须先失能，才能在 MIT、位置-速度和速度之间切换。
- 独立温度监测页：驱动/电机双曲线、当前/最低/最高温度、可调预警阈值和 CSV 导出。
- 独立运动监测页：目标/实际位置、速度、力矩曲线，实时误差、实际最低/最高值和 CSV 导出。

软件启动后默认选择：

```text
GB5753H 样机 · DM USB2CAN 串口桥（推荐）
接口：/dev/ttyACM0
```

这个适配器表现为串口设备，不会生成 `can0`，这是正常现象。

### 温度监测

“温度监测”页使用 DM 反馈帧中的驱动温度和电机温度，不额外发送查询命令。单次控制通常产生一个温度采样点；启用周期控制时会形成连续曲线。页面保留最近 10000 个采样点，可导出为 UTF-8 CSV。

默认预警/报警阈值为 `70℃ / 85℃`，仅用于界面提示，不会自动使电机失能，也不能替代驱动器保护。正式带载测试前应使用厂家给出的温度限制调整阈值。

### 运动监测

“运动监测”页使用最后一次成功发送的控制值作为目标，并与 DM 反馈帧中的实际位置、速度和力矩对比。位置-速度模式中的目标速度表示梯形轨迹最高速度，不是驱动器内部的瞬时轨迹设定值。未由当前模式提供的目标量显示为“—”。

## 运行

```bash
cd /home/robot/Motor_GB5753H
python3 -m pip install -r requirements.txt
python3 run.py
```

如果换用能注册 `can0` 的 gs_usb/CANable 等适配器，可选择“达妙 · 经典 SocketCAN（1M）”，并先配置：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0
```

## DM 帧依据

当前实现依据 DM 标准控制帧，并与 USB2CAN Python 示例交叉核对：

- MIT：CAN ID 为 `ID`。
- 位置-速度：CAN ID 为 `0x100 + ID`，数据是两个 Float32 小端数。
- 速度：CAN ID 为 `0x200 + ID`，数据是一个 Float32 小端数。
- 使能、失能、设置零点、清错：均发送到基础电机 ID，末字节分别为 `FC`、`FD`、`FE`、`FB`。
- 反馈数据：位置 16 位，速度和力矩各 12 位，另含状态及温度。

默认 ID=1 时，三种模式的控制 ID 分别是 `0x001`、`0x101`、`0x201`。

## 首次实机测试顺序

1. 先选择“离线模拟”，检查使能、失能、控制报文和反馈显示。
2. 电机输出轴架空，人员远离夹点，接好急停并使用限流电源。
3. 断电并拔掉 USB 后测量 CAN-H 与 CAN-L 之间电阻：约 `60Ω` 通常表示总线两端各有一个 `120Ω` 终端；`120Ω` 表示仅一个终端；无穷大表示开路；接近 `0Ω` 表示短路。
4. 确认 CAN-H 对 CAN-H、CAN-L 对 CAN-L，不能只按线色判断。
5. 连接串口桥，但先不要勾选运动安全确认，也不要使能；先观察接收区是否有反馈。
6. 向厂家确认这台 GB5753H 的 `PMAX / VMAX / TMAX` 后，再填写量程并从零目标、低增益开始测试。

界面暂用的 `PMAX=12.5`、`VMAX=30`、`TMAX=10` 只是 DM 通用协议示例值，不是 GB5753H 已确认参数。量程不一致会造成目标值与反馈值缩放错误，因此不能直接用于首次运动。

## 当前硬件诊断结果

- Linux 已识别适配器为 `/dev/ttyACM0`，当前用户具备串口访问权限。
- 上位机可按官方串口封装向适配器写入经典 CAN 帧。
- 已发送过安全的失能帧，未发送使能或运动命令。
- 当前尚未收到电机反馈，因此实机链路还不能判定已打通。

下一步优先检查 CAN-H/CAN-L 接线和终端电阻，并观察上位机发送时适配器的 TX/RX 指示灯。也可在 Windows 上用同一适配器运行厂家提供的 `HY_Motion_Studio_V1p2.exe` 做对照：若官方软件也无回包，问题大概率在物理链路或适配器设置；若官方软件有回包，则可抓取其串口数据进一步校准本工具的适配器封装。

## 测试

```bash
python3 -m unittest discover -s tests -v
QT_QPA_PLATFORM=offscreen python3 -c "import sys; sys.path.insert(0, 'src'); from PyQt5.QtWidgets import QApplication; from gb5753_tool.app import MainWindow; app=QApplication([]); w=MainWindow(); print(w.protocol_combo.currentData(), w.channel_edit.text()); w.close()"
```
