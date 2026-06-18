# 视觉输入到机械臂打击完整流程说明

本文档用于说明当前“视觉输入 -> Python-MATLAB 通信 -> 机械臂轨迹规划与力控打击 -> 可视化反馈”的完整 demo 流程，可作为课程报告中系统设计与实现部分的参考。

## 1. 系统目标

本系统模拟一个视觉驱动的打地鼠任务。手机摄像头通过虚拟摄像头实时观察九宫格板面，视觉程序识别蓝色圆柱体所在的格子编号 `1..9`。Python 桥接程序将稳定识别出的目标编号通过 UDP 发送给 MATLAB。MATLAB 收到目标后，将九宫格编号映射到机械臂工作空间中的目标点，并调用现有的 UR5 轨迹规划、动力学仿真和高速碰撞力控算法完成敲击动作。

当前 demo 的核心闭环是：

```text
蓝色圆柱体出现在九宫格某一格
  -> 视觉识别输出 target_id
  -> Python 桥接器做 5 帧稳定性判断
  -> UDP JSON 发送给 MATLAB
  -> MATLAB 映射到 board/world 坐标
  -> UR5 规划 hover/contact/hit/back 轨迹
  -> 力控敲击并判断 hit success
  -> 可视化界面显示目标 id、接触力和成功提示
```

## 2. 模块划分

### 2.1 视觉识别模块

外部视觉工程路径：

```text
E:\learning\大二春夏\机器人学1\大作业\test_virtual_camera
```

核心脚本：

```text
test_virtual_camera/red_grid_detector.py
```

该脚本读取手机虚拟摄像头画面，检测蓝色圆柱体以及九宫格黑色网格线，并输出 JSON 检测结果，例如：

```json
{"valid": true, "target_id": 5, "row": 2, "col": 2, "method": "circle_first"}
```

其中 `target_id` 是九宫格编号，按行优先排列：

```text
1 2 3
4 5 6
7 8 9
```

### 2.2 Python UDP 桥接模块

本仓库中的桥接脚本：

```text
python/communication/vision_grid_target_sender.py
```

它负责启动视觉脚本、读取视觉脚本 stdout 中的 JSON 行、进行稳定性判断，并把目标编号封装为 MATLAB 可解析的 UDP JSON。

桥接器默认策略：

```text
连续 5 帧都是同一个合法 target_id
并且该 target_id 与上一次发送给 MATLAB 的 id 不同
才发送给 MATLAB
```

这样可以减少视觉抖动造成的误触发。例如蓝色圆柱体在格子边缘附近移动时，视觉输出可能在相邻格子之间短暂跳变；5 帧稳定性判断可以过滤这种瞬时抖动。

桥接器默认还会给视觉脚本附加：

```text
--print-every 0
```

使视觉脚本尽量每帧输出检测 JSON，从而让“5 帧稳定”更接近真实帧级判断。如果用户显式传入 `--print-every`，桥接器不会覆盖用户设置。

### 2.3 MATLAB UDP 接收与目标映射模块

MATLAB 主入口：

```text
matlab/main_udp_impact_force_control.m
```

通信解析函数：

```text
matlab/communication/read_latest_udp_board_target.m
matlab/target/board_target_from_id.m
```

MATLAB 端收到 UDP JSON 后，优先解析 `target_id`，并根据 `config_board.m` 中定义的九宫格参数映射到 board 坐标和 world 坐标。

board 参数来自：

```text
matlab/config/config_board.m
```

当前九宫格 board 坐标为：

```text
x_list = [-0.20, 0, 0.20]
y_list = [-0.13, 0, 0.13]
```

目标点映射关系：

```text
id 1 -> board [-0.20, -0.13]
id 2 -> board [ 0.00, -0.13]
id 3 -> board [ 0.20, -0.13]
id 4 -> board [-0.20,  0.00]
id 5 -> board [ 0.00,  0.00]
id 6 -> board [ 0.20,  0.00]
id 7 -> board [-0.20,  0.13]
id 8 -> board [ 0.00,  0.13]
id 9 -> board [ 0.20,  0.13]
```

### 2.4 轨迹规划与力控打击模块

MATLAB 收到目标点后，会基于当前机械臂状态直接规划到目标点对应的关键位姿：

```text
hover   : 目标点上方
contact : 接触高度
hit     : 下压/击打位置
back    : 回撤位置
```

核心规划与控制函数包括：

```text
matlab/planning/make_hit_keyposes.m
matlab/planning/make_hit_motion_boundaries.m
matlab/planning/plan_joint_traj.m
matlab/robot/solve_ik.m
```

实时力控仿真逻辑在：

```text
matlab/main_udp_impact_force_control.m
matlab/config/config_impact_force_control.m
```

控制过程分为：

```text
approach : 机械臂末端接近目标上方
impact   : 末端向下接触目标
force    : 接触后使用导纳式力反馈修正 z 方向命令
retract  : 达到击倒条件后回撤
```

击打成功条件由接触力阈值和最小接触保持时间决定。成功后，MATLAB 命令行和可视化 HUD 会输出 `hit success`。

## 3. UDP 通信协议

Python 桥接器发送 UTF-8 JSON datagram。视觉输入产生的 payload 示例：

```json
{
  "valid": true,
  "source": "vision_grid_detector",
  "seq": 0,
  "timestamp": 1710000000.0,
  "target_id": 5,
  "row": 2,
  "col": 2,
  "board": [0.0, 0.0],
  "world": [0.45, 0.0, 0.05],
  "x": 0.45,
  "y": 0.0,
  "z": 0.05,
  "vision_method": "circle_first",
  "vision_confidence": 0.91,
  "vision_center": [320.0, 240.0],
  "vision_stable_frames": 5
}
```

MATLAB 当前主要依赖：

```text
valid
target_id
row
col
board
world
```

`vision_*` 字段用于调试和报告记录，例如说明检测方法、检测置信度、目标中心像素坐标和稳定帧数。

UDP 的设计理由：

```text
1. 视觉目标是实时数据，只关心最新目标。
2. UDP 延迟低，实现简单。
3. MATLAB 端会清空积压 datagram，只处理最新合法目标。
4. 运动过程中若又收到目标，只保留最新 pending 目标，避免队列无限增长。
```

## 4. MATLAB 目标执行策略

当前 MATLAB 端不是抢占式控制，而是“当前目标打完后再执行下一个目标”。

逻辑如下：

```text
如果机械臂空闲：
    收到目标后立即规划并执行

如果机械臂正在打击：
    新目标暂存为 pendingUdpTarget
    若运动中又收到更新目标，则覆盖旧 pending

当前 activeTraj 执行结束后：
    若 pendingUdpTarget 非空，则执行最新 pending 目标
```

这样做的好处是视觉输入即使有新目标变化，也不会让机械臂在一次敲击过程中频繁中断和重规划，更适合课堂展示和报告实验。

## 5. 可视化输出

MATLAB 会打开两个窗口：

```text
UDP Impact Force-Control Target Board
UDP Impact Force-Control Robot
```

目标板窗口显示：

```text
当前九宫格目标位置
当前 target id
seq
source
峰值接触力
hit_success 状态
```

机器人窗口显示：

```text
UR5 当前运动状态
目标圆柱体/地鼠模型
末端运动轨迹
规划关键点
接触力 HUD
当前 target id
hit status
phase
peak force
source
```

打击成功后，界面和命令行都会提示：

```text
hit success target id=..., peak force=... N
```

## 6. 运行方法

### 6.1 启动 MATLAB 接收与打击程序

在 MATLAB 中进入本仓库根目录，运行：

```matlab
run("matlab/main_udp_impact_force_control.m")
```

如果默认端口 `5005` 被占用，可以换端口：

```matlab
udpLocalPort = 5010;
run("matlab/main_udp_impact_force_control.m")
```

### 6.2 启动视觉桥接器

在 PowerShell 中进入本仓库根目录，运行：

```powershell
python -m python.communication.vision_grid_target_sender
```

如果 MATLAB 使用了其他端口，例如 `5010`：

```powershell
python -m python.communication.vision_grid_target_sender --port 5010
```

### 6.3 传递视觉脚本参数

桥接器自身参数写在前面，传给视觉脚本的参数写在 `--` 后面。例如：

```powershell
python -m python.communication.vision_grid_target_sender -- --index 1 --backend msmf --fourcc YUY2
```

开启视觉调试叠加：

```powershell
python -m python.communication.vision_grid_target_sender -- --hough-lines --hough-circles --auto-grid-debug
```

如果要修改稳定帧数，例如要求连续 8 帧一致：

```powershell
python -m python.communication.vision_grid_target_sender --stable-frames 8
```

### 6.4 通信基准测试

如果要排除视觉问题，只测试 Python-MATLAB 通信和 MATLAB 打击流程，可以运行随机目标发送器：

```powershell
python -m python.communication.random_board_target_sender --count 1 --seed 7
```

## 7. 报告中可描述的关键设计点

### 7.1 稳定性判断

视觉检测结果必须连续多帧一致才发送给 MATLAB。当前默认参数是：

```text
stable_frames = 5
```

这可以降低误识别、边界抖动和单帧噪声带来的错误触发。

### 7.2 最新目标优先

UDP 接收端会读取所有积压 datagram，只保留最新合法目标。机械臂运动中收到的新目标也只保存最新一个 pending 目标。这符合实时视觉任务“只关心当前最新目标”的特点。

### 7.3 非抢占式打击

机械臂当前敲击动作完成后才执行下一个目标，避免视觉输入抖动导致频繁中途重规划，使 demo 行为更稳定。

### 7.4 分阶段力控

系统不是只做位置轨迹跟踪，而是模拟了接触力和接触后的力反馈控制。接触后，控制器根据力误差调整末端法向位置命令，从而实现击打和保持接触力。

## 8. 常见问题

### 8.1 MATLAB 提示 UDP 端口被占用

换一个端口：

```matlab
udpLocalPort = 5010;
run("matlab/main_udp_impact_force_control.m")
```

Python 同步使用：

```powershell
python -m python.communication.vision_grid_target_sender --port 5010
```

### 8.2 视觉窗口能运行但 MATLAB 没有动作

检查：

```text
1. MATLAB 是否已经显示 Listening for Python UDP board targets。
2. Python 和 MATLAB 是否使用同一个端口。
3. 视觉桥接器终端是否出现 sent seq=... id=...。
4. target_id 是否连续稳定达到 5 帧。
5. MATLAB 是否仍在执行上一个目标；若是，新目标会先进入 pending。
```

### 8.3 视觉识别不稳定

可以尝试：

```powershell
python -m python.communication.vision_grid_target_sender --stable-frames 8
```

或给视觉脚本传入调试参数：

```powershell
python -m python.communication.vision_grid_target_sender -- --hough-lines --hough-circles --auto-grid-debug
```

如果蓝色目标较暗，可以降低阈值：

```powershell
python -m python.communication.vision_grid_target_sender -- --sat-min 45 --val-min 25 --blue-hue-min 80 --blue-hue-max 145 --min-area 300
```

## 9. 相关文件索引

视觉工程：

```text
E:\learning\大二春夏\机器人学1\大作业\test_virtual_camera\red_grid_detector.py
```

Python 桥接与协议：

```text
python/communication/vision_grid_target_sender.py
python/communication/random_board_target_sender.py
python/communication/board_protocol.py
```

MATLAB UDP 与目标映射：

```text
matlab/main_udp_impact_force_control.m
matlab/communication/read_latest_udp_board_target.m
matlab/target/board_target_from_id.m
matlab/config/config_board.m
```

MATLAB 规划、机器人和力控：

```text
matlab/planning/make_hit_keyposes.m
matlab/planning/plan_joint_traj.m
matlab/robot/solve_ik.m
matlab/config/config_impact_force_control.m
```
