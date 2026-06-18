# Python-MATLAB UDP 视觉打地鼠 demo

本分支基于 `matlab-impact-force-control`，用于打通“Python 实时目标输入 -> MATLAB 接收目标 -> 现有轨迹规划与高速碰撞力控敲打 -> 可视化显示结果”的闭环。

当前支持两个输入源：随机九宫格目标用于通信基准测试；手机/虚拟摄像头视觉识别用于完整 demo。视觉识别程序位于同级目录 `test_virtual_camera`，会输出蓝色圆柱体所在的九宫格序号，桥接器会把这个序号转成 MATLAB 可接收的 UDP JSON。

## 快速运行

### 1. 视觉输入完整 demo

先在 MATLAB 中进入项目根目录，启动 UDP 接收与实时敲打主程序：

```matlab
run("matlab/main_udp_impact_force_control.m")
```

然后在第二个终端中启动视觉桥接器：

```powershell
python -m python.communication.vision_grid_target_sender
```

默认情况下，桥接器会启动：

```text
..\test_virtual_camera\red_grid_detector.py
```

视觉桥接器默认要求连续 5 帧输出同一个合法 `target_id=1..9`，并且该 id 与上一次发送给 MATLAB 的 id 不同，才会发送 UDP，避免视觉抖动或同一目标周期性重复触发。完整流程说明见 [docs/vision_to_robot_hit_pipeline.md](docs/vision_to_robot_hit_pipeline.md)，运行参数和调试流程见 [docs/vision_udp_integration.md](docs/vision_udp_integration.md)。

如果要把参数传给视觉脚本，直接追加在命令末尾，例如：

```powershell
python -m python.communication.vision_grid_target_sender -- --index 1 --backend msmf --fourcc YUY2 --hough-lines --hough-circles
```

### 2. Python-MATLAB UDP 随机目标 demo

先在 MATLAB 中进入项目根目录，启动 UDP 接收与实时敲打主程序：

```matlab
run("matlab/main_udp_impact_force_control.m")
```

然后在第二个终端中启动 Python 随机目标发送器：

```powershell
python -m python.communication.random_board_target_sender --host 127.0.0.1 --port 5005 --period-s 2.5
```

运行后，Python 会每 2.5 秒随机选择一个 `target_id=1..9` 发送给 MATLAB。MATLAB 目标板窗口会显示当前目标点和目标 id，机器人窗口会实时显示 UR5、轨迹、接触力 HUD、红色圆柱地鼠下压效果；敲击成功后命令行、目标板状态框和机器人 HUD 会显示 `hit success` 与峰值接触力。

停止方式：

```text
MATLAB: 在目标板窗口按 Esc，或关闭两个图窗
Python: Ctrl+C
```

有限次调试可以使用：

```powershell
python -m python.communication.random_board_target_sender --count 3 --seed 7
```

如果要改 UDP 端口，可在 MATLAB 运行前设置：

```matlab
udpLocalPort = 5010;
run("matlab/main_udp_impact_force_control.m")
```

Python 端同步使用 `--port 5010`。

### 3. 原 MATLAB-only 力控 demo

在 MATLAB 中进入项目根目录后运行：

```matlab
run("matlab/main_impact_force_control.m")
```

如果想鼠标选目标点并立即执行力控敲击，运行：

```matlab
run("matlab/main_interactive_impact_force_control.m")
```

然后在目标板窗口中连续点击目标点。新的点击会立即抢占当前动作，机械臂从当前关节状态直接规划到目标 hover/contact/hit/back 关键点，并用五次多项式连接，不会等待上一轮敲击播放完成；界面会实时显示接触力和地鼠下压效果。

非交互入口运行后会生成：

```text
results/figures/impact_force_control_results.png
results/videos/impact_force_control_animation.gif
shared/impact_force_control_result.json
```

其中 GIF 会显示红色地鼠圆柱被机械臂接触后压入洞口，不再只是静态平面目标点；场景左上角会实时显示接触力、滤波力、峰值力、阈值和当前控制阶段。

## UDP 数据格式

Python 每次发送一个 UTF-8 JSON datagram。当前随机 demo 的 payload 示例：

```json
{"valid":true,"source":"random_board_demo","seq":0,"timestamp":1710000000.0,"target_id":5,"row":2,"col":2,"board":[0.0,0.0],"world":[0.45,0.0,0.05],"x":0.45,"y":0.0,"z":0.05}
```

MATLAB 端优先使用 `target_id`，也兼容 `id`、`row/col`、`board` 或 `world` 字段。若收到连续多个 UDP 包，主循环会清空积压包并只处理最新的合法目标。九宫格编号遵循 `config_board.m` 中的行优先顺序：

```text
1 2 3   y = -0.13
4 5 6   y =  0.00
7 8 9   y =  0.13
```

视觉桥接器会在 payload 中额外带上 `vision_method`、`vision_confidence`、`vision_center` 等字段，但 MATLAB 当前只依赖 `target_id`/`row`/`col`/`board`/`world` 执行动作。

## 报告实验

用于作业报告和课堂展示的三组实验可以一键运行：

```matlab
run("matlab/experiments/run_impact_force_experiments.m")
```

实验包括单次典型敲击、不同敲击速度对峰值力的影响、有无导纳力反馈对比、不同击倒最小接触力阈值下的力曲线和末端速度曲线。结果和分析见：

```text
matlab/experiments/impact_force_experiments.md
matlab/experiments/results/
```

## 核心入口

```text
matlab/main_impact_force_control.m
```

流程：

1. 加载 UR5、目标板、力阈值和高速碰撞力控参数。
2. 随机选择一个地鼠孔位作为目标。
3. 求解目标上方 hover 姿态作为初始姿态。
4. 执行高速下压、碰撞接触、力反馈保持和回撤。
5. 保存力曲线、穿透深度、末端速度、关节力矩和结果摘要。

## 新增 MATLAB 文件

```text
matlab/main_udp_impact_force_control.m
matlab/communication/read_latest_udp_board_target.m
matlab/target/board_target_from_id.m
matlab/config/config_impact_force_control.m
matlab/control/simulate_impact_force_control.m
matlab/visualization/plot_impact_force_control.m
matlab/main_impact_force_control.m
matlab/main_interactive_impact_force_control.m
docs/impact_force_control.md
```

新增 Python 文件：

```text
python/communication/board_protocol.py
python/communication/random_board_target_sender.py
python/communication/vision_grid_target_sender.py
```

## 控制思想

高速碰撞第一瞬间的峰值力主要由碰撞速度、接触刚度、阻尼、机械臂等效质量和采样周期决定，不能依赖反馈精确控制。因此本分支采用分阶段策略：

```text
approach: 末端从 hover 点向地鼠上方接近
impact: 以设定速度向下接触地鼠
force: 接触后使用导纳式力反馈修正 z 方向命令
retract: 达到阈值并保持足够时间后回撤
```

动力学仿真中，接触力不是事后计算，而是在每个仿真步作为外力矩进入关节动力学：

```text
M(q) qdd + C(q,qd) + G(q) = tau_cmd + J(q)' F_contact
```

控制器使用操作空间阻抗控制：

```text
F_task = Kx (x_cmd - x) + Dx (xd_cmd - xd)
tau_cmd = Jv' F_task + tau_posture + G(q)
```

接触后，力误差通过一阶导纳近似修正末端法向位置：

```text
z_cmd = z_cmd - K_adm (F_des - F_filtered) dt
```

详细原理、参数和全流程见 [docs/impact_force_control.md](docs/impact_force_control.md)。

## 仍保留的 MATLAB 基线

原 MATLAB 打地鼠基线仍在：

```text
matlab/main_sim.m
matlab/main_interactive_sim.m
```

它们用于对比原来的位置/PD 控制流程。新增高速碰撞力控实验请优先运行：

```matlab
run("matlab/main_impact_force_control.m")
```

交互式点击目标实验运行：

```matlab
run("matlab/main_interactive_impact_force_control.m")
```
