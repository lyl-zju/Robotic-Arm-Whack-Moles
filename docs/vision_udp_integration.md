# 视觉输入接入 Python-MATLAB 打地鼠链路

本文档说明如何把 `test_virtual_camera` 中的蓝色圆柱体九宫格识别结果接入 MATLAB 高速碰撞力控打地鼠 demo。

## 运行结构

当前链路由三个部分组成：

```text
手机/虚拟摄像头
  -> test_virtual_camera/red_grid_detector.py
  -> python.communication.vision_grid_target_sender
  -> UDP JSON
  -> matlab/main_udp_impact_force_control.m
  -> 轨迹规划、力控敲打、可视化输出
```

视觉脚本负责实时识别蓝色圆柱体所在格子，并在终端输出 JSON：

```json
{"valid": true, "target_id": 5, "row": 2, "col": 2, "method": "circle_first"}
```

桥接器读取这些 JSON 行，把 `target_id` 映射到 MATLAB 目标板坐标。默认只有连续 5 帧输出同一个合法 `target_id`，且该 id 与上一次发送给 MATLAB 的 id 不同，才通过 UDP 发送给 MATLAB。

## 启动顺序

先在 MATLAB 中进入本仓库根目录，运行：

```matlab
run("matlab/main_udp_impact_force_control.m")
```

MATLAB 终端应出现：

```text
Listening for Python UDP board targets on 0.0.0.0:5005
```

然后在本仓库根目录打开 PowerShell，运行视觉桥接器：

```powershell
python -m python.communication.vision_grid_target_sender
```

默认视觉脚本路径是：

```text
..\test_virtual_camera\red_grid_detector.py
```

如果你的视觉工程不在默认位置，可以显式指定：

```powershell
python -m python.communication.vision_grid_target_sender --detector-script "E:\learning\大二春夏\机器人学1\大作业\test_virtual_camera\red_grid_detector.py"
```

## 传递视觉参数

桥接器自身参数写在前面；需要传给 `red_grid_detector.py` 的参数写在后面。

常用示例：

```powershell
python -m python.communication.vision_grid_target_sender -- --index 1 --backend msmf --fourcc YUY2
```

带视觉调试叠加：

```powershell
python -m python.communication.vision_grid_target_sender -- --hough-lines --hough-circles --auto-grid-debug
```

如果蓝色目标较暗：

```powershell
python -m python.communication.vision_grid_target_sender -- --sat-min 45 --val-min 25 --blue-hue-min 80 --blue-hue-max 145 --min-area 300
```

如果想用手动标定结果：

```powershell
python -m python.communication.vision_grid_target_sender -- --grid-mode config
```

## UDP payload

桥接器发送给 MATLAB 的 UDP JSON 示例：

```json
{"valid":true,"source":"vision_grid_detector","seq":0,"timestamp":1710000000.0,"target_id":5,"row":2,"col":2,"board":[0.0,0.0],"world":[0.45,0.0,0.05],"x":0.45,"y":0.0,"z":0.05,"vision_method":"circle_first","vision_confidence":0.91,"vision_center":[320.0,240.0]}
```

MATLAB 当前只依赖 `target_id`、`row`、`col`、`board`、`world` 执行动作；`vision_*` 字段用于调试和日志追踪。

## 稳定性与重复目标策略

桥接器默认自动给 `red_grid_detector.py` 加上：

```text
--print-every 0
```

这样视觉脚本会尽量每帧输出一次检测 JSON，桥接器可以做连续帧稳定性判断。默认发送条件是：

```text
连续 5 帧都是同一个合法 target_id
并且该 target_id 与上一次发送给 MATLAB 的 id 不同
```

也就是蓝色圆柱体刚移动到新格子时，不会立刻触发敲打；必须稳定识别 5 帧后才发送 UDP。

如果要修改稳定帧数：

```powershell
python -m python.communication.vision_grid_target_sender --stable-frames 8
```

如果你显式传入视觉脚本参数 `--print-every`，桥接器不会覆盖它。例如：

```powershell
python -m python.communication.vision_grid_target_sender -- --print-every 0.1
```

这样稳定性判断就基于视觉脚本实际打印出来的 JSON 次数，而不是严格摄像头帧数。

默认不重复发送同一个 id，原因是：

```text
同一个蓝色圆柱体停在同一格时，只应触发一次敲打。
圆柱体移动到新格时，再触发下一次敲打。
```

如果你确实需要在目标不变时定时重发，可以使用：

```powershell
python -m python.communication.vision_grid_target_sender --resend-same-after-s 5.0
```

## 停止方式

```text
MATLAB: 在目标板窗口按 Esc，或关闭两个图窗
Python/视觉桥接器: Ctrl+C
视觉窗口: q 或 Esc
```

## 调试建议

先单独确认视觉脚本能稳定输出目标：

```powershell
python ..\test_virtual_camera\red_grid_detector.py --once
```

再确认随机 UDP demo 能触发 MATLAB：

```powershell
python -m python.communication.random_board_target_sender --count 1 --seed 7
```

最后运行视觉桥接器。如果终端只有 `vision invalid`，优先检查：

```text
摄像头是否被其他软件占用
EV 虚拟摄像头是否正在推流
蓝色圆柱体是否在九宫格内
黑色九宫格线是否能被视觉脚本识别
是否需要 --grid-mode config 使用手动标定
```
