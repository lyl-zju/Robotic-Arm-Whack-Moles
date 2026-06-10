# 接口约定

## 目标输入

所有目标最终都统一成如下字段：

```text
target.id
target.position_board = [x, y]
target.position_world = [X, Y, Z]
target.appear_time
target.mode = random / mouse / color
```

## Python 到 MATLAB

文件：`shared/target.json`

- `valid`：是否检测到有效目标。
- `source`：当前固定为 `color`，后续可扩展。
- `confidence`：颜色检测的面积归一化置信度。
- `pixel`：图像中心点 `[u, v]`。
- `board`：目标板坐标 `[x, y]`。
- `world`：机械臂基坐标系下 `[X, Y, Z]`。
- `timestamp`：检测相对耗时或时间戳。

## MATLAB 到 Python

文件：`shared/q_traj.csv`

第一列为 `t`，后续为 `q1...qn`。若 PyBullet 回放暂不做，可以只保留该文件作为导出结果。

## MATLAB 结果

文件：`shared/result.json`

保留命中成功、命中误差、响应时间、控制器类型、路径类型和跟踪误差摘要。

