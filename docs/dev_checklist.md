# MATLAB 力控分支检查清单

## 基础 MATLAB 主线

- [ ] UR5 能通过 `loadrobot("universalUR5")` 加载。
- [ ] 3x3 目标板孔位坐标正确。
- [ ] 所有孔位的 hover 姿态 IK 可达。
- [ ] `matlab/main_sim.m` 仍能运行原位置控制基线。
- [ ] `matlab/main_interactive_sim.m` 仍能用于交互式 RRT 对比。

## 高速碰撞力控

- [ ] `matlab/main_impact_force_control.m` 能完整运行。
- [ ] 仿真包含 `approach -> force -> retract` 状态切换。
- [ ] 接触力在每个仿真步进入动力学方程，而不是事后估计。
- [ ] 力传感器模型包含延迟、噪声和滤波。
- [ ] 接触后导纳式力反馈能调节末端 z 命令。
- [ ] 峰值力和超过阈值时长能正确写入结果。
- [ ] 最大下压深度、最大接触力、关节力矩和关节速度有限幅。

## 输出

- [ ] 生成 `shared/impact_force_control_result.json`。
- [ ] 生成 `results/figures/impact_force_control_results.png`。
- [ ] 生成 `results/videos/impact_force_control_animation.gif`。
- [ ] 文档 `docs/impact_force_control.md` 与代码参数一致。
