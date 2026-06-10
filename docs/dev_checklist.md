# 开发检查清单

## MATLAB 主线

- [ ] UR5 或备选机械臂能加载显示。
- [ ] 3x3 目标板和孔位坐标正确。
- [ ] 所有孔位 IK 可达。
- [ ] 固定单目标能完成 `hover -> hit -> back`。
- [ ] 随机目标能连续敲击 10 次。
- [ ] 鼠标点击目标能映射到目标板坐标。
- [ ] RRT 能绕开圆形或矩形障碍物。
- [ ] PD 或重力补偿 PD 有误差曲线。
- [ ] `shared/result.json` 与 `shared/q_traj.csv` 能导出。

## Python 视觉保底

- [ ] PyBullet 中目标板可见。
- [ ] 红色目标能随机出现在孔位上。
- [ ] 相机截图保存到 `data/screenshots/`。
- [ ] 颜色识别能输出中心像素。
- [ ] 单应性映射能输出目标板坐标。
- [ ] `shared/target.json` 格式能被 MATLAB 读取。

## 集成

- [ ] 先运行 Python 视觉侧写入 `target.json`。
- [ ] 再运行 `matlab/main_hit_from_vision.m`。
- [ ] 命中误差、响应时间和跟踪误差能保存。
- [ ] 演示视频至少包含随机目标、鼠标点击、RRT 和颜色识别闭环。

