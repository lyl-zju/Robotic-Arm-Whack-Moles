# MATLAB 高速碰撞力控打地鼠实验

本分支 `matlab-impact-force-control` 是一个 MATLAB-only 实验分支，专门用于测试机械臂与地鼠发生高速碰撞时的接触力建模、冲击响应和接触后的力反馈控制。

本分支已经移除 Python、OpenCV、PyBullet、URDF 回放和相关依赖，保留并扩展 MATLAB 侧的 UR5 建模、目标板、轨迹、动力学、力控和可视化代码。

## 快速运行

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
matlab/config/config_impact_force_control.m
matlab/control/simulate_impact_force_control.m
matlab/visualization/plot_impact_force_control.m
matlab/main_impact_force_control.m
matlab/main_interactive_impact_force_control.m
docs/impact_force_control.md
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
