# Python 目录说明

Python 侧负责三件事：

1. 构建 PyBullet 目标板/相机场景。
2. 用 OpenCV 做红色目标检测，并把目标写入 `shared/target.json`。
3. 读取 MATLAB 导出的 `shared/q_traj.csv`，在 PyBullet 中播放 UR5 轨迹并实时检测接触力。

## 目录结构

```text
python/
  common/
    config.py
  vision/
    camera_capture.py
    detect_color_target.py
    homography.py
    run_color_pipeline.py
  simulation/
    pybullet_scene.py
    replay_matlab_force_hit.py
  io_utils/
    replay_matlab_traj.py
    write_target_json.py
```

顶层只保留 `README.md` 和 `__init__.py`。所有程序入口都通过 `python -m ...` 运行。

## 功能分组

### `common/`

公共配置：

- `common/config.py`：项目路径、目标板尺寸、孔位、相机参数。

### `vision/`

视觉目标检测链路：

- `vision/camera_capture.py`：只截图，保存 `data/screenshots/camera_rgb.png`。
- `vision/detect_color_target.py`：HSV 红色目标检测，返回像素中心、bbox 和置信度。
- `vision/homography.py`：根据 `data/calibration/board_corners.json` 将像素坐标映射为目标板坐标。
- `vision/run_color_pipeline.py`：完整视觉入口，截图/读图、检测、坐标转换、写入 `shared/target.json`。

### `simulation/`

PyBullet 场景和力检测：

- `simulation/pybullet_scene.py`：目标板、孔位、红色目标、相机截图场景。
- `simulation/replay_matlab_force_hit.py`：读取 MATLAB 轨迹，在 PyBullet 中播放 UR5/锤头，并实时采样接触力。

### `io_utils/`

共享数据接口：

- `io_utils/write_target_json.py`：生成并写入 `shared/target.json`。
- `io_utils/replay_matlab_traj.py`：轻量检查 `shared/q_traj.csv` 的占位/调试脚本。

## 推荐运行命令

### 视觉检测并写入目标

```powershell
python -m python.vision.run_color_pipeline --gui
```

无界面：

```powershell
python -m python.vision.run_color_pipeline --nogui
```

使用已有图片：

```powershell
python -m python.vision.run_color_pipeline --image .\data\screenshots\camera_rgb.png
```

输出：

```text
data/screenshots/camera_rgb.png
data/screenshots/color_detection_annotated.png
shared/target.json
```

### MATLAB 轨迹 + PyBullet 力检测

先在 MATLAB 运行 `main_sim.m` 或 `main_hit_from_vision.m`，生成：

```text
shared/q_traj.csv
shared/result.json
```

然后在 `robotic1` 环境运行：

```powershell
python -m python.simulation.replay_matlab_force_hit --mode robot
```

如果只想快速无界面验证：

```powershell
python -m python.simulation.replay_matlab_force_hit --nogui --fast --mode robot --physics-hz 500 --log-hz 100 --extra-time 0.1
```

如果 UR5 模型路径需要手动指定：

```powershell
python -m python.simulation.replay_matlab_force_hit --mode robot --robot-urdf .\data\urdf\ur5\ur5_robot.urdf
```

输出：

```text
results/data/pybullet_force_log.csv
shared/pybullet_force_result.json
```

## 导入方式

新代码直接从子包导入，例如：

```python
from python.common.config import BOARD
from python.vision.detect_color_target import detect_red_target
from python.simulation.pybullet_scene import create_scene
```

## 注意事项

- PyBullet 在 Windows 中文路径下加载 URDF 可能乱码，`replay_matlab_force_hit.py` 已经把本地 URDF 转成相对路径传给 `loadURDF()`。
- `data/urdf/ur5/ur5_robot.urdf` 与 MATLAB `loadrobot("universalUR5")` 使用同一 UR5 Xacro 运动学来源，详见 `docs/ur5_urdf_source_check.md`。
- `shared/result.json` 中的 `target_world` 会被力检测回放优先使用；没有该文件时可以用 `--target-world X Y Z` 手动指定目标点。
