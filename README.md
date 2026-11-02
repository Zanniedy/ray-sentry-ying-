# ray-sentry-ying-
to a girl

哨兵导航与 Gazebo（Ignition Fortress）仿真工作空间，ROS 2 Humble。

## 目录
- `src/cs_robot_description`（包名 `rm_9_2`）：哨兵模型，云台可折叠（Z ⇄ 一），碰撞体为基础形状
- `src/cs_interface`：`ChassisCmd.srv`（`vx vy w fold`）
- `src/mid360_driver`：Mid-360 驱动
- `dependencies.repos`：第三方包（rmoss、rmu_gazebo_simulator 等），固定 commit
- `docs/img`：云台两种状态示意图

## 构建
```bash
git lfs pull                       # 网格文件用 Git LFS 存储
vcs import src < dependencies.repos
source ./make.sh
```

## 控制
```bash
ros2 service call /chassis_cmd cs_interface/srv/ChassisCmd "{vx: 1.0, vy: 0.0, w: 0.0, fold: false}"
```
