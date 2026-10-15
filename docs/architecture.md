# 系统架构

## 数据流

用户 (RViz)
│ 点 Nav2 Goal
▼
mission_manager（任务调度）
│
├── 阶段 1：导航
│ └── Nav2
│ ├── 自研 A* 规划器
│ └── 自研 Pure Pursuit 控制器
│
└── 阶段 2：视觉停靠
└── docking_node
├── ArUco 检测
├── PnP 位姿估计
├── TF 转换
└── 五状态状态机
│
▼ /cmd_vel
Gazebo 机器人

## 模块职责

| 模块 | 职责 | 类型 |
|---|---|---|
| astar_planner | 全局路径规划 | 自研插件 |
| pursuit_controller | 路径跟踪控制 | 自研插件 |
| vision_docking | ArUco 检测 + 视觉停靠 | 自研节点 |
| mission_manager | 任务调度、控制权切换 | 自研节点 |
| robot_bringup | 一键启动 | 启动包 |

## 控制权切换

同一时间只有一个节点发布 `/cmd_vel`：

| 阶段 | 控制权 |
|---|---|
| 导航 | pursuit_controller |
| 视觉停靠 | docking_node |

mission_manager 通过 `/docking_start` 触发切换。

## 核心话题

| 话题 | 类型 | 发布者 | 订阅者 |
|---|---|---|---|
| /goal_pose | PoseStamped | RViz | mission_manager |
| /plan | Path | planner_server | controller_server |
| /cmd_vel | Twist | controller_server / docking_node | Gazebo |
| /docking_status | String | docking_node | mission_manager |
| /camera/image_raw | Image | Gazebo | docking_node |

## 关键实现

### A* 规划器
- 8 邻域搜索 + 禁止穿角
- 下采样 + 贝塞尔平滑
- 代价地图膨胀

### Pure Pursuit
- 预瞄距离自适应
- 曲率 κ = 2·Ly / Ld²
- 接近减速 + 大角度降速

### 视觉停靠
- ArUco 检测 4 个角点
- PnP 求标记相对相机位姿
- TF 转换到 base_link
- 图像宽度保护作为独立停车判据
