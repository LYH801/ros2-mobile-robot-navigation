# ROS2 移动机器人导航与视觉停靠系统

基于 ROS2 Humble + Nav2 框架，独立实现移动机器人自主导航与视觉精确停靠系统。

## 项目简介

本项目在 ROS2 Nav2 框架基础上，通过插件机制替换默认的全局规划器和控制器，用自研算法实现自主导航，并集成基于 ArUco 标记的视觉停靠系统。用户可在 RViz 中一键触发从远处导航到精确定位停靠的完整任务。

## 核心功能

| 模块 | 说明 |
|---|---|
| **自研 A* 全局规划器** | 8 邻域搜索 + 禁止穿角 + 贝塞尔路径平滑 |
| **自研 Pure Pursuit 控制器** | 预瞄距离自适应 + 接近减速 + 大角度降速 |
| **视觉停靠系统** | ArUco 检测 + PnP 位姿估计 + TF + 五状态状态机 |
| **任务管理器** | RViz 一键触发完整任务，自动切换控制权 |

## 实验数据

### 导航实验（15 次）

| 算法 | 成功率 | 平均耗时 |
|---|---|---|
| NavFn + DWB（基线） | 80% | 17.7s |
| **A* + Pure Pursuit（自研）** | **100%** | **9.9s** |

### 视觉停靠实验（10 次）

| 指标 | 值 |
|---|---|
| 成功率 | **100%** |
| 平均耗时 | **5.8s** |

### 规划器对比

| 指标 | 自研 A* | NavFn |
|---|---|---|
| 规划时间 | 11.3ms | 27.4ms |
| 平均转折数 | 26.1 | 10.1 |

## 环境要求

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Classic
- Python 3.10
- OpenCV 4.5+

## 安装

### 1. 安装依赖

```bash
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-turtlebot3 \
  ros-humble-turtlebot3-gazebo \
  ros-humble-cv-bridge \
  ros-humble-tf2-geometry-msgs \
  python3-opencv
