# ada-localization

面向 RoboMaster 哨兵的 **Adaptive-LIO 松耦合里程计 + 独立 T-DT 因子图后端**，当前验证环境为 Ubuntu 24.04 / ROS 2 Jazzy，包含 Mid-360 专用配置、ScanContext 裁剪前点云接口、后端修复补丁和数据验证记录。

本仓库延续 [ECUST-Origin-Sentry-Team/Adaptive-LIO](https://github.com/ECUST-Origin-Sentry-Team/Adaptive-LIO) 的工程改造，独立后端来自 [mozihe/z-localization](https://github.com/mozihe/z-localization)。**没有切换为上游 ZLIO 紧耦合前端。** 当前仓库名为 `ada-localization`，ROS 包名仍为 `adaptive_lio`。

## 当前实现

```mermaid
flowchart LR
    L[Mid-360 LiDAR] --> F[Adaptive-LIO 松耦合 ESKF]
    I[IMU] --> F
    F --> O[高频局部 /odom]
    F --> S[同一帧末位姿和未作导航高度裁剪的点云]
    S --> B[T-DT legacy 后端]
    O --> B
    B --> C[ScanContext 检索 / 几何验证 / PCM / NIS]
    C --> G[因子图 / map 到 odom / 优化关键帧轨迹]
    F --> N[导航点云 Z 和车体裁剪 / livox/scan]
```

- 后端接口为 `/adaptive_lio/backend/odometry` 和 `/adaptive_lio/backend/cloud`，采用相同帧末时间戳；高频 `/odom` 独立输出。
- 原 ScanContext 回调和后端点云入口位于导航高度、车体裁剪之前，避免世界 Z 裁剪破坏描述子。此处点云仍经过前端采样和去畸变，并非原始全分辨率点云。
- Mid-360 前端使用几何自适应激光观测协方差；原 `mapping_m.yaml` 的默认配置保留。
- 后端补丁增加可选重访配准初值、起点历史邻域，并修复历史 PCM 支持数为 0 时被归一化强制改成 1 的问题。
- Mid-360 专用后端采用 Huber 核、至少三个当前一致候选和完整六维 NIS；禁用不适用的固定世界 Z 残差与历史 PCM 支持要求。其他配置不自动启用这些专用策略。

后端源码通过**固定上游提交 + 本仓库补丁**交付，不包含第二份完整后端树。补丁及应用说明见 [tools/patches](tools/patches/README.md)。仅构建本仓库的前端不会自动安装后端。

## 已验证结果及口径

SEU library Mid-360 原包约 429.83 秒，保留消息与原始时间戳，以 1 倍速回放。最终完整运行观察器收到 85,965 条 IMU，无停链，相关进程退出码均为 0。

| 统计对象 | 3D 首尾距离 |
|---|---:|
| 原配置在线后端消息 | 32.083 m |
| 最终配置在线前端消息 | 6.027 m |
| 最终配置在线后端消息 | 6.029 m |
| **最终保存的优化关键帧轨迹** | **0.161 m** |
| 原始首尾点云独立配准估计 | 约 0.16～0.17 m |

最终接受了关键帧 `762 → 5` 的跨整圈约束，6 个候选形成一致团，NIS 概率 `0.0178481` 通过 `0.001` 门限。最后的闭环优化发生在最后一条 IMU 之后，因此最后在线消息保留修正前值；历史在线消息不会被回溯重写。**0.161 m 是优化轨迹闭合距离，不是在线末消息偏差，也不是真值 ATE。** 数据没有外部真值，不能由单包闭合推导全场景精度或稳定性结论。

此前验证通过前端 5 组 CTest、后端 80 个用例；这些是对应修复版本的测试记录，本次仓库整理没有重新完整回放。原始 bag、构建产物、Docker 镜像不随仓库分发。

- [完整修复报告、失败对照和限制](docs/validation/mid360-fix-report.md)
- [最终指标 JSON](docs/validation/mid360-fixed-results.json) · [优化关键帧](docs/validation/mid360-fixed-keyframes.txt) · [关键日志](docs/validation/mid360-fixed-evidence.log)
- [原始首尾点云独立检查](docs/validation/seu-endpoint-check.md)
- [Avia quick-shack 验证与输入时间故障](docs/validation/quick-shack-report.md)

![轨迹对照，灰色为在线历史消息，蓝色为最终优化关键帧](docs/validation/mid360-fixed-trajectories.png)

## 构建及运行联合定位

先按下文“前端工程说明”的依赖和编译步骤构建 `adaptive_lio`。在新终端中准备独立后端：

```bash
# 在 ada-localization 仓库根目录执行；两个仓库放在同级目录。
ADA_ROOT="$(pwd)"
git clone https://github.com/mozihe/z-localization.git ../z-localization
git -C ../z-localization checkout 76e126328b670adc92981d9b4900cc059a6f2343
git -C ../z-localization apply --check "$ADA_ROOT/tools/patches/z-localization-adaptive-lio.patch"
git -C ../z-localization apply "$ADA_ROOT/tools/patches/z-localization-adaptive-lio.patch"
```

后端除 ROS/PCL/Eigen/TBB 依赖外，需要 **GTSAM 4.2、nanoflann 1.4.3、libmetis-dev**。本次 Jazzy 环境曾遇到系统 nanoflann 1.5.4 接口不兼容，以及 GTSAM 包导出 CppUnitLite 但缺少库的问题；后者通过编译 GTSAM 4.2 对应 CppUnitLite 源码解决，没有删除测试或使用空库。详见 [集成说明](docs/tdt-integration.md)。这不是已验证的全新机器一键安装脚本。

在上述依赖已安装后，构建独立后端：

```bash
source /opt/ros/jazzy/setup.bash
cd ../z-localization
rosdep install --from-paths src/backend src/localization_interfaces \
  --ignore-src -r -y --rosdistro jazzy
# 指向包含 1.4.3 版 nanoflann.hpp 的实际目录
NANOFLANN_INCLUDE_DIR=/absolute/path/to/nanoflann-1.4.3/include
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build \
  --packages-select localization_interfaces backend \
  --cmake-args -DBUILD_TESTING=ON -DNANOFLANN_INCLUDE_DIR="$NANOFLANN_INCLUDE_DIR"
source install/setup.bash
ctest --test-dir build/backend \
  -R '^backend_(descriptor|relocalization|localization|graph|measurement|lifecycle)_test$' \
  --output-on-failure
```

**启动前调整路径：** 当前受测 YAML 保留云环境绝对路径。将 `config/tdt_backend.yaml` 的 `extends` 指向本机后端 `config/backend.yaml`，将 `config/tdt_backend_mid360.yaml` 的 `extends` 指向本机 `tdt_backend.yaml`，并调整 `trajectory.save_directory`。配置位于 `src/Adaptive-LIO/config/`。`tools/build_tdt_backend.sh` 与回放工具也包含云环境路径，不能原样当作通用部署入口。

在同一个终端 source ROS、前端和后端的安装空间，然后启动：

```bash
source /opt/ros/jazzy/setup.bash
source /absolute/path/to/ada-localization/install/setup.bash
source /absolute/path/to/z-localization/install/setup.bash
ros2 launch adaptive_lio tdt_backend.launch.py \
  frontend_config:=/absolute/path/to/ada-localization/src/Adaptive-LIO/config/mapping_mid360.yaml \
  backend_config:=/absolute/path/to/ada-localization/src/Adaptive-LIO/config/tdt_backend_mid360.yaml \
  use_sim_time:=true
# 在另一已 source ROS 的终端播放 ROS2 包：
ros2 bag play /absolute/path/to/ros2_bag --clock --rate 1
```

实机运行将 `use_sim_time` 设为 `false`，另行启动雷达驱动。检查 `/livox/lidar`、`/livox/imu`、消息类型、点时间、外参和加速度单位：专用配置的 `gnorm=9.805` 假设输入加速度单位为 g；已经是 m/s² 时不能再次这样缩放。此联合入口默认 mapping，未启用 UWB/先验地图；不是 Nav2 或底盘控制系统。

复算最终保存轨迹指标（Python 环境需要 numpy）：

```bash
python tools/summarize_tdt_replay.py /path/to/replay_output \
  --optimized-keyframes docs/validation/mid360-fixed-keyframes.txt
```

`/path/to/replay_output` 是含 metrics.json 和前后端日志的回放输出目录，不是 bag 目录；全部原始运行输出未打包进 Git。仓库已保存汇总和小型证据。

## 遗留问题与部署边界

| 问题 | 当前状态 / 影响 |
|---|---|
| IMU 时间故障 | 曾出现一次 `TimeFault(reason=8)` 并停链。独立观察器同段时间戳连续，前端实际接收序列未逐条记录，传输/调度原因尚未定位。没有关闭故障保护，不能宣布长期稳定性通过。 |
| 运动起步初始化 | 静态初始化逻辑未重写；移动起步可能导致误判或延迟初始化，不同运行的首条定位时间可能不同。尚未实现可靠的动态初始化。 |
| 世界 Z 与重力方向 | 初始世界 Z 不保证重力竖直。导航固定世界 Z 裁剪仍有语义限制；将描述子入口放到裁剪前并未解决重力坐标系本身。 |
| 尾部异步优化与在线输出 | bag 结束后完成的闭环可能尚未反映到最后一条在线定位消息。最终优化轨迹必须单独保存和评估，不能把两个口径混用。 |
| 真值与跨场景鲁棒性 | SEU library 无外部真值；未完成其他场景、重复走廊、升降坡道、动态遮挡和长期实车验证，不能报告未经验证的 ATE/RPE 或普适参数。 |
| 标定与统计门限 | 外参、驱动是否预变换、加速度单位需要逐设备核实。自适应协方差、世界 Z/历史 PCM 策略和 Huber 配置为当前实验配置，未完成跨设备统计标定。 |
| 重置和会话 | 当前使用 legacy 接口，未实现 ZLIO 原子会话/暖重置协议；故障或重新播放应一起重启前后端。 |
| 性能与实时性 | 最终回放在线输出最大时间戳间隔约 334 ms，未完成实时延迟预算验收。回放无崩溃不代表控制系统可直接使用。 |
| 可移植部署 | YAML、工具包含 `/workspace/Adaptive-LIO` 等绝对路径；后端和依赖需另行准备，尚非全新机器一键构建。 |

## 来源、许可与工程说明

前端沿用原仓库历史及各组件许可声明；独立后端及其补丁遵循上游 `GPL-2.0-only`。第三方代码保留各自声明，不以新仓库名替换上游归属。原论文引用和前端内部设计见下文。

以下保留并更新原前端工程文档。其“默认配置”指单独运行的 `mapping_m.yaml`，不等同于上文显式选择的 Mid-360 联合定位配置。

---

## 前端工程说明


**面向 RoboMaster 哨兵机器人的 LiDAR–IMU 里程计 | ROS 2 Jazzy**

[![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-22314E)](https://docs.ros.org/en/jazzy/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-E95420)](https://ubuntu.com/)
[![C++](https://img.shields.io/badge/C%2B%2B-17-00599C)](https://isocpp.org/)
[![Original Paper](https://img.shields.io/badge/Paper-Adaptive--LIO-004088)](https://ieeexplore.ieee.org/document/10806842)

> 本仓库是基于 [原版 Adaptive-LIO](https://github.com/chengwei0427/Adaptive-LIO) 的工程化改造版本，主要用于 ECUST Origin Sentry 的 LiDAR–IMU 里程计开发与独立验证。**前端不是原论文的逐行复现；本仓库通过上文独立后端提供闭环定位集成，不包含完整导航或底盘控制系统。**
>
> 本仓库以 `main` 发布；原前端开发基线来自 `ljb`，下文默认参数指前端独立配置。

## 项目概览

该系统以 **LiDAR 几何观测约束漂移、IMU 高频传播提供及时运动状态** 为核心思路，采用内外双层 ESKF 数据流：

- **内层 ESKF（权威后验）**：IMU 传播 → 点云去畸变 → Ceres 帧末位姿配准 → 观测质量门控 → ESKF 校正；只有被接受的观测才能更新地图。
- **外层 ESKF（高频输出）**：安装内层提供的完整状态与协方差快照，重放快照之后的 IMU 历史，为 `/odom` 和 TF 提供连续状态。
- **点与时间语义统一**：使用传感器时间；逐点去畸变到当前 LiDAR 帧末 IMU 坐标；Ceres 只估计帧末位姿，不重复执行旧式插值补偿。
- **工程诊断**：保留几何退化分析、完整 6×6 观测协方差实验路径、IMU 质量检查，以及分阶段耗时和逐帧处理耗时统计。

**当前默认状态**：纯 LiDAR + IMU；主雷达工作，副雷达融合关闭；固定 LiDAR 观测协方差；ScanContext 关键帧地图模块关闭；异常 IMU 的短时保持关闭。实验选项存在不等于已完成真实数据标定或实车验收。

### 系统数据流

下图按照实际运行线程分为三个部分：**传感器接入与同步、内层 LiDAR–IMU 融合、外层高频 IMU 里程计输出**。实线表示默认数据路径，虚线表示可选的副雷达路径。内层的 LiDAR 更新与外层的 IMU 传播并行进行，并不是逐帧串行等待地图发布。

```mermaid
flowchart TB
    subgraph INPUT["① 传感器接入与质量检查"]
        direction LR
        L0["主 LiDAR / CustomMsg<br/>帧时间戳 + 逐点 offset_time"] --> L1["格式转换与点时间解析<br/>随机降采样 / 主雷达入队"]
        I0["IMU / sensor_msgs/Imu<br/>角速度 + 加速度"] --> I1["gnorm 单位换算<br/>时间有效性 / 饱和与故障检查"]
        I1 --> IQ{"IMU 质量判定"}
        IQ -->|可用| I2["统一处理后的 IMU<br/>数值 + 质量掩码 / 噪声尺度"]
        IQ -->|过期或重复| DROP["丢弃该输入"]
        IQ -->|不可恢复故障| FAULT["两层滤波停止<br/>需要重启节点"]
        A0["副 LiDAR / 可选订阅<br/>默认关闭"] -.-> A1["时间偏移校正 / 裁剪到主帧<br/>外参 T_main_aux 变换"]
    end

    subgraph INNER["② 内层 ESKF：低频 LiDAR 约束与权威后验"]
        direction TB
        SYNC["主帧 [t0, t1] + IMU 队列<br/>等待 IMU 覆盖帧末及 delay_time"] --> INIT{"静态 IMU 初始化完成？"}
        INIT -->|否| INITWAIT["TryInitIMU<br/>完成初始化前不执行配准"]
        INIT -->|是| PROP["内层 ESKF 传播到 t1<br/>记录帧内 IMU 预测位姿轨迹"]
        PROP --> DESKEW["冻结预测轨迹 / 逐点去畸变<br/>原始采样时刻 → 帧末 IMU 坐标"]
        DESKEW --> FRAME["构建帧末工作点<br/>有效时间覆盖筛选 / 点云建帧"]
        FRAME --> SEED{"局部体素地图为空？"}
        SEED -->|是| SEED_CHECK{"有效预测且点数足够？"}
        SEED_CHECK -->|是| SEED_OK["种子帧：不构造 LiDAR 观测<br/>用有效预测状态初始化地图"]
        SEED_CHECK -->|否| REJECT["拒绝本帧<br/>保留内层传播先验"]
        SEED -->|否| MATCH["局部体素地图邻域搜索<br/>空间 / 法向分桶选择点到平面残差"]
        MATCH --> CERES["Ceres 优化帧末 SE(3) 候选位姿<br/>不重复插值运动补偿"]
        CERES --> GEOM["最终对应点几何诊断<br/>信息谱 / 有效秩 / 弱约束方向"]
        GEOM --> R["构造 LiDAR 观测协方差 R<br/>默认固定 0.01 I6 / 自适应实验选项"]
        R --> GATE{"支持数 / 匹配质量 / 创新门控<br/>平移与旋转硬阈值 + 创新分数"}
        GATE -->|通过| UPDATE["ObserveSE3：内层 ESKF 校正<br/>状态与协方差更新"]
        GATE -->|未通过| REJECT
        UPDATE -->|校正成功| ACCEPT["接受本帧后验<br/>推进 last_accepted 时间"]
        UPDATE -->|状态校正失败| REJECT
        SEED_OK --> ACCEPT
        ACCEPT --> COMMIT["从内层 ESKF 导出权威状态<br/>提交 p / R / v / 零偏等派生状态"]
        REJECT --> COMMIT
        COMMIT --> SNAP["完整值快照：导航状态、重力<br/>18×18 P、传播参数与 Q / 序号"]
        SNAP --> PRODUCTS{"本帧是否接受？"}
        PRODUCTS -->|接受或种子帧| WORLD["以后验位姿重新计算世界点<br/>非 Ceres 候选位姿直接入图"]
        PRODUCTS -->|拒绝| SKIP["不更新地图 / 关键帧<br/>不发布该帧注册点云"]
        WORLD --> MAP["局部体素地图增量维护<br/>视野裁剪 / 可选关键帧地图"]
        MAP --> CLOUD["注册点云经异步队列发布<br/>/livox/scan"]
    end

    subgraph OUTER["③ 外层 ESKF：独立 IMU 重放与高频输出"]
        direction TB
        HISTORY["保留同一质量判定结果的 IMU 历史<br/>不在重放时重新评估质量"] --> REPLAY["有新快照时恢复完整滤波状态<br/>连续传播 / 重放快照后的 IMU"]
        SNAP --> REPLAY
        REPLAY --> CHECK{"状态有效 / 时间单调<br/>距最近接受帧未超时？"}
        CHECK -->|是| PROJECT["将外层状态投影为 Odometry<br/>位姿 / 机体系速度 / 协方差"]
        CHECK -->|否| HOLD["不发布过期结果<br/>不伪造时间戳或重复 TF"]
        PROJECT --> ODOM["/odom<br/>nav_msgs/Odometry"]
        PROJECT --> TF["/tf<br/>odom → aft_mapped → base_link"]
        PROJECT --> PATH["/odometry_path<br/>限长且按条件发布"]
    end

    L1 --> SYNC
    I2 -->|内层 IMU 队列| SYNC
    I2 -->|外层 IMU 输入队列| HISTORY
    A1 -.->|开启 bundle_fusion 时合并到主帧点集| DESKEW
    A1 -.->|关闭融合但已订阅：只在接受帧追加发布| CLOUD
```

**读图提示：**种子帧只利用已验证的 IMU 预测状态初始化地图，不执行 `ObserveSE3`；匹配拒绝帧依然向外层发送内层传播状态的完整快照，但**不更新地图，也不发布该帧的注册点云**。外层只从已安装的权威快照向后进行 IMU 传播，不会对同一 LiDAR 候选观测做第二次融合。`/odom`、TF 和 Path 来自外层；局部地图与 `/livox/scan` 来自内层。副雷达相关虚线在默认配置下不激活。

## 快速开始

### 1. 环境

当前独立构建基线为 **Ubuntu 24.04 + ROS 2 Jazzy + C++17**，使用 Eigen、PCL、Ceres 2.x、glog、gflags、yaml-cpp 等。需要先按 [ROS 2 Jazzy 安装文档](https://docs.ros.org/en/jazzy/Installation.html) 配置 ROS 环境。

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git python3-colcon-common-extensions python3-rosdep \
  libeigen3-dev libpcl-dev libceres-dev libgoogle-glog-dev libgflags-dev \
  libyaml-cpp-dev libopencv-dev libsuitesparse-dev \
  ros-jazzy-ament-cmake-auto ros-jazzy-pcl-conversions \
  ros-jazzy-tf2-geometry-msgs ros-jazzy-message-filters \
  ros-jazzy-rosbag2 ros-jazzy-rosbag2-storage-mcap
```

仓库 `src/livox_ros_driver2/` 提供 `livox_ros_driver2` 消息类型和驱动源码；**回放已有 rosbag 时不需要启动硬件驱动**。在目标机器上还需要匹配相应架构的 Livox SDK 库。

### 2. 获取并编译

```bash
git clone https://github.com/movoQAQ/ada-localization.git adalio_ws
cd adalio_ws
source /opt/ros/jazzy/setup.bash

# 已初始化 rosdep 的机器：
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy

CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --symlink-install \
  --packages-up-to adaptive_lio --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

首次使用 `rosdep` 但尚未初始化时，先运行 `sudo rosdep init`。构建建议在**未 source 其他 ROS 工作空间**的新终端中进行，防止同名依赖和消息定义冲突。`--packages-up-to` 会连同需要的 `livox_ros_driver2` 一起构建。

### 3. 实时运行

先确认传感器驱动正在发布 YAML 中配置的话题，然后在已 source 的终端执行：

```bash
ros2 launch adaptive_lio run.launch.py
```

修改配置时可以指定绝对路径：

```bash
ros2 launch adaptive_lio run.launch.py \
  adaptive_lio_cfg_dir:="$(pwd)/src/Adaptive-LIO/config/mapping_m.yaml"
```

注意：当前 `run.launch.py` **只启动 Adaptive-LIO 节点**，不自动启动 Livox 驱动或 RViz；即使声明了 `rviz` 参数，RViz 节点也没有添加到 LaunchDescription。需要可视化时另行运行 `rviz2`。

### 4. 录制数据回放

仓库**不包含**原始 rosbag。根目录脚本默认面向 163 号主雷达的点云与 IMU：

| 终端 | 命令 | 功能 |
|---|---|---|
| A | `./start_bag_odometry.sh` | 启动 LIO 与配置好的 RViz（RViz 使用仿真时间） |
| B | `./play_bag_163.sh /absolute/path/to/rosbag_directory` | 以 1 倍速回放主雷达与 IMU，并发布 `/clock` |

如果数据放在仓库根目录下的 `rosbag2_2026_07_07-16_37_53/`，终端 B 可以省略路径。脚本会检查 `metadata.yaml` 是否存在；输入数据至少需要含有：

```text
/livox/lidar_192_168_1_163
/livox/imu_192_168_1_163
```

更换设备或 bag 时，**同时检查** `config/mapping_m.yaml` 中的话题及 `play_bag_163.sh` 中的 `--topics` 列表。播放前用 `ros2 bag info /path/to/bag` 确认消息类型与时间范围。初始化阶段需有足够有效 IMU 数据；正式工作后日志会出现 `init successful`。

## ROS 接口和坐标系

| 方向 | 接口 | 类型 / 说明 |
|---|---|---|
| 订阅 | `common.lid_topic` | 主 LiDAR 点云；默认 `/livox/lidar_192_168_1_163` |
| 订阅 | `common.imu_topic` | IMU；默认 `/livox/imu_192_168_1_163` |
| 发布 | `/odom` | `nav_msgs/msg/Odometry`；外层 IMU 重放后的状态，包含位姿、速度相关信息与协方差 |
| 发布 | `/tf` | `odom → aft_mapped → base_link` |
| 发布 | `/odometry_path` | `nav_msgs/msg/Path`；有限长度历史轨迹 |
| 发布 | `/livox/scan` | `sensor_msgs/msg/PointCloud2`；被接受或初始化种子帧对应的注册点云 |
| 发布 | `/velocity`、`/move_dist` | `std_msgs/msg/Float32`；保留的标量诊断输出 |
| 服务 | `/save_map_service` | `std_srvs/srv/Trigger`；与可选地图模块关联 |

**坐标系职责**：`odom` 为本地连续里程计参考系；`aft_mapped` 是里程计跟踪的 IMU 相关状态坐标系；`base_link` 通过配置外参连接到车体。默认外参在 `common.fusion_aft_to_base_xyzrpy`，需按传感器安装和整车 TF 规划重新标定。**前端本包不发布 `map → odom`**；联合入口由独立后端负责该变换。单独接入 Nav2、GICP 重定位或其他全局定位器时，必须由外部模块负责该变换，不应额外重复发布 `odom → base_link`。

基础检查命令：

```bash
ros2 topic hz /odom
ros2 topic echo /odom --once
ros2 topic hz /livox/scan
ros2 run tf2_ros tf2_echo odom aft_mapped
```

## 核心算法与状态一致性

### LiDAR 帧处理

1. 从主雷达消息生成点时间，保留原始点与采样时间；使用 **传感器时间**，而非回调到达的主机时间，计算运动补偿。
2. IMU 完成对应时间窗口的预测后，将点去畸变到**当前帧末 IMU 坐标系**；时间未覆盖或无效的点明确剔除。
3. 依据局部地图构建点到平面残差，Ceres 优化**帧末 6 自由度候选位姿**；按空间与法向分桶采样，避免仅依赖输入点顺序。
4. 根据支持数量、几何约束、创新等判定候选是否可靠。通过门控的候选作为 SE(3) 观测校正内层 ESKF，并用已提交的后验刷新世界点；失败则保留传播先验，跳过本帧地图产品。
5. 将完整内层快照发送给外层，包括导航状态、重力、18×18 协方差、传播配置和噪声信息；外层按同一 IMU 质量策略重放后续历史以产生及时输出。

这套设计**不是滑窗优化或全局因子图**。ScanContext 代码仍在仓库中，但默认不启动其地图工作线程，也不等于当前已经启用回环校正或全局重定位。

### IMU 质量与协方差

IMU 数据进入系统时统一进行单位换算和有效性检查。默认 `imu_quality.hold_seconds: 0`：检测到不可恢复的无效测量或已配置量程下的饱和故障时，系统采取保守的故障停止策略；短时轴保持是实验选项，**未经过具体传感器标定，不应直接在实车启用**。

LiDAR 观测协方差默认 `odometry.lidar_covariance_mode: fixed`。`adaptive` 几何退化感知模式已实现，但尺度、相关性膨胀和接受率尚需要独立数据校准。任何合成测试结果都不应等同于已证明实际里程计精度提升。

## 配置指南

默认配置：[`src/Adaptive-LIO/config/mapping_m.yaml`](src/Adaptive-LIO/config/mapping_m.yaml)；低等待时间对照：[`mapping_m_low_delay.yaml`](src/Adaptive-LIO/config/mapping_m_low_delay.yaml)。两者主要差别是 `delay_time`（分别为 **0.05 s** 与 **0.0 s**）。

| 配置键 | 默认值 | 含义 / 使用提示 |
|---|---|---|
| `common.lid_topic` / `common.imu_topic` | 163 设备话题 | 首先核对实际输入，设备不同必须修改 |
| `common.gnorm` | `9.805` | 按当前数据的 g 单位加速度换算为 m/s²；不要对已为 m/s² 的数据重复缩放 |
| `common.fusion_aft_to_base_xyzrpy` | 仓库预设外参 | `aft_mapped → base_link` 的安装变换；换车必须重检 |
| `preprocess.sweep_cut_num` | `1` | 当前测试按主雷达完整消息处理 |
| `delay_time` | `0.05` s | 等待 IMU 覆盖后的额外时间策略；`0.0` 需同包实测对照 |
| `odometry.point_time_model` | `IMU_DESKEW_END` | 唯一有效的当前点时间处理路径 |
| `odometry.lidar_covariance_mode` | `fixed` | `adaptive` 仍属于未校准实验配置 |
| `odometry.max_num_residuals` | `500` | Ceres 残差预算上限 |
| `aux_lidar.enable_bundle_fusion` | `false` | 主副雷达联合匹配默认关闭，副雷达话题默认不订阅 |
| `mapping_module.enable_mapping` | `false` | 默认关闭可选 ScanContext/关键帧地图模块 |
| `imu_quality.hold_seconds` | `0.0` | 默认不做异常 IMU 轴保持 |

旧版 `icpmodel`、`motion_compensation`、`point_to_plane_with_distortion` 等不再是可切换的有效运行配置。对照旧工程迁移时，应以当前 YAML 和 [`point_state_contract.md`](src/Adaptive-LIO/docs/point_state_contract.md) 为准，不要直接复制早期参数。

## 代码结构

```text
.
├── README.md                     # 本文档
├── start_bag_odometry.sh         # LIO + RViz 跑包入口
├── play_bag_163.sh               # 163 主雷达数据回放
├── bag_odometry.rviz             # 可视化配置
└── src/
    ├── Adaptive-LIO/
    │   ├── config/               # 传感器与算法参数
    │   ├── launch/               # ROS 2 启动文件
    │   ├── src/
    │   │   ├── apps/             # ROS 订阅、发布和高频输出
    │   │   ├── preprocess/       # 点云转换、逐点时间
    │   │   ├── lio/              # 配准、滤波状态提交与局部地图
    │   │   ├── algo/             # ESKF、IMU 质量、几何门控、去畸变
    │   │   ├── scantext_module/  # 可选 ScanContext 相关模块
    │   │   └── common/           # 配置、地图与性能统计
    │   ├── test/                 # 确定性测试与回放检查工具
    │   ├── docs/                 # 算法约定、改造与分析报告
    │   └── validation/           # 构建、回归与历史验证记录
    └── livox_ros_driver2/        # Livox 消息与驱动依赖
```

建议从 [`main_ada.cpp`](src/Adaptive-LIO/src/apps/main_ada.cpp) 阅读节点接口，然后看 [`lidarodom.cpp`](src/Adaptive-LIO/src/lio/lidarodom.cpp) 的帧处理事务，接着读 [`eskf.hpp`](src/Adaptive-LIO/src/algo/eskf.hpp)、[`imu_odom_predictor.h`](src/Adaptive-LIO/src/algo/imu_odom_predictor.h) 及 [`imu_odom_output.h`](src/Adaptive-LIO/src/apps/imu_odom_output.h)。

## 测试与验证边界

构建时使用 `-DBUILD_TESTING=ON`，然后运行：

```bash
ctest --test-dir build/adaptive_lio \
  -R '^adaptive_lio_(core|geometry|registration|pipeline|imu_quality)$' \
  --output-on-failure
```

五组测试覆盖状态快照与 IMU 重放、几何信息矩阵、Ceres 因子和去畸变、种子/接受/拒绝事务、IMU 质量与故障边界。若需要脱离 rosbag 的源码级检查，也可运行：

```bash
bash src/Adaptive-LIO/test/run_core_tests.sh
bash src/Adaptive-LIO/test/run_registration_tests.sh
```

仓库中存有 **2026-10-10 改造阶段** 的验证记录：当时最终 R6 代码通过编译和五组确定性 CTest，但**最终 R6 版本没有继续执行完整 bag/实车验收**。`validation/` 中的历史回放结果和计时不能直接当成当前任何新参数或代码修改后的性能结论。详见 [论文对照改造报告](src/Adaptive-LIO/docs/paper_alignment_report_2026-10-10.md)。

### 性能日志如何理解

`frame_callback_to_done` 是主 LiDAR 回调进入后至 LIO 处理返回的单帧墙钟统计，包含点转换、队列等待和 IMU 同步等待；**不包含回调前传输及回调返回后的异步发布**。`processMeasurement` 是 LIO 处理内部耗时。`predict` / `optimize` 等阶段的样本数可能不同，**不能直接相加各阶段平均值作为一帧总耗时**。回放和设备的传感器时间用于状态估计；耗时计时使用主机单调时钟，二者不可混作延迟测量。

遇到 `deskew_uncovered_or_invalid=N` 时，表示该帧有 N 个点因时间覆盖不足或无效被丢弃，**不等价于整帧配准失败**；还需结合 `lio_quality` 中的接受状态和原因判断。

## 已知限制

- 前端独立运行提供局部里程计；联合入口增加独立后端的闭环图优化和 map→odom，仍不包含 Nav2 整套导航。
- 主副雷达联合匹配、ScanContext 地图、自适应 LiDAR 协方差、IMU 短时保持均不是默认工作配置。
- 当前外参与 IMU 单位换算针对特定设备和驱动发布约定。移植时须核对**原始消息语义**，不可直接复用数值。
- 通过单元测试与 RViz 轨迹连续，不等于完成精度、退化鲁棒性、延迟预算或实车安全性验收。
- 原始数据集不随仓库分发；已有公开数据回放和闭合对照，但缺少真值精度与跨场景验收，因此不作性能领先声明。

## 技术文档

深入阅读可参考 [点与状态契约](src/Adaptive-LIO/docs/point_state_contract.md)、[几何与观测协方差](src/Adaptive-LIO/docs/geometry_covariance.md)、[IMU 质量与重放](src/Adaptive-LIO/docs/imu_quality.md)、[完整改造报告](src/Adaptive-LIO/docs/paper_alignment_report_2026-10-10.md)、[测试说明](src/Adaptive-LIO/test/README.md)。

## 上游与致谢

本项目在原版 **Adaptive-LIO: Enhancing Robustness and Precision through Environmental Adaptation in LiDAR Inertial Odometry** 基础上进行工程化开发。原版作者、论文、历史实验与原始开源许可应以[上游仓库](https://github.com/chengwei0427/Adaptive-LIO)和本项目的 [`src/Adaptive-LIO/README.md`](src/Adaptive-LIO/README.md) 为准。

- 原始项目：[chengwei0427/Adaptive-LIO](https://github.com/chengwei0427/Adaptive-LIO)
- 论文：[IEEE Xplore](https://ieeexplore.ieee.org/document/10806842) · [arXiv](https://arxiv.org/abs/2503.05077)
- 工程维护：**ECUST Origin Sentry Team**

如在自己的研究中使用原始算法，请引用原论文；使用本工程改造版本时，请同时注明本仓库及所使用的分支或提交版本。
