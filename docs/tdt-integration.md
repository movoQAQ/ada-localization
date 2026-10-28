# T-DT 后端接入记录

> 以下为历史云环境集成记录。当前完整后端已纳入 `src/tdt_backend/`，安装、统一构建与启动以根目录 README 为准；无需额外克隆后端或应用补丁。当前 `tdt_backend.yaml` 已展开继承且使用相对地图路径。

上游：https://github.com/mozihe/z-localization ，提交 76e126328b670adc92981d9b4900cc059a6f2343（GPL-2.0-only，独立工作空间保留原许可）。本机路径 `/workspace/z-localization`。

使用后端 `common.frontend: legacy`，不启动 ZLIO，不改变 Adaptive-LIO 的松耦合 ESKF。新增 `publish_backend_frames` ROS 参数默认关闭，开启后仅为接受/种子帧提供同一帧末的 `/adaptive_lio/backend/cloud`（odom 系、未作发布用 Z 裁剪）和 `/adaptive_lio/backend/odometry`（odom→aft_mapped 后验及位姿协方差）。高频预测仍使用 `/odom`。扫描末端消息只用于位姿配对，twist 未提供，不用于速度估计。

原有 `/livox/scan` 保持裁剪。原 ScanContext 回调移到裁剪之前：旧逻辑在世界坐标按 [-1,2] 米裁剪，既损失描述子的最大高度信息，又引入随机器人世界高度变化的偏差。这里的“未裁剪”指发布阶段；点云仍经过前端采样、时间覆盖筛选和去畸变，不是原始全分辨率雷达数据。关闭副雷达融合时，描述子与后端仍只接收主雷达。

新增 `config/tdt_backend.yaml` 继承上游配置；路径目前针对本云环境，换机器须修改 extends。默认关闭 UWB/先验地图；body 使用独立 tdt_sensor 单位外参，不能把其输出当作已标定的底盘位姿。关闭后端 legacy 的 map→body TF，避免与现有 TF 链冲突；后端保留正常 map→odom 输出。未实现 ZLIO 原子会话/重置协议；故障或重播时必须一起重启前后端，不可假装支持暖重置。

构建后 source ROS、Adaptive 安装空间、backend 安装空间，运行 `ros2 launch adaptive_lio tdt_backend.launch.py`。当前集成 launch 使用实时时钟及 mapping；真实 bag 验收需统一 use_sim_time、话题、单位、点时间和外参后再使用。不可直接对未转换的 ROS1 bag 运行。

## 数据与验收

已核查 FAST-LIO 官方 README 的 Avia 示例：
https://github.com/hku-mars/FAST_LIO#4-rosbag-example
下载：https://drive.google.com/drive/folders/1CGYEJ9-wWjr8INyan6q1BZz_5VtGB-fP
当前网络返回 403，尚未下载或回放。Avia 与本机 Mid-360 视场不同，可用于接口和场景退化验证，不能替代 Mid-360 实车验收。原始消息/ROS1 格式及真值须下载后核验，不能假定有真值。

另一个候选是 FAST-LIVO2-Dataset，官方入口：https://github.com/hku-mars/FAST-LIVO2 （README 数据集节链接至 HKU SharePoint）。尚未下载，传感器、真值、许可和具体序列需核查。

固定随机种子、相同前端/后端参数和相同建图/测试划分，对比裁剪与完整高度输入；禁止用同一帧同时建库和查询。报告 SC Recall@1/5、错误重定位率、几何验证后的成功率、失锁次数/时长、恢复耗时、延迟分位数及内存。另做升降/坡道、重复走廊、动态遮挡、删帧、IMU间隙、时间倒退及反向行驶。具有真值才报告 ATE/RPE；无真值只报告连续性与恢复行为。合成测试通过不构成真实鲁棒性结论。

## 当前云环境构建差异

后端以 Humble 为基线，本次在 Jazzy 编译。`z-localization/src/backend/CMakeLists.txt` 启用 C 语言以供 PCL/VTK 解析 MPI_C。依赖固定 nanoflann 1.4.3（官方 tag 原始头文件），系统 1.5.4 改变了 SearchParams 接口。Ubuntu GTSAM 4.2 包导出 CppUnitLite 但没有提供库；从 borglab/gtsam 4.2 tag 编译其 CppUnitLite 三个源文件、安装静态库与头文件补齐，未使用空库或删除校验。

`tools/build_tdt_backend.sh` 记录本机后端构建和代表性测试命令，需在安装依赖的 adaptive-lio-dev 容器中执行。

ScanContext 原旋转单测预期方向错误：circshift 为左移，输入正转 90° 对应 45/60 列；修正预期，并增加反向 15/60 列检查。未改变匹配算法以迎合测试。原有 descriptor 测试只验证构造/描述子/旋转，不验证真实场景识别准确率。

## 已执行结果

- 前端五组确定性 CTest 全部通过，pipeline 包含 [-0.1,0.1] 极端发布高度裁剪下仍保留描述子输入的回归。
- 原 ScanContext 三个单测通过（包含旋转正反向检查）。
- 后端六组测试、77 个用例通过：measurement、lifecycle、graph、relocalization、localization、descriptor。
- 联合启动已验证后端订阅新增帧末话题，前端正常注册传感器与 odom 接口。初次退出发现全局 ROS 对象延迟析构导致 SIGSEGV；显式在局部 node 存活时释放 imu_odom_output 和 g_tf_broadcaster 后，两节点退出均正常。
- 不包含真实传感器/数据集回放通过的结论。后端会话测试验证上游协议，不代表本 legacy 适配实现暖重置。

Google Drive 目录随后已可访问，确认存在 `2020-09-16-quick-shack.bag`，文件 ID `1YqxHuDKzWUcda80QKBV61lXI86TXsGjP`。下载跳转 `drive.usercontent.google.com` 后返回代理 403；两域名均已写入环境网络草稿，需应用后继续下载、检查 ROS1 消息及时间语义、转换和回放。仅查找到了数据，尚未验证其完整性、许可和可用真值。

Jazzy 另需 libmetis-dev，及将上游 node_setup.cpp 的服务 QoS 参数改为 rclcpp::ServicesQoS()。构建与测试日志位于 `/workspace/adaptive-lio-env/`。容器依赖保存为 `adaptive-lio:tdt`；需挂载 `/workspace` 并 source `/opt/ros/jazzy/setup.bash`、`/workspace/adaptive-lio-env/install/setup.bash`、`/workspace/adaptive-lio-env/backend-install/setup.bash` 后启动。当前工作树修改未提交。

## 2026-10-25 网络放通后的更新

官方 quick-shack 数据已下载、检查、转换并执行完整高度/裁剪对照。旧的“数据下载仍阻塞”记录为历史状态。结果与限制见 [公开数据验证报告](validation/quick-shack-report.md)。两次按传感器时间调度的回放能运行，但保留原投递节奏的一次运行触发 IMU 时间故障，尚不能宣称长期稳定或实车精度合格。联合 launch 现支持 frontend_config、use_sim_time、mode 参数。


## Mid-360 专用配置与闭环修复

SEU library 已完成原包检查、转换及多轮回放。使用 `mapping_mid360.yaml`（前端）和 `tdt_backend_mid360.yaml`（后端），而非默认Avia参数；联合launch显式传入 `frontend_config`、`backend_config` 及 `use_sim_time:=true`。详情、对照和未解决限制见 [Mid-360修复报告](validation/mid360-fix-report.md)。后端新增重访配准初值和历史邻域支持，需重新构建独立z-localization仓库backend；仅复制YAML不能给旧二进制增加该功能。前端继续使用松耦合ESKF，ScanContext与后端都在导航Z裁剪之前取点云。
