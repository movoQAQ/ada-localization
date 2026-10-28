# T-DT + 松耦合 Adaptive-LIO 公开数据验证

日期：2026-10-25。结论：在本序列上完成前后端联合回放；不能宣称已通过长期稳定性、定位精度或跨场景鲁棒性验收。

## 数据与输入语义

来源：[FAST-LIO 官方 Avia 示例](https://github.com/hku-mars/FAST_LIO#4-rosbag-example)，文件 `2020-09-16-quick-shack.bag`，Google Drive ID `1YqxHuDKzWUcda80QKBV61lXI86TXsGjP`。
原文件约 218 MiB，SHA256 `05a56e75898f952766f136d1e5db64a35d202e990e3b1052558369d8384d7ffe`（本次下载计算的复现标识，不是独立发布的上游校验值）。原包记录跨度 50.001 秒；点云 491 条、IMU 9953 条，无真值话题。

ROS1→ROS2 逐字段转换：去除 ROS1 Header.seq，Livox 类型名适配到 livox_ros_driver2；保留传感器时间、点内 offset_time、点数据与 IMU 测量。原包完整保留在 `/workspace/datasets/quick-shack.bag`。详细输入检查见 `quick-shack-inspection.json`。

IMU 初始模长中位数 0.9913，使用 g→m/s² 换算；角速度模长峰值 3.978 rad/s，p95 2.254 rad/s；原始 IMU Header 严格递增，最大间隔 5.195 ms。使用 FAST-LIO Avia 参考外参 `[0.04165,0.02326,-0.0284]` 和 4 m 盲区；这是参考配置，不是重新标定结果。设备为 Avia，不是 Mid-360。

原包记录时间为 Unix 时间，传感器 Header 为设备启动秒数，且开头存在集中记录的消息。正式对照使用每条消息的原始 Header 作为 bag 调度时间（约 49.099 秒），未改消息载荷。这检验传感器时间序列上的算法行为，不复现原始传输突发。

## 正式对照结果

均使用同一数据、参考前端配置与固定 LiDAR 协方差；后端 legacy、Scan Context、mapping，关闭 UWB/先验图。裁剪组使用 ROS relay 在世界坐标做旧 [-1,2] m Z 裁剪；full 组直接接收新接口。

| 指标 | 完整高度 | Z 裁剪 |
|---|---:|---:|
| 前端里程计观测条数 | 9600 | 9169 |
| 后端定位位姿观测条数 | 9599 | 9167 |
| 前端日志接受帧（含种子） | 476 | 454 |
| 前端日志拒绝帧 | 0 | 0 |
| 观察器采集云帧 | 459 | 441 |
| 后端状态消息 | 447 NORMAL | 433 NORMAL |
| 后端日志接受回环次数 | 10 | 10 |
| 后端输出时间戳间隔 p99 | 5.085 ms | 5.084 ms |
| 后端输出时间戳间隔最大值 | 15.046 ms | 14.998 ms |
| 非有限/时间倒退观察器错误 | 0 | 0 |
| 前端/后端/播放器退出码 | 0/0/0 | 0/0/0 |

时间戳间隔不是墙钟端到端延迟。状态消息计数不是独立定位试验次数；accepted loop 是算法接受的约束，不是有真值确认的正确回环。

两次前端有效帧数不同，观察器也未采集全部日志接受帧。启动发现、执行调度、best-effort 接收及前端 thread_local 随机采样可能影响可比性；不能把轨迹或帧数差归因于 Z 裁剪。前后端末端相对起点位移保留于 JSON，但无真值，不称为漂移或精度。没有量化 Recall@K、误重定位率或 ATE/RPE。

## 同帧裁剪对 ScanContext 信息的影响

在 full 组采集的 459 个相同点云上，直接比较裁剪前后，并用对应帧末后验变换回 IMU 系。诊断描述子使用本仓库原 ScanContext 的 20 环×60 扇区、80 m 范围、最大高度逻辑；不是 T-DT 默认 30×90、整平/高度偏移后的完整检索流水线。因此下面度量用于说明输入信息损失，不作为后端识别精度。

- 旧世界 Z 裁剪删除 **53.55%** 的点（按总点数加权）。
- 非空栅格均值：**43.12 → 27.73**，减少约 **35.7%**。
- **100%** 的采样帧描述子改变。

Avia 视场有限，非空栅格本来就少；裁剪又删除高度结构。建议保持已实现的接口分离：SC/后端使用未作发布 Z 裁剪的帧末点云，导航输出独立裁剪。该短序列裁剪组仍能工作，不能断言裁剪必然造成 SC 匹配错误；升降、斜坡、不同楼层、重复结构或错误回环风险仍需更有针对性的数据。

## 保留的失败结果

`replay-aligned-full` 保留原记录间隔、仅平移记录时间到传感器时基：前端触发 `ImuQuality::TimeFault`（reason=8），停止两条估计链，后端停留 INITIALIZING，无定位输出。进程退出码仍为 0，不能将其算作通过。原始 IMU Header 没有 >50 ms 缺口，也没有倒退；具体接收端丢失/调度原因尚未被直接捕获，不能把传输突发猜测写成已证实根因。故障阈值没有放宽，按传感器时间调度的成功回放不能覆盖这次失败。

此外，首轮脚本的 ROS 参数写法与程序 gflags 解析不兼容、裁剪 relay 初版未保留 PCL point_step，均已修正；相应失败日志保留，未计入正式对照。

## 复现与产物

工具：`tools/convert_avia_bag.py`（使用 host data-venv 的 rosbags）、`tools/align_bag_clock.py`、`tools/replay_tdt.py`、`tools/summarize_tdt_replay.py`。完整运行目录：

- `/workspace/adaptive-lio-env/replay-sensor-full`
- `/workspace/adaptive-lio-env/replay-sensor-cropped-v2`
- `/workspace/adaptive-lio-env/replay-aligned-full`（失败）

每个目录有原始节点/播放器日志、metrics.json、同帧描述子矩阵 descriptors.npz。可审查汇总为 `quick-shack-results.json`。配置为本目录 `quick-shack.yaml`。

在已 source ROS、前端与后端安装空间的容器中，使用独立 ROS_DOMAIN_ID、ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST，运行：

```bash
python3 /workspace/Adaptive-LIO/tools/replay_tdt.py \
  --bag /workspace/datasets/quick-shack-sensor-clock \
  --config /workspace/Adaptive-LIO/docs/validation/quick-shack.yaml \
  --output /workspace/adaptive-lio-env/new-run
# 裁剪组追加 --crop；输出目录必须是新目录。
```

联合 launch 已增加 `frontend_config`、`use_sim_time`、`mode` 参数，便于后续自有 Mid-360 bag 回放。仍需有重复访问/真值的数据验证地点识别，以及更长时间和真实投递条件下的稳定性；未实现 legacy 适配的自动暖重置协议。

## 投递节奏短复查

保留原记录间隔、平移到传感器时基，播放器等待 2 秒后回放前 10 秒；另加 reliable 深度 500 的 IMU 观察订阅。收到 1976 条 IMU，观察到的 Header 最大间隔 5.115 ms；前端 585 条里程计、后端 565 条定位位姿、88 个点云，未触发时间故障，所有进程正常退出。目录 `/workspace/adaptive-lio-env/replay-delivery-delay2`。

该短复查支持先等待 ROS 发现再播放的操作建议，但只覆盖开头 10 秒，且观察订阅也发生变化，不能单独证明先前故障的根因或长期可靠性。保留投递突发时，外层预测会批量处理 IMU，输出数量与平滑传感器调度不同；不能据输出条数直接推断丢掉了同样数量的 IMU。

脚本现在默认 `--delay 2`，默认完整播放 `--duration -1`；正式 full/crop 对照当时没有这个延迟（重现其条件可指定 `--delay 0`）。脚本也增加了前端故障日志、后端节点异常退出以及后端无输出的失败判定，不再仅依赖进程退出码。
