# Mid-360 数据来源与测试约束

官方来源：https://github.com/hku-mars/Voxel-SLAM#44-livox-mid360
核查提交：70fc8a28d63823d5989ff184daeea0787b672398。
官方 README 明确标注 Livox Mid360，示例 `jungle_challenge.bag`，注明从剧烈运动开始。
下载：https://1drv.ms/f/c/8b1ef18ae4181c8d/ErtuXCFhFrBErZxzS5vLASkBJEfgDB9R2CSCgKe8BwhneQ?e=zbr7NL
2026-10-25 环境恢复且本地网络策略为 unrestricted 后重试，Microsoft 页面仍返回 HTTP 403（The request is blocked）；公共分享 API 返回 401。未取得 jungle_challenge.bag。

上游 VoxelSLAM/config/mid360.yaml：话题 /livox/lidar、/livox/imu，盲区0.5m，LiDAR→IMU平移 [-0.011,-0.02329,0.04412]、单位旋转。此值也与 FAST_LIO_ROS2 的 Mid360 参考配置一致。
本目录 mid360-raw-reference.yaml 是针对未做驱动坐标补偿的原始数据准备的参考配置，不是实车标定结果，不覆盖用户现有 mapping_m.yaml。取得数据后仍须确认消息实际坐标系、IMU单位、点内时间、时间基准和静止段；如果驱动已补偿到IMU系，不可重复套用这一外参。

Adaptive-LIO 的 lidar_type=1 使用 Livox CustomMsg 处理路径，枚举名AVIA不代表只能用Avia；不能把其他传感器枚举号当作Mid360型号。ROS1 livox_ros_driver/CustomMsg 可以由 tools/convert_avia_bag.py 转换（脚本名来自首个验证样本，其字段转换也接受 driver2 消息）。实际bag类型须读取后判断。

当前前端 StaticIMUInit 要求至少1秒并通过角速度/加速度方差检查。此公开序列从剧烈运动开始，因此必须把初始化是否成功当作独立验收项；不能通过提高静止阈值伪造通过。若后续有真实静止段，可明确报告从该段开始的测试；没有静止段则是动态初始化能力限制。

Mid360的360度水平视场更适合环状描述子，但世界Z裁剪仍会丢失高度结构；不得把Avia上的53.55%点云损失率移用于Mid360。保持SC/后端完整高度输入、导航点云单独裁剪，待真实Mid360数据上重新量化。

## 新增可访问来源

已下载 SEU 校园 Mid-360 数据集的 library.bag（1,666,816,303 bytes）：
https://github.com/hzc333/Livox-Mid360-SLAM-Dataset-SEU

核查提交 34556c419473887bada1317ac357d82ef3b4ae22；Google Drive 文件 ID 1oJyPuq1aUv7BJnNSO4e-BDYsudklJDMH。发布者声明闭合路径，bag 内只有 /livox/lidar 和 /livox/imu，没有真值。检查结果见 seu-library-inspection.json。采集者没有提供驱动坐标补偿配置，因此先按原始 Livox 坐标使用参考外参，作为明确未核实的实验假设，不能用来证明实车标定正确。
