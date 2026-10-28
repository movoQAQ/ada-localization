# T-DT 后端来源

来源：https://github.com/mozihe/z-localization

固定基线：`76e126328b670adc92981d9b4900cc059a6f2343`。
本目录包含 backend、localization_interfaces、构建辅助文件、默认配置和完整 GPL-2.0 许可；不包含 ZLIO 前端。
已应用仓库 `tools/patches/z-localization-adaptive-lio.patch`，不要重复应用。
这些源码来自 2026-10-28 实际完成单雷达和副点云仅发布回放的本地后端。

本目录不附带 GTSAM、nanoflann 或本机构建依赖，安装步骤见仓库 README。
前后端仍是独立 ROS 节点，接口及各自许可不变。
