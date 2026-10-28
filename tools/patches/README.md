# 独立T-DT后端补丁

`z-localization-adaptive-lio.patch` 保存本云环境对独立后端的修改，包括Jazzy构建兼容、Mid360重访初值/历史邻域、PCM参数语义修复和回归测试。上游：https://github.com/mozihe/z-localization ，基线提交 `76e126328b670adc92981d9b4900cc059a6f2343`。补丁针对该独立GPL-2.0后端，不把其算法移植进Adaptive-LIO前端。

在干净的上述基线检出中执行：
```bash
git apply --check /path/to/Adaptive-LIO/tools/patches/z-localization-adaptive-lio.patch
git apply /path/to/Adaptive-LIO/tools/patches/z-localization-adaptive-lio.patch
```
然后按照 `tools/build_tdt_backend.sh` 构建和测试。当前 `/workspace/z-localization` 已应用这些修改，不要重复应用；云环境的extends路径需按实际检出位置调整。
