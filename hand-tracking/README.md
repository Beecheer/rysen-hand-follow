# Rysen 摄像头全手跟随

本项目目前支持本机摄像头识别、个人标定、官方 URDF 三维预览和左手机械手 SDK 跟随。日常使用从工作区根目录的 `Start-HandFollow.cmd` / `Stop-HandFollow.cmd` 进入，完整步骤见 [工作区使用说明](../README.md)。

## 说明索引

| 文档 | 内容 |
|---|---|
| [代码结构](docs/ARCHITECTURE.md) | 数据流、模块职责、参数所在文件、维护与验证 |
| [全手跟随](README-Full-Follow.md) | 当前实机流程、限幅、拇指保持、通信与停止机制 |
| [个人标定](README-Calibration.md) | 掌心朝前的八姿态采集、结构优化、动态检查与配置备份 |
| [拇指专项标定](README-ThumbCalibration.md) | 自然位、单方向往返、四组对指轨迹，独立配置与回退 |
| [当前交接与优化方向](docs/STATUS.md) | 停服状态、已保存标定、离线审查结论与下一步优先级 |
| [阶段收尾说明](docs/STAGE-CLOSEOUT.md) | 本阶段交付、启停入口、验证与已知问题，供下次恢复使用 |
| [A/B 识别对照](README-Comparison.md) | MediaPipe / WiLoR、独立 uv 环境、同源帧骨架、延迟与录像回放 |
| [三维映射](README-Mapping.md) | 仅预览，不发送运动命令 |
| [SDK 反馈与模拟](README-SDK-Monitor.md) | 前期反馈和模拟发送检查 |
| [食指跟随](README-Index-Follow.md) | 前期 0–10° 小范围食指试验 |
| [纯摄像头预览](docs/README-Camera-Preview.md) | 第一阶段摄像头识别与环境说明 |
| [验证记录](VALIDATION.md) / [零位审计](ZERO-POSE-AUDIT.md) | 当前标定验证及历史摄像头、硬件检查，注明验证范围 |
| [食指单次试验](hardware/README-Index-Trial.md) / [弯曲复位](hardware/README-Index-Cycle.md) | 历史硬件检查 |

## 开发验证

```powershell
cd D:\Dev\wslc\hand-tracking
uv sync --frozen
uv run --frozen python -m unittest discover -s tests -q
# 启停脚本隔离测试：不启动摄像头、GPU 或 SDK
pwsh -NoProfile -File tests/test_launchers.ps1
# 前端行为回归（需要 Node.js）
node tests/frontend-recovery.cjs
node tests/frontend-thumb-calibration.cjs
# 只开映射预览，不启动 SDK 跟随
.\Start-HandMapping.ps1
.\Stop-HandMapping.ps1
```

Python 环境继续由 uv 管理；不使用全局 pip 安装项目依赖。原生模拟检查在现有 wslc 容器中运行，测试头文件只用于模拟，不可用于真实 SDK 编译。详细命令见代码结构文档。
