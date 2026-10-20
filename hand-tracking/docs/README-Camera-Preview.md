# Windows 摄像头手部识别预览

本文件保留第一阶段的纯摄像头预览说明。当前完整流程见 [项目首页](../README.md)，仅三维预览见 [映射预览说明](../README-Mapping.md)。不同预览不要同时占用同一摄像头。

第一阶段：自带摄像头 → MediaPipe Hand Landmarker → 骨架、左右手标签和性能显示。所有图像在本机处理，不保存照片或视频，不导入 Rysen SDK，也不启动 wslc 服务或连接灵巧手。

## 启动

在 Windows PowerShell 中执行：

```powershell
& D:\Dev\wslc\hand-tracking\Start-HandPreview.ps1
```

使用 uv 管理 Python 3.11、项目 `.venv` 和依赖；`uv.lock` 锁定实际版本。系统 Python 无需更改。首次运行如模型不存在，会从 Google 官方地址下载约 8 MB 模型，此后模型推理可离线进行。启动脚本使用 `uv sync --frozen`，不会自行升级依赖。

快捷键（先点预览窗口取得焦点）：

| 按键 | 操作 |
|---|---|
| Q / Esc / 关闭窗口 | 退出并释放摄像头 |
| M | 切换镜像显示 |
| H | 手动交换 Left / Right 标签 |
| K | 显示或隐藏 0–20 关键点编号 |

Left 为绿色、Right 为蓝色。`label xx%` 是左右手分类分数，不代表所有关键点的定位精度。FPS 是整条处理循环的平滑帧率，inference 是单次模型推理耗时，两者不能等同于真实端到端相机延迟。

## 左右手与镜像

识别始终使用摄像头原始画面，镜像只改变预览画面与骨架的绘制，不修改识别数据。第一次请只举起自己的左手，确认标签是否为 Left；若摄像头驱动自带翻转或标签不符合实际，用 H 切换，并在后续启动时加 `-SwapHandedness`。该开关仅对本次运行生效，不会悄悄写入未来机械手控制配置。进入映射阶段前还需单独标定。

```powershell
& D:\Dev\wslc\hand-tracking\Start-HandPreview.ps1 -SwapHandedness
& D:\Dev\wslc\hand-tracking\Start-HandPreview.ps1 -NoMirror
```

## 摄像头与排错

- 默认摄像头索引 0，优先 DirectShow，失败后尝试 Media Foundation。
- 若选中了红外/虚拟摄像头，尝试 `-Camera 1` 或 `-Camera 2`；通过实际彩色预览确认设备。
- 黑屏或打开失败：关闭 Windows“相机”、Teams、会议软件和正在用摄像头的浏览器页；打开隐私挡板，检查“设置 → 隐私和安全性 → 相机 → 允许桌面应用访问摄像头”。
- 尝试 `-Backend msmf` 或 `-Backend dshow`。设备驱动异常时打开摄像头的耗时由驱动决定。
- 默认请求 640×480、30 FPS，驱动可能使用不同参数，以控制台打印的实际分辨率和窗口 FPS 为准。
- 在光线充足、背景简单时，手掌尽量完整入镜。遮挡、快速运动和侧掌会降低识别稳定性。当前不识别手势类别、不计算机械手目标角度。
- 模型下载失败时检查网络/代理，不要禁用 TLS 证书校验。
- PowerShell 执行策略阻止脚本时，可审查脚本后对当前进程使用 `Set-ExecutionPolicy -Scope Process Bypass`。

## 开发命令

```powershell
cd D:\Dev\wslc\hand-tracking
uv sync --frozen
uv run --frozen python -m hand_preview --help
uv run --frozen python -m hand_preview --download-model
# 不打开摄像头，测试模型加载和空图推理
uv run --frozen python -m hand_preview --self-test
# 读取摄像头并推理 60 帧后退出，不保存画面、不显示窗口
uv run --frozen python -m hand_preview --headless --max-frames 60
```

模型使用固定版本 `float16/1`，下载后检查 ZIP 完整性并输出 SHA256；模型路径为 `models/hand_landmarker.task`。识别使用 MediaPipe Tasks 的 VIDEO 模式，每帧传入递增的单调时间戳，以同一帧完成推理和绘制，便于第一阶段验证。后续控制阶段再分离采集、识别和控制循环。

官方资料：[MediaPipe Python 指南](https://developers.google.com/edge/mediapipe/solutions/vision/hand_landmarker/python)、[模型说明](https://developers.google.com/edge/mediapipe/solutions/vision/hand_landmarker)、[uv 项目管理](https://docs.astral.sh/uv/guides/projects/)。
