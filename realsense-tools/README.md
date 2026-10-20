# D435 基础相机工具

独立 Windows uv 项目，只读取 RealSense，不使用 WSL 或机械手 SDK。基础入口为工作区根目录 `Start-DepthCamera.cmd` / `Stop-DepthCamera.cmd`，页面 `http://127.0.0.1:8095/`。

新增 **Start-DepthHand.cmd / Stop-DepthHand.cmd**：在同一页面启用 MediaPipe＋逐指深度约束与双骨架对照。操作、参考建立、来源颜色与算法限制见 [手部深度融合说明](README-DepthHand.md)。两种模式共用相机和 8095，切换入口会切换服务模式。

## 使用

1. 使用支持 USB 3 数据传输的线直连 USB 3 接口，关闭占用 D435 的其他程序。
2. 双击 `Start-DepthCamera.cmd`。首次环境准备使用 uv；依赖版本保存在 `uv.lock`。
3. 左侧是 RGB，右侧为对齐到 RGB 的深度伪彩图。显示分辨率均为 640×480。USB 3 请求 30 FPS，检测到 USB 2 时降为 15 FPS；这不保证故障链路一定能采集。
4. 点击任意一侧，查看对应彩色像素的原始 Z、原像素 XYZ、5×5 邻域中位数/有效比例/标准差。Z 沿光轴；XYZ 相对相机光学坐标系，单位米，不是手关节中心。
5. 调整近端、远端只改变深度图配色，不过滤原始测距数据。黑色表示原始深度为 0（无效）。
6. “暂停并释放相机”会停止采集；连接恢复后点“恢复 / 重试相机”。关闭网页不会停止后台服务。
7. 点击“保存当前 RGB＋原始深度”，才会在 `captures/时间戳/` 写入 `color.png`、16 位 `depth-z16.png`、`metadata.json`。默认不保存画面。深度米值 = PNG 原始整数 × JSON 中 `depth_scale_m`；0 不代表零距离。
8. 完成后点页面“停止服务”，或双击 `Stop-DepthCamera.cmd`。停止脚本检查服务身份并等待 8095 关闭，不强制杀死未知程序。

## 命令

```powershell
# 工作区根目录
.\Start-DepthCamera.ps1 -NoBrowser
.\Stop-DepthCamera.ps1
.\Start-DepthCamera.ps1 -Restart
# 多设备时用序列号指定相机
.\Start-DepthCamera.ps1 -Serial '设备序列号'

cd D:\Dev\wslc\realsense-tools
uv sync --frozen
uv run --frozen python depth_camera.py --list
# 试采并自动释放设备，仅保存数值诊断，不保存画面
uv run --frozen python depth_camera.py --probe 3
# 离线测试（合成数据，不打开相机）
uv run --frozen python -m unittest -q
```

设备流信息保存在 `logs/devices.json`（含本机设备序列号），成功试采报告为 `logs/probe.json`。服务输出为 `logs/camera.stdout.log` / `camera.stderr.log`。不要在服务占用相机时运行 probe；先停止服务。

## 测距与诊断边界

- 5×5 区域可能跨越前景和背景，边缘处中位数不能保证属于目标物体。原像素无效时不生成 XYZ，邻域结果另行显示。
- 有效比例是全画面统计，受场景、距离、材质、遮挡和连接影响，不是精度评分。帧时间戳差是诊断量，不等同于已验证硬件同步。
- 建议先在约 0.4–0.8 米处用不透明平面检查，再测手掌；先保持静止，确认深度连续后再缓慢移动。具体可用距离以实际深度图为准。
- 当前已验证 USB 3.2 下 640×480 / 30 FPS 彩色与对齐深度连续到帧，短测 93 帧、约 30.1 FPS。当前 Python SDK 固定 2.56.5 系列，固件 5.17.0.10，未刷设备固件。有效深度比例受当前场景影响，不代表手部识别或绝对精度。详见 `VALIDATION.md`。

## USBIP 转发占用

本次换线后出现过：Windows / SDK 找不到 D435，但 `usbipd list` 显示 `8086:0b07` 为 Attached。相机被转发接管，不能同时供 Windows 本工具使用。一次 detach 后又被自动接管，因此通过管理员授权取消了该相机的共享；随后 SDK 报告 USB 3.2 并正常到帧。

先运行 `usbipd list` 核对。确实要交还 Windows 时，用管理员 PowerShell 执行：

```powershell
& D:\Dev\wslc\realsense-tools\release_usbip.ps1
```

该辅助脚本要求恰好一台已连接的 `8086:0b07`，仅取消它的共享，结果写入 `logs/usbip-release.json`；不修改其他设备。它改变了相机的 USBIP 共享设置，后续若要交回 Linux，需要自行重新配置该相机的共享。常规启动不会自动请求管理员权限或取消共享，发现 Attached 会给出提示。

接入依据：[官方深度对齐示例](https://github.com/realsenseai/librealsense/blob/master/wrappers/python/examples/align-depth2color.py)、[SDK 投影与反投影说明](https://github.com/realsenseai/librealsense/wiki/Projection-in-RealSense-SDK-2.0)、[SDK 发布记录](https://github.com/realsenseai/librealsense/releases)。
