# Rysen Apex 手部跟随

使用摄像头识别人手，并将手指动作映射到 Rysen Apex 灵巧手。本工作区包含 Windows 端识别与网页预览、个人标定、RealSense D435 工具、SDK 控制桥，以及 Explorer 子模块。

Windows Python 环境由 [uv](https://docs.astral.sh/uv/) 管理；机械手 SDK 运行于现有 `wslc` 镜像。启动脚本不会安装普通 WSL、下载或重新导入系统镜像。Explorer 与 SDK 跟随共用设备时不能同时运行。

## 演示

[【根本没有这样的工程末端】](https://www.bilibili.com/video/BV1amJR6tEe2/)

## 快速开始

需要 Windows、PowerShell 7（`pwsh`）、`uv` 和已配置的 `wslc` SDK 容器。实机跟随还需要摄像头、已连接的灵巧手和匹配当前识别方案的本地个人标定。首次运行时，按提示准备依赖和模型。

### 实机跟随：WiLoR

在工作区根目录打开 PowerShell：

```powershell
.\Start-HandFollow.ps1
```

默认使用 WiLoR 摄像头识别，随后打开 <http://127.0.0.1:8090/>。点击“恢复 / 重新捕获”并确认预览正常后，**再点击“开始全手跟随”使能机械手**。启动脚本不会自动使能。

安全暂停可在页面点击“暂停并失能”。退出时运行：

```powershell
.\Stop-HandFollow.ps1
```

停止不会自动回零，也不等于关闭机械手电源。跟随使用 SDK 原有速度设置，并保留输入有效性、硬件故障和通信超时保护；没有完整的手指自碰撞求解，首次使用请缓慢测试并观察干涉。

### D435 识别与预览

先不连接机械手进行预览：

```powershell
.\Start-DepthFollow.ps1 -PreviewOnly
```

确认画面、标定和识别正常后，可运行 `Start-DepthFollow.ps1` 启动 D435＋MediaPipe 实机链路。启动后仍需在网页手动开始跟随。D435 页面使用 <http://127.0.0.1:8090/>。

### A/B 对照和深度相机工具

- `Start-HandCompare.ps1`：WiLoR 与 MediaPipe 的纯识别对照，不连接机械手。
- `Start-DepthCamera.ps1`：D435 彩色和对齐深度预览、点击测距及图像采集；页面为 <http://127.0.0.1:8095/>。
- `Start-DepthHand.ps1`：MediaPipe＋逐指深度约束的独立手部识别预览。

对应停止脚本以 `Stop-` 开头。日常使用可双击根目录下的 `.cmd` 启动器；`.ps1` 可传入上面列出的参数。

## 识别方案

| 模式 | 识别与深度 | 用途 | 入口 |
|---|---|---|---|
| WiLoR | WiLoR 三维人手估计；可选 D435 深度辅助四指侧摆 | 全手预览或 SDK 跟随 | `Start-HandFollow.ps1` |
| D435＋MediaPipe | MediaPipe 关键点与逐指深度约束 | D435 预览或 SDK 跟随 | `Start-DepthFollow.ps1` |
| WiLoR / MediaPipe 对照 | 两种识别结果并排观察 | 识别调试，不驱动设备 | `Start-HandCompare.ps1` |
| D435 手部预览 | MediaPipe＋逐指深度约束 | 单独观察深度融合效果 | `Start-DepthHand.ps1` |

完整的标定步骤、映射方式、参数与恢复流程见 [手部跟随说明](hand-tracking/README-Full-Follow.md)、[D435 跟随说明](hand-tracking/README-D435.md) 和 [标定说明](hand-tracking/README-Calibration.md)。A/B 模型对照和许可信息见 [对照说明](hand-tracking/README-Comparison.md)。

## 项目结构

| 路径 | 内容 |
|---|---|
| `hand-tracking/` | 手部识别、网页、标定、URDF 预览、SDK 跟随和测试 |
| `realsense-tools/` | D435 彩色/深度操作及手部深度融合工具 |
| `wilor-backend/` | 独立 WiLoR GPU 推理服务 |
| `rysen-explorer/` | Rysen Explorer 上游 Git 子模块 |
| `Start-*.ps1`、`Stop-*.ps1` | 工作区级启动与停止脚本 |
| `vision_runtime.py` | Windows 视觉进程调度策略 |

## 运行与数据说明

- 主预览页面在 `127.0.0.1:8090`；D435 基础工具在 `127.0.0.1:8095`；SDK 桥和 WiLoR 后端分别监听本机 `8092`、`8093`。
- 各项目通过自己的 `pyproject.toml` 与 `uv.lock` 管理 Python 依赖。
- `rysen-explorer` 以 Git 子模块固定版本。克隆后运行 `git submodule update --init --recursive` 获取源码。
- 模型权重、Python 环境、缓存、日志、个人标定和本地控制令牌不包含在仓库中。个人标定需在本机重新采集；不要公开分享令牌。
- D435 或 USB/IP 设备访问遇到问题时，查看 [D435 连接说明](realsense-tools/README.md)。

## 故障排查

| 现象 | 检查方式 |
|---|---|
| 启动时报 SDK 容器不存在 | 按现有 Rysen 镜像和 SDK 文档创建、配置容器；启动器不会安装或重新导入镜像 |
| 网页没有手部画面 | 检查摄像头是否被其他程序占用，点击“恢复 / 重新捕获”，让完整左手进入画面 |
| 页面显示的识别状态较旧 | 在浏览器按 Ctrl+F5；更改后端代码后使用 `-Restart` 重启 |
| 推理帧率骤降 | 查看 `hand-tracking/logs/` 下映射服务日志及 `/api/diagnostics/hybrid`，确认 8093 WiLoR 后端健康 |
| SDK 或设备故障 | 保留页面显示的错误和日志；设备故障需按硬件流程检查，服务不会自动清故障或恢复使能 |
