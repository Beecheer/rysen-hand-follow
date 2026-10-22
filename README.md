# Rysen 左手摄像头跟随工作区

**D435 全链路入口：`Start-DepthFollow.cmd` / `Stop-DepthFollow.cmd`。** 使用 D435＋MediaPipe，两步骨架标定、图像主导四指关节链、直接角度拇指（无需逆解、末端固定）及真实 SDK 跟随，统一页面为 `http://127.0.0.1:8090/`。无机械手时用 `Start-DepthFollow.cmd -PreviewOnly`。新版配置独立保存，旧八姿态配置保留。详见 [D435 全链路说明](hand-tracking/README-D435.md)；下文 A/B 入口继续保留。

当前主入口是 **摄像头识别 → 个人标定 → 五指三维预览 → SDK 实机跟随**。Windows Python 由 uv 管理；SDK 运行于现有 wslc 镜像。脚本不安装普通 WSL，不重新导入镜像。

当前一键实机入口默认使用 **B / WiLoR**，加载其独立八姿态标定。A / MediaPipe 可通过 `Start-HandFollow.ps1 -Tracker mediapipe -Restart` 回退。`Start-HandCompare.cmd` 仍为仅摄像头的 A/B 对照，不连接 SDK。

## 演示视频

[【根本没有这样的工程末端】](https://www.bilibili.com/video/BV1amJR6tEe2/?share_source=copy_web&vd_source=b2be945a77657353d02cfa6f7e5d15c4)

**原 A/B 阶段已收尾。** 已保存 B 的八姿态基础标定及拇指专项参考。其功能、验证范围、已知问题和恢复步骤见 [阶段收尾说明](hand-tracking/docs/STAGE-CLOSEOUT.md)；新增 D435 工作及当前运行状态见 [D435 验证记录](hand-tracking/docs/D435-VALIDATION.md)。

## 日常使用

1. 接通左手机械手电源并连接网线，打开电脑摄像头挡板。
2. 双击 **Start-HandFollow.cmd**，等待启动完成，打开 <http://127.0.0.1:8090/>。
3. 点击“恢复 / 重新捕获”，完整展示左手，确认预览动作正常。
4. 点击“开始全手跟随”才会使能、运动。启动脚本不会自动使能。
5. 暂停运动用页面“暂停并失能”；结束使用双击 **Stop-HandFollow.cmd**，释放摄像头、停止 SDK 和专用容器，并等待 WiLoR GPU 服务退出。

重复启动会复用已经健康运行的全手服务，不重启、不打断当前跟随。重复停止可执行。停止不自动回零，不等于硬件断电。

启动失败会尝试回收摄像头、SDK 和 WiLoR；停止时各环节独立清理，最后汇总未确认的错误。8093 上服务身份不符时不会发送关闭请求。失败信息应保留，以便定位未退出的环节。

PowerShell 7 命令：

```powershell
& D:\Dev\wslc\Start-HandFollow.ps1
& D:\Dev\wslc\Stop-HandFollow.ps1
# 不打开浏览器
& D:\Dev\wslc\Start-HandFollow.ps1 -NoBrowser
# 修改代码后重新部署并重启；会中断已有跟随，重启后保持失能
& D:\Dev\wslc\Start-HandFollow.ps1 -Restart -NoBrowser
```

需要已安装的 `pwsh`、`uv`、`wslc` 在 PATH 中。当前 SDK 容器必须已存在；缺失时脚本报错，不自动安装或重建。摄像头被其他程序占用时先关闭占用程序。

## 工作区布局

D435 基础操作使用独立入口 **Start-DepthCamera.cmd / Stop-DepthCamera.cmd**，页面为 `http://127.0.0.1:8095/`，提供彩色、对齐深度、点击测距和手动保存。它不会启动手部识别、SDK 或机械手。操作与当前验证状态见 [D435 说明](realsense-tools/README.md)。

D435 手部识别入口 **Start-DepthHand.cmd / Stop-DepthHand.cmd**：MediaPipe＋逐指深度约束，提供 RGB 推断/深度约束骨架对照及五指角度。独立预览，不连接机械手；与基础模式共用 8095，切换入口会切换模式。详见 [深度融合使用说明](realsense-tools/README-DepthHand.md)。

识别方案 B 的独立试验入口为 `Start-HandCompare.cmd` / `Stop-HandCompare.cmd`，仅做摄像头预览，不连接 SDK。操作和限制见 [A/B 对照说明](hand-tracking/README-Comparison.md)。

| 路径 | 用途 |
|---|---|
| `Start-HandFollow.*` / `Stop-HandFollow.*` | 推荐的日常入口；cmd 可双击，ps1 可传参 |
| `hand-tracking/` | 当前跟随项目，代码、网页、标定、模型和专项说明 |
| `realsense-tools/` | 独立 uv 环境的 D435 彩色/深度基础工具，不连接机械手 |
| `rysen-explorer/` | 官方 Explorer 源码，保留独立使用 |
| `Start-Rysen.ps1` / `Stop-Rysen.ps1` | Explorer 启停，与实机 SDK 跟随互斥 |
| `motions/` | 之前编写的 Explorer 动作组 |
| `rysen_system_image.tar` | 已导入镜像的原始归档，日常启动无需再次加载 |

没有搬动标定、模型、日志或官方仓库，也没有删除历史硬件试验。项目内部职责与调试入口见 [代码结构](hand-tracking/docs/ARCHITECTURE.md)。

## 连接与运行约定

| 项目 | 当前配置 |
|---|---|
| 左手设备 | `192.168.0.102`，SDK TCP `5856/5857` |
| 电脑有线地址 | `192.168.0.50/24` |
| 预览和操作页面 | `http://127.0.0.1:8090/` |
| 本地 SDK 桥 | `127.0.0.1:8092`，控制接口需要本机令牌 |
| WiLoR GPU 推理 | `127.0.0.1:8093`，由全手入口启动和停止 |
| SDK 专用容器 | `rysen_sdk_readonly`（历史名称，现在也承载真实跟随） |
| 现有镜像 | `rysen_system:latest`；SDK 1.4.7，设备固件 3.2.8 |

Explorer/ROS 后端与 SDK 跟随不能同时占用机械手。若 Explorer 正在运行，先执行 `Stop-Rysen.ps1`，再启动跟随。普通映射预览、摄像头预览和实机页面也不能同时占用摄像头。

## 当前控制行为

- 全手 16 个独立目标，发送 21 个关节位置，DIP/IP 按模型联动。
- 不额外限制全手速度/加速度，使用 SDK 原有参数；识别保留自适应滤波与侧摆突跳确认。
- 四指屈伸目标 0–65°、侧摆 ±25°；拇指目标取两份 URDF 的交集并预留 2° 余量。
- 拇指 j1 额外按实际 SDK 日志上限 0.53 rad 收紧，预留余量后约 28.4°；逆解与发送端使用一致范围。运行中单指屈伸短暂异常可局部保持至 500 ms，持续失效再停止。
- 拇指短暂识别无效时保持已有姿态；SDK 越界拒绝时优先保持上次接受的拇指目标，让四指继续。首次目标或保持重发仍被拒绝时会停止。
- 无固定运行时限；仍保留目标/反馈 250 ms、页面心跳 600 ms 以及 SDK 硬件故障停止。连接恢复后不自动重新使能。
- 没有复杂自碰撞求解。SDK 保护不代表所有手指组合均不会干涉。

识别优化 A 已支持掌心朝前的八姿态个人标定：中立、张开、钩手、指根轻弯和四个对指。旧五姿态继续兼容；完成新采集并应用后，才启用个人骨长与掌面优化。详细行为见 [全手跟随](hand-tracking/README-Full-Follow.md)，采样和动态检查见 [标定说明](hand-tracking/README-Calibration.md)。

新增 [拇指专项连续运动标定](hand-tracking/README-ThumbCalibration.md)：保留基础标定，独立采集自然位、三个运动方向及四组对指往返，为逆解提供连续姿态参考。采样、应用和切换均暂停并失能，应用后先观察预览；A/B 独立保存并可停用回退。

## 常见问题

| 现象 | 检查方式 |
|---|---|
| 识别不动或 PAUSED | 打开摄像头挡板，点击“恢复 / 重新捕获”，让完整左手入镜 |
| 控制指令是 `—` | 本次进程尚未发送运动指令；先查看启动/停止原因，再手动开始 |
| 拇指保持、四指继续 | 目标被限幅/SDK 拒绝或拇指暂时识别无效；页面会提示 SDK 限制 |
| 提示本地 SDK 桥异常 | 看 `logs/follow-events.jsonl` 和 `logs/sdk-dry-run.jsonl`，区分本地 HTTP 与设备反馈问题 |
| 提示 SDK 故障或错误码 | 保留具体错误及日志，不把硬件故障当作识别问题；错误码 3 是参数范围错误，不专指拇指 |
| 页面还是旧文字 | Ctrl+F5 刷新；改过后端代码用 `-Restart` 重新部署 |
| Chrome 打开后处理 FPS 下降 | 先只保留一个前台页面，点“三维预览：关”作对照。页面已限制三维重绘至最多 30 FPS、视频展示 15 FPS、状态推送 10 Hz；后台识别与 SDK 控制频率不随这些展示设置改变 |
| 持续“正在捕获” | 最新版本修复低帧率时连续有效帧计数被处理延迟反复清零；使用 `-Restart` 应用更新。状态行会显示具体拒帧原因。左手映射现默认单手检测，使用时只举左手 |
| 标定收不到有效帧 | 查看标定面板的具体拒帧原因。新版已修复二维侧摆与三维方向冲突导致全部拒帧；更新后仍异常时，保持整手入镜并检查掌面朝向，不必清空已有配置 |
| 启动失败 | 查看 `hand-tracking/logs/mapping.stderr.log`，再看 SDK 容器日志；不必重装镜像 |

```powershell
# SDK 原生运行日志
wslc exec rysen_sdk_readonly tail -n 80 /tmp/hand-sdk-follow.log
# Windows 控制事件
Get-Content D:\Dev\wslc\hand-tracking\logs\follow-events.jsonl -Tail 20
```

日志、模型、个人标定和令牌放在项目运行目录，Git 已忽略。分享代码时不要附带 `profiles/sdk-control-token.txt`。说明索引与开发命令见 [项目 README](hand-tracking/README.md)。
