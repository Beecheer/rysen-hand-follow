> 2026-10-20 当前更新：D435 主服务使用 `rgb_depth_root_correction`。以 RGB 三维骨架为主，深度弱修正掌面和 MCP 屈伸／侧摆，PIP/DIP 与拇指路径保留；现有标定兼容。137 项 Python 测试通过，实景效果待操作者验证。实现及参数见 README-D435.md 末节。

# 代码结构与维护

## D435 全链路补充

`Start-DepthFollow.*` 使用 `--tracker d435`，由 `depth_session.DepthMappingSession` 复用下述标定、URDF、滤波、跟随、SDK 和网页。`depth_source` 负责相机身份绑定、米制参考及 MediaPipe 兼容结果适配，采集与融合从工作区 `realsense-tools/depth_camera.py` 和 `hand_fusion.py` 导入同一实现。

D435 当前使用 `d435-articulated.json`（schema 4）。`SimpleDepthCalibration` 只采伸直略分开、并拢伸直两步；`ArticulatedDepthEstimator` 以图像重投影拟合四指固定骨长关节链，掌部深度拟合坐标系，逐指表面深度与几何遮挡关系辅助修正。掌面使用腕侧区域、一致平面选择、掌部关键点和新鲜腕深度降级，避免把握拳的混合表面当成整手丢失。`TipDepthThumb` 优先由点 1/4 深度确定指尖空间位置，用 URDF 根部几何闭式解析得到 j0/j1/j2；指尖不可见时使用 `DirectThumb` 的点 1/2/3 方向回退，无迭代拟合，j3/j4 仍固定。主页面将相机融合器切换至 `observe_only`，不调用旧逐点 Z 优化。旧 `d435-current.json` 和拇指文件保留，独立 8095 仍是旧逐点融合诊断。

米制参考与基础姿态原子保存，绑定相机内参和算法；运行数据带参考哈希，控制源验证与当前保存配置一致。标定、左右手切换和末端固定值修改需 `IndexFollow.stop_confirmed()` 观测新鲜五指失能反馈，HTTP STOP 被接收不再等同于失能已确认。完整接口、操作及验证见 [D435 说明](../README-D435.md) 和 [验证记录](D435-VALIDATION.md)。

`thumb_static.StaticThumbCalibration` 提供可选四位置静态标定，使用根部/指尖相对坐标和可编辑的模型参考姿态，建立三维仿射位置对应，接入 `TipDepthThumb.calculate_tip`。远离采样支持时淡出至基础映射。新配置 `d435-articulated-thumb-static.json` 独立保存，绑定基础骨架、拇指固定设置和限位。参考预览通过快照 `thumb_calibration.reference_joints` 覆盖前端模型显示，不写入 `mapper.positions` 或 SDK 源；采集及预览仍需确认停止，退出参考预览需恢复识别。详情见 [静态标定](../README-ThumbStatic.md)。

原 A/B 架构如下。

## 数据流

```text
Windows 摄像头 → A / MediaPipe 或 B / WiLoR 21 关键点
    → geometry（旧模式）或 human_pose（八姿态个人结构）
    → calibration / thumb_filter / thumb_calibration / kinematics → PreviewMapper
    ├→ FastAPI :8090 → 网页 Three.js / 官方 URDF 预览
    └→ IndexFollow（目标限幅、页面心跳、识别有效性）
          → HTTP 长连接 → wslc SDK bridge :8092
          → C++ worker → SDK 1.4.7 → 左手机械手
机械手状态 → C++ worker → bridge → HardwareMonitor → 页面和日志
```

`IndexFollow`、`index_camera_worker.cpp` 的名称来自早期食指试验，现在通过全手模式开关共用实现。保留名称是为了兼容脚本、容器部署路径和历史试验，不要把它们当成只支持食指的废弃代码。

浏览器展示与控制分离：三维最多 30 FPS、渲染像素比为 1，只在新识别帧到达时更新关节；后台页和视口外的模型暂停绘制。视频展示最多约 15 FPS，状态推送 10 Hz；后台页面断开视频/WebSocket 订阅，回到前台再恢复。切换后台仍按原逻辑请求停止实机跟随，回到前台不会自动使能。网页提供独立三维开关用于定位浏览器渲染负载，关闭三维不停止识别或主动暂停机械手。首页不缓存，主脚本 URL 带内容哈希，避免浏览器混用旧页面脚本。

`/api/status` 的 `frame_diagnostics` 提供检测到的左右手标签及分类分数、整帧处理耗时、可用/拒绝帧计数和当前拒帧原因。它用于区分未检测到手、左右手不匹配、低分、掌部越界和结果过期，不代表新增逐关节置信度。

左手映射模式默认 `--num-hands 1`；第一阶段纯摄像头工具仍保留原有双手默认值。MediaPipe 视频跟踪在已跟踪手数少于配置上限时会重新调用掌部检测，因此单手控制不应默认要求两只手。参考[官方 `hand_landmarker_graph.cc`](https://github.com/google-ai-edge/mediapipe/blob/master/mediapipe/tasks/cc/vision/hand_landmarker/hand_landmarker_graph.cc) 的 `has_enough_hands` 门控逻辑。命令行仍可显式用 `--num-hands 2` 覆盖。

连续有效帧按采集时间之间的间隔判定。2026-10-19 修复了处理完成时间与上一帧采集时间比较，导致延迟被重复计入、约 10 FPS 时连续计数反复清零的问题。短暂的 200 ms HOLD 不再抹掉已经观测到的连续帧；无效帧或采集间隔达到 200 ms 会中断计数。结果本身超过 200 ms 仍无效，500 ms 丢手暂停仍需手动恢复。回归覆盖 10 FPS、110 ms 处理延迟的捕获场景。

八姿态 schema 3 启用 `human_pose`：以个人掌面模板和骨长拟合四指结构，二维方向辅助侧摆。拟合前将三维骨段链绕掌面法向对齐到二维侧摆，避免图像/世界坐标方向差异形成不可消除的残差；屈伸仍依赖三维观测。采样阶段也使用该层，但不使用旧个人骨长作为新采样约束。`human_structure` 返回掌面质量及各指残差，`calibration.rejections` 保留本次采样拒帧原因计数。采样与保存细节见 [标定说明](../README-Calibration.md)。

## 模块职责

| 目录/文件 | 职责 |
|---|---|
| `hand_preview/app.py`, `__main__.py` | 纯摄像头预览、模型下载和基础参数 |
| `hand_preview/mapping_app.py` | 三维预览入口、摄像头线程、HTTP/WebSocket、生命周期 |
| `hand_preview/geometry.py` | 四指弯曲和侧摆几何解算 |
| `hand_preview/human_pose.py` | 方案 A：个人掌面、固定骨长四指拟合、拇指结构重建和动态一致性统计 |
| `hand_preview/comparison.py` | 同源帧 A/B 对照、最新帧队列、独立 B 映射/标定、可选录像 |
| `hand_preview/wilor_detector.py` | 实机 B 直接推理适配、异常帧无效化和 HTTP 连接恢复 |
| `../wilor-backend/` | 独立 uv 3.10 CUDA 推理进程、固定模型/依赖、仅本机 8093 接口 |
| `../wilor-backend/Stop-Backend.ps1` | 识别 GPU 服务身份、请求关闭并确认 8093 释放，全手和对照入口共用 |
| `hand_preview/kinematics.py` | URDF 正运动学、拇指约束拟合 |
| `hand_preview/calibration.py` | 标定采样、中立偏置和拇指对指约束 |
| `hand_preview/thumb_filter.py` | 拇指任务空间中值、静止死区、按连续运动调整的时间滤波 |
| `hand_preview/thumb_calibration.py` | 独立八步拇指轨迹采样、参考节点、连续姿态先验与基础配置兼容校验 |
| `hand_preview/mapping.py` | 映射滤波、关节联动和识别状态 |
| `hand_preview/index_follow.py` | Windows 目标裁剪、跟随状态与浏览器心跳 |
| `hand_preview/follow_bounds.py` | 预览/SDK 限位交集、拇指余量、联动域；启动时生成限位文件 |
| `hand_preview/bridge_client.py` | 本地 HTTP 长连接；失败控制请求不重放 |
| `hand_preview/hardware_monitor.py` | 设备反馈监控、模式保留、日志和早期模拟规划 |
| `hand_preview/sdk_follow_bridge.py` | 容器内 HTTP 桥、令牌校验、原生进程 stdin/stdout |
| `hand_preview/sdk_*control.py` | 容器内服务 PID 归属检查与启停 |
| `hardware/index_camera_worker.cpp` | 真实 SDK 连接、使能/失能、位置跟随、独立看门狗、拇指保持重发 |
| `hardware/test_*.py` | 编译并运行模拟 SDK 的回归检查，不驱动硬件 |
| `hardware/` 的 trial/cycle 程序 | 历史单指硬件试验；不由日常启动入口调用 |
| `web/` | 本地页面、Three.js、URDFLoader、控制状态和反馈表 |
| `assets/apex_left/` | 官方左手 URDF/STL；保留模型来源信息 |
| `tests/` | Windows 单元/本地 HTTP 测试及模拟 SDK 头文件 |

## 启停层次

根目录的 `Start-HandFollow.ps1` 检查现有服务并复用健康全手会话；`-Restart` 才强制重新部署。底层调用 `Start-HandFullFollow.ps1` → `Start-HandIndexFollow.ps1 -FullHand`。底层负责容器归属/互斥检查、上传桥接源码、生成限位、用真实 SDK 编译原生 worker、启动 Windows uv 应用。

根入口默认 `-Tracker wilor`，先确认独立 B 标定和 GPU 后端就绪；A 可显式选 `-Tracker mediapipe`。实机使用单一识别后端，不能启用 A/B 对照。`hardware/device-limits.json` 保存 SDK 日志证实的拇指 j1 边界及版本证据，和 URDF 共同生成限位；实际 SDK/固件版本不匹配则不进入使能。停止根入口还会关闭由本工作区提供的 WiLoR 推理服务。

根目录的 `Stop-HandFollow.ps1` 调用 `Stop-HandFullFollow.ps1` → `Stop-HandIndexFollow.ps1`，请求 Windows 服务退出并停止目标转发，等待原生失能/退出，然后停止专用容器。失败时报告未确认环节；不通过强制杀死未知 PID 来伪装停止成功。

SDK 停止的摄像头释放请求先于容器检查，避免容器不可查询时遗漏相机。根停止入口即使 SDK 环节报错，也会继续尝试独立停止 WiLoR，最后汇总错误。WiLoR 通过身份验证后请求关闭，并等待 8093 释放；端口被未知服务占用时拒绝修改。启动入口把 GPU 准备纳入失败回收路径，重启异常时同时清理 GPU。

日常启动后机械手保持失能。网页手动开始才发送 START。所有运行方式共享同一摄像头与机械手，不要同时启动不同控制器。

## 运行文件与配置

- `uv.lock` / `.python-version`：依赖锁和 Python 3.11；修改依赖后应显式更新锁文件。
- `profiles/`：个人标定、控制令牌、启动时生成的 `full-hand-limits.json`。不要清理或公开令牌。
- `models/`：MediaPipe 模型；已存在时不需重新下载。
- `logs/mapping.stdout.log`, `mapping.stderr.log`：Windows 服务输出。
- `logs/follow-events.jsonl`：开始与停止原因。
- `logs/sdk-dry-run.jsonl`：实际 SDK 反馈与目标诊断，文件名为历史名称。
- `logs/communication-check.json`：最近一分钟失能通信测试结果，不代表实际运动长测。
- 容器 `/opt/hand-follow/`：部署源码、真实 worker、模式和限位文件。
- 容器 `/tmp/hand-sdk-follow.log`：SDK 与原生进程诊断。

现有设备 IP、端口和 SDK 模型路径保留在工作区启动脚本和 worker 中，当前是单台左手机械手配置。更换设备时应同时核对 IP、左右手、模型与限位，不只替换网页地址。

## 验证

Windows 检查：`uv run --frozen python -m unittest discover -s tests -q`。2026-10-19 最近一次共 84 项通过，覆盖几何、个人结构、八姿态及拇指专项采样、配置备份、动态统计、A/B 适配与隔离、可选录像回读、滤波、跟随、长连接与错误处理。前端检查为 `node tests/frontend-recovery.cjs` 和 `node tests/frontend-thumb-calibration.cjs`。实际验证记录见 [VALIDATION.md](../VALIDATION.md)。

原生模拟文件在启动部署之外上传到专用容器后，可执行：

```powershell
wslc exec rysen_sdk_readonly python3 /opt/hand-follow/test_index_camera.py
wslc exec rysen_sdk_readonly python3 /opt/hand-follow/test_full_hand_camera.py
```

这两个脚本及 `sdk_stub/rysen_apexhand_sdk.hpp` 是开发测试资料，日常启动不上传它们；当前工作区已上传过。真实编译只使用 `/workspace/rysen_sdk/include`，绝不能把 `tests/fixtures/sdk_stub` 加入真实编译路径。模拟通过后仍需用户手动开始做实际方向与响应验证。

PowerShell 脚本可用 `System.Management.Automation.Language.Parser.ParseFile` 检查语法。启停改动应实际验证启动、重复启动、停止、重复停止，并确认五指失能；无需发送运动来验证脚本。

`pwsh -NoProfile -File tests/test_launchers.ps1` 在临时目录用隔离桩验证启动、复用、重启、失败回收、停止错误汇总及未知服务保护；不会启动摄像头、GPU 或 SDK。阶段收尾按用户要求保持停服，因此本轮真实运行只验证停止与重复停止，启动场景采用桩测试，不能将其描述为本轮真实硬件启动通过。
