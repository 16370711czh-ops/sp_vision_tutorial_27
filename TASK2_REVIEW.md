# Task 2 提交版：审查、修改与验收说明

本交付以正式大作业 Task 2 为需求依据，以用户 fork 的 `final` 分支为接口依据。它包含完整实现、配置、构建入口、离线测试和补丁；当前主机未连接机器人，实机验收状态不能由离线测试代替。

## 1. 代码审查报告

### 1.1 实际检查了哪个版本

本次联网查询确认：

- 用户仓库：`https://github.com/16370711czh-ops/sp_vision_tutorial_27`。
- 用户分支：`final`，提交 `88f877caa395191a5789a7a99eebd486b8b498f9`。
- 课程上游：`TongjiSuperPower/sp_vision_tutorial_27` 的 `final_project_aim` 指向同一提交。

重新从 GitHub 取得了包括 task_2、Target、EKF、Solver、Gimbal、Trajectory、Camera、YOLO、YAML 与 CMake 在内的 28 份真实源文件；完整 72 份基准文件逐一核对 Git blob SHA-1。记录在 `review/SOURCE_MANIFEST.json`，不是根据文件名猜接口。

原 DOCX 的 Task 2 已重新读取，需求摘录在 `review/TASK2_REQUIREMENTS.txt`，清单记录了 DOCX 的 SHA-256。要求包括：约 3m 定点击打；低速 3～5rad/s、中速 5～9rad/s；角速度容许误差分别 ±0.6、±1rad/s；单发请求至多 5Hz；每挡前 10 颗实际弹丸计分；Plotter 显示控制命令与 EKF 角速度。

**远端 final 与先前本地交付是两个版本。** 远端仍是原模板，主循环为空；先前本地工程已有 TargetConfig、task2_aimer.hpp/.cpp、task_2_fire.hpp、SingleShot、task2.yaml 和 CMake 增补。本次保留这些已经成立的实现，再修正确定的问题。仅把某一个 .cpp 复制到原模板中并不能完成集成。

### 1.2 确定缺失或错误

P0 表示功能/集成阻断，P1 表示明显数学或运行风险，P2 表示可靠性与诊断问题。

| 程度 | 来源与问题 | 原因 | 本次处理 |
| --- | --- | --- | --- |
| P0 | 远端 task_2 主循环为空 | 相机、识别、解算、估计、控制没有串起来 | 接入完整 Task2Pipeline，与实机主程序共用跟踪逻辑 |
| P0 | 远端缺少 TargetConfig、新接口和 Aimer/SingleShot 配套文件 | 本地增强实现尚未进入该分支 | 成套提供声明、实现、YAML 与 CMake；原模板本身并非因为缺少这些未引用的文件而必然编译失败 |
| P1 | 远端 P0 的 r/l/h 方差为零、Q 中也为零 | 对应协方差行列始终为零，几何参数无法从测量学习 | 使用可配置的正初始方差 |
| P1 | update_ypda 清零 H 的位置对转角偏导 | H 与实际非线性 h 不一致 | 保留真实偏导，用中心有限差分检查 |
| P1 | 首次换板前直接清零 H 的几何列 | 清零后的矩阵不再是同一个 h 的雅各比 | 移除此不一致操作，依靠几何先验及完整协方差更新 |
| P1 | 两板模型仍循环访问最近三个候选 | armor_num=2 时越界 | 对实际板数匹配，不固定访问第三块 |
| P1 | Armor.priority 未由 YOLO 构造过程赋值，却被 Target 读取 | 读取未初始化数据 | Target 明确初始化优先级；默认 Target 的枚举、计数与标志也初始化 |
| P1 | Gimbal::q 可能无限等待、做区间外插值 | 无超时；早于最早样本时仍外推；相同时间戳还可能除零 | 有界四元数历史，只做夹在两个样本之间的 slerp，拒绝重复时间戳与大间隔 |
| P1 | 首个反馈包没有等待上限 | 断串口时程序停在构造阶段 | 首包 1s 超时，失败清理线程后报告错误 |
| P1 | 忽略 solvePnP 返回值和异常位姿 | 失败输出、坏关键点可能进入状态估计 | 校验四点、凸性、有限值、PnP 成功、正深度和重投影误差 |
| P1 | 标定向量直接通过 data() 映射 Eigen | 长度不对可能越界读 | 验证数组长度、有限值、旋转矩阵和内参 |
| P1 | 无独立命令过期机制 | 推理卡住时上次控制可能保持 | Task2 启用串口读线程中的命令超时停控；仍需电控自身看门狗 |
| P1 | YOLOBase 缺少虚析构，unique_ptr 通过基类删除派生对象 | 存在未定义行为及资源释放风险 | 添加虚析构；保留原识别算法 |
| P1 | crc.cpp 使用 std::size_t 却缺少显式 cstddef | 实际独立编译时报错，依赖其他头文件的间接包含不可移植 | 补齐必要的标准头，协议及 CRC 算法保持不变 |
| P2 | 相机容量为 1 的队列满后丢弃新图 | 处理较慢时取到积压旧帧 | 用已有 replace 接口保留最新图像 |
| P2 | 相机转换异常可能逃出采集线程 | 不支持的像素格式或 OpenCV 异常会终止进程 | 捕获帧转换错误，仍释放 SDK 图像缓冲 |
| P2 | S.inverse 和 P.inverse 用于增益/诊断 | 不必要的显式求逆；奇异协方差可能污染诊断 | LDLT 求解，Joseph 更新，对称化及有限/半正定检查；坏测量不提交校正 |
| P2 | 四维观测的“95%阈值”写成 1.5 | 不是对应分布的 95% 分位点 | 诊断阈值改为 9.487729；运行拒绝门限另设为可调的 18.5 |
| P2 | 以 posterior-prior 校正量命名 NEES | 真正 NEES 需要真实状态，角度 wrap 还会制造假跳变 | 改为切空间 correction_mahalanobis；旧 nees 键标为不可用 |
| P2 | 旧本地主程序和 Target 分别选一次板 | 两次评分不同，可能通过 A 板门限却融合 B 板编号 | 同一候选和 id 用于评分、门限与融合 |
| P2 | 无效观测容易触发整段跟踪重建 | 噪声帧使速度历史丢失 | 拒绝坏测量、短失保留、超时重建；不同类别不会直接劫持当前目标 |
| P2 | 弹速为零时无法做无发射跟随 | 关闭摩擦轮时反馈可能为零 | 明确提供非弹道跟随，ballistic_valid=0，不能通过发射门限 |

### 1.3 已经正确实现、继续复用的内容

- 课程的 11 维状态定义、车辆四板的交替半径和高度模型。
- 匀速平动、匀角速度 F；分段白噪声的 dt⁴/4、dt³/2、dt² 结构。
- EKF 先用先验态计算 innovation/S/K，再更新状态的顺序。
- Joseph 形式的后验协方差更新。
- yaw/pitch/distance/板朝向的四维测量；各角度残差按周期处理。
- 原 Solver 的 camera→gimbal→world 变换、外参约定和 yaw 优化策略。
- 既有 Task2Aimer 的副本预测、飞行时间迭代、角度连续化与无空气阻力弹道。
- SingleShot 的时间节流和 false 帧分隔。
- 相机读取超时 API、Plotter JSON/UDP API、已有串口包布局及 CRC。

未重写 YOLO 网络、PnP 算法、矩阵滤波器框架或弹道系统。R 和过程噪声数值是可运行起点，未称其为最优标定。

### 1.4 风险与确定错误的区别

真实 PnP 朝向误差、图像模糊、R 的相关性、接收时延、相机标定、弹速反馈及电控开火语义属于待实机核对的风险。当前没有实机数据，不能判定它们已经发生，也不能判定已排除。见第 7 节。

## 2. 修改文件清单与兼容范围

### 新增

- `tasks/auto_aim/task2_aimer.hpp/.cpp`：与先前本地版本一致的基础上增加显式非弹道跟随状态。
- `tasks/auto_aim/task2_tracker.hpp/.cpp`：关联、确认、目标生命周期及就绪状态。
- `tasks/auto_aim/task2_pipeline.hpp/.cpp`：实机与仿真共用的估计/控制决策入口。
- `tasks/auto_aim/task2_config.cpp`：类型化参数读取和验证。
- `src/task_2_fire.hpp`：5Hz 单发请求节流，默认不允许发射。
- `tools/quaternion_history.hpp`：有界、线程安全的姿态插值缓存。
- `configs/task2.yaml`：完整示例配置，继承 standard.yaml 的设备参数。
- `tests/CMakeLists.txt`、`task_2_test.cpp`、`task2_acceptance.cpp`、`task2_solver_roundtrip.cpp`、`run_task_2.sh`。
- `.github/workflows/task2.yml`：Ubuntu 离线 CI 定义；2026-10-22 本地验收时未触发 GitHub Actions；GitHub 上的运行状态以 Actions 页面为准。
- 本审查报告、测试报告、运行说明、原始日志和来源证据。

### 修改

| 文件 | 修改内容 |
| --- | --- |
| src/task_2.cpp | 补齐视觉数据流、模式检查、无发射默认值、控制及 Plotter |
| target.hpp/.cpp | 配置/接口、初始化、真实 H、统一创新评分、拒绝校正、边界检查 |
| extended_kalman_filter.hpp/.cpp | LDLT、Joseph 数值检查、原子提交、诊断修正 |
| solver.hpp/.cpp | 标定、四元数、PnP 输入/输出及重投影检查 |
| gimbal.hpp/.cpp | 一致反馈快照、有界姿态、可选目标编号、过期停控与写入状态 |
| hikrobot.hpp/.cpp | 初始化采集状态、最新帧队列、转换异常处理 |
| yolo.hpp | 独立头文件所需 include 和虚析构 |
| trajectory.hpp/.cpp | 初始化无解输出、输入检查、低弹道根的数值稳定性 |
| 根、tools、io、auto_aim 的 CMakeLists.txt | 完整依赖、源文件集成、离线测试开关，尊重 OpenVINO_DIR |
| tools/crc.cpp、.gitignore | 补齐 cstddef；仅放行新增 Task2 CI 文件，其余 GitHub 目录内容仍忽略 |

### 保持不变

针对当前远端 final 的补丁不修改 `src/task_1.cpp`、`src/task_3.cpp`、YOLOV5 推理与后处理实现、armor.cpp、Camera 工厂接口、串口库源码、Plotter 实现、原模型和相机 SDK 资产，也不改原 standard.yaml/sentry.yaml。

共享 Target 保留此前第三部分已完成的 outpost 字段/模型，旧 Armor/void update 接口保留；新的 NIS 拒绝通过 Task2 的 try_update 接口显式启用，避免给 Task3 旧调用增加隐式拒绝语义。Gimbal 原四参数 send 调用保留，新增目标编号为默认参数。串口线格式不变。

共享依赖改动仍可能影响实际设备行为，因此合并包保留前两次任务成果，并运行可执行的 Task1/Task3 兼容性测试；测试结果以 `TASK2_TEST_REPORT.md` 为准。

## 3. 完整工程与补丁

交付提供完整工程 ZIP，以及两份不同基准的补丁：

- `task2-final.patch`：针对用户当前 final 的上述提交；只修改 Task2 及必要依赖，不覆盖两个其他主程序。
- `task2-upgrade-from-task3.patch`：针对上次已完成三部分的本地完整包，保留已有 Task1/Task3 实现。

ZIP 保留此前三个任务的成果，并采用本次完善的 Task2 和共享依赖。正式 Task2 的差异范围请以 `task2-final.patch` 和 `review/CHANGED_FILES.txt` 为准。

在正确基准的工作区执行：

```bash
git apply --check /你的路径/task2-final.patch
git apply /你的路径/task2-final.patch
git diff --stat
git diff
```

若已经应用过之前的大作业改动，使用对应升级补丁并先检查。补丁经过可应用性检查及应用后逐文件内容比对，没有伪造用户提交日志。

## 4. 数学模型与状态估计流程

### 4.1 状态、F、P0

```text
x = [cx, vx, cy, vy, cz, vz, a, omega, r, l, h]^T
```

位置单位 m，线速度 m/s，a 为 rad，omega 为 rad/s。r/l/h 均为 m。四板模型中两组半径分别 r 和 r+l，高度分别 cz 和 cz+h。

F 为单位阵，并设置 F(0,1)=F(2,3)=F(4,5)=F(6,7)=dt。dt 来自 steady_clock 图像时刻差，不能以固定帧率替代。重复或倒退时刻不融合；超时按丢失重建处理。

初始中心为观测板位置加 r×朝向单位向量，初始速度和 omega 为零。P0 对角为 `[1,64,1,64,1,64,0.4,100,0.01,0.01,0.01]`，后三项可配置。它们是方差，0.01m² 对应 0.1m 标准差，不是几何尺寸。

### 4.2 Q

对每个位置/速度或转角/角速度二元组，采用分段常加速度白噪声：

```text
Q_pair = variance × [[dt^4/4, dt^3/2],
                     [dt^3/2, dt^2]]
```

平面加速度方差默认 0.5，高度方向 0.1，角加速度方差 4.0。几何量按常值处理，Q 对应项为零，但 P0 非零，所以可以学习。此模型的方差含义是每步加速度的不确定性，不应与连续白噪声谱密度的 dt³/3 模型混用。

### 4.3 h 和 H

第 i 块板：

```text
ai = wrap(a + i×2π/N)
ri = r+l  （四板的i=1或3）；其他为r
pi = [cx-ri cos(ai), cy-ri sin(ai), cz+hi]
hi = h    （四板的i=1或3）；其他为0
h(x,i) = [atan2(yi,xi), atan2(zi,hypot(xi,yi)), norm(pi), ai]
```

H 是该 h 的真实导数，采用链式法则 `H = J_xyz_to_ypd × J_state_to_armor`，并补上装甲板朝向行。关键偏导包括 `dx/da=ri sin(ai)`、`dy/da=-ri cos(ai)`，交替板对 l/h 的偏导按组生效。没有清零这些真实导数。测试在两板/四板、±π 附近逐列进行中心有限差分，角度差按周期求差。

### 4.4 R、关联和校正

观测 z 使用同一空间板的 `[world_yaw, world_pitch, distance, armor_yaw]`。前三项由 PnP 的 world xyz 计算，最后一项来自 Solver 优化后的装甲板朝向。不能把云台 yaw 当作板的 yaw。

R 沿用仓库的随距离与板朝向变化的对角模型：

```text
R_yaw = R_pitch = 5e-4
R_distance = (1e-2 + 5e-3 d²) × (log(abs(delta_angle)+1)+1)
R_armor_yaw = 5e-2 + 1e-2 d² + 2e-1/d
```

这些量作为方差使用。对角近似忽略了 PnP 位置与朝向之间可能存在的相关性，需用实际残差检查其适用性。

对满足类别、尺寸、空间距离和朝向硬门限的候选板，计算创新 r=z-h 和 S=HPHᵀ+R，以 rᵀS⁻¹r 评分，使用同一对候选/id 做 try_update。矩阵运算用 LDLT 求解，不显式构造逆矩阵。超过 max_nis 或数值无效时，不提交测量校正。

这里“不提交”指该次 update 不改变 x/P；同一帧先前已经完成的时间预测仍保留。坏候选不会让目标直接变成另一个单位。目标失效/超时之后才允许重新确认身份。

接受测量后执行 Joseph 更新并对称化 P。P 小只代表模型内部不确定度小，不代表相对真实 omega 的误差已达标；合成测试使用已知真值直接计算误差，实机要另有转速真值或标定参考。

### 4.5 预测、控制与时间坐标

图像接收时间与姿态接收时间都来自主机 steady_clock。使用同一时钟避免时间域混用，但仍有传输与曝光延迟：

```text
pose_query_time = image_receive_time - camera_to_pose_offset_ms / 1000
```

此参数表示图像接收延迟减姿态接收延迟，允许正负，默认 0 是未测量起点。姿态必须有时间包围样本，超过 50ms 的姿态间隔不插值；不以外推姿态冒充同步结果。

Solver 继续使用 camera→gimbal→world 变换。相机位置/姿态使用图像时刻，控制反馈 yaw/pitch 和弹速使用最新一致快照。世界原点随机器人移动，因此自身加速仍是模型误差来源；题目要求先到固定位置再击打。

弹道预测在副本上迭代 `lead_time=frame_age+fire_delay+fly_time`。pitch 沿用仓库击打模块向上为负的控制约定；所有控制角度均为 rad，YAML 中注明的角度参数才按 degree 读入并转换。

无有效弹速时可输出显式非弹道跟随，ballistic_valid=0。只有有效弹道、观测、模式、反馈、估计状态、角度误差和单发频率全部满足时，人工启用的发射选项才可能请求一发。

## 5. 测试报告

实际执行结果、失败/受限项目、原始日志和仿真条件见 `TASK2_TEST_REPORT.md` 与 `validation/`。不要把目标文件编译、离线链接、图像合成往返、无发射仿真和实机验收当成同一结论。

## 6. 使用说明

完整命令和验收步骤见 `TASK2_RUNBOOK.md`。首次使用保持 `--fire=false`，先执行离线 CTest，再用 `--dry-run=true` 检查真实视觉链路，最后做无发射跟随。需要发弹时由人工核对电控单发语义、摩擦轮/挡位、标定和弹丸计数后执行。

## 7. 剩余风险与人工确认项

1. 本机没有机器人、相机或实际串口，未进行真实跟随、真实弹频、命中率和 Plotter 软件截图验收。
2. 完整 Linux/MVS/OpenVINO 程序的配置、链接和启动状态，以测试报告中的实际环境结果为准；Windows 离线测试通过不等于这部分通过。
3. 设备内外参、图像旋转/颜色、yaw/pitch 补偿和相对时延必须实测。示例配置继承课件设备参数，不是本次标定结果。
4. 原 Solver 的固定装甲板倾角和有限角度搜索仍存在；真实 PnP 可能有系统偏差。R 未经实机辨识。
5. 无空气阻力弹道没有完整描述空气阻力、枪口偏移和云台动态，fire_delay 需要标定。
6. 协议没有摩擦轮状态或考核轮次字段。程序不按 omega 穿越 5rad/s 擅自重置 10 发预算；shot_limit 是进程请求总上限。两挡实际前 10 发应由考核人员结合电控计数确认，不能把请求计数等同实际出弹。
7. 5Hz 和 false 帧只验证了视觉请求序列；固件是否按每次 fire 请求单发，需要人工确认。主机串口写成功也不等于电控接收或出弹成功。
8. 软件超时停控是尽力措施；主机断电、进程被强制终止、串口彻底断开时，最终必须依赖电控的命令超时保护。
9. 严重低帧率、遮挡、同类目标重叠与转速采样混叠无法由小协方差排除。需观察帧龄、观测、track_id、创新及真实转速对照。
