# Task 3 本机 Ubuntu 检查与验证记录

验证日期：2026-10-23。实际通过 Windows → VMware SSH 执行命令，已核验用户提供的 Ubuntu 主机 ED25519 指纹。以下结果来自用户当前 Ubuntu，不能解释为实机验收。

## 环境和 Git 基线

- Ubuntu 22.04.5 LTS，x86_64，内核 6.8.0-138-generic。
- GCC/G++ 11.4.0，CMake 3.22.1，Ninja 1.10.1，OpenCV 4.5.4。
- OpenVINO 原有 `/opt/intel/openvino_2024.6.0`；实际运行 build `2024.6.0-17404-4c0f47d2335-releases/2024/6`，可用设备 CPU。Task3 默认配置使用 CPU。
- MVS 链接仓库 `io/hikrobot/lib/amd64/libMvCameraControl.so`；依赖解析到 x86_64 动态库。没有替换已安装的 OpenVINO/MVS/驱动。
- 远程 `https://github.com/16370711czh-ops/sp_vision_tutorial_27.git`，检查起点 `final=f89d8ab561a1f788947b194466799c2f52becd28`；GitHub API 和 Ubuntu `git ls-remote` 一致。
- 独立检查工作区 `/home/sp/robomaster-ubuntu-audit.uXASDR/project`，分支 `audit/ubuntu-task1-task2`；在此前四行 GCC JSON 修正基础上集成 Task3，测试时尚未本地提交。用户原 `~/sp_vision_tutorial_27` main 与未跟踪文件保留。

## 审查结论及修复

| 级别 | 来源和位置 | 确定问题/风险 | 处理 |
|---|---|---|---|
| P0 | 起点 `src/task_3.cpp` | GitHub 仍是 Your code 空模板，不能以其可编译表示功能完成 | 替换为完整采集、检测、PnP、跟踪、控制、Plotter 主循环 |
| P0 | `src/task_1.cpp`、`src/task_2.cpp` Plotter JSON | 此前本机 GCC11 已报 packed float 无法绑定转发引用 | 每个文件仅 yaw/pitch 两处转 double；本轮同时实际重编译通过 |
| P1 | 旧本地 `outpost_tracker.cpp` | 关联挑选时没有保存板号，调用旧 void Target::update 又自行选板且绕过 NIS；更新计数没有确认接受 | 统一 NIS 选观测和 id，调用 try_update，成功后才累计和刷新 last_seen |
| P1 | Solver 配置读取 | 原先只读取 task2 重投影参数，Task3 配置下退化为无限门限 | 识别 task3 节点的同名参数；Task1/Task2 回归通过 |
| P1 | 旧 Task3 主循环 | 旧的阻塞姿态查询、分离状态读取、异常路径和 Plotter 请求量不足以匹配当前接口 | 复用 try_q、原子反馈快照、帧龄/反馈龄检查和最终发送快照 |
| P1 | 图纸/旧 Task3 配置 | 图纸底面原点与云台世界原点不同，示例 z 窗口不能证明排除了顶部板 | 三板相对高度估计；显式高度范围；未确认窗口默认拦截发射 |
| P1 | 开火接口 | Task2 单发5Hz逻辑与 Task3 连发12Hz不同 | 单独连续允许电平；弹道无效/丢失/模式无效/过期等均不允许开火 |
| P2 | 初始参数 | 示例标定、半径强先验、延迟和 R 经验模型尚未经实物验证 | 保留参数入口和明确标定要求，不声称最优或实测准确 |

已正确、保持复用的部分：当前 Target 的 120° 三板模型、相同半径、两个有符号相对高度、固定中心 F/Q、实际 h/H 和周期残差、Joseph/LDLT EKF、Task2Aimer 的副本预测与 Trajectory、Gimbal 的最终反馈/模式检查及指令看门狗。没有为 Task3 覆盖这些已经正确的实现。

## 实际执行结果

| 检查 | 状态 | 实测结果 |
|---|---|---|
| 离线 CMake 配置 | PASS | `BUILD_VISION_APP=OFF, BUILD_TESTING=ON`，退出0 |
| 离线 GCC/Ninja 构建 | PASS | 真实 Target/EKF/Tracker/Pipeline/Aimer/Solver 源码参与编译，退出0 |
| 全部 CTest | PASS | **13/13**，84,841 条断言；含原 Task1/Task2 全部回归 |
| Task3 几何、Jacobian、P0/F/Q/R、跟踪 | PASS | **35,613** 条；三板 H 的33个列向量中央差分、π跨越、静止、噪声、丢帧、误检、切板、恢复、副本预测、弹道代入 |
| Task3 pipeline/配置 | PASS | **1,995** 条；独立图纸高度夹具、默认禁发、dry-run、未验证窗口、反馈/图像/姿态无效、NIS拒绝不计数、非法 YAML |
| Task3 独立 PnP 夹具 | PASS | **55** 条；27组世界姿态与三板高度、负15°板倾角、云台三轴姿态及平移；Task3重投影门限读取 |
| 完整机器人 CMake | PASS | `BUILD_VISION_APP=ON`，使用原有官方 OpenVINO，退出0 |
| `task_1 task_2 task_3` 构建链接 | PASS | 三个真实 ELF64 x86_64 可执行文件，退出0 |
| `--help` 与 ldd | PASS | 三程序帮助页退出0；三个 ldd 无 not found；Task3 不加载 setupvars 的 ldd 也解析成功 |
| 当前 Task3 配置/CPU YOLO | PASS | 原模型实际加载、空图像返回空，三帧720×1280黑图推理0检测，约13.65/9.86/10.85ms |
| 实际 Plotter UDP loopback | PASS | 收到并解析 yaw/pitch/control/fire/send_ok/angular_velocity 数值字段，合计9条运行探针检查；没有构造 Camera/Gimbal |
| 首次运行探针 | FAIL（已解决） | 探针自身配置路径漏写 task3 子目录，YAML::BadFile，退出134；修正探针后退出0。不是 Task3 主程序失败 |
| 实物相机、有效目标视频、串口云台 | NOT TESTED | 遵循用户约束，未打开设备；黑图推理不能证明实际 outpost 检出和 PnP 质量 |
| 实物角速度误差±0.4、真实跟随 | NOT TESTED | 需要同步考核真值、真实装甲板与机器人 |
| 12Hz、30s血量/命中评分、发弹 | NOT TESTED | 没有发射，也没有确认控制器连发语义或物理开火挡位保护 |

没有将首次失败隐藏为通过；失败 transcript 与修正后 transcript 均保存在本机审查日志。没有新的源码编译失败。GitHub Actions 已扩展覆盖三任务，但它是另一套执行环境，不能用它替代本表的本机结果。

### 合成轨迹的独立真值误差

带8mm位置噪声、0.025rad姿态噪声、20%时间抖动、周期丢帧、顶部板/远距误检。每段12s；前4s为预热，误差按余下样本与生成器的真值比较，**不是根据滤波方差反推误差**。

| omega真值(rad/s) | FPS | 平均绝对误差(rad/s) | 最大绝对误差(rad/s) | 最大高度误差(m) |
|---:|---:|---:|---:|---:|
| 2.1 | 50 | 0.00236 | 0.00830 | 0.00270 |
| 2.5 | 50 | 0.00230 | 0.00879 | 0.00360 |
| 2.9 | 50 | 0.00316 | 0.00926 | 0.00345 |
| -2.1 | 50 | 0.00251 | 0.00903 | 0.00323 |
| -2.5 | 50 | 0.00286 | 0.00905 | 0.00325 |
| -2.9 | 50 | 0.00258 | 0.00931 | 0.00322 |
| 2.5 | 25 | 0.00293 | 0.01111 | 0.00615 |
| -2.5 | 25 | 0.00525 | 0.01544 | 0.00228 |

这些结果只适用于测试给定的模型、噪声和种子，未验证真实关键点偏差、运动模糊、反光、姿态同步误差及发弹命中。无法据此承诺实物 ±0.4rad/s 或35分。

## 日志、修改范围和复现

Ubuntu 日志：`/home/sp/robomaster-ubuntu-audit.uXASDR/task3/logs/`；最终明细 `final_ctest.txt`、`final_robot_build.txt`；实际 CPU/UDP 探针源及构建脚本在同目录 `task3/checks/`。Windows 也保留 SSH 原始 transcript、退出码与脚本哈希。项目测试源码已经随本提交进入 tests/；探针是额外审查工具，其记录单独打包，不作为相机/串口驱动替身。

文件清单、完整模型、保护条件和全部构建/启动命令见 [TASK3_GUIDE.md](TASK3_GUIDE.md)。根 CMake 未变；Target、EKF 和 Task2Aimer 未改。Task1/Task2 唯一主程序改动是 packed JSON 字段的四行强制转换。

结论：Task3 工程代码完整，在用户 Ubuntu 上可编译、可加载、离线测试通过。可以提交代码并安排人工监督的下一阶段；当前默认高度窗口和示例标定尚待测量，不能直接宣称符合全部实机评分要求。任何带配置主程序运行都涉及真实设备，`--dry-run` 也不是无设备模式；本轮未执行。
