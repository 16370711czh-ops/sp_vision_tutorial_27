# Task 2 编译、无发射验证与人工验收

先阅读 TASK2_REVIEW.md，再使用本说明。实际执行记录见 TASK2_TEST_REPORT.md。默认启动不发送 fire 请求。

## 1. 使用哪个交付

- 完整 ZIP：保留上次已完成的 Task1/Task3，并更新本次 Task2；适合查看、独立编译或作为完整参考。
- task2-final.patch：应用到用户当前 final（88f877c）上，只修改 Task2 和必要依赖。
- task2-upgrade-from-task3.patch：应用到上次完整三任务交付上。

本仓库 final 分支已直接集成 Task 2 修改，正常使用不需要再应用补丁。只有使用此前本地交付包时，才按对应基准先执行 git apply --check；不要把两份不同基准的补丁连续应用。

## 2. 先构建离线测试

在 Ubuntu 22.04 的工程根目录，安装课程环境尚未具备的离线依赖：

```bash
sudo apt-get update
sudo apt-get install -y cmake ninja-build g++ libeigen3-dev libopencv-dev \
  libyaml-cpp-dev libfmt-dev libspdlog-dev nlohmann-json3-dev

cmake -S . -B build-offline -G Ninja \
  -DBUILD_VISION_APP=OFF -DBUILD_TESTING=ON
cmake --build build-offline -j2
ctest --test-dir build-offline --output-on-failure
```

也可以运行 `bash tests/run_task_2.sh`。这条路径不实例化 Camera、Gimbal 或 OpenVINO，不接触串口，不发送弹丸请求。它编译并运行真实生产状态估计和控制决策代码，另用真实 OpenCV 执行 Solver/PnP 合成往返。

标准 Ubuntu 的 libopencv-dev 存在时，包含 task2_solver_roundtrip；如果只提供 OpenCV 头文件，CMake 明确报告该运行测试未启用，不能把它标为通过。若完整包中有旧 Task1/Task3 的测试和源码，也会增加兼容性测试。

Windows 可使用支持 C++17 的编译器和对应依赖执行离线构建；完整视觉程序使用 Linux/MVS/UDP 接口，不支持直接在本工程的 Windows 模式下启用。

## 3. 构建机器人程序

使用课程的 OpenVINO 2024.6 和 MVS 环境，另需 libusb 开发库：

```bash
sudo apt-get install -y libusb-1.0-0-dev
source /opt/intel/openvino_2024.6.0/setupvars.sh

cmake -S . -B build-robot -G Ninja \
  -DBUILD_VISION_APP=ON -DBUILD_TESTING=OFF \
  -DOpenVINO_DIR=/opt/intel/openvino_2024.6.0/runtime/cmake
cmake --build build-robot --target task_2 -j2
ldd build-robot/task_2
```

确认 ldd 没有 not found。OpenVINO_DIR 也可以由课程环境的 CMAKE_PREFIX_PATH 提供，本工程不再无条件覆盖它。x86_64/aarch64 使用对应仓库中的 MVS 动态库；驱动与动态库架构必须一致。

始终从工程根目录运行，使 assets/yolov5.xml 等相对路径有效。本次没有在真实 Linux 机器人上执行上述完整程序构建/启动；这一步的现场结果应单独记录。

## 4. 核对配置

configs/task2.yaml 从 standard.yaml 复制设备部分，必须对应当前机器人：

| 内容 | 检查方式 |
| --- | --- |
| enemy_color | 与考核靶机颜色相符，red 或 blue |
| camera_name、vid_pid、exposure_ms、gain、rotation_angle | 对应相机与图像方向，不能照搬另一台设备 |
| camera_matrix、distort_coeffs | 当前分辨率和图像旋转后的标定 |
| R_camera2gimbal、t_camera2gimbal、R_gimbal2imubody | 检查方向与单位；旋转 3×3，平移单位 m |
| com_port、波特率 | 与电控串口一致，默认协议 921600 |
| yaw_offset、pitch_offset | 单位 degree；继承的示例值并非本次实测 |
| task2.armor_num | 普通四板靶机为 4；两板靶机才设 2 |
| initial_radius / 几何方差 | 半径单位 m；方差单位 m² |
| 相对时延 | camera_to_pose_offset_ms，单位 ms，允许正负，默认 0 未经标定 |
| command_timeout_ms | 本 Task2 要求 20～200ms，示例 100ms |

相对时延约定：`查询姿态时刻 = 图像接收时间 - offset`。offset 是相机接收延迟减姿态接收延迟；两者都用主机时钟，仍不能保证没有传输差。负 offset 可能需要等待之后的姿态样本，超过 pose_timeout_ms 就跳过本帧。

degree 参数包括 association_angle、max_observation_facing、pitch_limit、max_facing_angle、fire_facing_angle、yaw_tolerance、pitch_tolerance。输出和内部计算统一为 rad。fire_delay/lost_time/max_frame_age/feedback_timeout/warmup_time 单位 s。

max_nis=18.5、过程噪声、5px 重投影阈值及初始几何方差均是起始设置。调整前先看真实残差、位姿和实际帧率，不把小协方差当作真值精度证明。

## 5. 第一轮：真实视觉链路，只记录

```bash
./build-robot/task_2 configs/task2.yaml --dry-run=true --show=true --fire=false
```

程序连接真实设备，但不接管云台，仍发送明确的 mode=0 停控命令。按设备流程由人工进入 AUTO_AIM，以便完整跟踪分支运行。

检查：

1. 图像方向/颜色/关键点正确，目标不是其他同色单位。
2. image_ok、pose_ok、feedback_fresh 连续有效；不要带着大量姿态超时继续调滤波参数。
3. detected、pnp_valid、observed、tracking 与图像一致。
4. radius、radius_diff、height_diff 的物理值合理。
5. 静止目标 angular_velocity 应接近零；旋转方向与带符号的 angular_velocity 相符。
6. 四板持续旋转时 track_id 不应反复增加；armor_id/seen_mask 可以随换板变化。
7. 干扰目标、短时遮挡和重新出现时，更新与重建行为符合预期。
8. requested_control、requested_fire、control、fire 始终为零；send_ok 只表示串口写入状态。

本次交付没有执行这轮真实设备测试。离线 pipeline 测试只覆盖了同一控制决策代码的合成输入。

## 6. 第二轮：无发射跟随

```bash
./build-robot/task_2 configs/task2.yaml --show=true --fire=false
```

人工核对挡位与设备状态后检查 yaw/pitch 指令、反馈、方向和机械范围。退出自瞄、丢失目标、没有同步姿态、反馈变旧或程序退出时应停控，不能出现突然追逐另一类别目标。

如果弹速反馈为零，follow_without_ballistics=true 允许几何跟随；ballistic_valid=0，fly_time=0，fire 保持零。这种跟随用于观测，不能当作已经完成预测击打验证。

如果推理速度超过命令超时，串口线程会停控。先解决过慢推理、设备驱动或配置问题；不要单纯放大时限掩盖旧帧。Plotter 的 processing_time 到主循环采样时刻为止，串口日志会额外记录命令过期。

## 7. Plotter 要看哪些字段

默认使用原 Plotter 的 127.0.0.1:9870。若软件在另一台电脑上，按实际网络配置接收端与发送地址。

| 分组 | 字段 |
| --- | --- |
| 作业必需输出 | angular_velocity、angular_speed、yaw、pitch、control、fire |
| 同步与性能 | image_ok、pose_ok、feedback_fresh、feedback_age、frame_age、processing_time |
| 状态估计 | omega_variance、radius、radius_diff、height_diff、nis |
| 目标生命周期 | observed、tracking、track_id、updates、seen_mask |
| 指令与反馈 | requested_control、requested_fire、send_ok、aim_yaw、aim_pitch、gimbal_yaw、gimbal_pitch、yaw_error、pitch_error |
| 弹道 | ballistic_valid、fly_time、lead_time、facing_angle、armor_id |
| 射击反馈 | shot_requests、bullet_count、bullet_speed |

yaw/pitch 是最近一次串口命令尝试的字段，send_ok 表示该次完整写入成功；仍不代表电控确认。命令过期线程可以更改实际 mode，所以同时保留了 requested_* 字段。aim_* 是算法建议，即使 dry-run 禁止接管也可用于观察。

tracking=0 时角速度的数值零表示无状态，不代表真实目标静止。nis 是最近接受测量的统计量，必须结合 observed；协方差和 NIS 都不能替代对真实转速的误差测量。

## 8. 第三轮：交给人工的正式实机验收

前两轮通过之后，由人工检查单发协议、摩擦轮、弹速反馈、实际弹丸计数以及开火挡位，再决定是否使用 `--fire=true`。本次没有执行任何真实发射。

按照原文，先低速后中速，分别记录真实转速、稳定后的估计值、Plotter 截图与实际前 10 发命中数。误差容许范围为低速 ±0.6rad/s、中速 ±1rad/s。默认转速筛选 2～12rad/s 是运行门限，不是验收真值。

SingleShot 保证请求不超过 5Hz，true 之间至少一帧 false。shot_requests 是视觉请求次数，bullet_count 的物理含义需要电控确认；两者均不能直接当作命中数。

协议没有明确的低/中速轮次或摩擦轮状态。shot_limit=0 不限制进程总请求数；设置 10 会限制整个进程共 10 次请求，不会自动变成两挡各 10 次。程序不会根据估计转速跨过 5rad/s 自动重置。按原文考核流程由人工控制每挡实际弹丸数，并只统计前 10 发。

需要自动执行严格 10+10 时，应先由电控提供可信的轮次/摩擦轮状态接口，再实现状态机；当前代码没有伪造该接口。

## 9. 提交前检查

- git diff 与 review/CHANGED_FILES.txt 对应，头文件、实现、配置和构建文件一起提交。
- 离线 CTest 全部通过，并保存 TASK2_TEST_REPORT.md 与原始日志。
- 在目标 Linux 环境构建 task_2，检查 ldd，记录环境与依赖版本。
- 保存真实 dry-run、无发射跟随的结果；按异常条件逐项检查停控。
- 由人工完成低/中速角速度与实际弹丸验收，保存 Plotter 截图。
- 本次 Task 2 工程修改已整理为个人仓库 final 分支上的独立 Git 提交；后续实机调整请另行提交，并保留原始测试记录。
