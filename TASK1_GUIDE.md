# Task 1 云台跟随：文件、编译与验收

先看 [要求复核](TASK1_REVIEW.md) 和 [测试报告](TASK1_TEST_REPORT.md)。代码已接入个人仓库 `final` 分支。
Task1 只做手持装甲板跟随，所有路径均禁止发射，不使用 EKF。

## 文件说明

| 文件 | 内容 |
| --- | --- |
| `src/task_1.cpp` | 完整取图、识别、颜色过滤、PnP、姿态同步、控制及 Plotter 主循环 |
| `src/task_1_angles.hpp` | 三维世界位置到视线角度、最近等价 yaw、pitch 限位 |
| `src/task_1_follow.hpp` | 参数读取校验和与离线测试共用的控制判断 |
| `configs/task1.yaml` | 步兵配置示例、独立角度补偿、时间门限和命令看门狗 |
| `tests/task_1_angles_test.cpp` | 角度、全周范围、补偿与异常输入测试 |
| `tests/task1_follow_test.cpp` | 丢失恢复、模式切换、干跑、异常时间戳、配置及 float 命令测试 |
| `tests/task1_solver_follow.cpp` | 独立投影→真实 OpenCV/PnP→世界坐标→跟随指令测试 |
| `tests/CMakeLists.txt` | 新测试与现有 Task2 回归测试一起构建 |
| `tests/run_task_1_angles.sh` | 无相机、无 OpenCV 的最小角度测试脚本 |
| `.github/workflows/task2.yml` | Ubuntu 的 Task1/Task2 离线测试及完整 Task1 构建检查 |
| `review/TASK1_REQUIREMENTS.json`、`validation/task1/` | 正式文档原文/hash 与实际验证记录 |

直接复用当前仓库的 Camera、YOLO、Solver、Gimbal、Plotter 与 QuaternionHistory；这些共享类不在本次 Task1 中重写。根 CMake 已有 task_1 目标，无需再手动添加 main。完整改动以 Git 提交 diff 为准。

## 1. 先跑离线测试

Ubuntu 22.04 项目根目录：

```bash
sudo apt-get update
sudo apt-get install -y cmake ninja-build g++ libeigen3-dev libopencv-dev \
  libyaml-cpp-dev libfmt-dev libspdlog-dev nlohmann-json3-dev
cmake -S . -B build-offline -G Ninja -DBUILD_VISION_APP=OFF -DBUILD_TESTING=ON
cmake --build build-offline -j2
ctest --test-dir build-offline --output-on-failure
# 只跑Task1三项测试
ctest --test-dir build-offline -R '^task1_' --output-on-failure
# 最小纯角度测试
bash tests/run_task_1_angles.sh
```

这条路径不实例化 Camera/Gimbal，也不运行 YOLO。完整 OpenCV 缺失时，PnP 测试不会被构建，不能据此声称 PnP 通过。

## 2. 编译真实 Task1 程序

使用课程 Ubuntu 22.04、OpenVINO 2024.6 和海康 MVS 环境，另外安装 libusb：

```bash
sudo apt-get install -y libusb-1.0-0-dev
source /opt/intel/openvino_2024.6.0/setupvars.sh
cmake -S . -B build-vision -G Ninja -DBUILD_VISION_APP=ON -DBUILD_TESTING=OFF \
  -DOpenVINO_DIR=/opt/intel/openvino_2024.6.0/runtime/cmake
cmake --build build-vision --target task_1 -j2
./build-vision/task_1 --help
```

OpenVINO 路径以真实安装为准。也可通过官方 PyPI 包的 `openvino.utils.get_cmake_path()` 取得 CMake 路径，CI 的完整构建使用此路径；参见 [OpenVINO 官方安装说明](https://docs.openvino.ai/2024/get-started/install-openvino/install-openvino-pip.html)。两条安装方式择一，不能把多个版本的头文件与库混用。

## 3. 接设备后先干跑

`configs/task1.yaml` 复制了课程 standard.yaml 的设备参数，属于示例，不是当前机器人的真实标定。接设备前核对相机内参/畸变、外参、图像方向、串口、波特率、enemy_color。模型和配置使用相对路径，始终从工程根目录启动。

```bash
./build-vision/task_1 configs/task1.yaml --dry-run=true
# 有图形桌面才打开窗口
./build-vision/task_1 configs/task1.yaml --dry-run=true --show=true
```

干跑仍需要相机和持续合法的云台反馈，但不请求接管，也不开火。首先检查识别框、distance、pose_ok、aim_yaw/aim_pitch、图像/反馈年龄。按原配置使用 GPU；没有可用 GPU 时须在预检阶段设置 `device: CPU` 并确认延迟能满足门限。

## 4. 无发射跟随与正式考核

准备阶段由人工核对角度符号和机械范围，再运行：

```bash
./build-vision/task_1 configs/task1.yaml
```

默认 dry-run=false；只有目标有效、图像/反馈新鲜并处于自瞄挡位时才接管。无目标、PnP/姿态失败、退出自瞄挡位均不沿用上一帧命令。所有 send 的 fire 恒为 false，不存在开启发射的 Task1 参数。

预检时依次检查：固定目标的左右/上下方向；缓慢移动；yaw 跨 ±π 与全周；pitch 接近实际 ±20°边界；遮挡和恢复；退出/重进自瞄挡位；图像或反馈中断。先确认 mode=0 对应电控的退出行为，程序无法代替电控自身的保护。

`command_timeout_ms:100` 启用当前 Gimbal 的命令看门狗；检测过程过慢时允许停控，不应为追求看起来连续而直接删除检查。`task1_max_frame_age:0.15` 是图像接收年龄上限；`task1_feedback_timeout:0.1` 不超过 Gimbal 内部固定的 100ms 上限。相对时间偏移需测量后设置，默认 0 不代表已经校时。

考核前固定代码、标定与配置，保存提交号。正式考核由考核人员运行程序、操作机器人进入自瞄挡位并移动手持装甲板；开始后考生不得修改代码或操作机器人。这是正式文档的流程要求。

## 5. Plotter 截图

使用战队 Plotter 接收 UDP `127.0.0.1:9870`。软件在另一台电脑时，需要在预检阶段调整 Plotter 的发送 host，不能在正式考核中临时改动。

截图至少显示 `yaw/pitch` 控制指令，建议同时显示 `gimbal_yaw/gimbal_pitch`、`control`、`send_ok`、`fire` 和 `yaw_error`。角度用 rad；跨 ±π 时允许等价绝对角超过 π，应结合周期角差判断。
`send_ok` 仅确认串口写入成功，不确认电控已经执行；写入失败时 yaw/pitch 可能是缓存的上一条报文，必须配合该标志解读。

持续平稳跟随与真实 Plotter 截图是最终验收依据。离线数学/PnP 通过和可执行文件加载成功，都不能代替实机考核。
