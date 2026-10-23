# Task 1 实际测试报告

日期：2026-10-23。软件基准：GitHub `final` 的 `d0df61f`，叠加本次 Task1 修改。
本地 Windows 主机没有接入机器人，相机、串口和机械运动均未执行。没有使用桩接口伪造硬件测试。

## 实际执行并通过

| 检查 | 结果 | 实际范围 |
| --- | --- | --- |
| 基准文件一致性 | 109/109 文件 Git blob 哈希相同 | 对照 GitHub 最新 tree，确认本地准备工程保留上次 Task2 提交 |
| CMake 配置、编译、链接 | 通过 | `BUILD_VISION_APP=OFF`，真实 yaml-cpp、Eigen、OpenCV 4.5.4、fmt、spdlog、JSON，Clang18/llvm-mingw；运行真实生产辅助函数及 Solver |
| CTest | 10/10 通过 | 3 项 Task1 + 原有 7 项 Task2；合计 47,178 项断言 |
| Task1 角度 | 1,464 项通过 | 左右上下、±20°限位、360°扫描、±π、多圈反馈、补偿及无效输入 |
| Task1 控制与配置 | 2,882 项通过 | 丢失恢复、模式切换、干跑停控、时间戳异常、无效配置、float 表示范围和持续两圈序列 |
| Task1 PnP→控制链路 | 1,448 项通过 | 独立解析相机投影、真实 OpenCV/PnP、四元数插值、世界坐标和控制角；181 帧从 170°到 530°的 yaw 及 ±18° pitch |
| Task2 回归 | 7/7，41,384 项通过 | 原有模型、关联、滤波、时序、参数、控制及 Solver 测试 |
| Linux C++ 编译 | 3 个 x86_64 ELF 目标文件通过 | 实际 `src/task_1.cpp` 与两份新增测试；不是仅检查文本语法，也不代表已完成 Linux 整程序链接 |
| 头文件独立包含 | 2/2 通过 | task_1_angles.hpp、task_1_follow.hpp |
| clang-format | 6 份 C++ 文件通过 | 主程序、两个头文件、三份测试 |

PnP/跟随夹具的最大位置误差约 `2.95263e-6 m`，最大角度指令误差约 `4.65718e-7 rad`。
这些数字来自理想无噪声投影、已知标定和简化的一阶响应，不能作为真实相机精度或机器人跟随误差。
该集成测试没有运行 YOLO、相机 SDK、真实串口或 Plotter 接收软件。

记录见 `validation/task1/`：cmake_configure.txt、cmake_build.txt、ctest.txt、format_check.txt、linux_*.txt、header_*.txt。

## 执行失败后已修复

新增 PnP 测试初次使用 auto 接收 OpenCV 逗号初始化表达式，编译器无法将其传给 cameraMatrix。
改为显式 cv::Mat 后，重新完成构建、CTest、Linux 对象与头文件检查，全部通过。
初次错误原始记录见 `development_compile_failure.txt`。最终构建另有旧版 fmt 与 Clang18/libc++ 的弃用警告，未影响本次 C++17 构建；没有声称零警告。

## GitHub Linux 验证

此前 Task2 提交的 [Ubuntu 离线 CI](https://github.com/16370711czh-ops/sp_vision_tutorial_27/actions/runs/39891308720) 已完成并通过。
本次工作流在此基础上新增 Task1 测试，并增加 `BUILD_VISION_APP=ON` 的完整 task_1 构建、链接和 `--help` 加载检查。
本报告生成时本次提交尚未运行该 CI，其结果以当前提交的 GitHub Actions 记录为准，不能把“已配置工作流”当作“已执行通过”。

`--help` 只检查程序加载与 CLI，不实例化设备。即使此步骤通过，也不说明相机、GPU 推理、串口或云台跟随已经通过验收。

## 尚未执行，不能标为通过

- 考核机器人完整依赖安装与真实设备启动。
- 实际曝光、旋转图像与标定、YOLO 识别质量和处理延迟。
- 串口协议、角度正负号、yaw 零点及连续角、pitch 机械限位的一致性。
- 实际 UDP Plotter 收包及控制命令截图。
- 手持装甲板全周/上下运动，遮挡恢复和退出自瞄挡位的机械行为。
- 正式考核流程：考生不操作机器人、不修改代码，考核人员记录持续无异常跟随。

本实现始终 fire=false；没有进行真实发射，也没有必要通过发射验证 Task1。
