# Task 2 文件与阅读入口

本目录为个人仓库 `final` 分支的 Task 2 工程修改，基于 `88f877c`。默认禁止发射。
源码与此前已测试的 Task 2 交付版一致；本次上传只补充、更新说明文档。

## 先阅读这些文档

1. [代码审查与数学模型](TASK2_REVIEW.md)：问题分级、模型、修改原因和剩余风险。
2. [测试报告](TASK2_TEST_REPORT.md)：已执行的构建与测试，以及无法执行的实机验证。
3. [编译与无发射验收说明](TASK2_RUNBOOK.md)：离线编译、视觉程序启动和 Plotter。

本地 Task 2 测试为 7/7 通过，共 41,384 项检查。完整机器人程序的实机编译、相机、串口及真实跟踪尚需在课程 Linux 环境和设备上验收，不能将离线通过视为实机通过。

## 核心文件及作用

| 文件 | 作用 |
| --- | --- |
| `src/task_2.cpp` | Camera、YOLO、PnP、状态估计、跟随控制和 Plotter 主流程 |
| `src/task_2_fire.hpp` | 单发请求节流及默认禁止发射的安全状态 |
| `tasks/auto_aim/target.hpp/.cpp` | 11 维整车模型、非线性观测及雅可比 |
| `tasks/auto_aim/task2_tracker.hpp/.cpp` | 装甲板关联、确认、丢失与重新初始化 |
| `tasks/auto_aim/task2_aimer.hpp/.cpp` | 预测、弹道补偿与跟随角度 |
| `tasks/auto_aim/task2_pipeline.hpp/.cpp` | 运行模式、反馈有效性和控制决策 |
| `tasks/auto_aim/task2_config.cpp` | Task 2 参数加载与校验 |
| `tasks/auto_aim/solver.hpp/.cpp` | PnP、坐标转换和测量检查 |
| `tools/extended_kalman_filter.hpp/.cpp` | EKF 预测、更新、协方差和数值检查 |
| `tools/quaternion_history.hpp` | 四元数历史与图像时刻插值 |
| `tools/trajectory.hpp/.cpp` | 弹道计算与无效输入保护 |
| `io/gimbal/gimbal.hpp/.cpp` | 云台反馈、姿态时间同步和命令保护 |
| `io/hikrobot/hikrobot.hpp/.cpp` | 相机最新帧和异常处理 |
| `configs/task2.yaml` | Task 2 全部配置；默认禁止发射 |
| `tests/task2_acceptance.cpp` | 模型、跟踪、控制决策、时间同步和配置测试 |
| `tests/task2_solver_roundtrip.cpp` | 真实 OpenCV 的 PnP 与坐标转换测试 |
| `tests/task_2_test.cpp` | 保留兼容性与合成旋转验证 |

根目录和子目录 CMake 完成构建集成；`tasks/auto_aim/yolo.hpp`、`tools/crc.cpp` 含必要的接口或编译修正；`.github/workflows/task2.yml` 提供 Ubuntu 离线 CI。

本次提交不替换 `src/task_1.cpp`、`src/task_3.cpp`。共享依赖的行为以审查报告为准。
`review/` 保存正式需求与基准源码证据，`validation/` 保存实际测试日志。

## 本次提交的完整 57 文件清单

- [.github/workflows/task2.yml](.github/workflows/task2.yml)
- [.gitignore](.gitignore)
- [CMakeLists.txt](CMakeLists.txt)
- [SOURCE.json](SOURCE.json)
- [TASK2_GUIDE.md](TASK2_GUIDE.md)
- [TASK2_REVIEW.md](TASK2_REVIEW.md)
- [TASK2_RUNBOOK.md](TASK2_RUNBOOK.md)
- [TASK2_TEST_REPORT.md](TASK2_TEST_REPORT.md)
- [TASK2_VALIDATION.md](TASK2_VALIDATION.md)
- [configs/task2.yaml](configs/task2.yaml)
- [io/CMakeLists.txt](io/CMakeLists.txt)
- [io/gimbal/gimbal.cpp](io/gimbal/gimbal.cpp)
- [io/gimbal/gimbal.hpp](io/gimbal/gimbal.hpp)
- [io/hikrobot/hikrobot.cpp](io/hikrobot/hikrobot.cpp)
- [io/hikrobot/hikrobot.hpp](io/hikrobot/hikrobot.hpp)
- [review/CHANGED_FILES.txt](review/CHANGED_FILES.txt)
- [review/SOURCE_MANIFEST.json](review/SOURCE_MANIFEST.json)
- [review/TASK2_REQUIREMENTS.txt](review/TASK2_REQUIREMENTS.txt)
- [src/task_2.cpp](src/task_2.cpp)
- [src/task_2_fire.hpp](src/task_2_fire.hpp)
- [tasks/auto_aim/CMakeLists.txt](tasks/auto_aim/CMakeLists.txt)
- [tasks/auto_aim/solver.cpp](tasks/auto_aim/solver.cpp)
- [tasks/auto_aim/solver.hpp](tasks/auto_aim/solver.hpp)
- [tasks/auto_aim/target.cpp](tasks/auto_aim/target.cpp)
- [tasks/auto_aim/target.hpp](tasks/auto_aim/target.hpp)
- [tasks/auto_aim/task2_aimer.cpp](tasks/auto_aim/task2_aimer.cpp)
- [tasks/auto_aim/task2_aimer.hpp](tasks/auto_aim/task2_aimer.hpp)
- [tasks/auto_aim/task2_config.cpp](tasks/auto_aim/task2_config.cpp)
- [tasks/auto_aim/task2_pipeline.cpp](tasks/auto_aim/task2_pipeline.cpp)
- [tasks/auto_aim/task2_pipeline.hpp](tasks/auto_aim/task2_pipeline.hpp)
- [tasks/auto_aim/task2_tracker.cpp](tasks/auto_aim/task2_tracker.cpp)
- [tasks/auto_aim/task2_tracker.hpp](tasks/auto_aim/task2_tracker.hpp)
- [tasks/auto_aim/yolo.hpp](tasks/auto_aim/yolo.hpp)
- [tests/CMakeLists.txt](tests/CMakeLists.txt)
- [tests/run_task_2.sh](tests/run_task_2.sh)
- [tests/task2_acceptance.cpp](tests/task2_acceptance.cpp)
- [tests/task2_solver_roundtrip.cpp](tests/task2_solver_roundtrip.cpp)
- [tests/task_2_test.cpp](tests/task_2_test.cpp)
- [tools/CMakeLists.txt](tools/CMakeLists.txt)
- [tools/crc.cpp](tools/crc.cpp)
- [tools/extended_kalman_filter.cpp](tools/extended_kalman_filter.cpp)
- [tools/extended_kalman_filter.hpp](tools/extended_kalman_filter.hpp)
- [tools/quaternion_history.hpp](tools/quaternion_history.hpp)
- [tools/trajectory.cpp](tools/trajectory.cpp)
- [tools/trajectory.hpp](tools/trajectory.hpp)
- [validation/cmake_build.txt](validation/cmake_build.txt)
- [validation/cmake_configure.txt](validation/cmake_configure.txt)
- [validation/compatibility_cmake_build.txt](validation/compatibility_cmake_build.txt)
- [validation/compatibility_cmake_configure.txt](validation/compatibility_cmake_configure.txt)
- [validation/compatibility_ctest.txt](validation/compatibility_ctest.txt)
- [validation/ctest.txt](validation/ctest.txt)
- [validation/development_q_guard_failure.txt](validation/development_q_guard_failure.txt)
- [validation/format_check.txt](validation/format_check.txt)
- [validation/linux_objects.json](validation/linux_objects.json)
- [validation/opencv_dependency_configure.txt](validation/opencv_dependency_configure.txt)
- [validation/vision_configure_attempt.txt](validation/vision_configure_attempt.txt)
- [validation/yaml_dependency_configure.txt](validation/yaml_dependency_configure.txt)
