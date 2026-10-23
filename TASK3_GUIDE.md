# Task 3：前哨站建模与验收说明

本实现以用户提供的《robomaster 2027算法组招新大作业.docx》Task 3 和结构图为依据，接入 `final` 的真实接口，采用 C++17。工程可编译和离线算法通过，不等于相机、云台、发射及正式评分已经通过实机验收。

## 1. 对照正式要求

| 要求 | 实现与验收界限 |
|---|---|
| 建模三块悬空中部板，不击打顶部板 | 只接收 `outpost/special` 与指定颜色；世界高度窗口排除顶部。窗口必须在实际哨兵坐标系测量 |
| 约 2.5 rad/s，误差不超过 ±0.4 rad/s | `omega = x[7]` 从 0 开始估计；支持正反转，不把 2.5 当作输出或观测。离线用独立真值计算误差 |
| 约 3m 水平距离 | 合成轨迹和独立 PnP 夹具覆盖该距离；实际距离、相机内外参仍需核对 |
| 12Hz 射击 | 输出连续允许电平，复用 `Gimbal::send`；不用 Task2 的 5Hz 单发调度器。12Hz、物理开火挡位和断联停火由电控确认 |
| 30s、1500HP、20HP/命中 | 正式倒计时和血量由考核人员/设备设置，本程序没有从进程启动误算 30s。未以仿真推算命中率或评分 |
| Plotter 输出指令、EKF 角速度 | 输出实际串口发送状态快照、yaw/pitch、fire/control、带符号 angular_velocity、绝对值 angular_speed 等 |
| 最多三次发弹调试 | 本次没有操作发弹设备；任何实机调试由人工监督并记录次数 |

## 2. 模型与图纸对应

状态保持课程已有 11 维接口：

```
x = [cx, vx, cy, vy, z0, vz, theta, omega, r, dh1, dh2]
```

三板 `i = 0,1,2` 的中心与朝向：

```
alpha_i = wrap(theta + i * 2*pi/3)
p_i = [cx - r*cos(alpha_i), cy - r*sin(alpha_i), z0 + delta_i]
delta_0 = 0, delta_1 = dh1, delta_2 = dh2
```

三板相位差为 120°，共用一个半径。`dh1`、`dh2` 为两块板相对于初见参考板的高度差，可正可负；不按高度排序硬编码物理板号，更不将 Task2 四板的“交替半径/交替高度”套过来。转过 120° 后应切换观测板号，而不是把同一物理板的高度改掉。

图中 Φ553mm 对应初始半径取 `0.553/2 = 0.2765m`。这是采用该结构图对应装置时的几何先验；实物版本、标注基准、装甲板中心位置应现场核对。半径通过很小的初始方差约束，不能指望 EKF 自动纠正一个明显错误的结构半径。

图示的 1116/1216/1316mm 等尺寸属于图纸的底面基准，不能直接写成世界 z。应使用：

```
z_world = H_plate_above_drawing_base + H_drawing_base_above_floor
          - H_gimbal_axis_above_same_floor
```

必须核对是否有底座/台阶以及对应尺寸是否确实到板中心。测试夹具按三板离地 `1.116/1.216/1.316m`、假设云台轴高 `0.95m`、同底面构造 `0.166/0.266/0.366m`，验证换算和板号置换；**0.95m 是测试假设，绝不是用户哨兵的实测参数**。配置中的 `0.10..0.50m` 也仅为示例。

Solver 沿用课程的前哨站装甲板倾斜约定 `pitch=-15°`，等价于板面相对水平约 75°；灯条关键点模型为宽 129mm、高 56mm。独立测试手工构造投影，覆盖该倾角、三种高度、非零云台 yaw/pitch/roll、相机平移及 yaw/pitch 轴平移。实物板角、模型关键点定义、相机标定仍需现场确认。

前哨站在世界中固定，机器人也应按文档停在固定考核位置。`vx/vy/vz` 保留接口但不推进中心，实际可观测模型为 8 个量；机器人明显平移时，这一模型不成立，应停下后重新锁定，不宣称当前代码补偿了底盘运动。

### F、Q、h、H、R

- `F = I`，仅 `F(6,7)=dt`，并将中心的 `F(0,1)、F(2,3)、F(4,5)` 置零；角度预测后取周期归一。
- 中心过程噪声为 `Q(0,0)=Q(2,2)=Q(4,4)=center_noise*dt`，单位 m²。角运动使用逐区间白加速度模型：`Q_theta_omega = sigma_a² * [[dt^4/4,dt^3/2],[dt^3/2,dt^2]]`。这里 `angular_acceleration_variance` 是加速度方差，不能混用连续白噪声谱密度的 dt³ 模型。
- 初始 `omega=0`；`P0 = diag(.1,.01,.1,.01,.1,.01,.4,100,radius_variance,height1_variance,height2_variance)`，表示初始角速度未知、半径已有结构先验。
- 测量来自 PnP 的世界 `xyz` 与装甲板 `yaw`，实际更新的 `z=[atan2(y,x), atan2(z,hypot(x,y)), norm(p), alpha_i]`；所有角度为 rad，位置 m，时间 s。
- `h(x)` 对预测板位置使用相同球坐标变换；`H = J_xyz_to_ypd * J_plate_to_xyz`，最后一行为相位导数。位置雅可比中 `dx/dtheta=r*sin(alpha)`、`dy/dtheta=-r*cos(alpha)`，`dx/dr=-cos(alpha)`、`dy/dr=-sin(alpha)`；高度列按 id=1/2 分别取 1。
- `R = diag(5e-4,5e-4,(.01+.005*d²)*(log(abs(beta)+1)+1), .05+.01*d²+.2/d)`，其中 `beta=wrap(measured_armor_yaw-atan2(y,x))`。这是仓库已有的经验噪声模型，不是经过本机实测优化的参数。
- 方位角、俯仰角、装甲板 yaw 的残差取周期差；EKF 使用已有 LDLT/Joseph 更新及有限值检查。三板所有 11 列 H 经过中央有限差分检查；另直接验证 P0、F/Q 传播与 R 数值。

### 关联与状态管理

每帧至多融合一块板，避免将同帧相关的姿态误差当作多次独立信息。先筛类别/颜色、PnP、世界高度、距离、朝向，再对所有候选板编号计算位置/角度门限和 NIS。选中的 **观测、板号对** 必须一并交给 `Target::try_update`。更新被拒绝时，不增加计数、不刷新成功观测时间、不增加三板 seen_mask。

短时丢失保留估计，但该帧不发送有效控制/发射请求；超过 lost_time 清除，恢复后重新学习和预热。非自瞄或反馈过期清除旧模型；重复/倒退图像时间戳拒绝。只有三板都见过、更新次数及预热时间满足、方差和转速在范围内才标记 estimate_ready。

`estimate_ready` 是运行门槛，**小方差不是实际误差达标证明**。真实 ±0.4rad/s 必须以考核设备独立真值和同步日志验证。

## 3. 运行数据流与保护

```
Camera timestamp -> signed camera/pose offset -> try_q (bracketed SLERP)
 -> YOLO outpost/color -> Solver PnP/world -> OutpostTracker -> Target/EKF
 -> Task2Aimer (predict a copy + Trajectory) -> Task3Pipeline -> Gimbal
 -> last_command snapshot -> Plotter UDP
```

复用原有 Camera、YOLO、Solver、Target、EKF、Trajectory、Task2Aimer、Gimbal、Plotter。未来预测不会修改实时滤波器。图像时间先减 `camera_to_pose_offset_ms` 再查询姿态；该值可正可负，0 只是待标定起点。反馈年龄须在 `[0,feedback_timeout]`，帧龄在 `[0,max_frame_age]`。云台四元数沿用 `wxyz`。

Yaw 与反馈采用同一世界参考，并映射到离反馈最近的 2π 分支。Pitch 沿用原项目负号约定，YAML 用度、内部用 rad。无有效弹速可采用无弹道跟随，且禁止发射。主循环处理读图/PnP异常；Gimbal 再校验反馈、模式、角度有限值，100ms 指令看门狗用于卡帧停止。进程退出发送停止请求；若物理串口已经断开，软件不能保证停止包到达，电控仍须有断联保护。

`--fire` 默认 false。`--dry-run=true` 对 pipeline 控制/发射全禁，但主程序依然创建 Camera/Gimbal、打开设备、接收反馈并可能发停止包，**不是无硬件模式**。本轮未执行带配置的主程序。

## 4. 文件清单

新增：

```
configs/task3.yaml
src/task_3_fire.hpp
tasks/auto_aim/outpost_tracker.hpp
tasks/auto_aim/outpost_tracker.cpp
tasks/auto_aim/task3_pipeline.hpp
tasks/auto_aim/task3_pipeline.cpp
tasks/auto_aim/task3_config.cpp
tests/task_3_test.cpp
tests/task3_acceptance.cpp
tests/task3_solver.cpp
TASK3_GUIDE.md
TASK3_VALIDATION.md
```

替换/修改：

```
src/task_3.cpp                  # 原空模板 -> 完整程序
tasks/auto_aim/CMakeLists.txt    # 明确加入 Task3 实现
tests/CMakeLists.txt            # 三项 Task3 验收测试
tasks/auto_aim/solver.cpp        # Task3 重投影误差门限，保留原 Task2 行为
src/task_1.cpp                  # 两处 packed float -> double 后写 JSON，修复 GCC11 构建
src/task_2.cpp                  # 同上，仅两处强制转换
.github/workflows/task2.yml     # 回归测试及完整 task_1/2/3 构建
README.md                      # 增加说明入口
```

保持不变：`target.hpp/.cpp`、EKF、Trajectory、Camera、YOLO、Gimbal、Task2Aimer 及 Task1/2 跟踪逻辑。根 CMake 本来就声明 task_3，无须再修改。不要复制旧 Task3 包中的整份 Target 覆盖当前文件。

## 5. Ubuntu 复现

本次验证目录是独立检查工程 `/home/sp/robomaster-ubuntu-audit.uXASDR/project`，未覆盖用户原 `~/sp_vision_tutorial_27` 的 main 和未跟踪文件。下列命令应在包含最新 final 的完整仓库根目录运行。

纯离线，不访问相机或串口：

```bash
cmake -S . -B build-offline -G Ninja -DBUILD_VISION_APP=OFF -DBUILD_TESTING=ON
cmake --build build-offline -j2
ctest --test-dir build-offline --output-on-failure
./build-offline/tests/task3_model_tracking
./build-offline/tests/task3_acceptance configs/task3.yaml
./build-offline/tests/task3_solver configs/task3.yaml
```

完整编译和安全的帮助页加载：

```bash
source /opt/intel/openvino_2024.6.0/setupvars.sh
cmake -S . -B build-robot -G Ninja \
  -DBUILD_VISION_APP=ON -DBUILD_TESTING=OFF \
  -DOpenVINO_DIR=/opt/intel/openvino_2024.6.0/runtime/cmake
cmake --build build-robot --target task_1 task_2 task_3 -j2
ldd build-robot/task_3
./build-robot/task_3 --help
```

只有经人工同意连接设备、确认物理禁发并完成参数核对后，才进行以下硬件观测命令（本轮未执行）：

```bash
./build-robot/task_3 configs/task3.yaml --dry-run=true --fire=false --show=true
```

它仍打开设备并发送停止状态。测量全部三块中部板和顶部板的候选 z，核对内参、外参、旋转图像、颜色、半径、姿态时延、角度参考系及 pitch 方向，设定可分离顶部的窗口，再由人工确认 `middle_height_verified`。随后再安排监督下无发射跟随。没有提供一键自动发弹命令。

默认 Task3 的 `device: CPU` 适配当前 VMware 可用的 OpenVINO CPU 设备。相机标定与 `yaw_offset/pitch_offset` 来自课程示例，需要替换为本机标定；不是推荐最优参数。

Plotter 默认 UDP `127.0.0.1:9870`，接收/查看程序应运行在 Ubuntu 内；Windows 的 localhost 不是 Ubuntu 的 localhost。主要看 `yaw/pitch/control/fire/send_ok/angular_velocity`，配合 `observed/seen_mask/omega_variance/z0/z1/z2/nis/frame_age/feedback_age`。`candidate_z_i` 是当前帧候选顺序，不是固定物理板号。`send_ok` 只代表串口写入成功，不能作为电控确认或真实命中证据。

## 6. 提交与实机条件

代码、配置、声明、构建和离线测试已成套。当前 Ubuntu 的真实构建与测试结果见 `TASK3_VALIDATION.md`。进入实机前仍须完成中部板高度窗口及相机标定、USB/串口权限、反馈参考系与 pitch 方向核验、时延/弹道标定、电控12Hz连发及人工挡位/断联保护确认。正式30s血量/命中和真实角速度误差尚未验证。
