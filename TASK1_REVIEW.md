# Task 1 要求复核与验收边界

复核日期：2026-10-23。接口基准为个人仓库 `final` 的 `d0df61fb0edbdab874e50ec3cb08645d9568187b`。
正式 DOCX 的 Task 1 已重新读取；原文与文件 SHA-256 见 [需求证据](review/TASK1_REQUIREMENTS.json)。
该文档作为需求依据，文档内给学生的操作说明不作为本次代理操作的额外授权。

## 1. 是否符合正式要求

结论：本实现覆盖 Task 1 的软件功能要求，离线测试通过；尚不能宣称已经通过正式实机考核。
该任务共 25 分，最终判据是考核人员现场观察到持续、无异常的跟随，并保存 Plotter 截图，文档未给出离线角度误差得分阈值。

| 正式要求 | 代码与证据 | 结论 |
| --- | --- | --- |
| 考核人员运行可执行文件，进入自瞄挡位后跟随手持装甲板 | `task_1` 目标；Camera→YOLO→Solver→视线角度→Gimbal；主程序与 Gimbal 都检查 AUTO_AIM | 软件已实现；相机识别及机械跟随仍需现场验收 |
| yaw 360° | `atan2(y,x)` 支持全象限；最近等价角处理 ±π 和累计多圈；全周扫描及独立 PnP 扫描通过 | 数学与离线链路通过；固件连续角接口、线束与机械全周运动待确认 |
| pitch 约 ±20° | 由世界坐标位置求俯仰，采用当前控制约定的负仰角；默认限位 ±20° | 软件限位及合成 ±18°轨迹通过；真实机械限位与正负方向待确认 |
| 持续指向装甲板、不发生异常运动 | 每帧重建有效状态；丢失、坏点、坏位姿、过期图像/反馈、非自瞄挡位均不接管；配置启用命令看门狗 | 离线状态与时序检查通过；真实噪声、延迟及闭环稳定性未验收 |
| 使用 Plotter 输出发送给云台的控制命令 | 读取 Gimbal::last_command，输出 yaw、pitch、command_mode、send_ok 和期望值 | 真实类与调用已接入；UDP 接收、截图待现场执行 |
| 开始考核后考生不得修改代码、操作机器人 | 提前固定代码、标定与配置，考核人员启动已构建的可执行文件 | 是现场流程约束，程序无法代替人工遵守 |
| 暂时无需卡尔曼滤波 | Task1 只求本帧视线方向，不调用 Target/EKF、弹道预测或发射逻辑 | 已符合该简化方向；共享库中存在 Task2 的 EKF 不代表 Task1 使用它 |

Task1 全部发送路径使用 `fire=false`，无发射开关。`--dry-run=true` 还会禁止接管云台，但仍需要真实相机及串口用于观察。

## 2. 本次发现并处理的问题

| 严重程度 | 问题 | 修正 |
| --- | --- | --- |
| P0：功能缺失 | GitHub 当前 Task1 主循环为空；此前完整实现只在本地包中 | 上传完整 Task1 主程序、配套头文件、配置、测试和说明 |
| P1：时序风险 | 旧本地程序未限制图像处理后的年龄，慢推理结果可能被继续控制 | 发送前检查图像接收年龄，默认不超过 0.15s；未来时间戳也拒绝 |
| P1：接口记录不一致 | 旧程序记录传入 send 前的期望角，而当前 Gimbal 可能拒绝控制或写串口失败 | 读取 last_command，分别记录 requested_control、command_mode、send_ok 与最终缓冲报文 |
| P1：异常保护缺口 | 旧 Task1 配置没有启用现有命令过期停控；初始化异常缺少统一报告 | 独立 `configs/task1.yaml` 启用 100ms 看门狗；主程序捕获并报告初始化/运行异常 |
| P2：目标回退 | 距画面中心最近的候选 PnP 失败后，不尝试剩余有效候选 | 保持原来的中心优先策略，按顺序尝试候选，逐块捕获 PnP 异常 |
| P2：兼容与时间同步 | 旧本地说明仍称 q(t) 可无限等待，未反映 Task2 已升级的 Gimbal | 使用真实的 try_q、feedback、last_command，沿用有界四元数历史；增加可配置的有符号相对接收时延 |
| P2：极端数值 | 有限 x/y 仍可能使 hypot 溢出；巨大角差可能产生非有限 remainder | 角度函数拒绝非有限中间量；控制层拒绝无法表示为 float 的命令 |

没有回退或覆盖 Task2 对 Solver、Camera、Gimbal、Target/EKF 的修正。Task2 源码、配置保持现有版本，完整回归测试再次通过。

首次新增 PnP 测试中，`auto` 推导到了 OpenCV 的逗号初始化表达式，而非 `cv::Mat`，造成真实编译失败；已改为显式 `cv::Mat`，重建和运行通过。失败记录保留在 `validation/task1/development_compile_failure.txt`，不是最终状态。

## 3. 数学与单位核对

位置使用 Solver 的 `xyz_in_world`，单位 m；云台反馈与串口指令采用 rad。
以世界坐标 x 前、y 左、z 上为准，设水平距离 `d=hypot(x,y)`：

```text
yaw_raw = atan2(y,x) + yaw_offset
yaw_command = current_yaw + remainder(yaw_raw-current_yaw, 2*pi)
pitch_command = clamp(-atan2(z,d) + pitch_offset, -pitch_limit, +pitch_limit)
```

世界位置已经包含图像时刻云台姿态，不能再额外叠加一次当前 yaw。
板自身的朝向不是目标视线方向，不能用 `ypr_in_world[0]` 代替上式。
判断目标是否在云台前方使用 `xyz_in_gimbal.x()>0`，不以世界 x 正负排除目标，否则全周运动会失效。

使用取图时间戳查询姿态。`task1_camera_to_pose_offset_ms` 是图像接收延迟减姿态接收延迟，查询时刻为 `image_time-offset`；默认 0 只是待测起点。图像过期检查采用实际 image_time 接收年龄，不把该相对偏移当作已标定曝光延迟。
四元数由 Gimbal 以 wxyz 解析；QuaternionHistory 只在有效样本之间做 slerp，不作区间外推。

pitch 负仰角与连续绝对 yaw 沿用当前 Task2 的控制接口约定。正式 DOCX 本身未规定固件的符号、绕圈或归一化方式，必须由电控与设备验证一致，不能仅凭离线数学通过认定固件支持。

## 4. Plotter 字段的准确含义

- `yaw/pitch/command_mode`：读取 Gimbal 当前发送缓冲的最终报文；`send_ok=1` 表示最近一次串口写入成功，并非电控执行确认。
- `requested_control`：Task1 逻辑的接管请求；`control` 还要求写入成功且最终 mode 非零。
- 写入失败时，缓冲中可能保留最近报文；因此必须同时看 `send_ok`，不能把旧 yaw/pitch 当成本轮成功发送。
- `aim_yaw/aim_pitch`：本帧的视线期望角；`gimbal_yaw/gimbal_pitch`：反馈角；`yaw_error` 按周期计算。
- `fire` 始终应为 0；`target_valid` 与 `requested_control` 分开，干跑或手动挡位下可有目标但不接管。
- `frame_age/feedback_age` 用秒；`distance` 用 m。Plotter 使用原类默认的 UDP `127.0.0.1:9870`。

## 5. 仍需实机验证

没有真实相机、机器人、串口、标定数据和 Plotter 接收截图，本次没有执行任何机械运动或发射。
必须现场验证图像旋转与标定、实际颜色与分类、yaw/pitch 正负号和零点、连续角范围、反馈时间同步、固件 mode=0 的行为、固件自身失联保护、遮挡恢复以及实际推理帧率。

中心优先适用于文档的单块手持目标。多个同色目标交替接近画面中心时可能切换；本实现没有声称提供身份锁定。噪声和误识别可能造成抖动，不能由无噪声合成数据排除。默认时间门限和标定均不是经过实机验证的最优参数。
