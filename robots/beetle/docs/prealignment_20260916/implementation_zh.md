# 固定合体参考系与低速相对修正：2026-10-21 更新

## 使用

在运行合体脚本的电脑使用更新后的本地工作区：

```bash
roslaunch beetle assembly_motion.launch module_ids:=2,3 real_machine:=true
```

Python 和 YAML 修改不涉及飞控固件或 C++ 编译。程序未由本次修改自动启动，也未通过 SSH 部署。`assembly_motion.py` 同时兼容私有参数 `~module_ids/~real_machine` 和原有同名普通参数；launch 将参数加载到 `/assembly_motion`。

配置：`robots/beetle/config/DockingConfig.yaml`，可通过 launch 的 `docking_config:=...` 指定替代文件。直接 rosrun 而不加载 YAML 时，代码默认的核心阈值与本文件相同。

## 固定参考与职责划分

两机稳定悬停后再启动合体脚本。程序在双方 HOVER、flight_state 和共同时间 TF 均有效的第一帧锁定参考，不额外增加“低差分速度保持 1 s”的启动条件。

- 参考原点：leader 初始 FC 位置，加上共同目标姿态下 FC→接口的偏移。这样调平 leader 时，名义接口位置仍与其保持的 FC 位置相容。
- 参考姿态：接口 roll/pitch = 0，yaw = 初始 leader 接口 yaw。冻结后不追踪 leader 的实时位置或姿态。采用接口标定旋转时，两机 root/FC 的目标姿态会有相应补偿，不一定数值相同。
- leader：只发送 roll/pitch/yaw 的 POS_MODE，位置为 NO_NAVIGATION，保留原有悬停位置目标。因此启动前应先结束其它位置/速度导航指令。
- follower：在固定参考系中生成预对齐与轴向接近轨迹；两机姿态从各自初始实测姿态以不超过 10°/s 的旋转速率过渡到共同目标。轴向前馈方向不随 leader 摆动旋转。
- 相对 Y/Z 修正：仅修正 follower，0.5 s 低通、0.3/s 积分增益，修正向量速率上限 0.01 m/s、总偏移上限 0.05 m。粗 Y/Z 目标已接近零且姿态达标后启用；最终保持、缓退与 TF 中断时冻结。粗对齐目标自身仍以各轴 0.05 m/s 归零。
- X 不加入持续相对跟踪；相对间隙用于减速、暂停、缓退和到位判断。leader 的轴向持续偏移可能使预对齐间隙条件不满足，此时应恢复悬停位置或停止后重新建立参考，而不是让目标持续追逐 leader。
- leader 的 FC 相对捕获位置偏移超过 5 cm 时，中止任务并保持 follower；稳定后需重新启动合体脚本，参考不会在飞行中自动重置。

代码按职责组合，已删除旧的实时 leader 跟随式 Standby/Approach 实现及其重复 TF、导航发布器、紧急逃离逻辑：

| 文件 / 类 | 职责 |
| --- | --- |
| `docking_reference.py` / `FixedDockingReference` | 固定几何、共同姿态平滑、低速相对 Y/Z 微调 |
| `docking_guidance.py` / `DockingGuidance` | 相对几何门槛、固定坐标轨迹、暂停与缓退 |
| `assembly_alignment.py` / `DockingMotion` | ROS TF、飞行状态、导航消息与异常保持 |
| `assembly_hardware.py` / `AssemblyHardware` | 接口舵机准备、锁扣及合体标志交接 |
| `assembly_api.py` | 组合上述对象，构建一对模块的 SMACH 状态机 |
| `task/assembly_motion.py` | 按 module_ids 顺序组织多对模块 |

## 门槛与异常处理

状态仍为 `Standby → PreAlign → Approach → Assembly`，需要时转 `Retreat → PreAlign`。

- 预对齐名义间隙 15 cm，实际间隙误差 <2 cm；固定参考轴下的接口 Y/Z 各 <3 cm。姿态检查同时要求双方相对姿态、各自相对固定参考姿态各轴误差 <4°，避免两机一起倾斜却被判为已对齐。
- 使用完整 1 s 新鲜观测窗口，至少 90% 样本达标且当前样本达标。PreAlign 不使用差分速度门槛。
- 轴向目标推进速度上限 **0.1 m/s**，临近预对齐点和最终接触点自动减速。该值是轨迹上限，不是实际飞机速度或三轴合成速度的硬约束。轴向目标领先实测位置不超过 2 cm。
- 失准先停止轴向推进、保持一次性捕获的固定坐标 X 目标，继续调姿/横向修正；同样的几何窗口确认恢复后继续。3 s 未恢复或误差达到 6 cm / 8° 时，近距离缓退，否则返回预对齐。
- 缓退上限 0.03 m/s，13 cm / 14.5 cm 滞回。最终锁扣要求 Y <2 cm、Z <1.5 cm，轴向间隙在 [-5 cm, 2 cm) 内且姿态达标；保留最多 5 cm 的向内目标偏置。
- 最终几何到位后，一次性捕获固定参考系下的 X 保持目标，冻结已有 Y/Z 轨迹目标和相对微调，轴向前馈归零。连续 **0.3 s** 几何达标后进入锁扣；速度估计仅供诊断，不再参与放行。保持分支跳过正常的实测位置领先量裁剪，避免裁剪重新移动保持目标。
- 保持期间任意最终几何条件失效，即取消计时、停止轴向推进并恢复纠偏，即使误差仍在粗对齐容差内也不放行。使用原恢复窗口确认最终几何恢复，再重新完整保持 0.3 s；持续/严重失准沿用缓退逻辑。
- 0.3 s 是几何保持时间，不证明实际速度为零。首次进入到位区仍可能有惯性运动，需在仿真中验证末段制动和接触效果。
- TF 中断发送最后有效实测位姿的零速保持，清空几何计时。恢复时保持原参考系和微调量，从当前实测姿态重新平滑调姿，近距离先恢复间隙。重复时间戳不会推进轨迹或计时。
- 退出 HOVER、flight_state 超时或 `/emergency_assembly_interuption` 中止任务；舵机没有订阅连接时不再假定准备/锁扣已完成。已经发送的机械动作无法撤回。

默认连接点为 root±0.26 m，并非实测插接面。`leader_connector_xyz/rpy`、`follower_connector_xyz/rpy` 可填写标定值；完全对齐时两个接口参考坐标系的轴应平行。标定误差仍会影响合体成功率。

`[Docking]` 日志输出参考捕获位置和 yaw，以及 `gap`、`yz`、`attitude_error_deg`、`trim_yz`、`reason`、`window`、`pass`、`paused`、`axial_cmd`、`final_hold`。最终保持原因显示为 `final_geometry_hold`，`final_hold` 显示已保持时间/要求时间。角度日志现在是三组姿态误差逐轴绝对值的最大值，不再是有符号的双方相对 RPY。

本轮没有改动 PID 或 unified 控制算法。调整发生在合体前的导航层；锁扣后仍通过原有 assembly_flag 进入后续控制。

最终保持配置为 `~docking/final_hold_time: 0.3`。已删除 `velocity_tolerance` 和 `final_settle_time`，自定义 YAML 应同步移除旧项；它们不再控制锁扣条件。

## 验证与下一次仿真

```bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m unittest discover \
  -s robots/beetle/test -p 'test_docking_*.py' -v
```

52 项离线测试通过，覆盖原几何窗口与恢复流程，以及固定目标抗 leader 扰动、共同倾斜拒绝放行、共同姿态平滑、yaw 跨 ±π、FC 偏移补偿、镜像与标定接口、微调方向/限速/饱和、TF 恢复无目标跳变、leader 漂移中止、锁扣异常和完整理想跟踪流程；新增最终目标/微调冻结、0.3 s 连续保持、速度噪声/未知速度不阻塞、严格最终容差失效后重新计时、失准后恢复微调和 TF 中断清除保持的测试。ROS 通信使用替身，未运行 Gazebo 或实机闭环。

下一次仿真使用：

```bash
roslaunch beetle assembly_motion.launch module_ids:=2,3 real_machine:=false
```

两机已稳定悬停后再启动。检查双方 `/uav/nav`：调姿过渡结束后，roll/pitch/yaw 目标应保持固定；follower 的轴向目标平滑推进，横向/高度只有慢速修正。记录 `/tf`、双方 `/uav/nav`、`flight_state`、实际姿态和 servo angle，重点比较“目标变化是否先于实际摆动”。若卡住，结合 `attitude_error_deg`、`gap`、`trim_yz` 和 `reason` 判断实际误差，勿直接放宽门槛。

此前 `new_guidance_replay.json`、`real_bag_gate_check_20261020.json` 是旧实验的几何门槛回放，不包含固定参考与微调后的闭环响应，不能用来声称新控制已经解决实机问题。
