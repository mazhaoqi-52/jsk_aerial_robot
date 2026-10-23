# 合体流程与相对位置修正：2026-10-23

本轮将合体流程收敛为“接口准备 → 预对齐 → 接近 → 锁扣”。保留位置与姿态联合导航，删除为抵抗扰动而额外添加的等待、退出和恢复状态。

## 使用

两机悬停后，在运行合体脚本的电脑启动：

```bash
roslaunch beetle assembly_motion.launch module_ids:=2,3 real_machine:=true
```

仿真使用 `real_machine:=false`，模块编号按实际场景填写。Python 和 YAML 修改无需编译飞控或 C++；重启脚本即可使用。配置位于 `robots/beetle/config/DockingConfig.yaml`，可通过 `docking_config:=...` 指定替代文件。

## 当前行为

1. **Standby：接口准备。** 等待双方报告 HOVER，然后准备公母连接舵机；准备完成即进入 PreAlign。
2. **PreAlign：非接触距离精对齐。** 接口间隙目标保持 15 cm，X 误差 <2 cm；新增独立的 Y/Z 阈值各 **8 mm**、相对 yaw <**2°**，相对 roll/pitch 各 <3°。Y/Z 和姿态持续修正，一次达标即进入 Approach，不等待速度窗口或保持时间。原来 Y/Z 各 2 cm 的粗对齐不再直接放行插接。
3. **Approach：较快插入。** 进入时根据当前 leader 接口确定一次固定 X 终点（名义接触点内侧 5 cm）。15 cm 内 X 目标速度上限为 **0.05 m/s**，靠近固定终点时按剩余距离减速。Y/Z 在整个插入期间继续有界纠偏，首次进入 5 cm 接触区后，额外 Y/Z 目标调整额度分别为 ±1 cm / ±1.5 cm。删除进入 4 cm 后永久冻结 Y/Z 的逻辑；几何误差回到 1.4 cm 时仍可继续纠正。
4. **Assembly：锁扣。** 实际名义接口间隙在 [-5 cm, 2 cm)、Y 误差 <2 cm、Z 误差 <1.5 cm、双方接口相对姿态各轴 <8.6° 时，立即转入原锁扣和 assembly_flag 交接。不再等待差分速度达标或额外保持 0.3 s。

预对齐 X 目标速度上限保持 **0.1 m/s**。`GapLimitedApproachSpeed` 负责插入速度：`min(approach_speed, near_approach_speed * max(1, gap / approach_slowdown_distance))`。默认 `near_approach_speed: 0.05`、`approach_slowdown_distance: 0.15`；15 cm 内不超过 5 cm/s，15～30 cm 从 5 增至 10 cm/s。精对齐放行后，Y/Z/yaw 误差不再把插入速度乘到最低倍率；删除 `AlignmentAwareApproachSpeed`、`approach_alignment_*` 和 `approach_min_speed_scale`。这限制的是目标速度，实际闭合速度仍取决于跟踪和外部扰动。

预对齐 X 相对位置修正由 `RelativePositionCorrection` 类完成：目标速度为 `-relative_correction_gain * 相对误差`，先限速，再按控制周期积分到上次目标。增益保持 **0.5 /s**，预对齐 X 限速为 `approach_speed`。

Y 使用 `DampedLateralCorrection`，在预对齐和接近阶段均采用 `v_y = clip(-relative_correction_gain * e_y - lateral_velocity_damping * filtered_relative_vy, ±correction_speed)`。Y 增益保持 **0.5 /s**，目标速度上限保持 **0.02 m/s**；新增阻尼系数 **0.4**（`lateral_velocity_damping`）及滤波时间常数 **0.25 s**（`lateral_velocity_filter_time`），阻尼允许设为 0 作对照。接口仍在一侧、但正快速接近对齐位置时，目标会提前减速或反向调整。插入完成前持续纠偏，速度前馈来自条件积分后实际施加的目标增量。

Y/Z 共用 `BoundedDampedCorrection` 的速度阻尼和条件积分实现，分别保存滤波历史与接触基准；`DampedLateralCorrection`、`DampedHeightCorrection` 仅负责各轴参数映射。Y 新增 `lateral_tracking_error_limit: 0.03`，`contact_lateral_correction_limit: 0.01`，`contact_lateral_correction_gap: 0.05`。Y 目标与实测 FC 位置在固定对接 Y 轴上的跟踪偏差超过 3 cm 时，不继续增加该方向的积分；首次进入 5 cm 范围后，额外目标变化限于 ±1 cm。即使两机一起横向移动，也不会反复刷新接触额度。

Z 使用独立的 `DampedHeightCorrection`，在预对齐和接近阶段采用相同控制律：`v_z = clip(-height_correction_gain * e_z - height_velocity_damping * filtered_relative_vz, ±height_correction_speed)`，其中 `e_z` 为 follower 接口高度减 leader 接口高度。根据相邻高度误差和本地控制周期求相对速度，再做一阶低通平滑。例如静止的 follower 接口低 4 cm 时，默认 Z 目标以 0.8 cm/s 向上移动；若 follower 正快速上升，速度阻尼会提前减小或反转目标速度。不把瞬时实测位置直接复制为目标。导航消息同时发布 XYZ 目标位置和实际施加的目标速度前馈。

| Z 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `height_correction_gain` | 0.2 /s | 高度误差修正增益 |
| `height_correction_speed` | 0.01 m/s | 高度目标速度上限 |
| `height_velocity_damping` | 0.4 | 相对垂直速度阻尼，允许设为 0 作对照 |
| `height_velocity_filter_time` | 0.25 s | 相对速度低通时间常数 |
| `height_tracking_error_limit` | 0.03 m | 导航目标与实测 follower FC 高度的跟踪偏差限制 |
| `contact_height_correction_limit` | 0.015 m | 首次进入接触区后，目标相对当时高度的额外调整范围 ±1.5 cm |

高度采用条件积分：若本次修正会继续扩大超出 3 cm 的 FC 目标跟踪偏差，就停止该方向的累积；靠近边界时只允许剩余可用的增量，反向修正仍可执行。反馈移动导致旧目标已在边界外时，不直接裁剪或拖动旧目标，避免重新引入“实测位置变化带着保持目标移动”的问题。

单机跟踪偏差小不代表接口误差在减小：接触受阻时，两机可能一起升降。因此首次进入 `contact_height_correction_gap` 范围（保持 **0.05 m**）还会记录高度目标，之后的高度修正被限制在其 ±1.5 cm 内；即使暂时离开再进入该范围，也不会刷新调整额度。Y/Z 接触额度的起点分别配置，离开再进入不会刷新额度。Y/Z 的边界均通过条件积分实现，不拖动已有目标。Z 跟踪限幅仍为 3 cm，阻尼、滤波和速度上限保持原值；只将接触后高度额度从 ±1 cm 增至 ±1.5 cm。

`contact_lock_gap` 与 `correction_locked` 已删除。只有最终合体条件满足、进入锁扣交接时才保持目标；完成时使用实测 FC 的等效轨迹坐标，保持 FC 位置连续，避免用实际接口位置配合目标姿态重新生成 FC 位置而产生跳变。

**接口对齐与 FC 跟踪分别计算。** `FixedDockingReference.tracking_position()` 将实测 FC 位置按本周期的期望接口姿态映射成等效轨迹坐标。`command - tracking_position` 对应固定对接坐标轴上的实际 FC 跟踪误差。接口实际位置/姿态仍用于对齐反馈；Y/Z 条件积分的 3 cm 边界则使用 FC 跟踪误差。参考姿态先平滑更新，再用相同姿态计算限幅和发布目标。支持镜像、标定后的接口旋转以及非零 root→FC 偏移；条件积分模块不依赖 ROS/TF。

共同参考系在首次有效 TF 时捕获：参考姿态为水平接口姿态和初始 leader 接口 yaw，原点考虑 leader FC 到接口的几何偏移。leader 保留原悬停位置目标，只更新姿态；follower 的位置修正在这一固定坐标系中进行，不重新捕获原点或旋转对接轴。姿态以 10°/s 平滑调整到共同目标。相对几何同时用于位置修正、阶段到位判断和接近速度限制。

### 20:08:34 真机 bag 对应的问题

`2026-10-21-20-08-34.bag` 全程停在 PreAlign。启动时捕获的 leader FC 高度约 0.770 m，follower 水平姿态下的导航目标随后固定为 0.770314 m。leader 原控制器高度目标保持 0.800000 m，follower 的控制器高度目标为 0.777730 m；控制器目标与导航 FC 目标存在坐标偏移，不能直接混为同一高度。两机控制器高度目标相差约 2.23 cm，且实际相对误差未参与固定目标修正。

预对齐开始后 12～41 s 的日志中，实际间隙中位数约 11.35 cm、横向误差中位数约 -5.26 cm、高度误差中位数约 -1.95 cm。收紧阈值只会阻止失准时继续接近，无法纠正参考捕获偏差或 leader 后续偏移。本次相对位置修正用于解决这一目标生成问题：leader 回升后，即使旧世界坐标目标已到达，follower 的高度目标仍会继续向上修正；预对齐 X/Y 的残余误差也会参与修正。

### 20:31:32 真机 bag 对应的问题

`2026-10-21-20-31-32.bag` 中，预对齐高度目标约 0.751～0.816 m，follower 实测 FC 高度约 0.648～0.830 m；进入 Approach 后约 0.8 s，实际接口高度差达到约 5.2 cm。原控制律只根据位置误差积分目标，没有利用相对垂直速度提前减速。新实现将 Z 增益从共享的 0.5 独立降至 0.2，将 Z 限速从 0.02 降至 0.01 m/s，并增加上述阻尼与条件积分。

该 bag 后期名义间隙约 4.4 cm、接口高度差约 2 cm，未触发几何 Y/Z 锁定，目标仍持续下调至约 0.727 m。新增接触区高度调整范围用于限制这种误差不减小、两机却一起移动时的继续累积。

### 2026-10-23 20:08:22 卡接后的调整

该次实验接触附近横向误差约 2 cm，随后在名义间隙约 4.5 cm 时锁定 Y/Z；高度差后来增至约 2 cm，但目标已无法继续修正。视频确认已接触并卡住。上一轮给 Y 加入相对速度阻尼，并将 Y/Z 几何锁定距离从 5 cm 改为 4 cm，在此前保留限速纠偏；原高度跟踪限幅、5 cm 开始的 ±1 cm 调整额度及 Z 阻尼均保留。轴向速度和任务状态连接不变。

### 2026-10-23 20:39:29 对齐不足与受阻积分

该次切入 Approach 时 Y=-1.89 cm、Z=+1.98 cm、yaw=-3°，均在预对齐放行边界附近。随后名义间隙长期约 4 cm，Z 仍约 2 cm，Y 多为 0.3～1 cm；`correction_locked` 全程 False。进入 5 cm 区域后 Y 目标继续累计移动约 4.7 cm，表明只有速度上限仍可能持续推拉。上一轮采用连续接近减速、Y 条件积分，并将接触区 Z 额度适度增至 ±1.5 cm。本轮保留 Y/Z 条件积分和额度，改为精对齐后以 5 cm/s 上限插入。高度误差的最终到位阈值仍为 1.5 cm；可调范围与成功判定阈值是两个不同概念。共同固定姿态和 yaw PID 保持现有设置。

### 2026-10-23 20:57:30 提前冻结与跟踪参考点

该次进入 Approach 时接口 Z 约 +1.88 cm，接触附近变为 -2.04 cm。FC 高度目标先因跟踪限幅停在约 0.788731 m，随后接口仍低约 1.44 cm 就触发永久 Y/Z 锁定，约 10 s 不再纠偏。接触区 ±1.5 cm 额度当时未耗尽。接近阶段 +6.2 s 附近，FC 跟踪偏差约 1.95 cm，而含 pitch 几何项的接口跟踪偏差约 3.47 cm；这说明必须分开对齐反馈和实际控制点跟踪限幅。

本轮删除提前冻结、使用 FC 等效跟踪坐标，并把 Y/Z 8 mm、yaw 2° 的精对齐放在原 PreAlign；随后恢复更快的插入上限，减少近距离停留。原记录轨迹在原进入 Approach 后约 1.3 s、名义间隙约 15 cm 时满足新条件；这是记录反馈检查，不预测修改后的闭环运动或脱卡能力。

默认接口坐标为 root±0.26 m，不代表实测插接面。可配置双方 `connector_xyz/rpy`；完全对齐时两个接口坐标系的轴应平行。持续的定位偏差仍可能使基本到位条件不满足，应结合标定与实际控制误差分析。

## 已移除与保留的处理

保持此前删除的 leader 5 cm 漂移退出、状态消息接收超时退出、TF 与地面时钟的差值检查、TF 采样间隔触发的重新预对齐、相对速度门槛、1 s/90% 通过窗口、最终 0.3 s 保持、误差超限暂停、3 s 恢复计时、Retreat 状态及滞回，以及实测位置领先量裁剪。本次相对位置修正不增加恢复状态或自动重试。

保留人工 `/emergency_assembly_interuption` 急停，以及收到实际非 HOVER 状态（例如 LAND）后的停止处理；不会仅因状态消息暂时未更新而退出。TF 查询失败或数据非有限数时不发送无效目标，发送最后有效实测位姿的零速保持。TF 恢复后在原阶段从当前实测位置继续生成轨迹，保留本次固定 X 终点及 Y/Z 接触区基准；重置 Y/Z 差分与速度滤波历史，避免恢复瞬间的差分尖峰，不自动缓退或重新建立参考。

TF 仍按同一共同时间查询；轨迹步长使用本地 ROS 控制时钟，不比较机载 TF 时间与地面时间，也不使用 TF 时间差作为状态转换条件。

日志输出 `phase`、`gap`、`yz`、`relative_rpy_deg`、`target_xyz`、`axial_cmd`、`target_fc_z`、`relative_vz/vy`、`height_limited/lateral_limited`、`fc_tracking_yz` 和 `height_limit_reason/lateral_limit_reason`。限幅原因区分 `none`、`tracking`、`contact` 或 `tracking+contact`，避免把“没有积分限幅”误读成“目标一定仍在修正”。

## 模块职责与验证

| 文件 | 职责 |
| --- | --- |
| `docking_guidance.py` | X 相对位置修正、精对齐和插入、Y/Z 阻尼和条件积分、到位判断 |
| `docking_reference.py` | 固定坐标转换、共同姿态平滑、实测 FC 等效跟踪坐标 |
| `assembly_alignment.py` | TF 与飞行状态读取、导航消息、人工中断 |
| `assembly_hardware.py` | 舵机准备、锁扣及合体交接 |
| `assembly_api.py` | 四个 SMACH 状态的组合 |
| `task/assembly_motion.py` | 按 module_ids 组织模块对 |

```bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m unittest discover \
  -s robots/beetle/test -p 'test_docking_*.py' -v
```

86 项离线测试覆盖完整理想跟踪与持续偏差流程、精对齐误差分别阻止过早插入、失败 bag 的粗对齐数据不再放行、达标后立即以 5 cm/s 上限插入、4 cm 区域小偏差继续纠正、FC 与接口参考点分离、pitch -3°/FC 落后 2 cm 时允许继续上调、跟踪受限后恢复可重新纠偏、带剩余 pitch 的完成保持不跳 FC 位置、标定/镜像情况下的 FC 误差等价关系、Y/Z 跟踪与接触额度、TF 恢复、人工急停、LAND 和原四状态连接。

上一轮使用 20:31:32 bag 的记录反馈重建并回放 918 个控制周期（当时接触额度为 1 cm）：新 Z 目标速度不超过 0.01 m/s，接触区相对首次高度目标的变化不超过 1 cm；在保持原实测轨迹的回放中，末端生成的 FC 高度目标约为 0.774 m，原记录目标约为 0.727 m。回放只验证目标生成、限速及防累积行为，不预测修改后的实际飞机运动。

ROS 通信使用替身，未运行新的 Gazebo 或实机闭环。先前的 bag 门槛回放文件反映历史实现，不能用于证明当前闭环效果。
