# 简化合体流程：2026-10-21

本轮将合体流程收敛为“接口准备 → 预对齐 → 接近 → 锁扣”。保留位置与姿态联合导航，删除为抵抗扰动而额外添加的等待、退出和恢复状态。

## 使用

两机悬停后，在运行合体脚本的电脑启动：

```bash
roslaunch beetle assembly_motion.launch module_ids:=2,3 real_machine:=true
```

仿真使用 `real_machine:=false`，模块编号按实际场景填写。Python 和 YAML 修改无需编译飞控或 C++；重启脚本即可使用。配置位于 `robots/beetle/config/DockingConfig.yaml`，可通过 `docking_config:=...` 指定替代文件。

## 当前行为

1. **Standby：接口准备。** 等待双方报告 HOVER，然后准备公母连接舵机；准备完成即进入 PreAlign。
2. **PreAlign：预对齐。** follower 向接口前方 15 cm 的固定中间目标移动，同时控制位置与 roll/pitch/yaw。接口间隙相对 15 cm 的误差、Y/Z 误差各 <5 cm，双方接口相对姿态各轴 <8.6°（约原来的 0.15 rad），一次达标即进入 Approach。
3. **Approach：接近。** 保持共同姿态，向名义接触点内侧 5 cm 的目标推进。途中误差不触发暂停、自动缓退、重新预对齐或任务退出，由底层控制器继续跟踪目标。
4. **Assembly：锁扣。** 实际名义接口间隙在 [-5 cm, 2 cm)、Y 误差 <2 cm、Z 误差 <1.5 cm、双方接口相对姿态各轴 <8.6° 时，立即转入原锁扣和 assembly_flag 交接。不再等待差分速度达标或额外保持 0.3 s。

预对齐及接近的轴向目标速度上限为 **0.1 m/s**，Y/Z 目标各轴以最高 0.05 m/s 变化；靠近固定目标时按剩余目标距离减速。这里限制的是目标轨迹速度，实际飞机速度取决于控制跟踪。轨迹不再使用“目标距离实测位置最多 2 cm”的裁剪，避免测量变化把原本的保持目标向内拖动。

共同参考系在首次有效 TF 时捕获：参考姿态为水平接口姿态和初始 leader 接口 yaw，原点考虑 leader FC 到接口的几何偏移。leader 保留原悬停位置目标，只更新姿态；follower 按固定参考系导航。姿态以 10°/s 平滑调整到共同目标。不存在额外的相对 Y/Z 误差积分微调；双方实际相对几何仅用于两个阶段的到位判断。

默认接口坐标为 root±0.26 m，不代表实测插接面。可配置双方 `connector_xyz/rpy`；完全对齐时两个接口坐标系的轴应平行。持续的定位偏差仍可能使基本到位条件不满足，应结合标定与实际控制误差分析。

## 已移除与保留的处理

删除 leader 5 cm 漂移退出、状态消息接收超时退出、TF 与地面时钟的差值检查、TF 采样间隔触发的重新预对齐、相对速度门槛、1 s/90% 通过窗口、最终 0.3 s 保持、误差超限暂停、3 s 恢复计时、Retreat 状态及滞回、相对误差积分微调，以及实测位置领先量裁剪。对应配置、实现和旧行为测试同步删除。

保留人工 `/emergency_assembly_interuption` 急停，以及收到实际非 HOVER 状态（例如 LAND）后的停止处理；不会仅因状态消息暂时未更新而退出。TF 查询失败或数据非有限数时不发送无效目标，发送最后有效实测位姿的零速保持。TF 恢复后在原阶段从当前实测位置继续生成轨迹，不自动缓退或重新建立参考。

TF 仍按同一共同时间查询；轨迹步长使用本地 ROS 控制时钟，不比较机载 TF 时间与地面时间，也不使用 TF 时间差作为状态转换条件。

日志输出 `phase`、`gap`、`yz`、`relative_rpy_deg`、`target_xyz`（固定参考系中的轨迹目标）及 `axial_cmd`，已删除旧窗口/暂停/保持进度字段。

## 模块职责与验证

| 文件 | 职责 |
| --- | --- |
| `docking_guidance.py` | 预对齐/接近轨迹及基本到位判断 |
| `docking_reference.py` | 固定坐标转换、共同姿态平滑 |
| `assembly_alignment.py` | TF 与飞行状态读取、导航消息、人工中断 |
| `assembly_hardware.py` | 舵机准备、锁扣及合体交接 |
| `assembly_api.py` | 四个 SMACH 状态的组合 |
| `task/assembly_motion.py` | 按 module_ids 组织模块对 |

```bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m unittest discover \
  -s robots/beetle/test -p 'test_docking_*.py' -v
```

35 项离线测试通过，包括完整理想跟踪流程、速度限制、单次到位放行、扰动期间继续接近、末端无时间/速度门槛、leader 高度偏移 6 cm 不退出、状态消息年龄不退出、TF 恢复保持阶段、±0.45 s 时钟差、坐标与姿态转换、人工急停、实际 LAND 后停止、舵机准备及四状态连接。

ROS 通信使用替身，未运行新的 Gazebo 或实机闭环。先前的 bag 门槛回放文件反映历史实现，不能用于证明当前闭环效果。
