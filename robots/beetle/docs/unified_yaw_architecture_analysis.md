# Beetle Unified 模式 Yaw 控制架构分析

## 1. 背景：为什么 Yaw 之前采用"独立通道"（旁路分配）

### 1.1 原始设计意图

Beetle unified 模式最初将 yaw 排除在 QP 分配之外（`yaw_in_allocation: false`），原因如下：

1. **降低分配器复杂度**：6-DOF QP 分配比 5-DOF 更难求解，特别是在编队几何不对称时，yaw 列可能导致分配矩阵条件数变差。去掉 yaw 后矩阵变为 5×N，更稳定。

2. **避免 yaw 与分配器耦合**：在编队中，roll/pitch 力矩需要各模块差异化分配（不同模块到质心距离不同，需要不同推力），但 yaw 力矩可以通过"所有模块发相同指令"来产生——这更简单，不需要优化分配。

3. **级联架构中的角色分工**：unified 模式的 roll/pitch 采用 PC (I-only) + Spinal (P+D at 1000Hz) 级联。yaw 不需要这种分割——PC 端全量 P+I+D 计算后，只需要一个标量广播到各 Spinal，Spinal 用 `cascade_yaw_d` 做高频阻尼即可。

4. **历史延续**：GimbalRotor 在 `gimbal_calc_in_fc && i_term_rp_calc_in_pc` 模式下（Beetle 的基础配置），yaw 在 wrench 中也被置零（`target_ang_acc_z = 0`）。Beetle 延续了这个设计。

### 1.2 独立通道的工作方式

```
PC (40Hz):
  yaw_pid_raw = pid_controllers_[YAW].result()   // 完整 P+I+D
  target_wrench_acc(5) = 0.0                      // 不进入分配
  ↓
computeUnifiedAllocation():
  candidate_yaw_term_ = yaw_pid_raw × max_yaw_scale   // 标量
  ↓
旧集中式路径中由 publishCommands() / 新架构中由各模块本地 pickup 后发给各自 Spinal:
  angles[2] = candidate_yaw_term_                 // 所有模块收到相同值
  ↓
Spinal (1000Hz):
  yaw_term[i] = TorqueAllocInv[i][yaw_col] × candidate_yaw_term_
  target_thrust[i] += yaw_term[i]                 // 加到各电机
  // 另外 cascade_yaw_d 做角速度 D 阻尼
```

**关键特征**：yaw 是一个标量广播，分配矩阵的 yaw 列虽然被发送到 Spinal（用于 per-motor 分解），但 PC 端的 QP 优化不考虑 yaw 自由度的力矩平衡。

### 1.3 独立通道的局限

1. **无法利用编队几何优势**：编队中不同模块与质心的力臂不同，万向节角度不同，对 yaw 力矩的贡献能力差异很大。独立通道给所有模块发相同 `candidate_yaw_term_`，相当于放弃了优化分配的机会。

2. **与 GimbalRotor/Ninja 不一致**：在 GimbalRotor 独立模式下（`i_term_rp_calc_in_pc=false`），yaw 始终参与 6-DOF 分配矩阵。Beetle unified 控制的模块数量更多、执行器更多，反而不利用 yaw 分配不合理。

3. **无法独立调参**：由于 `applyUnifiedGains()` 不修改 yaw 增益，unified 模式（惯量更大、阻尼不同）和独立模式共用一组 yaw PID，限制了调参空间。

---

## 2. GimbalRotor 的 Yaw 控制架构

### 2.1 概述

GimbalRotor 是 Beetle 的父类控制器。它支持多种配置（quad/tri/bi/tri_omni/dragonfly），但 **没有 unified 模式**——每个机器人独立运行自己的控制器。

### 2.2 Yaw 在分配矩阵中的位置

GimbalRotor 的 `controlCore()` 构建一个 **6×(rotor_coef × motor_num)** 的分配矩阵：

```
integrated_map =
  ┌─────────────────────────────────────┐
  │ Row 0-2: 力加速度 (fx, fy, fz)/M   │
  │ Row 3:   Roll 角加速度 / I_xx       │
  │ Row 4:   Pitch 角加速度 / I_yy      │
  │ Row 5:   Yaw 角加速度 / I_zz        │  ← yaw 始终在此
  └─────────────────────────────────────┘
```

每个旋翼的 3×3 力-力矩映射包含：
```cpp
wrench_map.block(3,0,3,3) = skew(r_i) + dir_i × m_f_rate × I₃
```
- `skew(r_i)`: 力臂 × 推力 → 力矩（杠杆效应）
- `m_f_rate`: 反扭矩系数（旋翼自旋产生的反向力矩）

经万向节遮罩后 → 伪逆 → `target_vectoring_f_ = integrated_map_inv × target_wrench_acc_cog`

**Yaw 力矩需求通过伪逆在所有旋翼间最优分配**。

### 2.3 两种模式下 Yaw 进入 wrench 的方式

| 条件 | `target_ang_acc_z` | Yaw 在 wrench 中 |
|------|-------------------|------------------|
| `gimbal_calc_in_fc && i_term_rp_calc_in_pc` | `0` | **零**（但 `candidate_yaw_term_` 仍计算） |
| 其他情况 | `YAW.result()` (完整 P+I+D) | **参与分配** |

**重要发现**：GimbalRotor 所有标准配置的 YAML 中 **都没有设置 `i_term_rp_calc_in_pc`**（默认 `false`），因此走的是 else 分支——**yaw 完整 PID 进入分配矩阵**。

只有 Beetle（`i_term_rp_calc_in_pc: true`）和 Ninja HW 使用了级联模式，此时 yaw 在 wrench 中为零。

### 2.4 `candidate_yaw_term_` 的双通道机制

即使 yaw 进入了分配矩阵（已影响 `target_vectoring_f_`），GimbalRotor **同时还计算** `candidate_yaw_term_`：

```cpp
candidate_yaw_term_ = pid_controllers_[YAW].result() × max_yaw_scale;
```

并通过 `angles[2]` 发送到 Spinal。Spinal 使用 `TorqueAllocationMatrixInv` 的 yaw 列将其分解到各电机。

这是 **双通道架构**：
1. PC 分配矩阵：yaw PID → 优化分配 → 改变各旋翼的基础推力和万向节角度
2. Spinal 重建：`candidate_yaw_term_` → per-motor yaw 修正 at 1000Hz

两者协同工作，分配矩阵决定"期望的力分布"，Spinal 做高频率修正。

---

## 3. Ninja 的 Yaw 控制架构

### 3.1 继承关系

```
NinjaController → BeetleController → GimbalrotorController → PoseLinearController
```

Ninja 的 `controlCore()` 调用 `BeetleController::controlCore()`，后者调用 `GimbalrotorController::controlCore()`。

### 3.2 Yaw 处理

- Ninja **没有 unified 模式**——即使组装后，每个模块独立运行自己的控制器
- Yaw PID 通过 GimbalrotorController 标准路径处理
- **硬件配置**（`i_term_rp_calc_in_pc: true`）：yaw 在 wrench 中为零，走 `candidate_yaw_term_` 旁路
- **仿真配置**（`i_term_rp_calc_in_pc: false`）：yaw 完整参与分配矩阵

### 3.3 模块间协调

Ninja 不使用中央统一控制，而是：
- 关节 PID（`joint_pitch`, `joint_yaw`）控制组装关节角度
- 交互力估计 + 前馈补偿实现模块间协调
- 每个模块独立稳定自身，yaw 增益不需要区分"组装/未组装"

### 3.4 2-DOF 万向节的全驱动优势

Ninja 每个旋翼有 **2-DOF 万向节**（roll + pitch），3 个旋翼 × 3 个控制量 = 9 个执行器输入，可独立控制全部 6-DOF。Yaw 力矩通过万向节推力矢量化 + 反扭矩共同产生。

---

## 4. 三种平台对比

| 特性 | GimbalRotor (标准) | Beetle unified (改进前) | Beetle unified (改进后) | Ninja |
|------|-------------------|----------------------|----------------------|-------|
| Yaw 在分配矩阵中 | **是** | **否** (`yaw_in_allocation=false`) | **可选** | 取决于 `i_term_rp_calc_in_pc` |
| `candidate_yaw_term_` | 始终计算发送 | 始终计算发送 | 始终计算发送 | 始终计算发送 |
| Yaw 独立增益 | 一套 | 一套（独立=unified 共用） | **两套**（unified_yaw 独立调） | 一套 |
| 运行时 Yaw 调参 | `controller/yaw` dynreconf | `controller/yaw` dynreconf | `controller/yaw` + **`controller/unified_yaw`** dynreconf | `controller/yaw` dynreconf |
| Yaw 力矩优化分配 | 通过伪逆 | 不优化（标量广播） | 可通过 QP 优化 | 通过伪逆 |
| Spinal Yaw D 阻尼 | 有（自动） | `cascade_yaw_d` | `cascade_yaw_d` | 有（自动） |
| Unified 模式 | 无 | 有 | 有 | 无 |

---

## 5. 当前改进后的架构

### 5.1 新增功能

1. **`unified_yaw` YAML 配置节**：在 `BeetleControl.yaml` 和 `BeetleControl_sim.yaml` 中新增，支持完整 P/I/D + limits
2. **`unified_yaw_gains_` 成员**：保存 unified 模式专用的 yaw PID 参数
3. **`applyUnifiedGains()` 扩展**：切换到 unified 模式时自动保存独立模式 yaw 增益，应用 unified yaw 增益
4. **`restoreIndependentGains()` 扩展**：退出 unified 模式时恢复独立模式 yaw 增益
5. **`controller/unified_yaw` 动态参数服务器**：运行时通过 `rqt_reconfigure` 实时调整 unified yaw PID

### 5.2 调参路径

```
独立模式下调 yaw:
  rqt_reconfigure → controller/yaw → pid_controllers_[YAW]

Unified 模式下调 yaw:
  rqt_reconfigure → controller/unified_yaw → unified_yaw_gains_
                                            → pid_controllers_[YAW] (实时推送)
```

### 5.3 `yaw_in_allocation` 配合

`unified_yaw` 增益与 `yaw_in_allocation` 标志独立工作：

- `yaw_in_allocation: false` + `unified_yaw`：yaw 仍走旁路（`candidate_yaw_term_`），但 PID 增益可以针对 unified 模式优化
- `yaw_in_allocation: true` + `unified_yaw`：yaw 进入 QP 分配 + 旁路双通道，增益可独立调整

**推荐**：先用 `yaw_in_allocation: false` 验证 `unified_yaw` 增益的效果，稳定后可尝试 `yaw_in_allocation: true` 获得完整的 6-DOF 优化分配。

---

## 6. 修改的文件清单

| 文件 | 修改内容 |
|------|---------|
| `robots/beetle/config/BeetleControl.yaml` | 新增 `unified_yaw:` 配置节 |
| `robots/beetle/config/BeetleControl_sim.yaml` | 新增 `unified_yaw:` 配置节 |
| `robots/beetle/include/beetle/control/beetle_controller.h` | 添加 `unified_yaw_gains_`, `saved_yaw_gains_`, `unified_yaw_reconf_server_` |
| `robots/beetle/src/control/beetle_controller.cpp` | `rosParamInit()` 加载 unified_yaw; `applyUnifiedGains()` 切换 yaw; `restoreIndependentGains()` 恢复 yaw; 创建 `controller/unified_yaw` dynreconf 服务器 |
