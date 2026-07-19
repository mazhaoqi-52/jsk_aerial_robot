# Beetle unified mode QP 设计与可迁移性分析

日期：2026-05-28  
当前工作分支：`develop/Beetle_hyper_qp`  
参考分支：`develop/assemble_quadrotors`，`develop/spidar/pedipulate`

本文先按 `~/ros/jsk_aerial_robot_ws/CLAUDE.md` 的要求区分“代码中已经实现的事实”和“可以作为论文/后续实现的设计建议”。结论先写在前面：

1. 当前 Beetle unified mode 的核心确实已经从 leader-follower 的残差补偿，转向了“整机 6-DoF wrench allocation + QP 约束分配”。这是论文里最应该展开的部分。
2. 但是，当前代码中的 QP 还不是严格意义上的“模块间内力约束 QP”。它现在主要约束每个 rotor 的矢量推力分量和 gimbal 角度，并用一个很小的 secondary objective 把解拉回 balanced hover reference。
3. 当前 `unified_internal_wrench_secondary_gain` 默认是 0，所以 legacy LF 的 internal/inter-wrench 计算在 unified mode 中默认只做诊断和 log，不进入控制。
4. 当前 QP 没有能量约束、passivity constraint、energy tank，也没有 thrust rate penalty。它依靠 PID limit、desired wrench timeout、gimbal/thrust component bound、warmup/reset 等工程保护减少风险，但这不能证明“防止系统缓慢发散”。
5. `develop/spidar/pedipulate` 的 Spidar QP 对 Beetle 很有参考价值，特别是 `A1 f + b1` 这种“actuator/contact force 到 joint torque/internal load”的线性映射。若要把“模块间内力不要太大”变成 unified mode 的主要贡献，应在 Beetle QP 中加入类似 Spidar 的 interface-load map，并把它放进 objective 和/或 constraints。

## 1. 术语：关于“内力/外力”的实事求是说法

老师的评论是合理的。Beetle 当前代码里 `internal_wrench`、`inter_wrench_list_`、`wrench_comp_list_` 这些名字来自 leader-follower 模式下的 weak-link/chain-like residual compensation。它们在代码中表示“模块观测到的残差 wrench 经过递推得到的 joint-cut/inter-module residual”，而不是 unified rigid-body model 里的严格内力变量。

在 unified mode 中，更准确的说法应是：

- `formation_desired_wrench_` 是任务层给整机的外部期望 wrench，例如 towing force。
- `est_wrench_list_` 是每个模块 momentum observer 估计到的外部/接触/建模误差综合 residual。
- `est_wrench_task_list_` 是 demo/task 层预测的任务引起的每模块 wrench。
- `est_residual = est_wrench - est_wrench_task` 后，`inter_wrench_list_` 更像“inter-module residual/joint-cut residual diagnostic”。
- 这些量在 unified allocation 的主问题里目前没有作为 hard internal-force constraints。

因此论文里建议避免把 unified mode 的主要概念说成“内力/外力分解”。更稳妥的表述是：

> Unified mode treats the assembled Beetle as one redundant full-vectoring aerial robot. The key problem is a constrained whole-body allocation QP that distributes actuator effort among modules according to task-dependent load-sharing objectives and physical/interface constraints.

中文可写为：

> unified mode 将合体后的 Beetle 视为一个冗余全矢量推力空中机器人；核心问题不是区分所谓内力/外力，而是在满足整机 wrench 跟踪和执行器约束的前提下，通过 QP 的目标函数与约束条件实现任务相关的模块出力分配，并限制模块接口载荷。

## 2. QP 是什么，为什么适合 unified allocation

QP，即 Quadratic Programming，是如下形式的凸优化问题：

```text
minimize    0.5 x^T P x + q^T x
subject to  l <= C x <= u
```

其中 `x` 是待优化变量，`P` 是半正定矩阵，`C x` 是线性约束。OSQP/OsqpEigen 正是求解这种问题的工具。

在 multirotor allocation 中，`x` 通常是所有电机/矢量推力的分量。冗余系统往往有很多组 actuator force 都能产生同一个总 wrench。QP 的价值在于：

- 用 primary objective 追踪整机期望 wrench。
- 用 secondary objective 决定冗余自由度如何分配，例如某些模块多出力、某些模块少出力。
- 用 constraints 保证物理可行，例如推力上限、gimbal 角度、joint torque/interface wrench 上限、rate limit 等。
- 在不可行时通过 slack variable 或软约束明确暴露“哪些目标无法同时满足”。

这正对应 Beetle unified mode 的论文贡献空间。

## 3. 当前 Beetle unified mode 控制链路

### 3.1 控制入口

当前 `BeetleController::controlCore()` 在 `unified_control_mode_` 为 true 且模块为 LEADER/FOLLOWER 时，leader 和 follower 都进入 `runUnifiedControlCommon(is_leader)`，本地运行完整的 6-DoF outer PID 和 formation allocation。代码注释明确说这是“symmetric local control”，目的是避免 leader 到 follower 的延迟污染本地响应：`robots/beetle/src/control/beetle_controller.cpp:416-438`。

离开 unified mode 时，代码会恢复 independent hover、reset QP state、reset unified reference warmup，并把 navigator unified flag 置回 false：`robots/beetle/src/control/beetle_controller.cpp:443-470`。

### 3.2 unified navigation 与旧 LF 的差别

`BeetleNavigator::assemblyNavCallback()` 在 unified mode 中把用户给的 assembly CoG target 转成当前模块的 CoG target，使每个模块都能在本地重构同一个 formation CoG target：`robots/beetle/src/beetle_navigation.cpp:457-604`。

特别重要的是 roll/pitch 的处理：unified mode 不直接写 `target_rpy_` 去让外环追踪 roll/pitch，因为这会造成 R/P I-term windup。代码注释记录了一个验证过的失败现象：pitch=0.4 时 pitch_i 在 10 s 内涨到 11.6 Nm，消耗 Z thrust 直到 auto-land：`robots/beetle/src/beetle_navigation.cpp:607-612`。

这说明当前代码已经把一部分“慢发散”问题定位为控制架构和积分器路径问题，而不是 QP 能量约束问题。

### 3.3 任务 wrench 路径

任务层 wrench 在 unified mode 中走 `formation_desired_wrench_`，在 LF mode 中走 legacy `desired_external_wrench` 兼容路径。demo 脚本也明确写了：

- unified active 时发布到 `formation_desired_wrench`
- leader-follower active 时发布到 `desired_external_wrench`
- unified allocation 期望 formation-body wrench at CoG

见 `robots/beetle/script/demos/aerial_towing_demo/load_towing_formation.py:716-722`。

controller 端 `desiredExternalWrenchCallback()` 把 legacy topic 也当成 formation-level desired wrench alias。每个模块都存储完整的 6D formation wrench，leader 再转发给所有 follower：`robots/beetle/src/control/beetle_controller.cpp:2033-2088`。`formationDesiredWrenchCallback()` 复用这个逻辑：`robots/beetle/src/control/beetle_controller.cpp:2106-2112`。

在 unified 主循环里，`formation_desired_wrench_` 不作为 PID persistent FF 注入，而是在 `computeUnifiedAllocation()` 内部加入 allocation target：`robots/beetle/src/control/beetle_controller.cpp:2432-2453`。

### 3.4 6-DoF target wrench 的构成

`runUnifiedControlCommon()` 中，外环先计算 formation CoG 的位置/速度误差，得到 body/CoG frame 的 target acceleration。然后构造 6 维 acceleration-space target wrench：

- 前三维：CoG frame 的线加速度 target。
- roll/pitch：只放 PC 外环的 I-term，P+D 由 spinal 高频内环承担。
- yaw：若 `yaw_in_allocation_` 为 true，则放入 QP；否则走 spinal candidate yaw term。
- 加上 gravity feedforward，takeoff 时有 ramp。

对应代码在 `robots/beetle/src/control/beetle_controller.cpp:2455-2584`。最后调用：

```cpp
unified_controller_->computeUnifiedAllocation(target_wrench_acc,
                                              formation_wrench_cmd,
                                              yaw_pid_raw);
```

## 4. 当前 Beetle QP 的实现细节

### 4.1 formation allocation matrix

`BeetleUnifiedController::buildFormationAllocationMatrix()` 生成 integrated map：

```text
A = integrated_map_ ∈ R^{6 x (rotor_coef * total_rotors)}
```

它把所有 rotor 的矢量推力分量映射到 formation CoG 的 acceleration-space wrench：

```text
w_acc = A f
```

构造过程：

1. 对每个 rotor 建立 wrench map：force 部分是 identity，torque 部分是 `skew(rotor_pos) + dir * mf_rate * I`。
2. force 行除以 formation mass。
3. torque 行左乘 formation inertia inverse。
4. 乘以每个 rotor 的 mask，把局部可控 force component 映射到 3D force。

代码见 `robots/beetle/src/control/beetle_unified_controller.cpp:1046-1105`。formation inertia 通过 parallel-axis theorem 合成：`robots/beetle/src/control/beetle_unified_controller.cpp:1107-1129`。

当前默认 `gimbal_dof_=1`，所以 `rotor_coef_=2`，每个 rotor 的决策变量是 `[f_x, f_z]`。若有 `N` 个模块，每个模块 4 个 rotor，则变量维度是：

```text
n = 2 * 4N = 8N
```

### 4.1.1 两个 Beetle 合体时，总力/总力矩如何计算

假设两个 Beetle 合体后被看成一个 formation rigid body，总体受到的 wrench 不是简单把两个模块各自的 6D wrench 逐项相加就完事。正确原则是：

1. 所有力必须先表达在同一个坐标系下，再直接相加。
2. 所有力矩必须绕同一个参考点计算，通常选 formation CoG。
3. 如果某个模块的力矩是绕模块自身 CoG 表达的，那么换算到 formation CoG 时必须加上力臂项。

公式上，若第 `i` 个模块相对 formation CoG 的位置是 `r_i`，模块合力是 `F_i`，模块自身 CoG 处的力矩是 `tau_i`，则 formation CoG 处的总 wrench 是：

```text
F_total   = Σ_i F_i
tau_total = Σ_i (tau_i + r_i × F_i)
```

所以对于力，可以理解为“所有模块相加”；对于力矩，必须分开看：既有每个模块自身产生的力矩，也有该模块合力通过相对 formation CoG 的力臂产生的附加力矩。不能只把各模块的 `tau_i` 直接相加。

当前 Beetle unified allocation 实际上是在 rotor 层做这件事，而不是先粗略算模块层结果再相加。`buildFormationAllocationMatrix()` 对每个 rotor 使用：

```text
tau_rotor = (p_rotor - p_formation_cog) × F_rotor
          + rotor_direction * mf_rate * F_rotor
```

然后所有 rotor 的贡献线性叠加，最后 force 行除以 formation mass，torque 行乘 formation inertia inverse，得到 acceleration-space allocation matrix：`robots/beetle/src/control/beetle_unified_controller.cpp:1046-1105`。

代码中的两个诊断函数也体现了这个区别：

- `getRealizedModuleWrenchBody()` 计算某个模块自己的 rotor 贡献，torque 使用的是 `model.rotor_origins_from_cog`，也就是模块自身 CoG 周围的 wrench：`robots/beetle/src/control/beetle_unified_controller.cpp:839-868`。
- `getRealizedWrenchBody()` 不把模块 wrench 直接相加，而是用 integrated formation map `A * f` 先得到 formation acceleration-space wrench，再用 formation mass/inertia 转回 force/torque：`robots/beetle/src/control/beetle_unified_controller.cpp:909-946`。

因此，如果论文或实验 log 中要比较“两个 Beetle 合体后的总力/总力矩”，建议统一说清楚参考点和坐标系：

```text
frame: formation body / CoG frame
origin: formation CoG
force: sum of all rotor/module forces
torque: sum of rotor/module torques shifted to formation CoG
```

### 4.1.2 当前 external wrench estimation 是否按 formation 总 wrench 这样做

需要区分两个不同的 observer 路径。结论是：

- formation-level observer：是按 formation 总 wrench/formation CoG 的思路做的，但目前是 debug-only，不进入控制。
- per-module external wrench observer：不是按“两个模块 wrench 全部搬到 formation CoG 后相加”做的；它仍是每个模块自己的 momentum observer，用本模块质量/惯量和本模块 realized wrench 估计本模块 residual。

`FormationMomentumObserver` 的设计注释写得很明确：它使用 formation mass/inertia，observer input 是 allocation 结果的 realized wrench，而不是 PID command；当前默认语义仍是 diagnostic/model residual：`robots/beetle/include/beetle/control/formation_momentum_observer.h:4-18`。leader 在 unified loop 中调用它时，输入是：

```text
formation_mass
formation_inertia
formation CoG velocity
omega_body
realized_wrench = unified_controller_->getRealizedWrenchBody()
```

见 `robots/beetle/src/control/beetle_controller.cpp:2599-2631`。其中 `getRealizedWrenchBody()` 使用 integrated formation allocation map：

```text
w_acc = integrated_map_ * target_vectoring_f_
F_body = formation_mass * w_acc.head(3)
T_body = formation_inertia * w_acc.tail(3)
```

见 `robots/beetle/src/control/beetle_unified_controller.cpp:909-946`。由于 `integrated_map_` 在构造时已经用每个 rotor 相对 formation CoG 的位置计算 torque，所以这个 formation observer 的 realized wrench 输入符合“力相加、力矩统一到 formation CoG”的原则。

但是，`BeetleController::externalWrenchEstimate()` 这条 per-module observer 路径不同。unified mode 下它优先调用：

```cpp
unified_controller_->getRealizedModuleWrenchBody(my_id, target_wrench_cog)
```

见 `robots/beetle/src/control/beetle_controller.cpp:1904-1950`。而 `getRealizedModuleWrenchBody()` 的 torque 是用 `model.rotor_origins_from_cog` 计算的，也就是模块自身 CoG 周围的 rotor wrench：

```text
module_force  += F_rotor
module_torque += p_rotor_from_module_cog × F_rotor
               + rotor_drag_torque
```

见 `robots/beetle/src/control/beetle_unified_controller.cpp:839-868`。这条路径没有把每个模块的 local wrench 通过 `r_i × F_i` 搬到 formation CoG 后再相加。它输出的是 per-module residual estimate，然后进入 `est_wrench_list_`，供 `calcInteractionWrench()` 做 legacy residual/inter-wrench diagnostic 或 LF compensation：`robots/beetle/src/control/beetle_controller.cpp:1988-1996`。

因此当前代码状态应这样表述：

> Beetle 已经有一个 formation-level external wrench observer，其输入符合 formation CoG 总 wrench 的计算方式，但该 observer 仍是 debug-only。当前 residual/inter-wrench 逻辑使用的是每个模块自己的 external wrench estimate，并不是严格的 formation-level total wrench decomposition。

这也解释了为什么 unified QP 的主贡献最好不要建立在当前 `inter_wrench_list_` 的“严格内力”解释上；它更适合作为观测诊断，真正的 interface load constraint 仍应由 model-based `D f + d0` 来定义。

### 4.2 desired wrench 如何进入 QP

`computeUnifiedAllocation()` 中先从 PID 得到 `target_wrench_acc_cog`，再把外部任务 wrench 转换成 acceleration-space：

```text
w = target_wrench_acc_cog
if desired_ext_wrench exists:
    w_force  += desired_force / formation_mass
    w_torque += formation_inertia^{-1} desired_torque
```

代码见 `robots/beetle/src/control/beetle_unified_controller.cpp:338-347`。

注意这里 `desired_ext_wrench` 是 formation/body frame 的 6D wrench，而不是每个模块自己的补偿项。towing demo 在 unified mode 下也会先把 world force 转成 formation CoG wrench：`robots/beetle/script/demos/aerial_towing_demo/load_towing_formation.py:689-701`。

### 4.3 secondary reference

QP 的 reference `f_ref` 来自 `buildSecondaryAllocationReference()`：

1. 基础 reference 是每个模块按自身质量平均分给 4 个 rotor 的 hover thrust：

```text
f_ref_z(module, rotor) = module_mass * g / 4
```

2. 若 `internal_wrench_secondary_gain_ > 0`，则把 `wrench_comp_list_` 中的每模块 6D compensation 映射成该模块 allocation block 的 delta，并限制 delta 不超过 `0.25 * alloc_t_max_`。

代码见 `robots/beetle/src/control/beetle_unified_controller.cpp:451-521`。

但当前配置里：

```yaml
unified_internal_wrench_secondary_gain: 0.0
```

见 `robots/beetle/config/BeetleControl.yaml:37-45`。因此当前默认飞行配置下，legacy internal/inter-wrench 不会改变 `f_ref`，只是诊断。

### 4.4 QP objective function

当前 `solveFullVectorQP()` 的注释和代码一致。OSQP 求：

```text
min_f  0.5 f^T P f + q^T f
P = A^T A + λ I
q = -A^T w - λ f_ref
```

代码见 `robots/beetle/src/control/beetle_unified_controller.cpp:529-570`。

等价地，它在最优解意义上是：

```text
min_f  0.5 ||A f - w||^2 + 0.5 λ ||f - f_ref||^2
```

这里的 0.5 不改变最优解。含义是：

- 第一项：尽量让整机实现目标 acceleration-space wrench。
- 第二项：在冗余自由度里尽量接近 balanced hover allocation。

代码注释也明确说它“still a soft objective, not an internal-force controller yet”：`robots/beetle/src/control/beetle_unified_controller.cpp:535-537`。

这点很重要。当前 QP 的论文表述应说“已有 constrained whole-body allocation with secondary load-balancing reference”，不能说“已经实现了模块间内力 hard constraint”。

### 4.5 当前 constraints

当前 constraints 全部是 linear constraints，适合 OSQP：

每个 rotor 在 `rotor_coef == 2` 下有两个 gimbal angle 约束：

```text
 f_x + tan(theta_max) f_z >= 0
-f_x + tan(theta_max) f_z >= 0
```

在 `f_z >= 0` 的前提下，这等价于：

```text
|atan2(-f_x, f_z)| <= theta_max
```

然后是每个 component 的 bound：

```text
0 <= f_z <= alloc_t_max
-alloc_t_max <= f_x <= alloc_t_max
```

代码见 `robots/beetle/src/control/beetle_unified_controller.cpp:580-618`。

配置默认：

```yaml
use_constrained_alloc: true
alloc_lambda: 1.0e-4
alloc_t_max: 21.0
alloc_gimbal_limit_deg: 75.0
```

见 `robots/beetle/config/BeetleControl.yaml:52-58`。

### 4.6 当前 QP solver 行为

solver 设置：

- warm start: true
- max iteration: 500
- absolute tolerance: `1e-5`
- relative tolerance: `1e-4`
- polish: true
- 若变量/约束维度变化，则重建 solver；否则 update Hessian/gradient/bounds。

代码见 `robots/beetle/src/control/beetle_unified_controller.cpp:636-668`。

若带外部 wrench 的 QP fail，会尝试丢掉 external wrench 再解一次；仍失败则不更新命令：`robots/beetle/src/control/beetle_unified_controller.cpp:354-371`。

### 4.7 一个当前实现的限制：component bound 不是 thrust norm bound

代码中有一段诊断非常关键：

> QP constrains fx/fz components, while the spinal receives sqrt(fx^2+fz^2).

也就是说，QP 约束了 `|f_x| <= T_max` 和 `f_z <= T_max`，但 spinal 最终收到的是：

```text
T = sqrt(f_x^2 + f_z^2)
```

因此 `T` 可能超过 `alloc_t_max`。当前代码只做 warning/log，没有把 `sqrt(f_x^2 + f_z^2) <= T_max` 作为 hard constraint，因为这是二阶锥约束，不是 OSQP 的线性约束。见 `robots/beetle/src/control/beetle_unified_controller.cpp:680-749`。

可改进方向是用多边形近似 thrust cone/norm bound，仍保持 QP 线性约束形式。

## 5. 当前 leader-follower 模式在做什么

旧 LF 模式主要依赖 per-module observer residual 和 follower 侧 PID compensation。

`calcInteractionWrench()` 的流程：

1. 对每个 assembled module 计算：

```text
est_residual_i = est_wrench_i - est_wrench_task_i
```

代码注释说明 observer 输出可理解为 `c_i + d_i + b_i`，task prediction 试图建模 `c_i + d_i`，相减后留下 parasitic-only residual：`robots/beetle/src/control/beetle_controller.cpp:1409-1428`。

2. 求 residual 平均 `W_w`。

3. 递推得到 `inter_wrench_list_`，注释称其为 leader-to-i traversal 上的 parasitic joint-cut wrench：`robots/beetle/src/control/beetle_controller.cpp:1465-1477`。

4. 再从 leader 往左右累加得到 `wrench_comp_list_`：`robots/beetle/src/control/beetle_controller.cpp:1527-1554`。

5. 在 LF mode 中，follower 若开启 `pd_wrench_comp_mode_`，会把 `wrench_comp_list_[my_id]` 转成 acceleration/IComp/PersistentFF 注入 PID：`robots/beetle/src/control/beetle_controller.cpp:519-610`。

unified mode 中同一套 residual 计算默认只用于 log。只有 `unified_internal_wrench_secondary_gain_ > 0` 且 HOVER/control ready 时，才会把 `wrench_comp_list_` 作为 QP secondary reference 的 bias：`robots/beetle/src/control/beetle_controller.cpp:2404-2430`。

这正是老师说“内力/外力概念容易误导”的原因：LF 的 `internal_wrench` 命名是历史遗留，它是 observer residual 的递推产物；unified mode 的核心不应再围绕这个名字展开。

## 6. `develop/assemble_quadrotors` 分支给 Beetle 的启发

`develop/assemble_quadrotors` 不是 QP allocation。它更像 Beetle unified 的前身：固定双 quadrotor 合体后，用整机模型算 allocation matrix，再把整机命令切片发给 male/female。

### 6.1 FullyActuatedController 是 pseudoinverse allocation

`FullyActuatedController::controlCore()`：

1. 计算 `q_mat = robot_model_->calcWrenchMatrixOnCoG()`。
2. force 行除以 mass，torque 行乘 inertia inverse。
3. `q_mat_inv_ = pseudoinverse(q_mat_)`。
4. 用 `q_mat_inv_` 把 X/Y/Z target acceleration 分配到各 rotor。
5. 对 X/Y/Z term 做简单 saturation。
6. yaw 因 PC-spinal 带宽问题走 candidate yaw term。

见 `develop/assemble_quadrotors:aerial_robot_control/src/control/fully_actuated_controller.cpp:81-148`。

### 6.2 AssembleController 做整机命令切片

`AssembleController::sendCmd()` 在 assemble mode 中先取得 8 个 rotor 的 base thrust，再根据当前 airframe 是 male 还是 female 取前 4 或后 4 个发给本机 spinal。同时它也把整机 torque allocation inverse 切出本模块 4 行发送：`develop/assemble_quadrotors:robots/assemble_quadrotors/src/control/assemble_controller.cpp:99-149`。

Beetle unified 继承了这个思想，但做了三件更进一步的事：

- 从固定 2 模块扩展到动态 N 模块。
- 从 scalar thrust/pseudoinverse 扩展到 full-vector force components/QP。
- 从单 leader 整机模型转向每个模块都本地构造同一个 formation model 并切片发布。

### 6.3 optimal_thrust 不是 runtime QP

`robots/assemble_quadrotors/src/optimal_thrust.cpp` 使用 NLOPT 做设计/优化，用来搜索 rotor configuration/thrust design。它不是当前飞行时的 allocation QP。论文中可作为历史/设计工具提及，但不应混淆为 unified mode runtime QP。

## 7. `develop/spidar/pedipulate` 分支的 QP：最值得借鉴的部分

Spidar 的 QP 与老师提到的“多关节间的内力/载荷”更接近。它把 thrust force、foot contact force、joint torque 之间的静力关系显式写成线性 map。

### 7.1 关键线性映射

在 `WalkController::quadrupedCalcStaticBalance()` 里构造：

```text
A1_fr: rotor thrust vector force -> joint torque
A1_fe: foot contact force        -> joint torque
A2_fr: rotor thrust vector force -> body wrench
A2_fe: foot contact force        -> body wrench
b1: gravity/pseudo wrench induced joint torque
b2: gravity/pseudo wrench induced body wrench
```

然后组合：

```text
A1 = [A1_fr A1_fe]
A2 = [A2_fr A2_fe]
f_all = [fr; fe]
```

代码见 `develop/spidar/pedipulate:robots/spidar/src/control/walk_control.cpp:314-324`。

### 7.2 quadruped static balance QP

Spidar static balance QP 的 objective：

```text
min  f_all^T W1 f_all + (A1 f_all + b1)^T W2 (A1 f_all + b1)
```

constraints：

```text
A2 f_all = -b2
|A1 f_all + b1 + target_extra_joint_torque| <= torque_limit
```

代码见 `develop/spidar/pedipulate:robots/spidar/src/control/walk_control.cpp:326-373`。

这与 Beetle 当前 QP 的最大区别是：

- Spidar 把 body equilibrium 作为 equality constraint。
- Spidar 把 joint torque/internal load 作为 hard inequality constraint。
- Beetle 当前是把 wrench tracking 放在 soft objective 里，没有 joint/interface load hard constraint。

### 7.3 pedipulate single-arm/all-arm QP

Single arm：

- variables：单个 arm 两个 gimbal modules 的 6 维 thrust vector。
- objective：thrust effort + joint torque cost。
- constraint：只限制该 arm 的 joint torque。

见 `develop/spidar/pedipulate:robots/spidar/src/control/walk_control.cpp:653-734`。

All arm：

- variables：所有 rotor 的 3D thrust vector。
- constraints：body x/y force、roll/pitch/yaw torque、joint torque。
- joint torque constraint 形式仍是：

```text
|A1 f + b1| <= pedipulate_joint_static_torque_limit
```

见 `develop/spidar/pedipulate:robots/spidar/src/control/walk_control.cpp:736-869`。

### 7.4 GroundRobotModel 中的 QP 变体

`GroundRobotModel::updateRobotModelImpl()` 还保留了几种静力 QP 变体：

- contact force only，最小化 joint torque，zero total wrench equality。
- thrust + contact，最小化 thrust 和 joint torque，zero total wrench equality。
- thrust + contact，最小化 thrust，并加 joint torque inequality。
- thrust + contact，同时有 thrust/joint torque objective 和 joint torque inequality。

见 `develop/spidar/pedipulate:robots/spidar/src/model/ground_robot_model.cpp:187-416`。

这部分对 Beetle 的直接启发是：如果能建立“rotor force 到连接处 interface wrench/connector torque”的线性映射，就可以像 Spidar 一样把模块间载荷从模糊的 residual diagnostic 变成 QP 中的显式项。

## 8. Spidar 的 QP 如何用于 Beetle 研究

### 8.1 可以迁移的核心思想

Spidar 的核心不是某个具体参数，而是这个结构：

```text
joint_or_interface_load = A1 f + b1
body_or_formation_wrench = A2 f + b2
```

对 Beetle 来说，已有：

```text
formation_wrench_acc = A f
```

还缺：

```text
interface_load = D f + d0
```

其中 `D` 应描述每个 rotor force 对模块连接处/弱链接处 wrench 的贡献。若有 `D`，Beetle QP 就可以扩展为：

```text
min_f  0.5 ||A f - w||_Q^2
     + 0.5 ||f - f_ref||_R^2
     + 0.5 ||D f + d0 - tau_ref||_S^2

subject to
     gimbal constraints
     thrust/component constraints
     tau_min <= D f + d0 <= tau_max
```

这里第三项和最后的 inequality 就是“模块之间的内力不要太大”的严格数学版本。

### 8.2 Beetle 中 D 矩阵可能如何构造

有两种路线：

#### 路线 A：model-based interface load map

为每个模块间连接定义 connector frame/cut frame。对每个 cut，把 cut 一侧的模块集合看作 free body。该侧所有 rotor force 对 cut wrench 的贡献可以线性叠加：

```text
W_cut = Σ_{rotor in one side} [ I ; skew(p_rotor - p_cut) + rotor_drag_term ] F_rotor
      - W_external_side
      - W_inertia_side
```

在 quasi-static 或低频 allocation 层，可先忽略/简化 inertia_side，把它写成：

```text
W_cut ≈ D f + d0
```

然后加入 QP。

优点：可解释、可用于论文推导、能提前限制接口载荷。  
缺点：需要准确的 connector geometry、模块邻接关系、payload/外力分配、frame convention。

#### 路线 B：observer-assisted residual map

继续使用当前 `calcInteractionWrench()` 的 observer residual，把 `wrench_comp_list_` 或 `inter_wrench_list_` 作为 `tau_ref`/bias，影响 `f_ref` 或 `D f` 的目标。

当前代码已经有一个很初步的版本：当 `unified_internal_wrench_secondary_gain > 0` 时，把 `wrench_comp_list_` 映射到 secondary reference。见 `robots/beetle/src/control/beetle_controller.cpp:2404-2430` 和 `robots/beetle/src/control/beetle_unified_controller.cpp:469-520`。

优点：不用立刻建立完整 connector model。  
缺点：这是 feedback residual，受 observer drift、通信延迟、task prediction 误差影响；更像辅助阻尼，不适合作为论文里主要的“内力约束 QP”证明。

推荐路线是 A 为主，B 用于诊断/自适应 bias。

### 8.3 为什么这能实现“某些模块多出力，某些模块少出力”

仅靠 pseudoinverse，冗余分配通常由矩阵几何决定，不容易表达“任务场景下的偏好”。QP 可以通过 `R`、`f_ref`、`D` 做到这一点：

- 想让某个模块多承担 towing force：把该模块的 reference/bias 沿任务方向提高，或降低该模块 effort weight。
- 想保护某个模块/连接：提高该模块 effort weight，或对相关 interface load 加更严格的 bound。
- 想避免所有模块平均硬顶：加入 per-module thrust budget 或 module-wise effort penalty。
- 想使 hook module 不被 controller 反向抵消任务载荷：把 task wrench 作为 formation-level wrench 进入 `w`，而 residual compensation 只处理 parasitic residual。

这与当前 towing demo 的语义一致：任务 wrench 是整机目标，不应被 residual compensation 当作 disturbance 抵消。demo 已经通过 `est_wrench_task` 在 residual 前扣掉任务预测：`robots/beetle/script/demos/aerial_towing_demo/load_towing_formation.py:725-733`。

### 8.4 当前 Beetle QP 相比 Spidar QP 的不足

结合当前代码和 Spidar/pedipulate 分支，可以把不足归纳为以下几点：

1. Beetle 还没有显式的 interface load map。当前 Beetle 只有 `A f -> formation wrench`，没有 Spidar 中类似 `A1 f + b1 -> joint torque` 的结构。因此“模块间内力不要太大”目前还不能作为 hard constraint 表达。
2. Beetle 的 wrench tracking 是 soft objective。当前 objective 是 `0.5 ||A f - w||^2 + 0.5 λ ||f - f_ref||^2`。Spidar static balance 中很多场景把 body equilibrium 写成 equality constraint，例如 `A2 f = -b2`，再用 joint torque inequality 限制内部载荷。
3. Beetle 的 secondary objective 还比较弱。`f_ref` 默认只是按模块质量平均 hover thrust，`alloc_lambda=1e-4` 很小；而且 `unified_internal_wrench_secondary_gain=0` 时 observer residual 不影响分配。它还没有真正体现“towing 时 hook module 多出力/某些连接少受力”这类任务相关 load sharing。
4. Beetle 当前只有 component bound 和 gimbal angle bound，没有 thrust norm hard bound。代码已经诊断到 spinal 收到的是 `sqrt(f_x^2 + f_z^2)`，所以即使 `f_x` 和 `f_z` 分量各自没超，实际 thrust magnitude 仍可能超过 `alloc_t_max`。
5. Beetle 没有 rate bound/rate penalty。冗余分配在相邻控制周期之间可能在 nullspace 中缓慢漂移，只是目前靠小的 hover reference、PID limit、warm start 和任务场景约束住了。
6. Beetle 没有 slack variable。QP 失败时当前策略是“若外部 wrench 导致失败则丢掉外部 wrench 再试”，仍失败则不更新命令；这能保护飞行，但不利于论文中分析不可行性的来源。
7. Beetle 的 observer residual 是诊断优先。Spidar 的 joint torque 来自模型中的 Jacobian/static map，可直接进入 QP；Beetle 的 `inter_wrench_list_` 来自 momentum observer residual 递推，受 bias、task prediction、通信和 frame convention 影响，更适合作为 validation/bias，而不适合作为唯一的约束来源。

### 8.5 参考 Spidar 的下一步改进优先级

如果以论文贡献和实现风险综合考虑，推荐顺序如下：

1. 先补 `f_prev` rate penalty 和 `|f - f_prev| <= delta_f_max`。这不需要新物理模型，能直接减少 QP nullspace 漂移和命令突变。
2. 把 `alloc_lambda I` 改成 block diagonal 权重矩阵 `R`，支持 per-module/per-rotor effort weight。这样可以最早实现“不同场景下某些模块多出力、某些模块少出力”的 objective function 设计。
3. 为两模块/多模块连接定义 connector frame，建立 `D f + d0` 的 interface wrench map。这个是从 Spidar 迁移过来的核心：Beetle 中的 `D` 对应 Spidar 中的 `A1`。
4. 先把 `||D f + d0||_S^2` 作为 soft cost 加入 QP，不立即做 hard constraint。飞行 log 中比较 model-based `D f` 和 observer-based `inter_wrench_list_`，验证符号、frame 和数量级。
5. 验证后加入 hard/softened constraint：`tau_min <= D f + d0 <= tau_max`。若容易 infeasible，则加 interface slack，并高权重惩罚 slack。
6. 用线性多边形近似加入 thrust magnitude constraint，解决 component bound 与实际 thrust magnitude 不一致的问题。
7. 对 towing、valve rotation、hover 分别设计 `f_ref(scene)` 和权重矩阵。论文里重点展示：同一个 formation wrench 目标下，QP 如何根据 objective/constraints 改变模块出力分配和接口载荷。

### 8.6 为什么 Spidar 加这些功能，以及是否适合 Beetle 的“大出力”目标

从代码看，Spidar 加 QP 不是为了“单纯让机器人出更大的力”，而是为了在多关节/接触约束下求一个静力可行且不超过关节 torque limit 的分配。

Spidar 的变量包括 rotor thrust vector `fr` 和有足接触时的 contact force `fe`。它构造：

```text
A1 f + b1 -> link joint torque
A2 f + b2 -> baselink/body wrench
```

其中 `A1` 来自 thrust/contact point Jacobian 对关节侧的转置映射，`A2` 来自同一组 Jacobian 对 baselink wrench 的映射。`b1`/`b2` 则包含 gravity 和 pseudo baselink wrench 的影响：`develop/spidar/pedipulate:robots/spidar/src/control/walk_control.cpp:216-324`。

然后 QP 做两件事：

1. 保持静力/姿态平衡，例如 `A2 f = -b2` 或把 baselink x/y force、roll/pitch/yaw torque 限在小范围内。
2. 限制关节 torque，例如：

```text
|A1 f + b1 + target_extra_joint_torque| <= joint_static_torque_limit
```

见 `develop/spidar/pedipulate:robots/spidar/src/control/walk_control.cpp:326-373`。配置中也能看到这些 torque limit 的物理含义：servo max torque 是 6.5 Nm，但 static/pedipulate limit 更保守，分别是 3.0、1.5、1.0 Nm：`develop/spidar/pedipulate:robots/spidar/config/octo/control/WalkControlConfig.yaml:8-18`。

所以 Spidar 这些功能的真实目的更接近：

> 在满足 body balance/pedipulation 需求的同时，避免 link joint/servo 被 thrust/contact force 造成的静态 torque 压垮。

这对 Beetle 的“大出力”目标适合，但必须改写目标语义。Beetle 的目标不是最小化外力，而是在 towing/valve rotation 等任务中尽可能稳定地产生较大的 formation wrench，同时不让单个 rotor、单个模块或连接件超限。因此不能直接照搬 Spidar 的“最小 thrust effort + joint torque limit”作为主目标，否则如果 task wrench 只是 soft objective，QP 会倾向于少出力。

更适合 Beetle 的改法是：

1. 把“大出力”任务 wrench 放在 primary objective 或 hard/soft equality 中，权重必须高于 effort minimization。
2. 用 Spidar 的 `A1 f + b1` 思路建立 Beetle 的 `D f + d0` connector/interface load map。
3. 用 interface load constraint 保护连接件，而不是让它压低所有出力。
4. 用 per-module weight/reference 决定谁多出力，而不是让 QP 自动平均或自动最小化总 thrust。
5. effort/rate/norm constraint 只作为安全边界和 redundancy resolution，不应覆盖 task wrench priority。

也就是说，Spidar 的“关节载荷映射和约束思想”非常适合 Beetle；但 Spidar 的具体目标函数权重不能原样搬过来。Beetle 论文里的重点应是“大 wrench generation under actuator/interface constraints”，而不是“minimum effort static balance”。

## 9. 当前是否已有能量约束，能否防止慢发散

结论：当前 Beetle QP 没有能量约束。

代码中没有以下内容：

- actuator work 或 mechanical power bound；
- passivity inequality；
- energy tank；
- Lyapunov decrease constraint；
- `||f_k - f_{k-1}||` rate penalty；
- `f^T v <= P_max` 这类功率约束。

当前已有的安全/稳定性保护主要是工程性的：

- PID 各通道 `limit_sum/limit_p/limit_i/limit_d`。
- unified mode warmup，避免切换瞬间积分冲击。
- desired wrench timeout，外部 wrench 过期则清零。
- QP component/gimbal bounds。
- force landing 时 reset PID 和清目标。
- roll/pitch 不把 formation attitude command 写入 `target_rpy_`，避免 R/P I-term 无执行通道而 windup。
- observer 在 unified mode 默认只诊断，不直接注入 PID。

这些保护能减少发散风险，但不能作为“能量约束”或“passivity 保证”来写。

Spidar 分支也没有看到严格 energy tank/passivity constraint。它主要靠 static equilibrium equality、joint torque inequality、力/力矩权重和 torque limit 来保证静态载荷合理。

### 9.1 若要加入能量/慢发散抑制，可行设计

短期最容易落地的是 rate/effort regularization：

```text
min ... + 0.5 ||f - f_prev||_U^2
subject to |f - f_prev| <= delta_f_max
```

这不能证明 passivity，但能显著减少 allocation 在冗余空间里的慢漂移和命令抖动。它仍是标准 QP。

更严格的 passivity/energy tank 可以利用“当前速度已测量，QP 变量是 force”这一点。若 measured generalized velocity/twist `v` 已知，则 actuator power 对 `f` 是线性的：

```text
P_cmd(f) = v^T J_f f
```

因此可加线性约束：

```text
P_cmd(f) <= P_allowed
```

或 energy tank：

```text
P_cmd(f) * dt <= E_tank + P_diss dt
```

在单个控制周期内这仍可转成 OSQP 线性约束。但要把它写成论文级贡献，需要严格定义 frame、twist、wrench、energy tank 更新和离散时间 passivity 条件。当前代码还没有这些基础。

### 9.2 为什么没有能量约束，但系统看起来没有发散

目前系统没有明显发散，并不等于 QP 已经隐含满足能量/passivity 约束。更合理的解释是：当前实验场景、控制架构和工程保护让系统暂时工作在一个相对温和的区域。

主要原因包括：

1. QP 的 primary target 来自外环 PID，而 PID 本身有限幅。`target_wrench_acc` 不是任意增大的自由变量，X/Y/Z/R/P/Yaw 通道都有 limit。
2. `f_ref` 是 hover reference，`alloc_lambda` 虽小但会抑制一部分 nullspace 任意漂移。它不是能量约束，但能让冗余解偏向稳定 hover 分配。
3. unified mode 修掉了一个已知的慢发散源：formation roll/pitch command 不再直接写入 `target_rpy_` 导致 R/P I-term 无执行通道地增长。这个修正对“不发散”的贡献可能比 QP 本身更直接。
4. roll/pitch 采用 PC I-term + spinal P/D 的分工，PC 只处理慢模型误差，spinal 处理高频姿态稳定。这减少了 PC 侧把高频姿态误差错误地积分进 allocation 的风险。
5. 外部任务 wrench 有 timeout；wrench 过期后清零，避免旧任务力长期留在 allocation target 中。
6. 当前 `unified_internal_wrench_secondary_gain=0`，observer residual 不闭环注入 unified QP。这避免了 observer bias 或 residual 递推误差把系统慢慢推走。
7. QP 有 gimbal/component bound，虽然不是完整 thrust norm bound，但仍限制了单个变量的大小。

所以可以这样判断：

> 当前系统不发散，更多来自外环限幅、积分路径修正、任务 wrench timeout、默认关闭 observer residual feedback、以及 actuator/gimbal bound 的组合，而不是来自严格的能量约束证明。

这对论文表述很重要。可以说“current implementation includes practical boundedness safeguards”，但不能说“QP guarantees passivity/energy stability”。如果审稿或老师问为什么没有能量约束还能飞，回答应是：现阶段任务低速、外部 wrench 有限、积分器路径被控制住、observer feedback 默认不进闭环，所以没有激发慢漂移；但若要扩展到更强接触/更长时间任务，仍建议加入 rate/passivity/energy-aware allocation。

### 9.3 为什么这里没有直接建议先加能量约束

没有把能量约束列为第一优先级，原因不是它不重要，而是它目前不适合作为 Beetle QP 的第一步修改。

第一，当前 Spidar QP 本身也没有 energy tank/passivity constraint。Spidar 的核心保护是 static equilibrium、joint torque inequality 和 thrust/contact effort cost，而不是能量约束。因此如果我们说“参考 Spidar”，最直接应该先参考 `A1 f + b1` 这种 load map 和 torque/load limit，而不是凭空加一个 Spidar 没有的 energy layer。

第二，能量约束需要更完整的功率模型。若要写成：

```text
P_cmd(f) = v^T J_f f <= P_allowed
```

必须明确 generalized velocity、wrench frame、rotor/gimbal actuator power、formation/body twist，以及 energy tank 如何离散更新。当前 Beetle QP 的变量是 rotor force components，现有代码还没有把这些变量和可证明的 actuator/interaction power 统一起来。贸然加入容易得到“形式上像 passivity，物理上不闭合”的约束。

第三，Beetle 当前的论文目标是“大出力但受约束”。能量约束本质上会限制功率输入；如果没有设计好，它会直接削弱 towing/valve 任务中需要的持续大 wrench。对这个目标，第一优先级应是：

```text
task wrench priority
actuator/gimbal/thrust norm constraints
interface load constraints
rate smoothing
```

而不是先用 energy constraint 限制输出。

第四，当前没有明显发散的主要原因已经能从代码中解释：外环 PID limit、roll/pitch 积分路径修正、desired wrench timeout、observer residual 默认不进闭环、QP component/gimbal bound。这些不能替代能量证明，但足以解释为什么现阶段低速任务没有明显慢发散。

更合理的路线是：

1. 先加 rate penalty/rate bound，作为工程上最直接的 slow-drift 抑制。
2. 加 interface load map/constraint，解决连接件受力问题。
3. 在更强接触、更长时间或明显出现能量注入问题后，再设计 passivity/energy tank，并把它作为独立稳定性增强，而不是当前 QP load-sharing 的前置条件。

## 10. 建议的 Beetle QP 论文版设计

如果目标是把 QP 作为文章主要贡献，建议把当前 QP 扩展成“task-aware, interface-load-constrained allocation QP”。一个合理的 formulation：

```text
variables:
    f      all rotor vector force components
    s      optional wrench tracking slack

minimize:
    0.5 ||A f + s - w||_Q^2
  + 0.5 ||f - f_ref(scene)||_R^2
  + 0.5 ||D f + d0 - tau_ref||_S^2
  + 0.5 ||f - f_prev||_U^2
  + 0.5 ||s||_rho^2

subject to:
    gimbal linear constraints
    thrust/component/norm-polygon constraints
    tau_min <= D f + d0 <= tau_max
    |f - f_prev| <= delta_f_max
    optional module-wise thrust budget
```

各项意义：

- `A f + s ≈ w`：整机任务优先，slack 用于不可行时保持 solver 有解并量化误差。
- `f_ref(scene)`：任务相关 load sharing，例如 towing、valve rotation、hover、payload carrying。
- `D f + d0`：模块接口载荷或 joint-cut wrench。
- `tau_min/tau_max`：连接件/弱链接允许的最大力和力矩。
- `f_prev`：抑制冗余空间慢漂移和命令跳变。
- `Q/R/S/U/rho`：体现任务优先级和模块偏好。

### 10.1 与当前代码的最小差异实现路线

建议分阶段实现：

1. 在 `BeetleUnifiedController` 中保留当前 `solveFullVectorQP()` 框架，先加入 `f_prev` rate penalty 和 rate bound。这不需要新模型，风险最低。
2. 加入 per-module weight/reference：把 `alloc_lambda_ I` 改成 block diagonal `R`，每个模块可不同权重。
3. 定义 connector/interface geometry，新增 `buildInterfaceLoadMatrix()`，输出 `D` 和可选 `d0`。
4. 先把 `||D f||_S^2` 加入 objective，只做 soft interface load reduction。
5. log 对比 model-based `D f` 与 observer-based `inter_wrench_list_`，验证 frame 和符号。
6. 验证一致后，再加 hard bound `tau_min <= D f + d0 <= tau_max`。
7. 若 hard bound 经常导致 infeasible，引入 slack `epsilon_tau`，并用高权重惩罚。

### 10.2 需要新增/修改的主要代码位置

- `robots/beetle/include/beetle/control/beetle_unified_controller.h`
  - 新增 QP 权重、rate limit、interface load 参数和缓存。
  - 新增 `buildInterfaceLoadMatrix()` 声明。

- `robots/beetle/src/control/beetle_unified_controller.cpp`
  - 在 `rosParamInit` 读入新参数。
  - 在 `computeUnifiedAllocation()` 构造 `D`。
  - 在 `solveFullVectorQP()` 中扩展 Hessian/gradient/constraints。
  - 保存 `prev_vectoring_f_` 用于 rate penalty/bound。

- `robots/beetle/config/BeetleControl.yaml`
  - 新增 `alloc_module_weights`、`alloc_internal_weight`、`alloc_interface_force_limit`、`alloc_rate_limit` 等参数。

- `robots/beetle/src/control/beetle_controller.cpp`
  - 保留 observer residual log，但把它从“控制主路径”中降级为 validation/bias。
  - 对比 `D f` 和 `inter_wrench_list_` 的 diagnostic publisher。

- URDF/config
  - 增加 connector frame 或模块间连接点定义，否则 `D` 无法可靠构造。

## 11. 当前代码状态下可写入论文的表述边界

建议可以写：

- 已实现 dynamic formation model synthesis：根据 assembled IDs、模块质量/惯量/rotor origin 合成 formation mass、CoG、inertia。
- 已实现 full-vector constrained QP allocation：变量是所有 rotor 的 force components，约束包括 gimbal angle 和 force component bounds。
- 已实现 task wrench 的 unified path：formation-level desired wrench 直接进入 allocation target，避免 LF 中把任务力当 disturbance 反向抵消。
- 已实现 legacy inter-module residual diagnostic，并预留 secondary reference hook。
- 已实现 leader/follower symmetric local unified allocation，减少网络延迟导致的 follower 滞后。

不建议写成已经完成：

- “已实现模块间内力 hard constraint”。
- “QP 已有能量约束防止慢发散”。
- “observer internal wrench 已稳定闭环控制 unified mode”。默认配置下它是 diagnostic-only。
- “thrust magnitude 被严格限制在 `alloc_t_max`”。当前是 component bound，magnitude 可能超过。

## 12. 与老师研究方向的结合建议

老师在 Spidar/pedipulate 中做的 QP，最适合被抽象为：

```text
actuator/contact decision variables
-> body wrench/equilibrium map
-> joint/interface load map
-> effort and load objective
-> equilibrium and torque/load constraints
```

Beetle 可以用同一个思想，只是对象从“多关节地面/飞行混合机器人”变成“多模块合体空中机器人”：

```text
rotor vector force f
-> formation CoG wrench A f
-> connector/interface wrench D f
-> task-aware module load sharing
-> actuator/gimbal/interface/rate constraints
```

如果这部分实现并实验验证，论文贡献会更清楚：

1. 不是重新讨论内力/外力定义，而是提出一个统一的冗余分配 QP。
2. 通过 objective function 实现不同任务下的模块出力偏置。
3. 通过 interface load constraints 抑制模块连接处过大载荷。
4. 通过 slack/rate/passivity extension 处理不可行性和慢漂移。

## 13. 最后判断

当前 Beetle unified mode 的 QP 已经是一个很好的起点，但还处于“constrained allocation + balanced hover secondary objective”的阶段。若论文要把 QP 作为主要贡献，建议下一步不要继续围绕 legacy “内力/外力”名字展开，而是把 Spidar 那种 `A1 f + b1` 的 joint torque/load map 迁移成 Beetle 的 `D f + d0` interface load map。

一句话总结：

> 当前实现已经证明 Beetle unified mode 可以用 QP 做整机约束分配；下一步要成为论文核心贡献，需要把“场景相关出力分配”和“模块接口载荷约束”显式写进 QP，而不是只依赖 LF residual diagnostic 或 balanced hover reference。

## 14. 本次代码修改后的状态

本次已经把 QP 扩展接口接入 `BeetleUnifiedController`，但为保护实机飞行，新增影响控制量的参数默认关闭。也就是说：当前默认配置会发布连接处载荷诊断，但不会因为未经验证的连接载荷模型改变飞行分配；需要在 log 验证后再逐步打开 soft cost / hard bound。

### 14.1 新增 QP 项

当前 QP 变为：

```text
min_f  0.5 ||A f - w||^2
     + 0.5 ||f - f_ref||_R^2
     + 0.5 alloc_rate_weight ||f - f_prev||^2
     + 0.5 ||D f||_S^2

subject to:
     gimbal angle linear constraints
     component thrust bounds
     optional |f - f_prev| <= alloc_rate_limit
     optional component-wise |D f| <= interface_limit
```

其中：

- `R` 由 `alloc_lambda` 和 `alloc_module_weights` 组成。`alloc_module_weights` 是 module-id indexed multiplier；某模块权重越小，QP 越允许它偏离 hover reference，因此在大力矩/大外力任务中更容易“多出力”。
- `f_prev` 是上一帧成功 QP 解，用于平滑冗余空间中的分配漂移。
- `D f` 是 actuator-side interface cut-load proxy，按相邻模块之间的 cut 计算。

### 14.2 连接处载荷的具体定义与数值输出

对 `N` 个 assembled modules，代码会生成 `N-1` 个 cut。第 `k` 个 cut 位于第 `k` 和第 `k+1` 个模块 CoG 的中点。对 cut 右侧所有模块的 rotor actuator force 求和：

```text
W_cut,k = D_k f
        = [Fx, Fy, Fz, Tx, Ty, Tz]^T
```

其中每个 rotor 的 torque 用：

```text
tau_rotor_about_cut =
    (p_rotor - p_cut) × F_rotor
  + rotor_direction * mf_rate * F_rotor
```

这和 formation wrench matrix 的构造方式一致，只是参考点从 formation CoG 换成 cut midpoint。

代码会发布：

```text
/beetleX/unified_control/interface_load
```

消息类型是 `std_msgs/Float32MultiArray`。每个 cut 占 10 个数：

```text
[left_id, right_id,
 Fx, Fy, Fz,
 Tx, Ty, Tz,
 |F|, |T|]
```

同时每 1 秒 log 一次：

```text
[UnifiedCtrl InterfaceLoad] actuator_cut_proxy
max|F|=...N max|T|=...Nm
limits(F/T)=.../... weights(F/T)=.../...
cuts=[1-2:F=(...,...,...)|F|=... T=(...,...,...)|T|=...]
```

注意这个数值目前是 actuator-side proxy，不是完整物理 connector load。它还没有加入 gravity、payload、contact external wrench、inertial wrench。因此它适合用来比较 QP 分配导致的连接载荷变化，但不能直接当作连接件真实受力上限。比如单模块质量约 `2.82 kg` 时，单模块 hover thrust 约 `2.82 * 9.8 ≈ 27.6 N`；两模块水平合体 hover 时，actuator-side proxy 的 cut force 可能出现约一个右侧模块 lift 量级的数值，但真实连接件在准静态 hover 下还会被右侧模块重力抵消，不能把这个 proxy 直接解释成实际连接件拉力。

### 14.3 新参数

real/sim 两份配置均新增：

```yaml
alloc_rate_weight: 0.0
alloc_rate_limit: 0.0
alloc_module_weights: []
alloc_interface_force_weight: 0.0
alloc_interface_torque_weight: 0.0
alloc_interface_force_limit: 0.0
alloc_interface_torque_limit: 0.0
```

建议调参顺序：

1. 先只看 `/unified_control/interface_load` 和 log，不打开权重/限制。
2. 若 QP 解在冗余空间抖动，尝试 `alloc_rate_weight: 1.0e-4` 到 `1.0e-3`。
3. 若想让某个模块更愿意多出力，把对应 module id 的 `alloc_module_weights` 调低。例如 3 模块中想让 module 2 更愿意承担任务，可试 `[1.0, 0.5, 1.0]`。
4. 若 connection proxy 的方向和数量级经过 log 验证，再尝试小的 `alloc_interface_force_weight` / `alloc_interface_torque_weight`。
5. 最后再打开 `alloc_interface_force_limit` / `alloc_interface_torque_limit`。硬限制过紧会导致 QP infeasible。

### 14.4 是否已经实现“大力矩时自适应某些模块多出力、某些模块少出力”

结论是：已经具备 QP 机制，但默认配置下没有强行启用。

如果只保持默认参数，QP 仍主要是整机 wrench tracking + balanced hover reference；它会根据 actuator geometry 自然分配大力矩，但不会主动按连接载荷或模块偏好重排很多。

如果打开 `alloc_module_weights`，则可以人为指定“谁更愿意多出力”。这是场景相关的 load-sharing policy。

如果打开 `alloc_interface_force_weight` / `alloc_interface_torque_weight`，QP 会在保持 `A f ≈ w` 的同时自动寻找较小 `D f` 的解。由于 `D` 和当前 assembly geometry 每帧重建，这部分是在线自适应的：当大力矩任务让某个 cut 的 proxy torque 变大，QP 会尝试把一部分出力转移到对该 cut 更不敏感、或力臂更有利的模块/rotor 上。

不过这不是 magic。能否实现明显的“某些模块多出力、某些模块少出力”，取决于三个条件：

1. 系统确实有冗余自由度；如果目标 wrench 已经接近 actuator/gimbal limit，QP 没有足够自由度重分配。
2. interface proxy `D f` 的符号和数量级经过验证；否则 soft cost 可能惩罚了错误方向。
3. task wrench priority 必须高于 effort/interface cost；否则 QP 可能为了降低连接载荷而牺牲大力矩任务。

因此当前实现应被描述为：

> 支持 task-aware load sharing and interface-load-aware allocation；默认 conservative，需要通过参数逐步启用。大力矩操作下可以通过模块权重和 interface load cost 实现部分模块多出力、部分模块少出力，但真实连接件载荷约束还需要 gravity/external/inertia-compensated `D f + d0` 模型进一步完善。
