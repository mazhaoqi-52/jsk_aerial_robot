# rosbag 回放与 RViz 使用指南

本地分支：`senior/rosbag-visualization`，基于学长仓库
`Li-Jinjie/jsk_aerial_robot_dev` 的 `develop/MPC_tilt_mt`，基点 `8f40e97a`。
独立 worktree：`/home/ma/ros/senior_rosbag_viz`。
原工作区 `/home/ma/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot` 保持原分支。

本文适用于已经配置好的本机 **Windows + WSL Ubuntu 22.04 + ROS One** 环境。
脚本中的 ROS 路径和实验文件路径针对本机；换电脑时需要相应调整。
三个 `.bag` 文件位于 `/home/ma/rosbag`，没有上传到 GitHub，需另行准备。

## 1. 先进入正确的终端

**ROS 命令需要在 WSL Ubuntu 中执行。** 如果提示符是 `PS C:\Users\29855>`，
说明仍在 Windows PowerShell。先执行：

```powershell
wsl -d Ubuntu-22.04
```

进入后提示符通常类似 `ma@电脑名称:~$`。
下文所有 `bash` 命令都在这里执行；需要多个终端时，每个终端都要先进入 WSL。
也可以直接从 Windows Terminal 打开 Ubuntu-22.04 标签页。

复制文件名时保留普通下划线 `_`，不要写成 `\_`。

## 2. 三个实验的对应关系

| 编号 | 实验 | bag 文件 | 时长 |
| --- | --- | --- | --- |
| 1 | Push 20N：推墙成功 | `2026-04-17-18-27-35_push_wall_20N_success.bag` | 约 90.80 秒 |
| 2 | Push-and-slide 5N：直接推失败，推并滑动成功 | `2026-07-30-20-10-15_push_fail_push_and_slide_success.bag` | 约 125.50 秒 |
| 3 | Push-and-slide 10N：推并滑动成功 | `2026-07-30-20-29-19_push_and_slide_10N_success.bag` | 约 156.00 秒 |

选择以下一种方式即可：

| 目的 | 使用方式 | ROS master |
| --- | --- | --- |
| 看机器人姿态、轨迹和原始 RViz 显示 | 第 3 节：一键回放 + RViz | `http://localhost:11321` |
| 单独播放 bag，供 PlotJuggler 等工具订阅 | 第 4 节：手动 `rosbag play` | `http://localhost:11311` |

同一个 ROS master 上一次只播放一个实验，避免同名话题和 `/clock` 混合。

## 3. 同时启动 rosbag 回放和 RViz

### 3.1 选择一个实验

进入代码目录（无需重新编译控制器）：

```bash
cd /home/ma/ros/senior_rosbag_viz
```

**实验 1：Push 20N**

```bash
./robots/beetle_omni/scripts/play_experiment.sh 1
```

**实验 2：Push-and-slide 5N**

```bash
./robots/beetle_omni/scripts/play_experiment.sh 2
```

**实验 3：Push-and-slide 10N**

```bash
./robots/beetle_omni/scripts/play_experiment.sh 3
```

以上三条命令选一条运行。脚本会加载 ROS 环境、在端口 `11321` 启动或连接 ROS master，
加载机器人模型、恢复记录的静态 TF，并启动 RViz 和 bag 播放器。无需另开 `roscore`。

### 3.2 开始、暂停和退出

默认暂停；在**启动终端**按空格开始/暂停，按 `s` 单步。
操作前先点击终端，使键盘焦点位于终端。不要把 RViz 时间面板的 Pause 当作 bag 播放器暂停。
暂停在开头时尚无动态 TF，RViz 暂时显示等待数据属于正常情况。
`Ctrl+C` 或关闭 RViz 会结束这次回放。一次只运行一个实例。
播放结束后窗口保持，重新播放请退出后重新启动。

启动后立即从头播放：

```bash
./robots/beetle_omni/scripts/play_experiment.sh 1 pause:=false
```

半速播放：

```bash
./robots/beetle_omni/scripts/play_experiment.sh 1 rate:=0.5 pause:=false
```

### 3.3 跳到接触片段

下面的起点用于快速观察接触过程，不是与实验视频校准后的对齐时间。每次选择一条运行：

```bash
./robots/beetle_omni/scripts/play_experiment.sh 1 start:=45 pause:=false
```

```bash
./robots/beetle_omni/scripts/play_experiment.sh 2 start:=75 pause:=false
```

```bash
./robots/beetle_omni/scripts/play_experiment.sh 3 start:=110 pause:=false
```

`start` 是相对 bag 开始的秒数。
也可以把 `1` 换成完整 bag 路径；当前模型及坐标系针对这三个 beetle1/brush 实验。
仅回放可视化相关话题，模型由记录的 TF 驱动。

### 3.4 保留学长原始 RViz 配置

默认直接加载 `config/rviz/nmpc_exp.rviz`，文件内容与学长分支完全一致。
显示项、消息话题、颜色、透明度、箭头比例、TF 显示开关、视角和面板布局均使用原配置。
不自动替换原来的 wrench 话题，也不添加数值文字或墙面显示项。
回放辅助节点默认只恢复静态 TF，额外 wrench 转换默认关闭。

切换为学长的另一个原始配置：

```bash
./robots/beetle_omni/scripts/play_experiment.sh 1 \
  rviz_config:=/home/ma/ros/senior_rosbag_viz/robots/beetle_omni/config/rviz/nmpc.rviz
```

原配置中的某些话题（例如 `/arm/mocap/pose`、`/hand/mocap/pose`）不在这三个 bag 中，
因此相应显示项可能提示没有消息；保留原配置，不删除这些显示项或制造替代数据。
原配置的 `/beetle1/ext_wrench_est/value` 与此前要求的
`/beetle1/dist_w_f_cog_tq/ext` 是不同的话题，默认按学长原来的选择显示。

## 4. 单独播放 rosbag，不启动 RViz

这种方式适合用 PlotJuggler 订阅消息。以下各终端都在同一个 WSL Ubuntu 环境中运行，
并统一使用 `http://localhost:11311`。普通 `rosbag play` 会发布 bag 中记录的全部话题。

### 4.1 终端 A：启动 ROS master

```bash
source /opt/ros/one/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
unset ROS_IP
roscore -p 11311
```

保持此终端运行。如果该地址已经运行着你要使用的 master，则无需重复启动。

### 4.2 终端 B：设置回放环境

```bash
source /opt/ros/one/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
unset ROS_IP
rosparam set /use_sim_time true
```

然后选择下面一个实验。

**实验 1：Push 20N**

```bash
rosbag play --clock --pause /home/ma/rosbag/2026-04-17-18-27-35_push_wall_20N_success.bag
```

**实验 2：Push-and-slide 5N**

```bash
rosbag play --clock --pause /home/ma/rosbag/2026-07-30-20-10-15_push_fail_push_and_slide_success.bag
```

**实验 3：Push-and-slide 10N**

```bash
rosbag play --clock --pause /home/ma/rosbag/2026-07-30-20-29-19_push_and_slide_10N_success.bag
```

`--clock` 发布回放时钟；`--pause` 让播放器启动后先暂停，方便先启动订阅工具。
在**终端 B** 按空格开始/暂停，暂停时按 `s` 单步，`Ctrl+C` 结束当前实验。
切换实验时退出当前播放器，再运行另一个文件的命令，终端 A 的 master 可以保持运行。

原生命令使用 `--start`、`--rate` 参数；它们与一键脚本的 `start:=`、`rate:=` 写法不同：

```bash
rosbag play --clock --pause --start=45 --rate=0.5 /home/ma/rosbag/2026-04-17-18-27-35_push_wall_20N_success.bag
```

仅运行 `rosbag play` 不会加载机器人模型参数。如需完整模型的 RViz 显示，请使用第 3 节。

### 4.3 终端 C：用 PlotJuggler 查看 interaction wrench

本机已安装 PlotJuggler 及 ROS 插件。启动命令：

```bash
source /opt/ros/one/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
unset ROS_IP
rosrun plotjuggler plotjuggler
```

1. 在 PlotJuggler 的 Streaming 区域选择 ROS 话题订阅插件并启动。
2. 选择 `/beetle1/dist_w_f_cog_tq/ext`。
3. 回到终端 B，按空格开始播放。
4. 把 `wrench/force/x`、`wrench/force/y`、`wrench/force/z` 拖到一张曲线图。
5. 把 `wrench/torque/x`、`wrench/torque/y`、`wrench/torque/z` 拖到另一张曲线图。

字段在界面中的具体前缀可能不同，可以搜索 `dist_w_f_cog_tq/ext` 定位。
力使用世界坐标系，单位 N；力矩使用 CoG 坐标系，单位 N·m。
显示这些原始分量的时间曲线时，无需启用 RViz 的可选 wrench 转换。

如果希望 PlotJuggler 订阅第 3 节脚本正在回放的数据，启动 PlotJuggler 前把
`ROS_MASTER_URI` 改为 `http://localhost:11321`，直接订阅现有回放即可。

## 5. 常见问题

| 现象 | 处理方法 |
| --- | --- |
| PowerShell 提示无法识别 `rosbag` | 先执行 `wsl -d Ubuntu-22.04` 进入 Ubuntu，再加载 ROS 环境。 |
| WSL 中提示 `rosbag: command not found` | 执行 `source /opt/ros/one/setup.bash`。每个新终端都要加载。 |
| 提示 bag 文件不存在 | 检查 `/home/ma/rosbag` 下的文件名；复制代码块中的原始路径，不加反斜杠转义下划线。 |
| 启动后机器人不动或曲线不更新 | 命令默认暂停；点击播放器所在终端并按空格。 |
| PlotJuggler 找不到回放话题 | 核对 master 地址：一键脚本用 `11321`，本文手动回放用 `11311`；重启连接到错误 master 的 PlotJuggler。 |
| RViz 部分显示项提示没有消息 | 原配置包含 bag 中不存在的话题，例如手部动捕话题；这些显示项保留原状。 |
| RViz 提示缺少 `world` 或 TF | 若尚未开始播放，先按空格；若仍有问题，检查 master 地址、回放是否在运行及 TF 数据。 |
| RViz 只显示网格，没有机器人模型 | 使用第 3 节脚本，它会加载 `/beetle1/robot_description` 和静态 TF。 |
| RViz 面板隐藏或窗口布局不适合本机 | 可在 Panels 菜单打开 Displays；关闭时如不想覆盖学长配置，选择不保存。 |

检查 ROS 命令和实验文件：

```bash
source /opt/ros/one/setup.bash
command -v rosbag
ls -lh /home/ma/rosbag/*.bag
rosbag info /home/ma/rosbag/2026-04-17-18-27-35_push_wall_20N_success.bag
```

检查正在回放的话题（先根据所选方式设置 `ROS_MASTER_URI`）：

```bash
rostopic list
rostopic echo -n 1 /beetle1/dist_w_f_cog_tq/ext
```

`rostopic echo` 需要回放正在进行才能收到新消息，暂停时可能一直等待。

## 6. 可选的额外交互力显示

此前制作的 `rosbag_interaction.rviz` 仅保留为可选配置，默认不加载。
它包含不同的显示样式、交互力转换和数值文字。如需使用，必须显式指定：

```bash
./robots/beetle_omni/scripts/play_experiment.sh 1 interaction_wrench:=true \
  rviz_config:=/home/ma/ros/senior_rosbag_viz/robots/beetle_omni/config/rviz/rosbag_interaction.rviz
```

### 坐标系转换说明

bag 中的话题是 `/beetle1/dist_w_f_cog_tq/ext`，类型 `geometry_msgs/WrenchStamped`。
`wrench` 是消息字段，不是话题路径。记录的 `header.frame_id` 为空。

发布实现见 `aerial_robot_control/include/aerial_robot_control/wrench_est/wrench_est_base.h`：
`force = getDistForceW()`，`torque = getDistTorqueCOG()`。
这条调试消息混用了世界系力与 CoG 系力矩，不能直接补 `world` 或 `cog` 后交给 RViz。

仅在 `interaction_wrench:=true` 时，`rosbag_wrench_viz.py` 查询**消息时间戳**的 `world <- beetle1/cog` TF，
执行 `F_cog = R_world_from_cog.T * F_world`，保持 CoG 力矩不变，发布到
`/beetle1/rosbag_viz/interaction_wrench`。箭头从 CoG 出发，方向按世界中的实际力显示。
这里不更换力矩参考点，所以不添加平移叉乘项。
缺少对应时刻 TF 时最多等待 2 秒墙钟时间，之后丢弃并警告，不用最新姿态代替。

节点合并 bag 的全部静态 TF，并以 latched `/tf_static` 发布，支持从中间开始播放。
若同一个静态 child frame 在 bag 中改变变换，则明确报错。
原始 bag 文件不作修改。

## 7. 验证与范围

三个 bag 均已检查：可读取、模型的所有连杆具有记录 TF。
已核验世界系到 CoG 的旋转方向（单位姿态与 90° yaw）。
此前回放检查输出位于 `/home/ma/rosbag/rviz_validation`，包含每个实验的运行日志、
转换后 wrench 采样与 RViz 截图；这些截图对应此前的可选增强配置，不代表当前默认原版画面。
检查的是接触片段，并非三个 bag 的逐帧审计。

按本次确认的范围，只做 RViz 可视化；未处理实验视频对齐或录制。
