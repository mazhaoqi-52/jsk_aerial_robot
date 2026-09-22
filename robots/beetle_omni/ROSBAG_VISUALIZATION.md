# 本机 rosbag / RViz 回放

本地分支：`senior/rosbag-visualization`，基于学长仓库
`Li-Jinjie/jsk_aerial_robot_dev` 的 `develop/MPC_tilt_mt`，基点 `8f40e97a`。
独立 worktree：`/home/ma/ros/senior_rosbag_viz`。
原工作区 `/home/ma/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot` 保持原分支。

## 启动

在本机 WSL 终端执行（无需重新编译控制器）：

```bash
cd /home/ma/ros/senior_rosbag_viz
./robots/beetle_omni/scripts/play_experiment.sh 1
```

- `1`：push 20N，90.80 秒。
- `2`：push-and-slide 5N，125.50 秒。
- `3`：push-and-slide 10N，156.00 秒。

默认暂停；在**启动终端**按空格开始/暂停，按 `s` 单步。
暂停在开头时尚无动态 TF，RViz 暂时显示等待数据属于正常情况。
`Ctrl+C` 或关闭 RViz 会结束这次回放。一次只运行一个实例。
播放结束后窗口保持，重新播放请退出后重新启动。

直接观看接触片段：

```bash
./robots/beetle_omni/scripts/play_experiment.sh 1 start:=45 pause:=false
./robots/beetle_omni/scripts/play_experiment.sh 2 start:=75 pause:=false
./robots/beetle_omni/scripts/play_experiment.sh 3 start:=110 pause:=false
```

`start` 是相对 bag 开始的秒数；可用 `rate:=0.5` 半速播放。
也可以把 `1` 换成完整 bag 路径；当前模型及坐标系针对这三个 beetle1/brush 实验。
脚本使用本机 `/opt/ros/one`，独立 ROS master 端口 `11321`。
仅回放可视化相关话题，模型由记录的 TF 驱动。

## 默认使用学长原始配置

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

## 可选的额外交互力显示

此前制作的 `rosbag_interaction.rviz` 仅保留为可选配置，默认不加载。
它包含不同的显示样式、交互力转换和数值文字。如需使用，必须显式指定：

```bash
./robots/beetle_omni/scripts/play_experiment.sh 1 interaction_wrench:=true \
  rviz_config:=/home/ma/ros/senior_rosbag_viz/robots/beetle_omni/config/rviz/rosbag_interaction.rviz
```

## 为什么需要 wrench 转换节点

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

## 验证与范围

三个 bag 均已检查：可读取、模型的所有连杆具有记录 TF。
已核验世界系到 CoG 的旋转方向（单位姿态与 90° yaw）。
此前回放检查输出位于 `/home/ma/rosbag/rviz_validation`，包含每个实验的运行日志、
转换后 wrench 采样与 RViz 截图；这些截图对应此前的可选增强配置，不代表当前默认原版画面。
检查的是接触片段，并非三个 bag 的逐帧审计。

按本次确认的范围，只做 RViz 可视化；未处理实验视频对齐或录制。
