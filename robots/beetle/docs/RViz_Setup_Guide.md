# RViz轨迹可视化设置指南

## 三种RViz配置方案

### 方案1：使用默认RViz + 手动配置（推荐）
**适用场景**：与仿真环境无缝集成，避免多窗口混乱
**优点**：与其他系统集成，使用熟悉的RViz界面，无需额外配置

```bash
# 1. 启动仿真环境（已包含默认RViz）
roslaunch beetle bringup.launch headless:=false simulation:=true real_machine:=false

# 2. 启动阀门位置服务
roslaunch beetle valve_pos.launch simulation:=true real_machine:=false

# 3. 启动轨迹预览（无专用RViz）
roslaunch beetle valve_rotation_manual.launch preview_only:=true simulation:=true

# 4. 按照下方步骤手动添加Display（仅首次需要）
```

### 方案2：使用专用RViz配置
**适用场景**：需要详细轨迹分析，希望一键启动
**优点**：无需手动配置，所有轨迹可视化都已预配置
**注意**：会产生多个RViz窗口

```bash
# 启动时添加 rviz:=true 参数
roslaunch beetle valve_rotation_manual.launch preview_only:=true simulation:=true rviz:=true
```

### 方案3：无RViz纯命令行
**适用场景**：服务器环境，仅需控制功能
**优点**：节省资源，适合自动化脚本

```bash
# 启动时不添加rviz参数
roslaunch beetle valve_rotation_manual.launch preview_only:=true simulation:=true
```

## 快速参考表

| 需求 | 命令 | 说明 |
|------|------|------|
| 最简单的轨迹预览 | `roslaunch beetle valve_rotation_manual.launch preview_only:=true simulation:=true` | 默认RViz，手动添加Display |
| 专用RViz配置 | `roslaunch beetle valve_rotation_manual.launch preview_only:=true simulation:=true rviz:=true` | 专用RViz，一键启动 |
| 纯命令行操作 | `roslaunch beetle valve_rotation_manual.launch preview_only:=true simulation:=true` | 无可视化界面 |
| 完整仿真流程 | 参考方案1的完整步骤 | 推荐用法 |

## 方案1详细配置步骤（推荐用法）

### 完整启动流程：

1. **启动仿真环境**
   ```bash
   roslaunch beetle bringup.launch headless:=false simulation:=true real_machine:=false
   ```

2. **启动阀门位置服务**
   ```bash
   roslaunch beetle valve_pos.launch simulation:=true real_machine:=false
   ```

3. **启动轨迹预览**
   ```bash
   roslaunch beetle valve_rotation_manual.launch preview_only:=true simulation:=true
   ```

### 手动添加Display（仅首次需要）：

完成上述启动流程后，您需要在默认RViz中手动添加以下Display来查看轨迹可视化：

### 1. 添加轨迹预览 (MarkerArray)
1. 在RViz左下角点击 **"Add"** 按钮
2. 在弹出的"Create visualization"对话框中：
   - 展开 **"By display type"** 标签
   - 滚动找到并选择 **"MarkerArray"**
   - 点击 **"OK"**
3. 在新创建的MarkerArray Display中：
   - 展开该Display项（点击左侧的三角形）
   - 找到 **"Topic"** 字段
   - 点击Topic字段右侧的下拉箭头，选择 `/beetle1/trajectory_preview`
   - 或者直接在Topic字段中输入：`/beetle1/trajectory_preview`
   - 确保 **"Enabled"** 复选框已勾选（显示✓）

### 2. 添加阀门标记 (Marker)
1. 再次点击 **"Add"** 按钮
2. 在弹出的对话框中：
   - 选择 **"Marker"**
   - 点击 **"OK"**
3. 在新创建的Marker Display中：
   - **如果左侧三角形无法展开**，先设置Topic：
     - 直接在Topic字段中输入：`/beetle1/valve_preview`
     - 或点击Topic字段右侧的下拉箭头选择
   - **然后展开该Display项**（点击左侧的三角形）
   - 确保 **"Enabled"** 复选框已勾选（显示✓）

### 3. 添加UAV起始位置 (Marker)
1. 再次点击 **"Add"** 按钮
2. 选择 **"Marker"** 并点击 **"OK"**
3. 在新创建的Marker Display中：
   - **如果左侧三角形无法展开**，先设置Topic：
     - 直接在Topic字段中输入：`/beetle1/uav_start_preview`
     - 或点击Topic字段右侧的下拉箭头选择
   - **然后展开该Display项**（点击左侧的三角形）
   - 确保 **"Enabled"** 复选框已勾选（显示✓）

### 4. 检查坐标系设置
1. 在RViz顶部的"Global Options"中，确保 **"Fixed Frame"** 设置为 `world`
2. 如果没有 `world` 坐标系，可以尝试设置为 `map` 或 `odom`

### 5. 调整视角以查看轨迹
1. 使用鼠标左键拖动调整视角
2. 使用滚轮缩放（轨迹可能很大需要缩小查看）
3. 使用鼠标右键拖动平移视角
4. 如果看不到轨迹，可以点击Views面板中的"Zero"按钮重置视角

### 6. 保存配置（可选）
1. 完成配置后，点击 **File** → **Save Config As**
2. 保存RViz配置文件，下次启动时可以直接加载

## 操作提示：
- **Topic选择**：如果在下拉菜单中找不到话题，请确认轨迹预览程序正在运行
- **Display展开**：如果左侧三角形无法展开，请先设置Topic，等待1-2秒后再尝试展开
- **Display顺序**：建议按照上述顺序添加Display，这样层次更清晰
- **界面刷新**：如果遇到界面响应问题，可以在RViz菜单中选择View → Reload
- **颜色调整**：如果轨迹颜色不清晰，可以在每个Display中调整Color属性
- **大小调整**：如果轨迹太小或太大，可以调整MarkerArray和Marker的Scale属性

## 预期可视化效果

添加完成后，您应该能在默认RViz中看到：
- **蓝色轨迹线和箭头**：无人机的运动轨迹（从MarkerArray显示）
- **红色圆柱体**：阀门位置标记（从Marker显示）
- **黄色球体**：UAV起始位置标记（从Marker显示）
- **无人机3D模型**：已存在的Robot Model显示
- **坐标系轴线**：TF显示的坐标变换关系

## 常见问题解答

### Q: 为什么Marker Display的左侧三角形无法展开？
A: **这通常是因为Display还没有完全初始化或需要先设置Topic**。

**解决方法**：
1. **先设置Topic再展开**：
   - 直接在Topic字段中输入话题名称（如：`/beetle1/valve_preview`）
   - 等待1-2秒让Display加载完成
   - 然后再点击左侧三角形展开

2. **如果仍然无法展开**：
   - 在RViz菜单中选择 **View** → **Reload**
   - 或者关闭并重新打开RViz

3. **检查Display状态**：
   - 确保Display名称左侧有✓标记（表示启用）
   - 如果没有，请勾选Enabled复选框

### Q: 为什么点击Topic字段右侧的下拉箭头没有反应？
A: **这通常表示轨迹预览程序没有运行**。RViz的Topic下拉菜单只显示当前正在发布的话题。

**解决步骤**：
1. **检查轨迹预览程序是否运行**：
   ```bash
   rosnode list | grep -i valve
   ```
   如果没有输出，说明轨迹预览程序没有启动

2. **启动轨迹预览程序**：
   ```bash
   roslaunch beetle valve_rotation_manual.launch preview_only:=true simulation:=true
   ```

3. **验证话题是否在发布**：
   ```bash
   rostopic list | grep beetle1 | grep preview
   ```
   应该能看到：
   - `/beetle1/trajectory_preview`
   - `/beetle1/valve_preview`
   - `/beetle1/uav_start_preview`

4. **重新尝试添加Display**：话题发布后，下拉菜单应该就能正常工作了

### Q: 为什么看不到轨迹？
A: 请检查以下几点：
1. **轨迹预览程序是否正在运行**：`rosnode list | grep -i valve`
2. **话题是否在发布数据**：`rostopic echo /beetle1/trajectory_preview -n 1`
3. **Fixed Frame设置是否正确**：应该设置为`world`
4. **Display是否正确启用**：确保所有Display都有✓标记

### Q: 轨迹太大或太小看不清？
A: 
1. **缩放调整**：使用鼠标滚轮放大/缩小
2. **Scale调整**：在MarkerArray Display中调整Scale参数
3. **视角重置**：点击Views面板中的"Zero"按钮

### Q: 颜色不够明显？
A: 在每个Display中展开Color属性进行调整：
- 轨迹：建议蓝色或绿色
- 阀门：建议红色
- UAV起始位置：建议黄色或橙色

### Q: 添加Display后还是看不到？
A: 
1. 确认话题名称拼写正确
2. 检查Enabled复选框是否勾选
3. 尝试重新启动轨迹预览程序

## 故障排除

### 如果看不到轨迹：
1. 检查Fixed Frame是否设置正确
2. 确认Topic名称是否正确
3. 检查轨迹话题是否在发布：
   ```bash
   rostopic echo /beetle1/trajectory_preview
   ```

### 如果轨迹太小或太大：
1. 调整MarkerArray的Scale
2. 调整Marker的Scale

### 如果颜色不明显：
1. 调整MarkerArray的Color
2. 调整Marker的Color
