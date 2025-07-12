#!/usr/bin/env python
"""
阀门操作轨迹生成器使用示例
演示如何使用更新后的类来计算机体位置和姿态
包含RViz可视化功能
"""

import rospy
import math
from trajectory import AlignToGraspTrajectory, ValveRotationTrajectory, PolynomialTrajectory
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import Header, ColorRGBA
from tf.transformations import quaternion_from_euler

class TrajectoryVisualizer:
    """轨迹可视化类，用于在RViz中显示轨迹"""
    
    def __init__(self):
        # 发布器
        self.marker_pub = rospy.Publisher('/valve_trajectory_markers', MarkerArray, queue_size=10)
        self.trajectory_pub = rospy.Publisher('/valve_trajectory_path', Marker, queue_size=10)
        
        # 等待发布器就绪
        rospy.sleep(0.5)
    
    def publish_valve_model(self, valve_center, valve_yaw, valve_radius=0.1225):
        """发布阀门模型到RViz"""
        marker_array = MarkerArray()
        
        # 阀门中心点
        center_marker = Marker()
        center_marker.header.frame_id = "map"
        center_marker.header.stamp = rospy.Time.now()
        center_marker.ns = "valve_model"
        center_marker.id = 0
        center_marker.type = Marker.SPHERE
        center_marker.action = Marker.ADD
        center_marker.pose.position.x = valve_center[0]
        center_marker.pose.position.y = valve_center[1]
        center_marker.pose.position.z = valve_center[2]
        center_marker.pose.orientation.w = 1.0
        center_marker.scale.x = 0.05
        center_marker.scale.y = 0.05
        center_marker.scale.z = 0.05
        center_marker.color.r = 1.0
        center_marker.color.g = 0.0
        center_marker.color.b = 0.0
        center_marker.color.a = 1.0
        marker_array.markers.append(center_marker)
        
        # 阀门圆圈
        circle_marker = Marker()
        circle_marker.header.frame_id = "map"
        circle_marker.header.stamp = rospy.Time.now()
        circle_marker.ns = "valve_model"
        circle_marker.id = 1
        circle_marker.type = Marker.CYLINDER
        circle_marker.action = Marker.ADD
        circle_marker.pose.position.x = valve_center[0]
        circle_marker.pose.position.y = valve_center[1]
        circle_marker.pose.position.z = valve_center[2]
        circle_marker.pose.orientation.w = 1.0
        circle_marker.scale.x = valve_radius * 2
        circle_marker.scale.y = valve_radius * 2
        circle_marker.scale.z = 0.02
        circle_marker.color.r = 0.0
        circle_marker.color.g = 1.0
        circle_marker.color.b = 0.0
        circle_marker.color.a = 0.3
        marker_array.markers.append(circle_marker)
        
        # 阀门横梁（3根）
        for i in range(3):
            beam_angle = valve_yaw + i * 2 * math.pi / 3
            beam_marker = Marker()
            beam_marker.header.frame_id = "map"
            beam_marker.header.stamp = rospy.Time.now()
            beam_marker.ns = "valve_model"
            beam_marker.id = 2 + i
            beam_marker.type = Marker.CYLINDER
            beam_marker.action = Marker.ADD
            
            # 横梁位置
            beam_x = valve_center[0] + valve_radius * math.cos(beam_angle)
            beam_y = valve_center[1] + valve_radius * math.sin(beam_angle)
            beam_marker.pose.position.x = beam_x
            beam_marker.pose.position.y = beam_y
            beam_marker.pose.position.z = valve_center[2]
            
            # 横梁方向
            q = quaternion_from_euler(0, 0, beam_angle + math.pi/2)
            beam_marker.pose.orientation.x = q[0]
            beam_marker.pose.orientation.y = q[1]
            beam_marker.pose.orientation.z = q[2]
            beam_marker.pose.orientation.w = q[3]
            
            beam_marker.scale.x = 0.02
            beam_marker.scale.y = 0.02
            beam_marker.scale.z = 0.15
            beam_marker.color.r = 0.0
            beam_marker.color.g = 0.0
            beam_marker.color.b = 1.0
            beam_marker.color.a = 0.8
            marker_array.markers.append(beam_marker)
        
        self.marker_pub.publish(marker_array)
    
    def publish_trajectory_path(self, trajectory_points, namespace="trajectory", color=(0.0, 1.0, 1.0, 1.0)):
        """发布轨迹路径到RViz"""
        path_marker = Marker()
        path_marker.header.frame_id = "map"
        path_marker.header.stamp = rospy.Time.now()
        path_marker.ns = namespace
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.pose.orientation.w = 1.0
        
        # 线条设置
        path_marker.scale.x = 0.02  # 线宽
        path_marker.color.r = color[0]
        path_marker.color.g = color[1]
        path_marker.color.b = color[2]
        path_marker.color.a = color[3]
        
        # 添加轨迹点
        for point in trajectory_points:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = point[2]
            path_marker.points.append(p)
        
        self.trajectory_pub.publish(path_marker)
    
    def publish_uav_poses(self, positions, yaws, namespace="uav_poses"):
        """发布UAV位置和姿态到RViz"""
        marker_array = MarkerArray()
        
        for i, (pos, yaw) in enumerate(zip(positions, yaws)):
            # UAV机体marker
            uav_marker = Marker()
            uav_marker.header.frame_id = "map"
            uav_marker.header.stamp = rospy.Time.now()
            uav_marker.ns = namespace
            uav_marker.id = i
            uav_marker.type = Marker.ARROW
            uav_marker.action = Marker.ADD
            
            uav_marker.pose.position.x = pos[0]
            uav_marker.pose.position.y = pos[1]
            uav_marker.pose.position.z = pos[2]
            
            # 设置姿态
            q = quaternion_from_euler(0, 0, yaw)
            uav_marker.pose.orientation.x = q[0]
            uav_marker.pose.orientation.y = q[1]
            uav_marker.pose.orientation.z = q[2]
            uav_marker.pose.orientation.w = q[3]
            
            uav_marker.scale.x = 0.2  # 箭头长度
            uav_marker.scale.y = 0.05  # 箭头宽度
            uav_marker.scale.z = 0.05  # 箭头高度
            
            # 颜色渐变
            ratio = float(i) / max(len(positions)-1, 1)
            uav_marker.color.r = 1.0 - ratio
            uav_marker.color.g = ratio
            uav_marker.color.b = 0.0
            uav_marker.color.a = 0.8
            
            marker_array.markers.append(uav_marker)
        
        self.marker_pub.publish(marker_array)

def example_usage():
    """演示轨迹生成器的使用方法并可视化"""
    
    # 初始化ROS节点
    rospy.init_node('trajectory_visualization_example')
    
    # 创建可视化器
    visualizer = TrajectoryVisualizer()
    
    # 定义末端执行器相对于机体重心的偏移（来自URDF文件）
    end_effector_offset_x = 0.246     # 末端执行器在机体前方24.6cm
    end_effector_offset_y = 0.0       # Y方向无偏移
    end_effector_offset_z = 0.0743823 # 末端执行器在机体重心上方7.4cm
    
    # 示例阀门位置和朝向
    valve_center = (2.0, 1.0, 0.8)  # 阀门中心位置
    valve_yaw = math.pi / 6  # 阀门朝向30度
    valve_radius = 0.1225
    
    print("=== 阀门轨迹可视化示例 ===")
    print(f"阀门中心: {valve_center}")
    print(f"阀门yaw: {math.degrees(valve_yaw):.1f}°")
    
    # 发布阀门模型
    visualizer.publish_valve_model(valve_center, valve_yaw, valve_radius)
    
    print("\n=== 阶段1: 对准抓取点 ===")
    
    # 创建对准轨迹
    align_traj = AlignToGraspTrajectory(
        approach_duration=5.0,
        valve_center=valve_center,
        valve_pose_yaw=valve_yaw,
        grasp_height=valve_center[2],
        valve_radius=valve_radius,
        valve_beam_width=0.0185,
        end_effector_offset_x=end_effector_offset_x,
        end_effector_offset_y=end_effector_offset_y,
        end_effector_offset_z=end_effector_offset_z,
        rotation_direction=1,
        insertion_offset=0.015
    )
    
    # 设置起始位置
    start_pos = (1.0, 0.5, 1.0)
    align_traj.set_start_position(start_pos)
    
    # 生成对准轨迹点
    align_positions = []
    align_yaws = []
    
    rate = rospy.Rate(20)  # 20Hz
    while not rospy.is_shutdown() and not align_traj.is_complete():
        result = align_traj.get_next_position_and_yaw()
        if result is None:
            break
        pos, yaw = result
        align_positions.append(pos)
        align_yaws.append(yaw)
        rate.sleep()
    
    print(f"生成了 {len(align_positions)} 个对准轨迹点")
    
    # 发布对准轨迹
    visualizer.publish_trajectory_path(align_positions, "align_trajectory", (1.0, 0.5, 0.0, 1.0))
    
    print("\n=== 阶段2: 阀门旋转 ===")
    
    # 获取对准后的最终位置
    target_info = align_traj.get_target_info()
    final_body_pos = target_info['body_position']
    final_body_yaw = target_info['body_yaw']
    
    print(f"对准后机体位置: {final_body_pos}")
    print(f"对准后机体yaw: {math.degrees(final_body_yaw):.1f}°")
    
    # 创建旋转轨迹（直接构造，无需复杂的mocap数据解析）
    # 计算当前夹持器位置
    cos_yaw = math.cos(final_body_yaw)
    sin_yaw = math.sin(final_body_yaw)
    
    global_offset_x = (cos_yaw * end_effector_offset_x - 
                      sin_yaw * end_effector_offset_y)
    global_offset_y = (sin_yaw * end_effector_offset_x + 
                      cos_yaw * end_effector_offset_y)
    
    current_end_effector_x = final_body_pos[0] + global_offset_x
    current_end_effector_y = final_body_pos[1] + global_offset_y
    current_end_effector_z = final_body_pos[2] + end_effector_offset_z
    
    # 计算旋转半径和起始角度
    dx = current_end_effector_x - valve_center[0]
    dy = current_end_effector_y - valve_center[1]
    radius = (dx**2 + dy**2)**0.5
    start_angle = math.atan2(dy, dx)
    
    rotation_traj = ValveRotationTrajectory(
        rotation_duration=10.0,
        valve_center=valve_center,
        rotation_radius=radius,
        start_angle=start_angle,
        rotation_angle=math.pi/2,  # 旋转90度
        grasp_height=valve_center[2],
        rotation_direction=1,
        end_effector_offset_x=end_effector_offset_x,
        end_effector_offset_y=end_effector_offset_y,
        end_effector_offset_z=end_effector_offset_z
    )
    
    # 开始旋转轨迹
    rotation_traj.start_rotation()
    
    # 生成旋转轨迹点
    rotation_positions = []
    rotation_yaws = []
    
    while not rospy.is_shutdown() and not rotation_traj.is_complete():
        result = rotation_traj.get_next_position_and_yaw()
        if result is None:
            break
        pos, yaw = result
        rotation_positions.append(pos)
        rotation_yaws.append(yaw)
        rate.sleep()
    
    print(f"生成了 {len(rotation_positions)} 个旋转轨迹点")
    
    # 发布旋转轨迹
    visualizer.publish_trajectory_path(rotation_positions, "rotation_trajectory", (0.0, 1.0, 0.0, 1.0))
    
    print("\n=== 可视化完整轨迹 ===")
    
    # 发布完整轨迹的UAV位置
    all_positions = align_positions + rotation_positions
    all_yaws = align_yaws + rotation_yaws
    
    # 选择一些关键点进行可视化（避免过多marker）
    step = max(1, len(all_positions) // 20)  # 最多显示20个UAV位置
    selected_positions = all_positions[::step]
    selected_yaws = all_yaws[::step]
    
    visualizer.publish_uav_poses(selected_positions, selected_yaws)
    
    print(f"可视化了 {len(selected_positions)} 个UAV关键位置")
    print("\n=== 可视化完成 ===")
    print("请在RViz中查看以下topic:")
    print("- /valve_trajectory_markers (阀门模型和UAV位置)")
    print("- /valve_trajectory_path (轨迹路径)")
    print("\n建议RViz配置:")
    print("- Fixed Frame: map")
    print("- 添加MarkerArray display，topic: /valve_trajectory_markers")
    print("- 添加Marker display，topic: /valve_trajectory_path")
    
    # 保持节点运行以便在RViz中查看
    print("\n按Ctrl+C退出...")
    rospy.spin()


if __name__ == '__main__':
    try:
        example_usage()  # 调用可视化版本
    except rospy.ROSInterruptException:
        print("可视化示例被中断")
        pass