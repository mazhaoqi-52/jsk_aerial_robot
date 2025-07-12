#!/usr/bin/env python
"""
简单的轨迹可视化示例
直接从topic获取mocap数据，不需要复杂的create_from_mocap_data方法
"""

import rospy
import math
import threading
from trajectory import AlignToGraspTrajectory, ValveRotationTrajectory
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class SimpleTrajectoryVisualizer:
    """简单的轨迹可视化器，直接使用topic数据"""
    
    def __init__(self, module_id=1):
        self.module_id = module_id
        
        # 当前位置和姿态
        self.current_uav_pos = None
        self.current_uav_yaw = 0.0
        self.current_valve_pos = None
        self.current_valve_yaw = 0.0
        
        # 事件标志
        self.uav_received = threading.Event()
        self.valve_received = threading.Event()
        
        # 仿真标志
        self.is_simulation = rospy.get_param("~simulation", True)
        
        # 订阅者
        self.uav_sub = rospy.Subscriber(f"/beetle{module_id}/mocap/pose", PoseStamped, self.uav_callback, queue_size=1)
        
        if self.is_simulation:
            self.valve_sub = rospy.Subscriber("/valve/odom", Odometry, self.valve_sim_callback, queue_size=1)
        else:
            self.valve_sub = rospy.Subscriber("/valve/mocap/pose", PoseStamped, self.valve_callback, queue_size=1)
        
        # 发布者
        self.marker_pub = rospy.Publisher('/valve_trajectory_markers', MarkerArray, queue_size=10)
        self.path_pub = rospy.Publisher('/valve_trajectory_path', Marker, queue_size=10)
        
        rospy.sleep(0.5)  # 等待发布者就绪
        
    def uav_callback(self, msg):
        """UAV位置回调"""
        self.current_uav_pos = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        # 提取yaw角
        q = msg.pose.orientation
        _, _, self.current_uav_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.uav_received.set()
    
    def valve_sim_callback(self, msg):
        """阀门仿真位置回调"""
        self.current_valve_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        # 提取yaw角
        q = msg.pose.pose.orientation
        _, _, self.current_valve_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.valve_received.set()
    
    def valve_callback(self, msg):
        """阀门真实位置回调"""
        self.current_valve_pos = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        # 提取yaw角
        q = msg.pose.orientation
        _, _, self.current_valve_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.valve_received.set()
    
    def wait_for_data(self, timeout=5.0):
        """等待mocap数据"""
        rospy.loginfo("等待mocap数据...")
        uav_ok = self.uav_received.wait(timeout)
        valve_ok = self.valve_received.wait(timeout)
        
        if uav_ok and valve_ok:
            rospy.loginfo(f"收到数据 - UAV位置: {self.current_uav_pos}, yaw: {math.degrees(self.current_uav_yaw):.1f}°")
            rospy.loginfo(f"收到数据 - 阀门位置: {self.current_valve_pos}, yaw: {math.degrees(self.current_valve_yaw):.1f}°")
            return True
        else:
            rospy.logwarn("超时等待mocap数据")
            return False
    
    def publish_valve_model(self):
        """发布阀门模型"""
        if self.current_valve_pos is None:
            return
            
        marker_array = MarkerArray()
        valve_radius = 0.1225
        
        # 阀门中心
        center_marker = Marker()
        center_marker.header.frame_id = "map"
        center_marker.header.stamp = rospy.Time.now()
        center_marker.ns = "valve_model"
        center_marker.id = 0
        center_marker.type = Marker.SPHERE
        center_marker.action = Marker.ADD
        center_marker.pose.position.x = self.current_valve_pos[0]
        center_marker.pose.position.y = self.current_valve_pos[1]
        center_marker.pose.position.z = self.current_valve_pos[2]
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
        circle_marker.pose.position.x = self.current_valve_pos[0]
        circle_marker.pose.position.y = self.current_valve_pos[1]
        circle_marker.pose.position.z = self.current_valve_pos[2]
        circle_marker.pose.orientation.w = 1.0
        circle_marker.scale.x = valve_radius * 2
        circle_marker.scale.y = valve_radius * 2
        circle_marker.scale.z = 0.02
        circle_marker.color.r = 0.0
        circle_marker.color.g = 1.0
        circle_marker.color.b = 0.0
        circle_marker.color.a = 0.3
        marker_array.markers.append(circle_marker)
        
        # 三根横梁
        for i in range(3):
            beam_angle = self.current_valve_yaw + i * 2 * math.pi / 3
            beam_marker = Marker()
            beam_marker.header.frame_id = "map"
            beam_marker.header.stamp = rospy.Time.now()
            beam_marker.ns = "valve_model"
            beam_marker.id = 2 + i
            beam_marker.type = Marker.CYLINDER
            beam_marker.action = Marker.ADD
            
            # 横梁位置
            beam_x = self.current_valve_pos[0] + valve_radius * math.cos(beam_angle)
            beam_y = self.current_valve_pos[1] + valve_radius * math.sin(beam_angle)
            beam_marker.pose.position.x = beam_x
            beam_marker.pose.position.y = beam_y
            beam_marker.pose.position.z = self.current_valve_pos[2]
            
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
    
    def publish_current_uav_position(self):
        """发布当前UAV位置"""
        if self.current_uav_pos is None:
            return
            
        marker_array = MarkerArray()
        
        # UAV当前位置
        uav_marker = Marker()
        uav_marker.header.frame_id = "map"
        uav_marker.header.stamp = rospy.Time.now()
        uav_marker.ns = "current_uav"
        uav_marker.id = 0
        uav_marker.type = Marker.ARROW
        uav_marker.action = Marker.ADD
        
        uav_marker.pose.position.x = self.current_uav_pos[0]
        uav_marker.pose.position.y = self.current_uav_pos[1]
        uav_marker.pose.position.z = self.current_uav_pos[2]
        
        # 设置姿态
        q = quaternion_from_euler(0, 0, self.current_uav_yaw)
        uav_marker.pose.orientation.x = q[0]
        uav_marker.pose.orientation.y = q[1]
        uav_marker.pose.orientation.z = q[2]
        uav_marker.pose.orientation.w = q[3]
        
        uav_marker.scale.x = 0.3  # 箭头长度
        uav_marker.scale.y = 0.08  # 箭头宽度
        uav_marker.scale.z = 0.08  # 箭头高度
        
        uav_marker.color.r = 1.0
        uav_marker.color.g = 1.0
        uav_marker.color.b = 0.0
        uav_marker.color.a = 1.0
        
        marker_array.markers.append(uav_marker)
        self.marker_pub.publish(marker_array)
    
    def generate_and_visualize_trajectory(self):
        """生成并可视化轨迹"""
        if not self.wait_for_data():
            return False
            
        # 发布阀门模型
        self.publish_valve_model()
        
        # 发布当前UAV位置
        self.publish_current_uav_position()
        
        # 末端执行器偏移（从URDF文件）
        end_effector_offset_x = 0.246
        end_effector_offset_y = 0.0
        end_effector_offset_z = 0.0743823
        
        rospy.loginfo("=== 生成对准轨迹 ===")
        
        # 创建对准轨迹
        align_traj = AlignToGraspTrajectory(
            approach_duration=5.0,
            valve_center=self.current_valve_pos,
            valve_pose_yaw=self.current_valve_yaw,
            grasp_height=self.current_valve_pos[2],
            valve_radius=0.1225,
            valve_beam_width=0.0185,
            end_effector_offset_x=end_effector_offset_x,
            end_effector_offset_y=end_effector_offset_y,
            end_effector_offset_z=end_effector_offset_z,
            rotation_direction=1,
            insertion_offset=0.015
        )
        
        # 设置起始位置（使用当前UAV位置）
        align_traj.set_start_position(self.current_uav_pos)
        
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
        
        rospy.loginfo(f"生成了 {len(align_positions)} 个对准轨迹点")
        
        # 发布对准轨迹
        self.publish_trajectory_path(align_positions, "align_trajectory", (1.0, 0.5, 0.0, 1.0))
        
        rospy.loginfo("=== 生成旋转轨迹 ===")
        
        # 获取对准后的最终位置
        target_info = align_traj.get_target_info()
        final_body_pos = target_info['body_position']
        final_body_yaw = target_info['body_yaw']
        
        rospy.loginfo(f"对准后机体位置: {final_body_pos}")
        rospy.loginfo(f"对准后机体yaw: {math.degrees(final_body_yaw):.1f}°")
        
        # 简化的旋转轨迹创建（不使用复杂的create_from_mocap_data）
        # 直接计算旋转参数
        cos_yaw = math.cos(final_body_yaw)
        sin_yaw = math.sin(final_body_yaw)
        
        # 计算末端执行器位置
        end_effector_x = final_body_pos[0] + cos_yaw * end_effector_offset_x - sin_yaw * end_effector_offset_y
        end_effector_y = final_body_pos[1] + sin_yaw * end_effector_offset_x + cos_yaw * end_effector_offset_y
        end_effector_z = final_body_pos[2] + end_effector_offset_z
        
        # 计算旋转半径和起始角度
        dx = end_effector_x - self.current_valve_pos[0]
        dy = end_effector_y - self.current_valve_pos[1]
        radius = math.sqrt(dx**2 + dy**2)
        start_angle = math.atan2(dy, dx)
        
        rospy.loginfo(f"旋转半径: {radius:.3f}m, 起始角度: {math.degrees(start_angle):.1f}°")
        
        # 创建旋转轨迹（简化版本）
        rotation_traj = ValveRotationTrajectory(
            rotation_duration=10.0,
            valve_center=self.current_valve_pos,
            rotation_radius=radius,
            start_angle=start_angle,
            rotation_angle=math.pi/2,  # 旋转90度
            grasp_height=self.current_valve_pos[2],
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
        
        rospy.loginfo(f"生成了 {len(rotation_positions)} 个旋转轨迹点")
        
        # 发布旋转轨迹
        self.publish_trajectory_path(rotation_positions, "rotation_trajectory", (0.0, 1.0, 0.0, 1.0))
        
        rospy.loginfo("=== 轨迹生成完成 ===")
        return True
    
    def publish_trajectory_path(self, trajectory_points, namespace, color):
        """发布轨迹路径"""
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
        
        self.path_pub.publish(path_marker)

def main():
    """主函数"""
    rospy.init_node('simple_trajectory_visualizer')
    
    # 获取参数
    module_id = rospy.get_param("~module_id", 1)
    
    rospy.loginfo(f"启动简单轨迹可视化器 - UAV{module_id}")
    
    # 创建可视化器
    visualizer = SimpleTrajectoryVisualizer(module_id)
    
    # 等待一下让订阅者准备好
    rospy.sleep(1.0)
    
    # 生成并可视化轨迹
    if visualizer.generate_and_visualize_trajectory():
        rospy.loginfo("轨迹可视化成功!")
        rospy.loginfo("请在RViz中查看以下topic:")
        rospy.loginfo("- /valve_trajectory_markers (阀门模型和UAV位置)")
        rospy.loginfo("- /valve_trajectory_path (轨迹路径)")
        rospy.loginfo("建议RViz设置: Fixed Frame = map")
        
        # 保持节点运行
        rospy.loginfo("按Ctrl+C退出...")
        rospy.spin()
    else:
        rospy.logerr("轨迹可视化失败!")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("轨迹可视化被中断")
        pass
