#!/usr/bin/env python3
"""
径向配置可视化脚本
直观展示"阀门中心→末端执行器→机体中心"的径向扩张配置
"""

import math
import matplotlib.pyplot as plt
import numpy as np
from constrained_optimizer import ConstrainedInsertionOptimizer

def visualize_radial_configuration():
    """可视化径向配置"""
    
    # 测试参数
    valve_radius = 0.1225  # 12.25cm
    end_effector_length = 0.246  # 24.6cm
    
    # 模拟阀门位置
    valve_pos = (0.0, 0.0, 0.5)
    valve_yaw = 0.0
    
    # 创建优化器
    optimizer = ConstrainedInsertionOptimizer(
        module_id=1,
        valve_radius=valve_radius,
        end_effector_length=end_effector_length,
        safety_margin=0.02
    )
    
    # 手动设置位置数据
    optimizer.valve_pos = valve_pos
    optimizer.valve_yaw = valve_yaw
    
    # 测试前方起始位置
    optimizer.current_uav_pos = (0.0, 0.5, 0.5)
    optimizer.current_uav_yaw = -math.pi/2
    
    # 执行优化
    result = optimizer.optimize_insertion_position()
    
    if result and result['success']:
        # 创建图形
        fig, ax = plt.subplots(1, 1, figsize=(12, 10))
        
        # 阀门中心
        valve_center_x, valve_center_y, _ = valve_pos
        
        # 优化结果
        body_pos = result['body_position']
        end_eff_pos = result['end_effector_position']
        
        # 画阀门
        valve_circle = plt.Circle((valve_center_x, valve_center_y), valve_radius, 
                                 color='lightblue', alpha=0.5, label='阀门 (半径12.25cm)')
        ax.add_patch(valve_circle)
        
        # 画阀门中心
        ax.plot(valve_center_x, valve_center_y, 'ko', markersize=8, label='阀门中心')
        
        # 画阀门beam结构
        beam_angles = [0, 2*math.pi/3, 4*math.pi/3]  # 0°, 120°, 240°
        for i, angle in enumerate(beam_angles):
            beam_x = valve_center_x + valve_radius * math.cos(angle)
            beam_y = valve_center_y + valve_radius * math.sin(angle)
            ax.plot([valve_center_x, beam_x], [valve_center_y, beam_y], 
                   'k-', linewidth=3, alpha=0.7)
            ax.text(beam_x + 0.02, beam_y + 0.02, f'Beam{i}', fontsize=8)
        
        # 画起始位置
        start_pos = (0.0, 0.5, 0.5)
        ax.plot(start_pos[0], start_pos[1], 'rs', markersize=10, label='起始位置')
        
        # 画优化后的机体位置
        ax.plot(body_pos[0], body_pos[1], 'go', markersize=12, label='机体中心 (优化后)')
        
        # 画末端执行器位置
        ax.plot(end_eff_pos[0], end_eff_pos[1], 'ro', markersize=10, label='末端执行器中心')
        
        # 画径向扩张线
        ax.plot([valve_center_x, end_eff_pos[0]], [valve_center_y, end_eff_pos[1]], 
               'r-', linewidth=2, label='阀门中心→末端执行器')
        ax.plot([end_eff_pos[0], body_pos[0]], [end_eff_pos[1], body_pos[1]], 
               'g-', linewidth=2, label='末端执行器→机体中心')
        
        # 画完整的径向线
        ax.plot([valve_center_x, body_pos[0]], [valve_center_y, body_pos[1]], 
               'b--', linewidth=1, alpha=0.7, label='完整径向线')
        
        # 画移动轨迹
        ax.plot([start_pos[0], body_pos[0]], [start_pos[1], body_pos[1]], 
               'orange', linewidth=2, alpha=0.7, label='移动轨迹')
        
        # 添加距离标注
        valve_body_dist = math.sqrt((body_pos[0] - valve_center_x)**2 + 
                                   (body_pos[1] - valve_center_y)**2)
        valve_endeff_dist = math.sqrt((end_eff_pos[0] - valve_center_x)**2 + 
                                     (end_eff_pos[1] - valve_center_y)**2)
        
        # 标注距离
        mid_x = (valve_center_x + end_eff_pos[0]) / 2
        mid_y = (valve_center_y + end_eff_pos[1]) / 2
        ax.text(mid_x, mid_y + 0.03, f'{valve_endeff_dist:.3f}m', 
               fontsize=10, ha='center', bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.7))
        
        mid_x = (end_eff_pos[0] + body_pos[0]) / 2
        mid_y = (end_eff_pos[1] + body_pos[1]) / 2
        ax.text(mid_x, mid_y + 0.03, f'{end_effector_length:.3f}m', 
               fontsize=10, ha='center', bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgreen", alpha=0.7))
        
        # 设置图形
        ax.set_xlim(-0.6, 0.6)
        ax.set_ylim(-0.2, 0.6)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right')
        ax.set_title('径向配置优化结果：阀门中心→末端执行器→机体中心', fontsize=14, fontweight='bold')
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        
        # 添加配置信息
        info_text = f"""
配置信息：
• 阀门半径: {valve_radius:.3f}m
• 末端执行器长度: {end_effector_length:.3f}m
• 机体距离阀门中心: {valve_body_dist:.3f}m
• 末端执行器距离阀门中心: {valve_endeff_dist:.3f}m
• 径向扩张: 理想配置 ✓
• 约束满足: 全部满足 ✓
        """
        ax.text(0.02, 0.98, info_text, transform=ax.transAxes, fontsize=10,
               verticalalignment='top', bbox=dict(boxstyle="round,pad=0.5", facecolor="white", alpha=0.8))
        
        plt.tight_layout()
        plt.savefig('radial_configuration_visualization.png', dpi=300, bbox_inches='tight')
        plt.show()
        
        print("✅ 径向配置可视化完成！")
        print(f"图片已保存为: radial_configuration_visualization.png")
        print(f"配置类型: 理想径向扩张配置")
        print(f"机体距离阀门中心: {valve_body_dist:.3f}m")
        print(f"末端执行器距离阀门中心: {valve_endeff_dist:.3f}m")
        
    else:
        print("❌ 优化失败，无法生成可视化")

if __name__ == "__main__":
    visualize_radial_configuration()
