#!/usr/bin/env python3
"""
Test the updated insertion depth calculations
"""

def test_insertion_depth():
    """Test the updated insertion depth calculations"""
    
    print("=== 测试修改后的插入深度计算 ===")
    
    # Parameters from the log
    valve_pos = [3.000, 0.0, 0.570]  # 阀门中心位置
    original_z_offset = 0.01  # 原始z_offset
    new_z_offset = -0.05  # 修改后z_offset
    
    original_pre_insertion = 0.015  # 原始pre_insertion_distance
    new_pre_insertion = 0.035  # 修改后pre_insertion_distance
    
    original_insertion_offset = 0.005  # 原始insertion_offset
    new_insertion_offset = 0.015  # 修改后insertion_offset
    
    print(f"阀门中心位置: {valve_pos}")
    print(f"阀门中心高度: {valve_pos[2]:.3f}m")
    
    print(f"\n=== 原始设置 ===")
    original_target_z = valve_pos[2] + original_z_offset
    original_total_offset = original_insertion_offset + original_pre_insertion
    print(f"  z_offset: {original_z_offset:.3f}m")
    print(f"  pre_insertion_distance: {original_pre_insertion:.3f}m")
    print(f"  insertion_offset: {original_insertion_offset:.3f}m")
    print(f"  目标高度: {original_target_z:.3f}m")
    print(f"  总插入偏移: {original_total_offset:.3f}m")
    
    print(f"\n=== 修改后设置 ===")
    new_target_z = valve_pos[2] + new_z_offset
    new_total_offset = new_insertion_offset + new_pre_insertion
    print(f"  z_offset: {new_z_offset:.3f}m")
    print(f"  pre_insertion_distance: {new_pre_insertion:.3f}m")
    print(f"  insertion_offset: {new_insertion_offset:.3f}m")
    print(f"  目标高度: {new_target_z:.3f}m")
    print(f"  总插入偏移: {new_total_offset:.3f}m")
    
    print(f"\n=== 对比分析 ===")
    z_change = new_target_z - original_target_z
    offset_change = new_total_offset - original_total_offset
    
    print(f"  目标高度变化: {z_change:.3f}m ({'下降' if z_change < 0 else '上升'})")
    print(f"  插入深度变化: {offset_change:.3f}m ({'增加' if offset_change > 0 else '减少'})")
    
    # 从日志中的实际结果分析
    actual_final_pos = [2.857, -0.098, 0.683]  # 实际最终位置
    actual_end_effector = [3.052, 0.052, 0.755]  # 实际末端执行器位置
    
    print(f"\n=== 实际结果分析 ===")
    print(f"  实际最终机体高度: {actual_final_pos[2]:.3f}m")
    print(f"  实际末端执行器高度: {actual_end_effector[2]:.3f}m")
    print(f"  末端执行器相对阀门高度: {actual_end_effector[2] - valve_pos[2]:.3f}m")
    
    print(f"\n=== 预期改进效果 ===")
    expected_body_height = new_target_z  # 期望的机体高度
    expected_end_effector_height = expected_body_height + 0.074  # 假设末端执行器偏移约7.4cm
    
    print(f"  预期机体高度: {expected_body_height:.3f}m")
    print(f"  预期末端执行器高度: {expected_end_effector_height:.3f}m")
    print(f"  预期末端执行器相对阀门高度: {expected_end_effector_height - valve_pos[2]:.3f}m")
    
    body_improvement = expected_body_height - actual_final_pos[2]
    end_eff_improvement = expected_end_effector_height - actual_end_effector[2]
    
    print(f"  机体高度改进: {body_improvement:.3f}m ({'下降' if body_improvement < 0 else '上升'})")
    print(f"  末端执行器高度改进: {end_eff_improvement:.3f}m ({'下降' if end_eff_improvement < 0 else '上升'})")
    
    if body_improvement < 0 and end_eff_improvement < 0:
        print(f"  ✓ 改进效果：插入深度将增加，末端执行器更深入阀门内部")
    else:
        print(f"  ⚠ 需要进一步调整参数")
    
    print(f"\n=== 总结 ===")
    print(f"  修改目标：解决'末端执行器打到阀门把手平面上，没有插入'的问题")
    print(f"  修改策略：降低目标高度，增加插入深度")
    print(f"  预期效果：末端执行器将更深入阀门内部，避免卡在表面")

if __name__ == "__main__":
    test_insertion_depth()
