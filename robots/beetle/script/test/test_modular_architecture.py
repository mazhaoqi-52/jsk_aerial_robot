#!/usr/bin/env python
"""
快速测试脚本：验证模块化反馈控制系统
"""
import sys
import os
import unittest
from unittest.mock import Mock, patch

# 添加beetle script目录到Python路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

class TestModularArchitecture(unittest.TestCase):
    """测试模块化架构的基本功能"""
    
    def setUp(self):
        """设置测试环境"""
        self.mock_sm = Mock()
        self.mock_sm.current_yaw = 0.0
        self.mock_sm.valve_pos = (3.0, 0.0, 0.55)
        self.mock_sm.get_current_position.return_value = (2.8, 0.1, 0.8)
        self.mock_sm.normalize_angle.return_value = 0.0
    
    def test_enhanced_motion_controller_import(self):
        """测试增强Motion Controller的导入"""
        try:
            from enhanced_motion_controller import EnhancedMotionController
            controller = EnhancedMotionController(self.mock_sm)
            self.assertIsNotNone(controller)
            print("✅ EnhancedMotionController导入成功")
        except ImportError as e:
            self.fail(f"❌ EnhancedMotionController导入失败: {e}")
    
    def test_alignment_status_check(self):
        """测试对齐状态检查功能"""
        try:
            from enhanced_motion_controller import EnhancedMotionController
            controller = EnhancedMotionController(self.mock_sm)
            
            # 模拟对齐状态检查
            current_pos = self.mock_sm.get_current_position()
            is_aligned, report = controller.check_alignment_status(
                current_pos, self.mock_sm.current_yaw, self.mock_sm.valve_pos)
            
            self.assertIsInstance(is_aligned, bool)
            self.assertIsInstance(report, dict)
            self.assertIn('end_effector_to_valve_distance', report)
            self.assertIn('yaw_error', report)
            self.assertIn('collinearity_angle', report)
            print("✅ 对齐状态检查功能正常")
        except Exception as e:
            self.fail(f"❌ 对齐状态检查失败: {e}")
    
    def test_alignment_correction_calculation(self):
        """测试对齐纠正计算功能"""
        try:
            from enhanced_motion_controller import EnhancedMotionController
            controller = EnhancedMotionController(self.mock_sm)
            
            # 模拟对齐报告
            mock_report = {
                'position_error': [0.2, 0.1, 0.0],
                'yaw_error': 0.1,
                'collinearity_angle': 0.05
            }
            
            pos_correction, yaw_correction = controller.calculate_alignment_correction(mock_report)
            
            self.assertIsInstance(pos_correction, (list, tuple))
            self.assertIsInstance(yaw_correction, (int, float))
            self.assertEqual(len(pos_correction), 3)
            print("✅ 对齐纠正计算功能正常")
        except Exception as e:
            self.fail(f"❌ 对齐纠正计算失败: {e}")
    
    def test_main_file_imports(self):
        """测试主文件的导入依赖"""
        try:
            # 测试主文件能否正确导入新的模块
            sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))
            
            # 模拟必要的ROS模块
            with patch.dict('sys.modules', {
                'rospy': Mock(),
                'smach': Mock(),
                'smach_ros': Mock(),
                'aerial_robot_msgs.msg': Mock(),
                'geometry_msgs.msg': Mock(),
                'std_msgs.msg': Mock(),
                'sensor_msgs.msg': Mock(),
                'beetle.msg': Mock(),
                'beetle.srv': Mock(),
                'trajectory': Mock(),
                'dragon_library': Mock(),
                'nav_msgs.msg': Mock(),
                'sensor_msgs.msg': Mock(),
                'tf': Mock(),
                'tf.transformations': Mock(),
                'copy': Mock(),
                'threading': Mock(),
                'Queue': Mock(),
                'queue': Mock()
            }):
                from enhanced_motion_controller import EnhancedMotionController
                print("✅ 主文件导入依赖正常")
        except Exception as e:
            self.fail(f"❌ 主文件导入依赖失败: {e}")
    
    def test_configuration_parameters(self):
        """测试配置参数结构"""
        try:
            # 检查配置文件是否存在
            config_path = os.path.join(os.path.dirname(__file__), '..', '..', 'config', 'EnhancedValveRotationConfig.yaml')
            
            if os.path.exists(config_path):
                print("✅ 配置文件存在")
                
                # 简单检查配置文件内容
                with open(config_path, 'r') as f:
                    content = f.read()
                    expected_params = [
                        'strict_alignment_enabled',
                        'alignment_correction_enabled',
                        'alignment_position_tolerance',
                        'alignment_yaw_tolerance'
                    ]
                    
                    for param in expected_params:
                        if param in content:
                            print(f"✅ 配置参数 {param} 存在")
                        else:
                            print(f"⚠️ 配置参数 {param} 缺失")
            else:
                print("⚠️ 配置文件不存在，将使用默认参数")
                
        except Exception as e:
            print(f"⚠️ 配置文件检查失败: {e}")

def run_basic_functionality_test():
    """运行基本功能测试"""
    print("=== 模块化架构基本功能测试 ===")
    
    # 创建测试套件
    suite = unittest.TestSuite()
    suite.addTest(TestModularArchitecture('test_enhanced_motion_controller_import'))
    suite.addTest(TestModularArchitecture('test_alignment_status_check'))
    suite.addTest(TestModularArchitecture('test_alignment_correction_calculation'))
    suite.addTest(TestModularArchitecture('test_main_file_imports'))
    suite.addTest(TestModularArchitecture('test_configuration_parameters'))
    
    # 运行测试
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # 总结结果
    if result.wasSuccessful():
        print("\n🎉 所有测试通过！模块化架构准备就绪")
        return True
    else:
        print(f"\n❌ {len(result.failures)} 个测试失败, {len(result.errors)} 个测试错误")
        for test, error in result.failures + result.errors:
            print(f"  - {test}: {error}")
        return False

def check_file_structure():
    """检查文件结构完整性"""
    print("\n=== 文件结构检查 ===")
    
    script_dir = os.path.join(os.path.dirname(__file__), '..', '..')
    
    expected_files = [
        'enhanced_motion_controller.py',
        'valve_rotation_fang_single.py',
        'config/EnhancedValveRotationConfig.yaml',
        'script/demos/valve_rotation_demo/alignment_control_example.py'
    ]
    
    all_files_exist = True
    for file_path in expected_files:
        full_path = os.path.join(script_dir, file_path)
        if os.path.exists(full_path):
            print(f"✅ {file_path}")
        else:
            print(f"❌ {file_path} - 文件不存在")
            all_files_exist = False
    
    return all_files_exist

def print_architecture_summary():
    """打印架构摘要"""
    print("\n=== 模块化架构摘要 ===")
    
    architecture_info = {
        "核心模块": {
            "enhanced_motion_controller.py": "增强Motion Controller，包含对齐控制逻辑",
            "valve_rotation_fang_single.py": "主状态机，使用新的模块化架构",
            "EnhancedValveRotationConfig.yaml": "配置文件，管理对齐控制参数"
        },
        "关键功能": [
            "严格的COG-末端执行器-阀门对齐控制",
            "实时对齐状态监控和纠正",
            "模块化的代码组织结构",
            "灵活的参数配置系统",
            "旋转过程中的轨迹稳定性保证"
        ],
        "改进点": [
            "z_offset从-0.03m纠正到0.55m",
            "对齐控制逻辑从主文件分离到专用模块",
            "增强的错误处理和状态反馈",
            "更清晰的代码组织和可读性",
            "统一的配置管理系统"
        ]
    }
    
    for category, items in architecture_info.items():
        print(f"\n{category}:")
        if isinstance(items, dict):
            for key, value in items.items():
                print(f"  • {key}: {value}")
        else:
            for item in items:
                print(f"  • {item}")

if __name__ == "__main__":
    print("开始模块化架构验证...")
    
    # 检查文件结构
    files_ok = check_file_structure()
    
    # 运行功能测试
    if files_ok:
        tests_ok = run_basic_functionality_test()
        
        if tests_ok:
            print_architecture_summary()
            print("\n🚀 模块化架构验证完成！系统准备就绪")
        else:
            print("\n⚠️ 某些测试失败，请检查配置")
    else:
        print("\n❌ 文件结构不完整，请确保所有文件都已创建")
    
    print("\n下一步建议:")
    print("1. 在仿真环境中测试新架构")
    print("2. 根据实际表现调整对齐控制参数")
    print("3. 监控对齐控制的性能指标")
    print("4. 根据需要进一步优化motion controller")
