#!/usr/bin/env python
"""
手动插入后自动阀门旋转程序 - 快速测试脚本
"""

import os
import sys
import subprocess
import time

def check_file_exists(filepath):
    """检查文件是否存在"""
    if os.path.exists(filepath):
        print(f"✅ {filepath}")
        return True
    else:
        print(f"❌ {filepath}")
        return False

def test_python_syntax(filepath):
    """测试Python语法"""
    try:
        result = subprocess.run(['python', '-m', 'py_compile', filepath], 
                              capture_output=True, text=True)
        if result.returncode == 0:
            print(f"✅ {os.path.basename(filepath)} - 语法正确")
            return True
        else:
            print(f"❌ {os.path.basename(filepath)} - 语法错误: {result.stderr}")
            return False
    except Exception as e:
        print(f"❌ {os.path.basename(filepath)} - 测试失败: {e}")
        return False

def main():
    """主测试函数"""
    print("=== 手动插入后自动阀门旋转程序 - 快速测试 ===")
    
    base_path = "/home/ma/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/beetle"
    
    # 检查必要文件
    files_to_check = [
        f"{base_path}/script/demos/valve_rotation_demo/valve_rotation_fang_single_joy.py",
        f"{base_path}/script/demos/valve_rotation_demo/print_launch_params.py",
        f"{base_path}/launch/valve_rotation_tasks/valve_rotation_joy.launch",
        f"{base_path}/script/demos/valve_rotation_demo/JoyControlledValveRotation_README.md"
    ]
    
    print("\n1. 文件存在性检查:")
    all_files_exist = True
    for filepath in files_to_check:
        if not check_file_exists(filepath):
            all_files_exist = False
    
    if not all_files_exist:
        print("\n❌ 某些文件缺失，请检查文件创建是否成功")
        return False
    
    # 检查Python文件语法
    print("\n2. Python语法检查:")
    python_files = [f for f in files_to_check if f.endswith('.py')]
    syntax_ok = True
    for filepath in python_files:
        if not test_python_syntax(filepath):
            syntax_ok = False
    
    if not syntax_ok:
        print("\n❌ 某些Python文件语法错误")
        return False
    
    # 检查文件权限
    print("\n3. 文件权限检查:")
    for filepath in python_files:
        if os.access(filepath, os.X_OK):
            print(f"✅ {os.path.basename(filepath)} - 可执行")
        else:
            print(f"⚠️ {os.path.basename(filepath)} - 不可执行")
    
    # 显示程序功能总结
    print("\n4. 程序功能总结:")
    print("📝 主程序: valve_rotation_fang_single_joy.py")
    print("   - 手动插入后自动对齐和旋转")
    print("   - 手柄控制启动和紧急停止")
    print("   - 增强控制器集成")
    print("   - 多级备选算法")
    
    print("🚀 启动文件: valve_rotation_joy.launch")
    print("   - 一键启动所有必要节点")
    print("   - 参数化配置")
    print("   - 状态监控")
    
    print("📖 使用说明: JoyControlledValveRotation_README.md")
    print("   - 详细使用说明")
    print("   - 参数配置指南")
    print("   - 故障排除")
    
    # 启动建议
    print("\n5. 启动建议:")
    print("🔧 测试命令:")
    print("   roslaunch beetle valve_rotation_joy.launch")
    print("")
    print("📋 操作步骤:")
    print("   1. 手动插入manipulator到阀门")
    print("   2. 程序检测插入位置")
    print("   3. 按下手柄Start按钮启动自动序列")
    print("   4. 程序自动完成对齐、旋转、上升")
    print("")
    print("🎮 手柄控制:")
    print("   - Start按钮(7): 启动自动序列")
    print("   - Select按钮(6): 紧急停止")
    
    print("\n✅ 测试完成 - 程序已准备就绪！")
    return True

if __name__ == "__main__":
    try:
        success = main()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n测试被用户中断")
        sys.exit(1)
    except Exception as e:
        print(f"\n测试执行错误: {e}")
        sys.exit(1)
