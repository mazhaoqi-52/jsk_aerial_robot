#!/usr/bin/env python3

"""
Fix valve_rotation_fang_single.py by removing duplicate RotateValveState class
and correcting indentation errors.
"""

import os

def fix_valve_rotation_file():
    file_path = "/home/ma/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/beetle/script/demos/valve_rotation_demo/valve_rotation_fang_single.py"
    
    # Read the file
    with open(file_path, 'r') as f:
        lines = f.readlines()
    
    # Find the problematic class definitions
    first_rotate_valve_line = None
    second_rotate_valve_line = None
    
    for i, line in enumerate(lines):
        if line.strip().startswith("class RotateValveState(SingleUAVStateBase):"):
            if first_rotate_valve_line is None:
                first_rotate_valve_line = i
            else:
                second_rotate_valve_line = i
                break
    
    print(f"First RotateValveState class at line {first_rotate_valve_line + 1}")
    print(f"Second RotateValveState class at line {second_rotate_valve_line + 1}")
    
    # Create new content: keep everything before first duplicate, 
    # skip the problematic section, keep everything from the second (correct) class onward
    new_lines = []
    
    # Keep everything before the first problematic class
    new_lines.extend(lines[:first_rotate_valve_line])
    
    # Skip everything until the second correct class and add the second class onward
    new_lines.extend(lines[second_rotate_valve_line:])
    
    # Write the fixed file
    with open(file_path, 'w') as f:
        f.writelines(new_lines)
    
    print(f"Fixed file: removed {second_rotate_valve_line - first_rotate_valve_line} problematic lines")
    print("File has been repaired!")

if __name__ == "__main__":
    fix_valve_rotation_file()
