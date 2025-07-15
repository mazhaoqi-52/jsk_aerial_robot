#!/usr/bin/env python3
"""
Visualize the "end-effector must be at valve radius" constraint
"""
import math
import matplotlib.pyplot as plt
import numpy as np

def visualize_valve_constraint():
    """Visualize the valve radius constraint"""
    
    # Valve parameters
    valve_center = (3.0, 0.0)
    valve_radius = 0.1225  # 12.25cm
    valve_yaw = 0.0  # beam0 aligned with X-axis
    
    # UAV parameters
    end_effector_length = 0.246  # 24.6cm
    safety_margin = 0.005  # 0.5cm
    
    # Create figure
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 7))
    
    # Plot 1: Valve structure and constraint
    ax1.set_title("Valve Structure and End-Effector Constraint", fontsize=14)
    ax1.set_aspect('equal')
    
    # Draw valve circle
    valve_circle = plt.Circle(valve_center, valve_radius, fill=False, color='blue', linewidth=2, label='Valve (radius=12.25cm)')
    ax1.add_patch(valve_circle)
    
    # Draw safety margin
    safety_circle = plt.Circle(valve_center, valve_radius + safety_margin, fill=False, color='red', linestyle='--', linewidth=1, label='Safety margin (+0.5cm)')
    ax1.add_patch(safety_circle)
    
    # Draw beams (120° apart)
    beam_angles = [0, 2*math.pi/3, 4*math.pi/3]  # 0°, 120°, 240°
    beam_labels = ['Beam0', 'Beam1', 'Beam2']
    
    for i, (angle, label) in enumerate(zip(beam_angles, beam_labels)):
        beam_x = valve_center[0] + valve_radius * math.cos(angle)
        beam_y = valve_center[1] + valve_radius * math.sin(angle)
        ax1.plot(beam_x, beam_y, 'ks', markersize=8)
        ax1.text(beam_x + 0.02, beam_y + 0.02, label, fontsize=10)
    
    # Draw gap regions
    gap_angles = [math.pi/3, math.pi, 5*math.pi/3]  # 60°, 180°, 300°
    gap_labels = ['Gap01', 'Gap12', 'Gap20']
    
    for i, (angle, label) in enumerate(zip(gap_angles, gap_labels)):
        gap_x = valve_center[0] + (valve_radius + 0.03) * math.cos(angle)
        gap_y = valve_center[1] + (valve_radius + 0.03) * math.sin(angle)
        ax1.plot(gap_x, gap_y, 'go', markersize=6)
        ax1.text(gap_x + 0.02, gap_y + 0.02, label, fontsize=9, color='green')
    
    # Example end-effector positions
    # Valid position (in gap, at valve radius)
    valid_angle = math.pi  # 180° (in gap12)
    valid_x = valve_center[0] + valve_radius * math.cos(valid_angle)
    valid_y = valve_center[1] + valve_radius * math.sin(valid_angle)
    ax1.plot(valid_x, valid_y, 'go', markersize=10, label='Valid end-effector')
    
    # Invalid position (too close to beam)
    invalid_angle = 2*math.pi/3  # 120° (too close to beam1)
    invalid_x = valve_center[0] + valve_radius * math.cos(invalid_angle)
    invalid_y = valve_center[1] + valve_radius * math.sin(invalid_angle)
    ax1.plot(invalid_x, invalid_y, 'ro', markersize=10, label='Invalid end-effector')
    
    # UAV body positions corresponding to valid end-effector
    body_yaw = invalid_angle + math.pi  # Point toward valve center
    body_x = valid_x - end_effector_length * math.cos(body_yaw)
    body_y = valid_y - end_effector_length * math.sin(body_yaw)
    ax1.plot(body_x, body_y, 'b^', markersize=10, label='UAV body')
    
    # Draw end-effector connection
    ax1.plot([body_x, valid_x], [body_y, valid_y], 'b-', linewidth=2, alpha=0.7)
    ax1.text((body_x + valid_x)/2, (body_y + valid_y)/2 + 0.03, '24.6cm', fontsize=9)
    
    ax1.plot(valve_center[0], valve_center[1], 'ko', markersize=8, label='Valve center')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    ax1.set_xlabel('X (meters)')
    ax1.set_ylabel('Y (meters)')
    ax1.set_xlim(2.5, 3.5)
    ax1.set_ylim(-0.5, 0.5)
    
    # Plot 2: Mathematical constraint explanation
    ax2.set_title("Mathematical Constraint Explanation", fontsize=14)
    ax2.text(0.1, 0.9, "Constraint 1: End-effector radius constraint", fontsize=12, weight='bold', transform=ax2.transAxes)
    ax2.text(0.1, 0.85, "valve_distance = √[(end_eff_x - valve_center_x)² + (end_eff_y - valve_center_y)²]", fontsize=10, transform=ax2.transAxes, family='monospace')
    ax2.text(0.1, 0.8, "constraint = safety_margin - |valve_distance - valve_radius|", fontsize=10, transform=ax2.transAxes, family='monospace')
    ax2.text(0.1, 0.75, "Must be ≥ 0 (satisfied when end-effector is at valve radius ± safety margin)", fontsize=10, transform=ax2.transAxes)
    
    ax2.text(0.1, 0.65, "Physical meaning:", fontsize=12, weight='bold', transform=ax2.transAxes)
    ax2.text(0.1, 0.6, "• End-effector must be exactly at valve perimeter", fontsize=10, transform=ax2.transAxes)
    ax2.text(0.1, 0.55, "• Distance from valve center = 12.25cm (±0.5cm safety)", fontsize=10, transform=ax2.transAxes)
    ax2.text(0.1, 0.5, "• This ensures proper engagement with valve structure", fontsize=10, transform=ax2.transAxes)
    
    ax2.text(0.1, 0.4, "Why this constraint?", fontsize=12, weight='bold', transform=ax2.transAxes)
    ax2.text(0.1, 0.35, "• Too close: End-effector collides with valve center", fontsize=10, transform=ax2.transAxes)
    ax2.text(0.1, 0.3, "• Too far: End-effector cannot reach valve beams", fontsize=10, transform=ax2.transAxes)
    ax2.text(0.1, 0.25, "• Just right: End-effector can insert between beams", fontsize=10, transform=ax2.transAxes)
    
    ax2.text(0.1, 0.15, "Example values:", fontsize=12, weight='bold', transform=ax2.transAxes)
    ax2.text(0.1, 0.1, f"• valve_radius = {valve_radius:.4f}m", fontsize=10, transform=ax2.transAxes, family='monospace')
    ax2.text(0.1, 0.05, f"• safety_margin = {safety_margin:.4f}m", fontsize=10, transform=ax2.transAxes, family='monospace')
    ax2.text(0.1, 0.0, f"• Acceptable range: {valve_radius-safety_margin:.4f}m to {valve_radius+safety_margin:.4f}m", fontsize=10, transform=ax2.transAxes, family='monospace')
    
    ax2.set_xlim(0, 1)
    ax2.set_ylim(0, 1)
    ax2.axis('off')
    
    plt.tight_layout()
    plt.savefig('/tmp/valve_constraint_visualization.png', dpi=150, bbox_inches='tight')
    # plt.show()  # Commented out to avoid GUI
    
    print("=== END-EFFECTOR RADIUS CONSTRAINT EXPLANATION ===")
    print(f"Valve radius: {valve_radius:.4f}m (12.25cm)")
    print(f"Safety margin: {safety_margin:.4f}m (0.5cm)")
    print(f"End-effector length: {end_effector_length:.4f}m (24.6cm)")
    print()
    print("The constraint ensures:")
    print("1. End-effector is exactly at valve perimeter")
    print("2. Proper distance for beam engagement")
    print("3. Avoids collision with valve center")
    print("4. Prevents over-extension beyond valve")
    print()
    print("Mathematical formulation:")
    print("  distance = √[(end_eff_x - valve_center_x)² + (end_eff_y - valve_center_y)²]")
    print("  constraint = safety_margin - |distance - valve_radius|")
    print("  constraint ≥ 0 for valid solutions")
    print()
    print("This is a CRITICAL constraint for successful valve operation!")

if __name__ == "__main__":
    try:
        visualize_valve_constraint()
    except ImportError:
        print("Matplotlib not available. Providing text explanation only.")
        print()
        print("=== END-EFFECTOR RADIUS CONSTRAINT EXPLANATION ===")
        print("Valve radius: 0.1225m (12.25cm)")
        print("Safety margin: 0.005m (0.5cm)")
        print("End-effector length: 0.246m (24.6cm)")
        print()
        print("The constraint ensures:")
        print("1. End-effector is exactly at valve perimeter")
        print("2. Proper distance for beam engagement")
        print("3. Avoids collision with valve center")
        print("4. Prevents over-extension beyond valve")
        print()
        print("Mathematical formulation:")
        print("  distance = √[(end_eff_x - valve_center_x)² + (end_eff_y - valve_center_y)²]")
        print("  constraint = safety_margin - |distance - valve_radius|")
        print("  constraint ≥ 0 for valid solutions")
        print()
        print("This is a CRITICAL constraint for successful valve operation!")
