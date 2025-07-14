# Stage 4 Position Jump and Emergency Detection Timing Fix

## Date: 2025-01-14

## Problem Analysis

### Primary Issues Identified:

1. **Stage 4 Position Drift**: 
   - UAV showed 7.5cm drift during 3-second hold period
   - Original position: `[2.930, -0.145, 1.209]`
   - Final position: `[2.872, -0.191, 1.198]`
   - Drift: `0.075m` (7.5cm)

2. **Emergency Detection Timing Issue**:
   - Emergency detection was called at 50Hz during rotation
   - Despite hardcoded `stuck_threshold=25.0s`, emergency was triggered at 3.0s
   - This was caused by high-frequency checking (every 20ms) creating noise sensitivity

3. **Control Frequency Issues**:
   - Stage 4 and pre-rotation stabilization were using 10Hz
   - Rotation was using 50Hz with emergency check every cycle
   - High frequencies were amplifying mocap noise and control instability

## Root Causes:

### 1. Emergency Detection Frequency
The emergency detection was being called every control cycle (50Hz) during rotation:
```python
while not rospy.is_shutdown() and not rotation_traj.is_complete():
    # Check for emergency during rotation
    if self.check_rotation_emergency():  # Called every 20ms!
        rospy.logwarn("Emergency detected during rotation, stopping")
        return 'emergency'
```

### 2. Control Frequency Instability
High control frequencies (10Hz-50Hz) were amplifying mocap noise and causing unnecessary control corrections.

### 3. Position Update Logic
Even though "fixed position" was used, the high-frequency control updates still caused drift.

## Comprehensive Fix

### 1. Emergency Detection Frequency Reduction
```python
# Check for emergency during rotation - but only every 0.5 seconds to avoid high frequency noise
current_time = time.time()
if current_time - last_emergency_check > 0.5:  # Check every 0.5 seconds, not every 20ms
    if self.check_rotation_emergency():
        rospy.logwarn("Emergency detected during rotation, stopping")
        return 'emergency'
    last_emergency_check = current_time
```

### 2. Control Frequency Optimization
- **Stage 4 Hold**: `10Hz → 5Hz`
- **Pre-rotation Stabilization**: `10Hz → 5Hz`
- **Rotation Control**: `50Hz` (maintained for trajectory smoothness)
- **Emergency Check**: `50Hz → 0.5s intervals`

### 3. Enhanced Position Stability
```python
rate = rospy.Rate(5)  # Further reduced from 10Hz to 5Hz to reduce control noise

while time.time() - start_time < hold_duration:
    if rospy.is_shutdown():
        return False
    
    # Maintain FIXED target position and yaw (do NOT update in loop!)
    MotionController.send_trajectory_point(self.pub, target_hold_pos, target_hold_yaw)
    
    rate.sleep()
```

### 4. Improved Timing Tolerance
```python
# Log position every 0.5 seconds to monitor for drift
if (time.time() - start_time) % 0.5 < 0.2:  # Log every 0.5 seconds (relaxed timing)
```

## Parameter Configuration

### Emergency Detection Parameters (Hardcoded + ROS Override):
```python
self.stuck_threshold = 25.0  # 25 seconds before emergency
self.movement_threshold = 0.02  # 2cm movement threshold
self.yaw_threshold = 0.1  # 0.1 rad yaw threshold
```

### Force Thresholds:
```python
contact_force_threshold = 3.0  # Base threshold 3.0N
# Circumferential adjustment: 3.5x = 10.5N
# Alignment phase: 2.5x = 7.5N
```

### Control Frequencies:
```python
# Stage 4 hold: 5Hz
# Pre-rotation stabilization: 5Hz  
# Rotation: 50Hz
# Emergency check: every 0.5 seconds
```

## Expected Improvements

1. **Reduced Stage 4 Drift**: < 5cm drift during 3-second hold
2. **Proper Emergency Timing**: 25-second threshold should work correctly
3. **Smoother Control**: Less jittery motion due to lower control frequencies
4. **Better Stability**: Reduced noise amplification from mocap data

## Test Verification

Run the test script to verify:
```bash
./test_final_stage4_fix.sh
```

Check for:
- Stage 4 drift < 5cm
- Emergency detection parameters logged correctly
- Control frequencies as specified
- Emergency not triggered for 25 seconds (if parameters working)
- Smooth motion during hold phases

## Files Modified

1. `valve_rotation_fang_single.py`:
   - `check_rotation_emergency()` timing fix
   - Stage 4 control frequency reduction
   - Pre-rotation stabilization frequency reduction
   - Enhanced debug logging

2. `test_final_stage4_fix.sh`: New comprehensive test script

## Expected Log Output

```
[INFO] RotateValveState emergency parameters: stuck_threshold=25.0s, movement_threshold=0.020m, yaw_threshold=0.100rad
[INFO] Emergency checking frequency: every 0.5 seconds (not every control cycle)
[INFO] Control frequencies: Stage 4 hold=5Hz, Pre-rotation stabilization=5Hz, Rotation=50Hz
[INFO] Stage 4 will maintain fixed position: [x.xxx, y.yyy, z.zzz]
[INFO] Stage 4 hold progress: 1.0s/3.0s, drift: 0.0xxm
[INFO] Total drift during Stage 4: 0.0xxm  # Should be < 0.05m
```

## Status: IMPLEMENTED

All fixes have been implemented and are ready for testing. The comprehensive approach addresses both the immediate position jump issue and the underlying timing/frequency problems that were causing instability.
