# Joy-Controlled Valve Rotation (Simplified)

This is a simplified version of the joy-controlled valve rotation program that directly imports and reuses the existing state machine components from `valve_rotation_fang_single.py`.

## Architecture

Instead of duplicating all the feedback control code, this version:

1. **Imports existing states**: Directly imports `InitializeStartPositionState`, `MoveToValveState`, `DescendAndContactState`, and `RotateValveState` from the original program
2. **Reuses state machine**: Uses the existing SMACH state machine architecture without modification
3. **Minimal wrapper**: Provides only the joy control logic as a lightweight wrapper around existing components

## Key Features

- **Manual insertion phase**: Press Square button to start initialization and move to valve
- **Automatic sequence**: Press Triangle button to start descent, contact, and rotation
- **Joy control toggle**: Press X button to enable/disable joy control
- **Thread-based execution**: Each phase runs in separate threads to avoid blocking
- **Status publishing**: Publishes completion status to `/valve_rotation_status` topic

## Usage

### Launch the program:
```bash
roslaunch beetle valve_rotation_joy.launch
```

### Control mapping:
- **X button (button 0)**: Toggle joy control on/off
- **Square button (button 2)**: Start manual insertion phase
- **Triangle button (button 3)**: Start automatic sequence

### Workflow:
1. Press X to enable joy control
2. Press Square to start manual insertion (initialization + move to valve)
3. Manually insert manipulator into valve (this step is done by operator)
4. Press Triangle to start automatic sequence (descent, contact, rotation)

## File Structure

- `valve_rotation_fang_single_joy.py`: Simplified joy control wrapper (268 lines)
- `valve_rotation_fang_single.py`: Original state machine with all feedback control (2000+ lines)
- `valve_rotation_joy.launch`: Launch file configuration

## Benefits of This Approach

1. **No code duplication**: Reuses existing tested components
2. **Maintainable**: Changes to original state machine automatically benefit this version
3. **Lightweight**: Only 268 lines vs 1000+ lines in previous bloated version
4. **Stable**: Uses proven state machine architecture
5. **Extensible**: Easy to add new joy control features without touching core logic

## Parameters

- `module_id`: UAV module ID (default: 1)
- `auto_start`: Whether to start automatically without joy control (default: false)
- All other parameters are handled by the original state machine

## Dependencies

- Original `valve_rotation_fang_single.py` and its dependencies
- `sensor_msgs` for Joy messages
- `smach` for state machine execution
- `threading` for parallel execution

This simplified approach demonstrates the power of proper software architecture and component reuse.
