std_msgs/Header header

# Command-model-equivalent thrust after the final PWM clamp [N].
# One entry per physical rotor.
float32[] thrust

# Final gimbal target after the FC-side low-pass update [rad].
# Ordered by rotor, then by gimbal DoF.
float32[] gimbal_angle

# Final PWM command sent by the FC, in the same 0..2000 convention as Pwms.
uint16[] pwm

# Per-rotor PWM/model flags using the constants below.
uint8[] rotor_flags

# Remaining scale after saturation handling (1.0 means no shedding).
float32 base_thrust_scale
float32 yaw_scale

uint8 saturation_flags
uint8 BASE_THRUST_SHED=1
uint8 YAW_SHED=2
uint8 PWM_MIN_CLIPPED=4
uint8 PWM_MAX_CLIPPED=8
uint8 THRUST_MODEL_INVALID=16
