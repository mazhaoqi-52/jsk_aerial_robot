std_msgs/Header header
uint8 id
float64 mass
# Body-fixed inertia/origins in physical baselink axes. The origin field keeps
# its legacy name for message compatibility; it is not in the virtual CoG axes.
float64[9] inertia
geometry_msgs/Vector3[] rotor_origin_from_cog
int8[] rotor_direction
float64 mf_rate
