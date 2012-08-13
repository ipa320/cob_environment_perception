# The interpolation mode to use during this step
uint8 interpolation_mode
uint8 LINEAR    = 0 # Positions will be linearly interpolated
uint8 SPHERICAL = 1 # Position and orientation will be interpolated in a spherical sense.

# Sets this as the camera attached (fixed) frame before movement.
# An empty string will leave the attached frame unchanged. 
string attached_frame

# When should this pose be reached?
duration time_from_start

# The frame-relative point for the camera.
geometry_msgs/PointStamped eye

# The frame-relative point for the focus (or pivot for an Orbit controller).
geometry_msgs/PointStamped focus

# The frame-relative vector that maps to "up" in the view plane.
# The zero-vector will default to +Z in the camera fixed_frame.
geometry_msgs/Vector3Stamped up

