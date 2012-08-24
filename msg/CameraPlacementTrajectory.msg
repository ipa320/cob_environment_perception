# When to start the camera placement trajectory
# (There is no header here because the frame is specified per-camera-placement.)
time start_time

# The vector of camera placements
# An empty vector can be sent if this message is being used to
# just send changes to the control parameters.
rviz_view_controllers/CameraPlacement[] placements

# The interaction style that should be activated when movement is done.
uint8 mouse_interaction_mode
uint8 NO_CHANGE = 0 # Leaves the control style unchanged
uint8 ORBIT = 1 # Activates the Orbit-style controller
uint8 FPS   = 2 # Activates the FPS-style controller

# A flag to enable or disable user interaction
bool interaction_enabled

# A flag indicating if the camera yaw axis is fixed to +Z of the camera attached_frame
bool maintain_fixed_up_axis
 

