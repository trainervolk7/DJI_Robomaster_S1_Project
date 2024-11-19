# Reports the state of a joystick's axes and buttons.

# The timestamp is the time at which data is received from the joystick.
std_msgs/Header header

# Left joystick
float32[2] left_joystick

# Right joystick
float32[2] right_joystick

# L2 value
float32 l2

# R2 Value
float32 r2

# Buttons pressed
string button

# Buttons pressed direction
string button_direction
