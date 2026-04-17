### Description
Receives motor velocity commands, applies the control law and calculates RELbot's state based on the encoder counts

### Inputs
Messages passed through RosXenoBridge over topic '/Ros2Xeno'
    Type: Ros2Xeno
    Combined float message. Consists of 'left_wheel_vel' and 'left_wheel_vel'

Encoder readings sent from FPGA

### Outputs
Messages passed through RosXenoBridge over topic '/Xeno2Ros'
    Type: Xeno2Ros
    Combined float message. Consists of 'x', 'y' and 'theta'

Wheel velocity commands sent to FPGA

### Run
In a terminal run the following commands
'sudo ./build/controller_test/controller_test'
'ros2 run ros_xeno_bridge RosXenoBridge'

### Core components
'run()': receives motor velocity commands, applies the control law, calculates RELbot's state based on the encoder counts and sends it over Xeno2Ros topic