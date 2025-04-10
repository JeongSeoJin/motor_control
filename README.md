# motor_control

## This repository for motor control and Actuators 
foc control, simplefoc, position control, impedance control and so on

Actuator_V2 folder is complete opensource for proprioceptive actuator. there's a source code for motor control, step files, elements for complete building.

And I am going to make some youtube video for Building Tutorials and specific sources about my own proprioceptive acutator. you can expect it!

Actuator V2 preview is below

![alt text](image.png)

And 'drv8302_sunnysky_impedance_arduino' and 'drv8302_sunnysky_impedance_esp' are not actually torque control with current. This is based on voltage systemically. The actual Impedance Control(Torque Control) is 'esp32_current_control_low_side'. This is the Torque Control(Q-axis Current Control) with FOC Algorithm. 

'master_two_drv8302_control_with_CAN', 'slave_two_drv8302_control_with_CAN' and 'two_drv8302_control_with_CAN' are motor control which is 'esp32_current_control_low_side' merged with ACAN library.

By the way, 'TWAI_Simplefoc_Node' is motor control which is 'esp32_current_control_low_side' merged with ESP32-TWAI-CAN library.