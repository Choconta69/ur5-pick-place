# ur5_pick_place
Nodo ROS 2 (Python) que simula una rutina Pick and Place con un UR5 + gripper en Gazebo.
- Mueve el brazo por 4 puntos: HOME, PICK, PLACE, HOME.
- Operaciones: mensajes informativos, apertura/cierre de gripper, espera de entrada digital
  (/digital_input_0) y activación de salida digital (/digital_output_0), simuladas con topics std_msgs/Bool.

## Uso
ros2 launch ur5_robotiq_bringup sim_gz_sgb.launch.py gazebo_gui:=false
ros2 run ur5_pick_place pick_place_node
ros2 topic pub --once /digital_input_0 std_msgs/msg/Bool "{data: true}"
