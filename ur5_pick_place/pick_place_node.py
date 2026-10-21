#!/usr/bin/env python3
"""Simulación de una rutina Pick and Place para UR5 + gripper."""
import time

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]
GRIPPER_OPEN = [0.04, 0.04]
GRIPPER_CLOSED = [0.004, 0.004]

# Puntos de interés (radianes). Ajustar si hay colisiones en la simulación.
HOME = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
PICK = [0.8, -1.2, 1.5, -1.87, -1.57, 0.8]
PLACE = [-0.8, -1.2, 1.5, -1.87, -1.57, -0.8]

# Cada punto: nombre, posición articular, tiempo de movimiento y acciones.
# Acciones: ('info', texto) | ('gripper', 'open'|'close')
#           ('wait_di', n) | ('set_do', n, True|False)
ROUTINE = [
    {'name': 'HOME', 'joints': HOME, 'time': 4.0,
     'actions': [('info', 'Inicio de rutina en HOME'), ('gripper', 'open')]},
    {'name': 'PICK', 'joints': PICK, 'time': 4.0,
     'actions': [('wait_di', 0), ('gripper', 'close'), ('set_do', 0, True)]},
    {'name': 'PLACE', 'joints': PLACE, 'time': 5.0,
     'actions': [('gripper', 'open'), ('set_do', 0, False),
                 ('info', 'Objeto liberado en PLACE')]},
    {'name': 'HOME', 'joints': HOME, 'time': 4.0,
     'actions': [('info', 'Rutina finalizada')]},
]


class PickPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_place_node')
        self.arm_pub = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory', 10)
        self.grip_pub = self.create_publisher(
            Float64MultiArray, '/gripper_controller/commands', 10)

        self.joint_pos = {}
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

        # Entradas/salidas digitales simuladas con topics Bool
        self.di_state = {}
        self.do_pubs = {}
        for i in range(2):
            self.di_state[i] = False
            self.create_subscription(
                Bool, f'/digital_input_{i}',
                lambda msg, idx=i: self._di_cb(idx, msg), 10)
            self.do_pubs[i] = self.create_publisher(
                Bool, f'/digital_output_{i}', 10)

    def _js_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.joint_pos[name] = pos

    def _di_cb(self, idx, msg):
        self.di_state[idx] = msg.data

    def spin_for(self, seconds):
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _reached(self, target, tol=0.03):
        return all(
            abs(self.joint_pos.get(j, 1e9) - t) < tol
            for j, t in zip(ARM_JOINTS, target))

    def move_arm(self, target, move_time):
        traj = JointTrajectory()
        traj.joint_names = ARM_JOINTS
        point = JointTrajectoryPoint()
        point.positions = target
        point.time_from_start = Duration(
            sec=int(move_time), nanosec=int((move_time % 1) * 1e9))
        traj.points = [point]
        self.arm_pub.publish(traj)

        # Esperar a llegar (la simulación puede correr más lento que tiempo real)
        deadline = time.time() + move_time * 3 + 10
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._reached(target):
                break
        else:
            self.get_logger().warn('Tiempo agotado esperando llegar al punto')
        self.spin_for(0.5)

    def move_gripper(self, state):
        msg = Float64MultiArray()
        msg.data = GRIPPER_OPEN if state == 'open' else GRIPPER_CLOSED
        self.grip_pub.publish(msg)
        self.get_logger().info(f'Gripper: {state}')
        self.spin_for(1.5)

    def set_do(self, idx, value):
        self.do_pubs[idx].publish(Bool(data=value))
        self.get_logger().info(f'Salida digital {idx} -> {value}')
        self.spin_for(0.3)

    def wait_di(self, idx):
        self.di_state[idx] = False
        self.get_logger().info(
            f'Esperando entrada digital {idx} en alto. Para continuar: '
            f'ros2 topic pub --once /digital_input_{idx} '
            f'std_msgs/msg/Bool "{{data: true}}"')
        while rclpy.ok() and not self.di_state[idx]:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f'Entrada digital {idx} recibida')

    def run(self):
        self.spin_for(2.0)  # descubrimiento de publishers/subscribers
        for step in ROUTINE:
            self.get_logger().info(f'--- Moviendo a {step["name"]} ---')
            self.move_arm(step['joints'], step['time'])
            for action in step['actions']:
                kind = action[0]
                if kind == 'info':
                    self.get_logger().info(f'INFO: {action[1]}')
                elif kind == 'gripper':
                    self.move_gripper(action[1])
                elif kind == 'wait_di':
                    self.wait_di(action[1])
                elif kind == 'set_do':
                    self.set_do(action[1], action[2])
        self.get_logger().info('Rutina Pick and Place completada')


def main():
    rclpy.init()
    node = PickPlaceNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
