#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class SingleJointCommander(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)

        # Joints wie in deiner Controller-Config
        self.joint_names = [
            "base_to_shoulder",
            "elbow_rotation",
            "wrist_extension_rotation",
            "wrist_rotation",
            "gripper_rotation"
        ]

        self.declare_parameter("position", 0.0)

        # Subscriber für aktuelle Positionen
        self.joint_states = {name: 0.0 for name in self.joint_names}
        self.create_subscription(JointState, "/joint_states", self.joint_states_cb, 10)

        # Publisher für Trajektorien
        self.publisher = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10
        )

    def joint_states_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_names:
                self.joint_states[name] = pos

    def move_joint(self, joint_name: str, target: float, duration: float = 2.0):
        if joint_name not in self.joint_names:
            self.get_logger().error(f"Unknown joint: {joint_name}")
            return

        # Basis: aktuelle Positionen aller Joints
        positions = [self.joint_states[j] for j in self.joint_names]

        # Zielwert für gewünschten Joint überschreiben
        idx = self.joint_names.index(joint_name)
        positions[idx] = target

        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)

        traj.points.append(point)

        self.get_logger().info(f"Moving {joint_name} to {target:.2f} rad in {duration:.1f}s")
        self.publisher.publish(traj)


def main_alt():
    rclpy.init()
    node = SingleJointCommander()

    # ein bisschen warten, bis joint_states angekommen sind
    rclpy.spin_once(node, timeout_sec=2.0)

    # Beispiel: base_to_shoulder auf 1.27 rad bewegen
    node.move_joint("base_to_shoulder", -1.27, duration=2.0)

    # noch kurz laufen lassen, damit es gesendet wird
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()


def main():
    node = None
    try:
        rclpy.init()
        try:
            node = SingleJointCommander("single_joint_commander")
            # ein bisschen warten, bis joint_states angekommen sind
            rclpy.spin_once(node, timeout_sec=1.0)

            value = node.get_parameter("position").get_parameter_value().double_value
            node.get_logger().info(f"position: {value}")
            # Beispiel: base_to_shoulder auf 1.27 rad bewegen
            node.move_joint("base_to_shoulder", value, duration=2.0)

            # noch kurz laufen lassen, damit es gesendet wird
            rclpy.spin_once(node, timeout_sec=2.0)

        except Exception as e:
            print(f"Fehler beim Erstellen des Nodes: {e}")
            return

    except KeyboardInterrupt:
        print("Sie haben STRG+C gedrückt!")

    finally:
        if node is not None:
            if rclpy.ok():
                node.get_logger().info(f"Node {node.get_name()} wird beendet!")
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
