# Import ROS 2 Python client library and utilities
import rclpy
from rclpy.node import Node
import os
import sympy as sp  # symbolic math library (unused here but imported)
from sympy import lambdify
from scipy.spatial.transform import Rotation as R  # for rotation conversions
import math
import numpy as np  # numerical library for array/matrix math
from cobot_unitn.robot_utils import compute_fk,compute_control
# Import ROS 2 message types for joint states and poses
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

class EndEffectorController(Node):
    """
    ROS 2 Node to control the robot end-effector by converting
    desired end-effector pose commands into joint commands using
    Jacobian-based inverse kinematics.
    """

    def __init__(self):
        super().__init__('end_effector_controller')

        self.deltaT = 0.1    # Control loop time step (s)
        self.lmb = 1e-2      # Damping factor for Jacobian pseudo-inverse

        # Publisher for joint target positions to be sent to the robot driver
        self.target_pub = self.create_publisher(JointState, 'joint_targets', 1)

        # Subscriber to receive desired end-effector pose commands
        self.target_sub = self.create_subscription(
            Pose, '/end_effector_target', self.trajectory_callback, 1)

        # Subscriber to receive current joint states from robot
        self.state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 1)

        self.index = 0       # trajectory index (if using trajectories)
        self.speed = 50.0     # joint velocity for publishing (dummy constant speed)

        self.start_time = self.get_clock().now()

        # Predefine joint names for the JointState message
        self.joint_msg = JointState()
        self.joint_msg.name = [
            "joint2_to_joint1",
            "joint3_to_joint2",
            "joint4_to_joint3",
            "joint5_to_joint4",
            "joint6_to_joint5",
            "joint6output_to_joint6",
        ]
        self.joint_msg.velocity = [self.speed] * 6  # constant velocity values

        self.pos_d_seq = []
        self.q_d_seq = []
        self.or_d_seq = []


    def joint_state_callback(self, joint_msg):
        """
        Callback to update current joint positions from feedback topic.
        Stores the joint positions and then unsubscribes to avoid further updates.
        """
        self.old_q_d = list(joint_msg.position)
        # self.old_q_d= [0]*6
        self.destroy_subscription(self.state_sub)
        self.get_logger().info(f"SUBSCRIBER DESTROYED")
        self.get_logger().info(f"Current joints state: {self.old_q_d}")

    def trajectory_callback(self, pose_msg):
        """
        Callback triggered on receiving a desired end-effector pose.
        Computes joint commands by inverse kinematics and publishes them.
        """
        # Desired position from the Pose message
        pos_d = np.array([
            pose_msg.position.x,
            pose_msg.position.y,
            pose_msg.position.z
        ])

        # Desired orientation as a quaternion, converted to rotation vector
        R_d = R.from_quat([
            pose_msg.orientation.x,
            pose_msg.orientation.y,
            pose_msg.orientation.z,
            pose_msg.orientation.w
        ])
        
        ori_d = R_d.as_matrix()

        self.get_logger().info(f"pos_d = {pos_d}")
        self.get_logger().info(f"or_d = {ori_d}")

        Kp_pos =6
        Kp_ori = 1

        self.q_d = compute_control(self.old_q_d,pos_d,ori_d,np.eye(3)*Kp_pos,Kp_ori,self.deltaT)

        self.get_logger().info(f"q_d = {self.q_d}")

        # Prepare and publish the joint target message
        self.joint_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_msg.position = self.q_d.tolist()
        self.joint_msg.velocity = [self.speed] * 6

        self.target_pub.publish(self.joint_msg)
        self.index += 1
        self.old_q_d = np.copy(self.q_d)


def main(args=None):
    """
    Main function: Initialize ROS 2 node and spin to process callbacks until shutdown.
    """
    rclpy.init(args=args)
    node = EndEffectorController()

    try:
        rclpy.spin(node)  # Keep node alive to process incoming messages
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Stopping [end_effector_controller] node.")
    finally:
        node.get_logger().info("Shutting down [end_effector_controller] node and ROS.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
