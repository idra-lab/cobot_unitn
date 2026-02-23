# Import symbolic math (SymPy) and numerical libraries
import sympy as sp
import numpy as np
from scipy.spatial.transform import Rotation as R
from sympy import lambdify

# ROS 2 message types
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

# ROS 2 core libraries
import rclpy
from rclpy.node import Node
import time

from cobot_unitn.robot_utils import compute_fk

class TrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('trajectory_planner')

        # Publisher to send the end-effector target pose
        self.target_pub = self.create_publisher(Pose, 'end_effector_target', 1)
        # Subscriber to receive joint states
        self.state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 1)

        # Flags and buffers
        self.q_ref = None                     # To store the initial joint positions
        self.joint_state_received = False     # Whether joint state has been received
        self.index = 0                        # Index for publishing trajectory points
        self.trajectory = None                # Placeholder for trajectory

        self.start_time = self.get_clock().now()

        # Start a timer to regularly publish trajectory points (every 0.1s = 10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def joint_state_callback(self, msg):
        """
        Callback triggered on first reception of joint states.
        Computes the trajectory based on current pose.
        """
        if not self.joint_state_received:
            self.q_ref = np.array(msg.position)
            self.joint_state_received = True
            self.get_logger().info("Initial joint state received. Computing trajectory...")
            self.destroy_subscription(self.state_sub)  # No need to listen anymore
            self.compute_trajectory()

    def compute_trajectory(self):
        """
        Reach a fixed position and then draw a circle
        Computes a circular trajectory in the workspace for the end-effector,
        starting from its current pose (computed via forward kinematics).
        """

        # --- Circular Position Trajectory ---
        radius = 0.04 
        n_points = 100
        angles = np.linspace(0, 2 * np.pi, n_points )
        circle_numbers=2

        orient_d = np.array([[0,-1,0],
                                [-1,0,0],
                                [0,0,-1]])
        orient_d = R.from_matrix(orient_d).as_rotvec()

        pos_init =  np.array([0.22,0,0.033])

        pos_traj = np.tile(pos_init,(30,1))
        pos_traj = np.hstack((pos_traj, np.tile(orient_d,(pos_traj.shape[0],1))))

        circle_center = pos_traj[-1,:3] + np.array([0,radius,0])
        trajectory_circle = circle_center  + radius*np.vstack((np.sin(angles),-np.cos(angles),np.zeros(n_points))).T
        trajectory_circle = np.hstack((trajectory_circle,np.tile(orient_d,(trajectory_circle.shape[0],1))))

        # --- Full 6D Trajectory: position + orientation ---
        self.raw_traj = np.vstack((pos_traj, np.tile(trajectory_circle,(circle_numbers,1))))

        # Downsample and pad the trajectory to smooth it at the start
        self.downsampled_traj = self.raw_traj[::1].tolist()
        self.downsampled_traj.insert(0, self.raw_traj[0].tolist())
        self.downsampled_traj.insert(0, self.raw_traj[0].tolist())
        self.downsampled_traj.insert(0, self.raw_traj[0].tolist())
        self.downsampled_traj.append(self.raw_traj[-1].tolist())
        self.downsampled_traj = np.array(self.downsampled_traj)

        self.downsampled_traj = self.raw_traj

        self.total_steps = self.downsampled_traj.shape[0]

        self.get_logger().info(f"Trajectory computed with {self.total_steps} steps.")
        self.get_logger().info(f"Trajectory content: \n{self.downsampled_traj} ")

        np.save('x_trajectory.npy',self.raw_traj)

    def timer_callback(self):
        """
        Periodically publishes the next pose in the trajectory.
        Stops when the end of the trajectory is reached.
        """
        if not self.joint_state_received or self.downsampled_traj is None:
            self.get_logger().info("Waiting for initial joint state and trajectory...")
            return

        if self.index < self.total_steps:
            X = self.downsampled_traj[self.index]

            # Create and publish Pose message
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = X[0:3]

            quat = R.from_rotvec(X[3:6]).as_quat()
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]

            self.target_pub.publish(pose)
            self.index += 1

        elif self.index == self.total_steps:
            self.get_logger().info("End-effector trajectory finished.")
            self.index += 1  # Prevents further publishing
            self.timer.cancel()


def main(args=None):
    """
    Main entry point. Initializes the node and spins until interrupted.
    """
    rclpy.init(args=args)
    node = TrajectoryPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
