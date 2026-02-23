import numpy as np
import matplotlib.pyplot as plt


# DH parameters in millimeters
# DH parameters in millimeters
d_mm = np.array([131.22, 0, 0, 63.4, 75.05, 45.6])  # Link offsets (along Z axis) in mm
a_mm = np.array([0, -110.4, -96, 0, 0, 0])  # Link lengths (along X axis) in mm
alpha = np.array([np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0])  # Link twists (around X axis)
offset = np.array([0, -np.pi/2, 0, -np.pi/2, np.pi/2, 0])  # Joint angle offsets

# Convert DH parameters to meters (by dividing by 1000)
d = d_mm / 1000  # Link offsets in meters
a = a_mm / 1000  # Link lengths in meters

def compute_A(a, alpha, d, theta):
    """
    Compute the homogeneous transformation matrix A_i for a single joint i
    using the Denavit-Hartenberg parameters and joint angle theta.
    Returns a 4x4 numpy array representing the transform from frame i-1 to i.
    """
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,              np.sin(alpha),                np.cos(alpha),               d],
        [0,              0,                            0,                           1]
    ])


def compute_fk(joint_angles):
    """
    Compute the forward kinematics of the robot given the current joint angles.
    Multiplies the chain of transformations to get the end-effector pose relative to base frame.
    Returns a 4x4 homogeneous transformation matrix of the end-effector.
    """
    T = np.eye(4)  # initialize transformation as identity
    for i in range(6):
        theta = joint_angles[i] + offset[i]  # apply joint angle offset
        A = compute_A(a[i], alpha[i], d[i], theta)  # transform for joint i
        T = T @ A  # chain transform
    return T


def compute_jacobian(joint_angles):
    """
    Compute the geometric Jacobian matrix for the robot at given joint angles.
    Returns a 6x6 Jacobian matrix composed of:
     - Top 3 rows: linear velocity components (partial derivatives of EE position)
     - Bottom 3 rows: angular velocity components (rotation axes)
    """
    T = np.eye(4)
    Ts = [T]  # store transformations from base to each joint

    # Compute transformation matrices for all joints
    for i in range(6):
        theta = joint_angles[i] + offset[i]
        A = compute_A(a[i], alpha[i], d[i], theta)
        T = T @ A
        Ts.append(T)

    o_n = Ts[6][:3, 3]  # End-effector position (from last transform)
    Jp = []  # Linear velocity part of Jacobian
    Jo = []  # Angular velocity part of Jacobian

    for i in range(6):
        z_i = Ts[i][:3, 2]  # z-axis of the i-th joint frame (rotation axis)
        o_i = Ts[i][:3, 3]  # origin of the i-th joint frame
        Jp_i = np.cross(z_i, o_n - o_i)  # linear velocity Jacobian component
        Jp.append(Jp_i)
        Jo.append(z_i)  # angular velocity Jacobian component

    # Stack linear and angular parts vertically to form full Jacobian
    J = np.vstack((np.array(Jp).T, np.array(Jo).T))
    return J

q_traj = np.load('x_trajectory.npy')

print(f'q shape {q_traj.shape}')

x_traj = []
for i in range(q_traj.shape[0]):
    x_traj.append(compute_fk(q_traj[i]))
x_traj = np.array(x_traj)

plt.figure()
plt.grid(True)
plt.axis('equal')

colors = np.linspace(0, 1, q_traj.shape[0])  # This will map from 0 to 1, affecting the color scale

scatter = plt.scatter(q_traj[:,1],q_traj[:,0],c=colors,cmap='coolwarm')
plt.colorbar(scatter)
plt.show()
plt.close()