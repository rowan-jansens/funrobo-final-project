import numpy as np
from scripts.utils import EndEffector, dh_to_matrix, wraptopi

class RobotKinematics:
    def __init__(self):
        """Initialize the robot kinematics model."""
        # Robot dimensions (in meters)
        self.l1 = 0.105  # Height of first joint
        self.l2 = 0.101  # Length of first arm segment
        self.l3 = 0.120  # Length of second arm segment
        self.l4 = 0.047  # Length of third arm segment
        self.l5 = 0.100  # End effector length

        self.num_dof = 5  # Degrees of freedom
        
        # Joint angles (radians)
        self.theta = np.zeros(self.num_dof)
        
        # Joint limits (radians)
        self.theta_limits = [
            [-np.pi*2/3, np.pi*2/3],   # Base joint
            [-np.pi/2, np.pi/2],       # Shoulder
            [-np.pi*2/3, np.pi*2/3],   # Elbow
            [-np.pi*5/9, np.pi*5/9],   # Wrist pitch
            [-np.pi/2, np.pi/2]        # Wrist roll
        ]

        # DH parameters [theta, d, a, alpha]
        self.DH = np.zeros((self.num_dof, 4))
        
        # Transformation matrices
        self.T = [np.eye(4) for _ in range(self.num_dof)]
        
        # End effector pose
        self.ee = EndEffector()
        
        # Robot joint positions
        self.points = np.zeros((self.num_dof + 1, 3))

    def calc_robot_points(self):
        """Calculate the positions of all robot joints."""
        T_current = np.eye(4)
        self.points[0] = [0, 0, 0]  # Base position
        
        for i in range(self.num_dof):
            T_current = T_current @ self.T[i]
            self.points[i + 1] = T_current[:3, 3]
        
        # Update end effector position and orientation
        self.ee.x = self.points[-1][0]
        self.ee.y = self.points[-1][1]
        self.ee.z = self.points[-1][2]
        
        # Extract rotation angles
        R = T_current[:3, :3]
        roll, pitch, yaw = self.rotm_to_euler(R)
        self.ee.rotx = roll
        self.ee.roty = pitch
        self.ee.rotz = yaw

    def rotm_to_euler(self, R):
        """Convert rotation matrix to Euler angles (roll, pitch, yaw)."""
        sy = np.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        singular = sy < 1e-6

        if not singular:
            roll = np.arctan2(R[2,1], R[2,2])
            pitch = np.arctan2(-R[2,0], sy)
            yaw = np.arctan2(R[1,0], R[0,0])
        else:
            roll = np.arctan2(-R[1,2], R[1,1])
            pitch = np.arctan2(-R[2,0], sy)
            yaw = 0

        return roll, pitch, yaw

    def calc_jacobian(self, theta):
        """Calculate the robot's Jacobian matrix."""
        J = np.zeros((6, len(theta)))
        T_current = np.eye(4)
        p_end = self.points[-1]
        
        for i in range(len(theta)):
            T_current = T_current @ self.T[i]
            z_axis = T_current[:3, 2]
            p_current = T_current[:3, 3]
            
            # Linear velocity component
            J[:3, i] = np.cross(z_axis, p_end - p_current)
            # Angular velocity component
            J[3:, i] = z_axis
            
        return J

    def damped_inverse_jacobian(self, theta, lambda_=0.01):
        """Calculate damped least squares inverse of Jacobian."""
        J = self.calc_jacobian(theta)
        return J.T @ np.linalg.inv(J @ J.T + lambda_**2 * np.eye(6))

    def calc_numerical_ik(self, EE: EndEffector, tol=0.01, ilimit=50):
        """ Calculate numerical inverse kinematics based on input coordinates. """

        xd = np.array([EE.x, EE.y, EE.z, EE.rotx, EE.roty, EE.rotz])  # Target pose
        theta = self.theta.copy()

        for i in range(ilimit):
            # forward kinematics and current EE pose
            self.calc_forward_kinematics(theta, radians=True)
            current = np.array([
                self.ee.x, self.ee.y, self.ee.z,
                self.ee.rotx, self.ee.roty, self.ee.rotz
            ])

            err = xd - current
            err[3:] = [wraptopi(a) for a in err[3:]]

            if np.linalg.norm(err) < tol:
                break

            J_inv = self.damped_inverse_jacobian(theta)
            dtheta = J_inv @ err[:3]  # Use only position part

            theta += dtheta
            for j in range(len(theta)):
                theta[j] = np.clip(theta[j], self.theta_limits[j][0], self.theta_limits[j][1])

        self.theta = theta.copy()
        self.calc_forward_kinematics(self.theta, radians=True)
        return np.rad2deg(self.theta)  # Return angles in degrees

    def calc_forward_kinematics(self, theta: list, radians=False):
        """Calculate forward kinematics for given joint angles."""
        if not radians:
            self.theta = np.deg2rad(theta)
        else:
            self.theta = theta

        # Set DH parameters
        self.DH[0] = [self.theta[0], self.l1, 0, np.pi/2]
        self.DH[1] = [self.theta[1] + np.pi/2, 0, self.l2, np.pi]
        self.DH[2] = [self.theta[2], 0, self.l3, np.pi]
        self.DH[3] = [self.theta[3] - np.pi/2, 0, 0, -np.pi/2]
        self.DH[4] = [self.theta[4], self.l4 + self.l5, 0, 0]

        # Compute transformation matrices
        for i in range(self.num_dof):
            self.T[i] = dh_to_matrix(self.DH[i])
        
        self.calc_robot_points()