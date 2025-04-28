import numpy as np

# Robot dimensions (in meters)
l1 = 0.105  # Height of first joint
l2 = 0.101  # Length of first arm segment
l3 = 0.120  # Length of second arm segment
l4 = 0.047  # Length of third arm segment
l5 = 0.100  # End effector length

# Target position
target_x = 0.25  # meters
target_y = 0.0   # meters
target_z = 0.05  # meters

# Initialize joint angles (radians)
theta = np.zeros(5)

# Joint limits (radians)
theta_limits = [
    [-np.pi, np.pi],                    # Base joint: ±180°
    [-np.pi/3, np.pi],                  # Shoulder: -60° to 180°
    [-np.pi+np.pi/12, np.pi-np.pi/4],  # Elbow: -165° to 135°
    [-np.pi+np.pi/12, np.pi-np.pi/12], # Wrist pitch: -165° to 165°
    [-np.pi, np.pi]                     # Wrist roll: ±180°
]

# Joint velocity limits (radians/sec)
thetadot_limits = [
    [-np.pi*2, np.pi*2],  # Base joint: ±360°/s
    [-np.pi*2, np.pi*2],  # Shoulder: ±360°/s
    [-np.pi*2, np.pi*2],  # Elbow: ±360°/s
    [-np.pi*2, np.pi*2],  # Wrist pitch: ±360°/s
    [-np.pi*2, np.pi*2]   # Wrist roll: ±360°/s
]

# DH parameters [theta, d, a, alpha]
DH = np.zeros((5, 4))

# Set DH parameters
DH[0] = [theta[0], l1, 0, np.pi/2]
DH[1] = [theta[1] + np.pi/2, 0, l2, np.pi]
DH[2] = [theta[2], 0, l3, np.pi]
DH[3] = [theta[3] - np.pi/2, 0, 0, -np.pi/2]
DH[4] = [theta[4], l4 + l5, 0, 0]

# Calculate transformation matrix from DH parameters
def dh_matrix(dh_params):
    theta, d, a, alpha = dh_params
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    
    return np.array([
        [ct, -st*ca, st*sa, a*ct],
        [st, ct*ca, -ct*sa, a*st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ])

# Iterative inverse kinematics
max_iterations = 50
tolerance = 0.01
lambda_ = 0.01  # Damping factor

for iteration in range(max_iterations):
    # Forward kinematics
    T = np.eye(4)
    current_points = np.zeros((6, 3))
    
    for i in range(5):
        DH[0] = [theta[0], l1, 0, np.pi/2]
        DH[1] = [theta[1] + np.pi/2, 0, l2, np.pi]
        DH[2] = [theta[2], 0, l3, np.pi]
        DH[3] = [theta[3] - np.pi/2, 0, 0, -np.pi/2]
        DH[4] = [theta[4], l4 + l5, 0, 0]
        
        Ti = dh_matrix(DH[i])
        T = T @ Ti
        current_points[i+1] = T[:3, 3]
    
    # Current end effector position
    current_x = current_points[-1][0]
    current_y = current_points[-1][1]
    current_z = current_points[-1][2]
    
    # Position error
    error = np.array([
        target_x - current_x,
        target_y - current_y,
        target_z - current_z
    ])
    
    # Check if we're close enough
    if np.linalg.norm(error) < tolerance:
        break
    
    # Calculate Jacobian
    J = np.zeros((3, 5))
    T_current = np.eye(4)
    p_end = current_points[-1]
    
    for i in range(5):
        T_current = T_current @ dh_matrix(DH[i])
        z_axis = T_current[:3, 2]
        p_current = T_current[:3, 3]
        J[:, i] = np.cross(z_axis, p_end - p_current)
    
    # Damped least squares
    J_inv = J.T @ np.linalg.inv(J @ J.T + lambda_**2 * np.eye(3))
    dtheta = J_inv @ error
    
    # Update joint angles
    theta += dtheta
    
    # Apply joint limits
    for i in range(len(theta)):
        theta[i] = np.clip(theta[i], theta_limits[i][0], theta_limits[i][1])

# Convert final angles to degrees and add gripper angle
final_angles = np.append(np.rad2deg(theta), 0.0)
print(f"Target position: ({target_x}, {target_y}, {target_z})")
print(f"Final joint angles (degrees): {final_angles}")