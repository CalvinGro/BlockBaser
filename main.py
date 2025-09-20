from DWL_to_degrees import decode_dwl5000xy
import numpy as np
import time
example_packet = bytes([0x71, 0x11, 0x22, 0x00, 0x01, 0x23,
                        0x00, 0xFF, 0x38, 0x00, 0x5A, 0x3C])

x, y = decode_dwl5000xy(example_packet)
print(f"X: {x:.4f}°, Y: {y:.4f}°")



# Assume hardware interfaces:
# read_level() -> [theta_x, theta_y] in radians
# vl53 = [sensor1, sensor2, sensor3]  # List of 3 VL53L4CD sensor objects, initialized via I2C
# actuate_piston(corner_id, power)  # corner_id: 0=A,1=B,2=C,3=D; power: 0-1 normalized

# Constants
L_x, L_y = 1000, 1000  # block dimensions in mm
p_test = 0.1  # small test power (normalized)
tolerance = np.array([0.001, 0.001, 0.1])  # [rad, rad, mm]
max_iters = 50
K_p = 0.5  # proportional gain for stability

# Fusion parameters for 3 lasers
num_samples = 10  # Samples per fusion
R = 2.25  # Measurement noise variance (mm², from sensor std dev ~1.5 mm)
Q = 0.1   # Process noise variance

# Global persistent state for Kalman (initialize outside loop)
h_pred = 0.0
P_pred = 1.0  # Initial estimate variance

def read_fused_height():
    global h_pred, P_pred
    readings = []
    for _ in range(num_samples):
        heights = np.array([s.get_distance() for s in vl53])  # Read from 3 lasers in mm
        readings.append(np.median(heights))  # Outlier-resistant median
        time.sleep(0.02)  # 50 Hz sampling
    h_avg = np.mean(readings)
    
    # Simple 1D Kalman filter
    K = P_pred / (P_pred + R)
    h_fused = h_pred + K * (h_avg - h_pred)
    P_pred = (1 - K) * P_pred + Q
    h_pred = h_fused  # Update prediction for next call
    
    return h_fused

def get_state():
    theta_x, theta_y = read_level()
    h = read_fused_height()  # Use fused height from 3 lasers
    return np.array([theta_x, theta_y, h])

def estimate_initial_deltas(state, L_x, L_y):
    theta_x, theta_y, h = state
    # Rough deltas without response model
    dx = L_x / 2
    dy = L_y / 2
    delta_A = -dx * theta_y - dy * theta_x + h
    delta_B = dx * theta_y - dy * theta_x + h
    delta_C = dx * theta_y + dy * theta_x + h
    delta_D = -dx * theta_y + dy * theta_x + h
    return np.array([delta_A, delta_B, delta_C, delta_D])  # but we won't use this directly

# Main loop
state = get_state()
iter = 0
while np.linalg.norm(state) > np.linalg.norm(tolerance) and iter < max_iters:
    if state[2] < -tolerance[2]:  # If h is too low (negative beyond tolerance)
        print("Block is too low; lift it up and add more gravel.")
        break
    
    # Step 1: Testing blows to build response matrix A (3x4)
    A = np.zeros((3, 4))
    for i in range(4):  # for each piston
        state_before = get_state()
        actuate_piston(i, p_test)
        state_after = get_state()
        delta_S = state_after - state_before
        A[:, i] = delta_S / p_test  # response per unit power
    
    # Step 2: Compute desired change
    delta_S_desired = -state
    
    # Step 3: Solve for powers p (least-squares with pseudoinverse)
    A_pinv = np.linalg.pinv(A)  # 4x3
    p = A_pinv @ delta_S_desired  # 4x1
    
    # Enforce non-negative (simple projection, may need optimization for better)
    p = np.maximum(p, 0)
    
    # Scale for stability
    p = K_p * p / np.max(p) if np.max(p) > 0 else p  # normalize to max 1
    
    # Step 4: Apply blows
    for i in range(4):
        if p[i] > 0:
            actuate_piston(i, p[i])
    
    # Step 5: Update state
    state = get_state()
    iter += 1

if iter < max_iters and state[2] >= -tolerance[2]:
    print("Block leveled and flushed.")
elif state[2] < -tolerance[2]:
    print("Block is too low; lift it up and add more gravel.")
else:
    print("Max iterations reached; check sensors or material.")