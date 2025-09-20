# Main script (updated for trapezoidal block)
import numpy as np
import time
from simulator import BlockSimulator  # Import the simulator
from DWL_to_degrees import decode_dwl5000xy
from degrees_and_laser_to_pos_change import parse_result_block

example_packet_DWL = bytes([0x71, 0x11, 0x22, 0x00, 0x01, 0x23,
                        0x00, 0xFF, 0x38, 0x00, 0x5A, 0x3C])
example_block = bytes.fromhex("01 00 00 00 00 00 00 00 00 00 00 C8")

dist = parse_result_block(example_block)
x, y = decode_dwl5000xy(example_packet_DWL)
print(f"X: {x:.4f}°, Y: {y:.4f}°")
print(f"distance: {dist}mm")

# Initialize simulator with trapezoidal dimensions
sim = BlockSimulator(width=12, length_short=14, length_long=16)

# Mock vl53 for compatibility
vl53 = [None] * 3  # Dummy

# Override hardware functions with sim
def read_level():
    return sim.get_level()

def actuate_piston(corner_id, power):
    sim.actuate_piston(corner_id, power)
    time.sleep(0.1)  # Simulate delay for settling

# Settings (now trapezoidal)
tolerance = [0.001, 0.001, 0.1]  # [rad, rad, mm]
max_iters = 50
test_power = 0.1
gain = 0.5

# Simple height from sim
def get_height():
    return sim.get_height()

def get_state():
    theta_x, theta_y = read_level()
    h = get_height()
    return np.array([theta_x, theta_y, h])

# Main leveling loop
state = get_state()
print(f"Initial state: tilts=({state[0]:.4f}, {state[1]:.4f} rad), height={state[2]:.1f}mm")
print(f"Block geometry: {sim.length_short/25.4:.1f}' x {sim.width/25.4:.1f}' trapezoid")

for i in range(max_iters):
    # Check if block is too low
    if state[2] < -tolerance[2]:
        print("Block is too low! Add gravel and lift it up.")
        break
    
    # Check if already level enough
    if np.linalg.norm(state) < np.linalg.norm(tolerance):
        print("Block is level and flush!")
        break
    
    # Build response matrix with test blows
    response = np.zeros((3, 4))
    for piston in range(4):
        before = get_state()
        actuate_piston(piston, test_power)
        after = get_state()
        response[:, piston] = (after - before) / test_power
    
    # Calculate needed corrections
    target_correction = -state
    powers = np.linalg.pinv(response) @ target_correction
    powers = np.maximum(powers, 0)  # No negative powers
    
    # Scale and apply
    if np.max(powers) > 0:
        powers = gain * powers / np.max(powers)
    
    # Apply corrections
    total_power = np.sum(powers)
    if total_power > 0:
        for piston in range(4):
            if powers[piston] > 0:
                actuate_piston(piston, powers[piston])
        print(f"Iter {i+1}: Applied powers=[{powers.round(3)}], total={total_power:.3f}")
    
    # Check new state
    state = get_state()
    print(f"  New state: tilts=({state[0]:.4f}, {state[1]:.4f}), height={state[2]:.1f}mm")
    print(f"  Corner heights: A={sim.z[0]:.1f}, B={sim.z[1]:.1f}, C={sim.z[2]:.1f}, D={sim.z[3]:.1f}mm")

else:
    print("Max iterations reached - check setup or material.")

print("Done.")
print(f"Final corner heights: {sim.z.round(1)} mm")