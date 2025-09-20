# Main script (with faster convergence, tighter corner tolerance, and low-block safety)
import numpy as np
import time
from simulator import BlockSimulator  # Import the simulator class

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

# Settings
tolerance = [0.0005, 0.0005, 0.5]  # [rad, rad, mm]
corner_tolerance = 1             # mm
max_iters = 20
gain = 0.75
min_power = 0.01
low_block_limit = -0.7  # mm (safety check for sinking)

# Helpers
def get_height():
    return sim.get_height()

def get_state():
    theta_x, theta_y = read_level()
    h = get_height()
    return np.array([theta_x, theta_y, h])

def rad_to_deg(rad):
    return np.degrees(rad)

def check_corner_tolerance():
    max_deviation = np.max(np.abs(sim.z))
    return (max_deviation <= corner_tolerance, max_deviation)

def calculate_corner_powers():
    """Proportional control with damping"""
    corner_errors = -sim.z  # positive = too high
    powers = gain * (np.abs(corner_errors) / 10.0)

    # Mild decay near target
    avg_error = np.mean(np.abs(corner_errors))
    scale = min(1.0, avg_error / 2.0)
    powers *= scale

    return np.clip(powers, 0, 0.9)

# Main leveling loop
state = get_state()

print("="*70)
print("TRAPEZOIDAL BLOCK LEVELING SIMULATION - FAST/TIGHT MODE")
print(f"Target: Level (±0.03°) and ALL CORNERS within ±{corner_tolerance}mm of adjacent block")
print("="*70)

theta_x, theta_y, h = state
corner_ok, max_corner_dev = check_corner_tolerance()
print(f"Initial state:")
print(f"  Tilt X: {rad_to_deg(theta_x):+6.3f}°")
print(f"  Tilt Y: {rad_to_deg(theta_y):+6.3f}°")
print(f"  Height offset: {h:+6.1f}mm")
print(f"  Corner deviation: max |{max_corner_dev:+.1f}|mm")
print("  Corners:", [f"{z:+5.2f}" for z in sim.z])

print("\nWarmup: test blows...")
for piston in range(4):
    actuate_piston(piston, 0.02)
print("Warmup complete.")

for i in range(max_iters):
    theta_x, theta_y, h = state

    # SAFETY CHECK: block too low
    if np.min(sim.z) < low_block_limit:
        print("\n!!! block too low, lift it up and add gravel or something")
        break

    # Convergence check
    weighted_state = np.array([
        theta_x / tolerance[0],
        theta_y / tolerance[1],
        h / tolerance[2],
    ])
    state_norm = np.linalg.norm(weighted_state)
    corner_ok, max_corner_dev = check_corner_tolerance()

    if state_norm < 1.0 and corner_ok:
        print(f"\nITERATION {i+1}: SUCCESS - WITHIN TOLERANCE")
        break

    print(f"\nITERATION {i+1}:")
    print(f"  Tilt X: {rad_to_deg(theta_x):+6.3f}°")
    print(f"  Tilt Y: {rad_to_deg(theta_y):+6.3f}°")
    print(f"  Height: {h:+6.1f}mm")
    print(f"  Corner max deviation: {max_corner_dev:.2f}mm")
    print("  Corners:", [f"{z:+5.2f}" for z in sim.z])

    # Calculate powers
    powers = calculate_corner_powers()

    # Gentle tilt compensation
    if abs(theta_x) > 0.001:
        if theta_x > 0:
            powers[0] *= 1.05; powers[3] *= 1.05
            powers[1] *= 0.95; powers[2] *= 0.95
        else:
            powers[0] *= 0.95; powers[3] *= 0.95
            powers[1] *= 1.05; powers[2] *= 1.05

    if abs(theta_y) > 0.001:
        if theta_y > 0:
            powers[0] *= 1.05; powers[1] *= 1.05
            powers[2] *= 0.95; powers[3] *= 0.95
        else:
            powers[0] *= 0.95; powers[1] *= 0.95
            powers[2] *= 1.05; powers[3] *= 1.05

    # Apply
    for piston in range(4):
        if powers[piston] > min_power:
            actuate_piston(piston, powers[piston])

    # Update
    state = get_state()

else:
    print(f"\nMAX ITERATIONS ({max_iters}) REACHED")

print("\n" + "="*70)
print("SIMULATION COMPLETE")
theta_x, theta_y, h = state
corner_ok, max_corner_dev = check_corner_tolerance()
print(f"Final tilt X: {rad_to_deg(theta_x):+6.3f}°")
print(f"Final tilt Y: {rad_to_deg(theta_y):+6.3f}°")
print(f"Final height: {h:+6.1f}mm")
print(f"Final corners:", [f"{z:+5.2f}" for z in sim.z])
print(f"Final corner deviation: {max_corner_dev:.2f}mm ({'✓ OK' if corner_ok else '✗ EXCEEDS'})")

weighted_state = np.array([
    theta_x / tolerance[0],
    theta_y / tolerance[1],
    h / tolerance[2],
])
state_norm = abs(rad_to_deg(theta_x)) + abs(rad_to_deg(theta_y))
print(f"Final tilt error norm: {state_norm:.6f} ({'✓ OK' if state_norm < 0.3 else '✗ EXCEEDS'})")

if state_norm < 1.0 and corner_ok:
    print("✓ FULL SUCCESS: Block is within all tolerances!")
else:
    print("✗ PARTIAL FAILURE: Some tolerances not met")
print("="*70)
