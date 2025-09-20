# simulator.py
import numpy as np

class BlockSimulator:
    def __init__(self, width=12, length_short=14, length_long=16):
        # Dimensions in inches (convert to mm internally for consistency)
        self.width = width * 25.4  # mm
        self.length_short = length_short * 25.4  # mm
        self.length_long = length_long * 25.4  # mm
        
        # Trapezoid geometry: bottom (A-B) is short side, top (D-C) is long side
        # For simplicity, assume linear taper - average length for center
        self.length_avg = (self.length_short + self.length_long) / 2
        self.width_half = self.width / 2
        
        # Corner positions (mm) - trapezoid with A-B at bottom (short), D-C at top (long)
        # Place centered at origin
        self.corners = {
            0: np.array([-self.length_short/2, -self.width_half]),  # A (bottom-left)
            1: np.array([self.length_short/2, -self.width_half]),   # B (bottom-right)
            2: np.array([self.length_long/2, self.width_half]),     # C (top-right)
            3: np.array([-self.length_long/2, self.width_half])     # D (top-left)
        }
        
        # Random initial corner heights (mm), reasonable misalignment: -10 to 10 mm
        self.z = np.random.uniform(-1, 30, 4)  # [z_A, z_B, z_C, z_D]
        
        # Stiffness factor: blow lowers by 0-5 mm per unit power, with noise
        self.k = 5.0  # max lowering per unit power
        self.noise_scale = 0.2  # 20% variability
        
        # Assume adjacent block at z=0, laser measures average "effective height offset"
        # Positive h: current block higher than adjacent (needs lowering)

    def actuate_piston(self, corner_id, power):
        if power <= 0:
            return
        # Lower the corner: delta_z = -power * k * (1 + noise)
        noise = np.random.uniform(-self.noise_scale, self.noise_scale)
        delta_z = -power * self.k * (1 + noise)
        self.z[corner_id] += delta_z

    def get_level(self):
        # Compute tilts (radians, small angle) for trapezoid
        # Use vector approach: fit plane to 4 points and find normal
        
        # Corner positions and heights
        points = [self.corners[i] + np.array([0, 0, self.z[i]]) for i in range(4)]
        
        # Fit plane: z = ax + by + c using least squares
        # Design matrix A: [x, y, 1] for each point
        A = np.array([[p[0], p[1], 1] for p in points[:3]])  # Use 3 points
        b = np.array([p[2] for p in points[:3]])
        
        # Solve for plane coefficients
        try:
            coeffs, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
            a, b, c = coeffs
        except:
            # Fallback if singular: use simple average slopes
            # theta_y (around x): average slope along length direction
            theta_y = ( (self.z[1] - self.z[0]) / self.length_short + 
                       (self.z[2] - self.z[3]) / self.length_long ) / 2
            # theta_x (around y): average slope along width direction
            theta_x = ( (self.z[3] - self.z[0] + self.z[2] - self.z[1]) / 
                       (2 * self.width) )
            return theta_x, theta_y
        
        # Convert plane coefficients to angles (small angle approximation)
        theta_x = np.arctan(-b)  # Rotation around y-axis
        theta_y = np.arctan(a)   # Rotation around x-axis
        
        return theta_x, theta_y

    def get_height(self):
        # Simulate fused laser: weighted average z based on trapezoid geometry
        # Weight by inverse distance from center for better "flush" representation
        center = np.array([0, 0])
        weights = 1 / (1 + np.linalg.norm([self.corners[i][:2] for i in range(4)], axis=1))
        h_weighted = np.average(self.z, weights=weights)
        
        # Add small measurement noise ~0.5 mm std dev
        noise = np.random.normal(0, 0.5)
        return h_weighted + noise

# Usage example:
# sim = BlockSimulator(width=12, length_short=14, length_long=16)
# print("Corner positions:", sim.corners)
# print("Initial heights:", sim.z)
# print("Level:", sim.get_level())
# print("Height:", sim.get_height())
# sim.actuate_piston(0, 0.5)
# print("After actuation:", sim.z)