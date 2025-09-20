import numpy as np

class BlockSimulator:
    def __init__(self, width=12, length_short=14, length_long=16):
        # Dimensions in inches → mm
        self.width = width * 25.4
        self.length_short = length_short * 25.4
        self.length_long = length_long * 25.4

        self.length_avg = (self.length_short + self.length_long) / 2
        self.width_half = self.width / 2

        # Corner coordinates (x, y)
        self.corners = [
            np.array([-self.length_short/2, -self.width_half]),  # A
            np.array([self.length_short/2, -self.width_half]),   # B
            np.array([self.length_long/2, self.width_half]),     # C
            np.array([-self.length_long/2, self.width_half])     # D
        ]

        # Random tilt ±5 degrees
        self.theta_x = np.radians(np.random.uniform(-5, 5))
        self.theta_y = np.radians(np.random.uniform(-5, 5))

        # Laser sensors (3 along bottom edge)
        self.laser_positions = [
            np.array([-self.length_short/4, -self.width_half]),
            np.array([0, -self.width_half]),
            np.array([self.length_short/4, -self.width_half])
        ]
        self.laser_offsets = np.random.uniform(-2, 2, 3)

        # Random laser readings 5–25 mm
        self.laser_readings = np.random.uniform(10, 25, 3)

        # Compute corner heights from tilt + bottom-center laser
        center_laser_height = self.laser_readings[1]
        self.z = np.zeros(4)
        for i, (x, y) in enumerate(self.corners):
            self.z[i] = center_laser_height + self.theta_x * y - self.theta_y * x

        # Actuation parameters
        self.k = 5.0
        self.noise_scale = 0.2

        print(f"Initial tilt X: {np.degrees(self.theta_x):.2f}°, Y: {np.degrees(self.theta_y):.2f}°")
        print(f"Laser readings: {self.laser_readings}")
        print(f"Initial corner heights: {self.z}")

    def actuate_piston(self, corner_id, power):
        if power <= 0:
            return
        noise = np.random.uniform(-self.noise_scale, self.noise_scale)
        delta_z = -power * self.k * (1 + noise)
        self.z[corner_id] += delta_z

    def get_level(self):
        points = np.array([np.append(c, z) for c, z in zip(self.corners, self.z)])
        X = points[:, :2]
        X = np.c_[X, np.ones(4)]
        Y = points[:, 2]
        coeffs, _, _, _ = np.linalg.lstsq(X, Y, rcond=None)
        a, b, _ = coeffs
        theta_x = np.arctan(-b)
        theta_y = np.arctan(a)
        return theta_x, theta_y

    def interpolate_height(self, x, y):
        x = np.clip(x, -self.length_avg/2, self.length_avg/2)
        y = np.clip(y, -self.width_half, self.width_half)

        if abs(y + self.width_half) < 1:  # bottom
            t = (x + self.length_short/2) / self.length_short
            return (1-t)*self.z[0] + t*self.z[1]
        elif abs(y - self.width_half) < 1:  # top
            t = (x + self.length_long/2) / self.length_long
            return (1-t)*self.z[3] + t*self.z[2]
        else:
            return np.mean(self.z)

    def get_height(self):
        readings = []
        for i, pos in enumerate(self.laser_positions):
            x, y = pos
            h = self.interpolate_height(x, y) + self.laser_offsets[i]
            h += np.random.normal(0, 1.5)
            readings.append(h)
        fused = np.mean(readings) + np.random.normal(0, 0.3)
        print(f"  Laser readings: {[f'{r:.2f}' for r in readings]} -> fused: {fused:.2f}mm")
        return fused

    def get_corner_status(self):
        status = []
        for i, z in enumerate(self.z):
            if z > 2.0:
                status.append(f"Corner {i} ({z:+.1f}mm): TOO HIGH")
            elif z < -2.0:
                status.append(f"Corner {i} ({z:+.1f}mm): TOO LOW")
            else:
                status.append(f"Corner {i} ({z:+.1f}mm): OK")
        return status
