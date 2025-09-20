

def decode_dwl5000xy(packet: bytes) -> tuple[float, float]:
    """
    Decode a 12-byte packet from the Digi-Pas DWL-5000XY sensor (dual-axis mode).

    Args:
        packet (bytes): The full 12-byte packet from the sensor.

    Returns:
        tuple[float, float]: (x_angle_deg, y_angle_deg)
    """
    if len(packet) != 12:
        raise ValueError("Packet must be exactly 12 bytes long")

    def decode_axis(b: bytes) -> int:
        """Convert 3 bytes (24-bit two's complement) into signed integer."""
        raw = (b[0] << 16) | (b[1] << 8) | b[2] # piece together three parts of the bit from the packet


        if raw & 0x800000:  # if sign bit (bit 23) is set
            raw -= 0x1000000  # subtract 2^24 to get negative value
        return raw

    # Y-axis: bytes 3-5, X-axis: bytes 6-8
    y_raw = decode_axis(packet[3:6])
    x_raw = decode_axis(packet[6:9])

    # Dual-axis mode scaling: 1000
    y_angle = y_raw / 1000.0
    x_angle = x_raw / 1000.0

    return (x_angle, y_angle)


