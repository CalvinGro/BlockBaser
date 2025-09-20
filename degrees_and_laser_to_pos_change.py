
def parse_result_block(block: bytes) -> int | None:
    """Parse 12-byte VL53L0X result block (logic layer). Returns distance in mm."""

    if block is None or len(block) != 12:
        return None
    
    # Last two bytes are distance MSB/LSB in mm:
    msb, lsb = block[-2], block[-1]
    dist = (msb << 8) | lsb

    return dist if 0 < dist < 65535 else None