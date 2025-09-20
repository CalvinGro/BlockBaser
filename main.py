from DWL_to_degrees import decode_dwl5000xy
from degrees_and_laser_to_pos_change import parse_result_block

example_packet_DWL = bytes([0x71, 0x11, 0x22, 0x00, 0x01, 0x23,
                        0x00, 0xFF, 0x38, 0x00, 0x5A, 0x3C])
example_block = bytes.fromhex("01 00 00 00 00 00 00 00 00 00 00 C8")

dist = parse_result_block(example_block)
x, y = decode_dwl5000xy(example_packet_DWL)
print(f"X: {x:.4f}°, Y: {y:.4f}°")
print(f"distance: {dist}mm")

