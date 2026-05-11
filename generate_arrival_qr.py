# generate_arrival_qr.py
"""
Generate the Patrol Pro arrival QR marker as an SVG file.

This intentionally has no third-party dependencies. It emits a QR Code Model 2,
Version 1, error correction level L marker for the default payload:
PATROLPRO:ARRIVAL:1
"""

import argparse
import os


DEFAULT_PAYLOAD = "PATROLPRO:ARRIVAL:1"
DEFAULT_OUTPUT = "arrival_marker.svg"
ALPHANUMERIC = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ $%*+-./:"
VERSION = 1
SIZE = 21
DATA_CODEWORDS = 19
ECC_CODEWORDS = 7
FORMAT_ECL_L = 1
MASK_PATTERN = 0


def append_bits(bits, value, length):
    for i in range(length - 1, -1, -1):
        bits.append((value >> i) & 1)


def payload_to_codewords(payload):
    for char in payload:
        if char not in ALPHANUMERIC:
            raise ValueError("Unsupported QR character: {}".format(char))

    bits = []
    append_bits(bits, 0x2, 4)
    append_bits(bits, len(payload), 9)

    index = 0
    while index + 1 < len(payload):
        value = ALPHANUMERIC.index(payload[index]) * 45
        value += ALPHANUMERIC.index(payload[index + 1])
        append_bits(bits, value, 11)
        index += 2

    if index < len(payload):
        append_bits(bits, ALPHANUMERIC.index(payload[index]), 6)

    capacity_bits = DATA_CODEWORDS * 8
    append_bits(bits, 0, min(4, capacity_bits - len(bits)))
    while len(bits) % 8 != 0:
        bits.append(0)

    codewords = []
    for start in range(0, len(bits), 8):
        byte = 0
        for bit in bits[start:start + 8]:
            byte = (byte << 1) | bit
        codewords.append(byte)

    pad = [0xEC, 0x11]
    pad_index = 0
    while len(codewords) < DATA_CODEWORDS:
        codewords.append(pad[pad_index % 2])
        pad_index += 1

    if len(codewords) != DATA_CODEWORDS:
        raise ValueError("Payload is too long for QR version 1-L")
    return codewords


def gf_multiply(x, y):
    product = 0
    while y:
        if y & 1:
            product ^= x
        x <<= 1
        if x & 0x100:
            x ^= 0x11D
        y >>= 1
    return product & 0xFF


def reed_solomon_divisor(degree):
    result = [0] * (degree - 1) + [1]
    root = 1
    for _ in range(degree):
        for j in range(degree):
            result[j] = gf_multiply(result[j], root)
            if j + 1 < degree:
                result[j] ^= result[j + 1]
        root = gf_multiply(root, 0x02)
    return result


def reed_solomon_remainder(data, degree):
    divisor = reed_solomon_divisor(degree)
    result = [0] * degree
    for byte in data:
        factor = byte ^ result.pop(0)
        result.append(0)
        for i, coefficient in enumerate(divisor):
            result[i] ^= gf_multiply(coefficient, factor)
    return result


def make_matrix(payload):
    data = payload_to_codewords(payload)
    codewords = data + reed_solomon_remainder(data, ECC_CODEWORDS)
    bits = []
    for byte in codewords:
        append_bits(bits, byte, 8)

    matrix = [[False for _ in range(SIZE)] for _ in range(SIZE)]
    function = [[False for _ in range(SIZE)] for _ in range(SIZE)]

    def set_function(row, col, dark):
        matrix[row][col] = bool(dark)
        function[row][col] = True

    def draw_finder(top, left):
        for row in range(top - 1, top + 8):
            for col in range(left - 1, left + 8):
                if row < 0 or row >= SIZE or col < 0 or col >= SIZE:
                    continue
                inner_row = row - top
                inner_col = col - left
                dark = (
                    0 <= inner_row <= 6 and
                    0 <= inner_col <= 6 and
                    (
                        inner_row in (0, 6) or
                        inner_col in (0, 6) or
                        (2 <= inner_row <= 4 and 2 <= inner_col <= 4)
                    )
                )
                set_function(row, col, dark)

    draw_finder(0, 0)
    draw_finder(0, SIZE - 7)
    draw_finder(SIZE - 7, 0)

    for i in range(8, SIZE - 8):
        set_function(6, i, i % 2 == 0)
        set_function(i, 6, i % 2 == 0)

    set_function(4 * VERSION + 9, 8, True)
    draw_format_bits(matrix, function)

    bit_index = 0
    upward = True
    col = SIZE - 1
    while col > 0:
        if col == 6:
            col -= 1

        for i in range(SIZE):
            row = SIZE - 1 - i if upward else i
            for current_col in (col, col - 1):
                if function[row][current_col]:
                    continue
                dark = bit_index < len(bits) and bits[bit_index] == 1
                if mask_bit(row, current_col):
                    dark = not dark
                matrix[row][current_col] = dark
                bit_index += 1

        upward = not upward
        col -= 2

    return matrix


def mask_bit(row, col):
    return (row + col) % 2 == 0


def format_bits():
    data = (FORMAT_ECL_L << 3) | MASK_PATTERN
    remainder = data
    for _ in range(10):
        remainder = (remainder << 1) ^ ((remainder >> 9) * 0x537)
    return ((data << 10) | remainder) ^ 0x5412


def bit_at(value, index):
    return ((value >> index) & 1) != 0


def draw_format_bits(matrix, function):
    bits = format_bits()

    def set_function(row, col, dark):
        matrix[row][col] = bool(dark)
        function[row][col] = True

    for i in range(6):
        set_function(i, 8, bit_at(bits, i))
    set_function(7, 8, bit_at(bits, 6))
    set_function(8, 8, bit_at(bits, 7))
    set_function(8, 7, bit_at(bits, 8))
    for i in range(8, 15):
        set_function(8, 14 - i, bit_at(bits, i))

    for i in range(8):
        set_function(8, SIZE - 1 - i, bit_at(bits, i))
    for i in range(8, 15):
        set_function(SIZE - 15 + i, 8, bit_at(bits, i))
    set_function(SIZE - 8, 8, True)


def write_svg(matrix, output_path, module_px=18, quiet_zone=4):
    module_count = len(matrix) + quiet_zone * 2
    size_px = module_count * module_px
    rects = []
    for row, values in enumerate(matrix):
        for col, dark in enumerate(values):
            if dark:
                x = (col + quiet_zone) * module_px
                y = (row + quiet_zone) * module_px
                rects.append('<rect x="{}" y="{}" width="{}" height="{}"/>'.format(
                    x, y, module_px, module_px
                ))

    body = "\n  ".join(rects)
    svg = """<svg xmlns="http://www.w3.org/2000/svg" width="{0}" height="{0}" viewBox="0 0 {0} {0}">
  <rect width="100%" height="100%" fill="#fff"/>
  <g fill="#000">
  {1}
  </g>
</svg>
""".format(size_px, body)

    with open(output_path, "w") as handle:
        handle.write(svg)


def main():
    parser = argparse.ArgumentParser(description="Generate Patrol Pro arrival QR marker")
    parser.add_argument("--payload", default=DEFAULT_PAYLOAD)
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    output_path = os.path.abspath(args.output)
    matrix = make_matrix(args.payload)
    write_svg(matrix, output_path)
    print("Wrote {}".format(output_path))
    print("Payload: {}".format(args.payload))


if __name__ == "__main__":
    main()
