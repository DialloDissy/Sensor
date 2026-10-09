import struct
import subprocess


def fnv1a(data):
    value = 2166136261
    for byte in data:
        value = ((value ^ byte) * 16777619) & 0xffffffff
    return value


def make_record(timestamp, sequence, sensor_id, flags, x, y, z):
    data = struct.pack(
        "<QI BBH iii", timestamp, sequence, sensor_id, flags, 0, x, y, z
    )
    return data + struct.pack("<I", fnv1a(data))


def make_trace():
    header = b"USENS001" + struct.pack("<HH", 32, 1)
    header += struct.pack("<I", fnv1a(header))

    records = [
        make_record(2_000_000_000, 60, 1, 1, 60, 0, 0),
        make_record(2_000_000_000, 10, 2, 1, 400, 0, 0),
        make_record(2_000_000_000, 3, 3, 1, 488566000, 23522000, 35000),
        make_record(2_000_000_000, 7, 5, 1, 1, 0, 0),
        make_record(2_050_000_000, 11, 2, 1, 410, 0, 0),
    ]
    return header + b"".join(records)


result = subprocess.run(
    ["./solution"],
    input=make_trace(),
    capture_output=True,
    check=True,
)
print(result.stdout.decode(), end="")
