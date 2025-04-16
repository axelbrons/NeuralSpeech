import struct

with open('audio8.txt', 'r') as f:
    values = [int(x.strip()) for x in f.readlines()]
with open('audio8.bin', 'wb') as f:
    for value in values:
        binary = struct.pack('<h', value)
        f.write(binary)