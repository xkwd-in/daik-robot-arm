"""Raw serial test - exact same logic as old working script"""
import serial, time, sys

port = sys.argv[1] if len(sys.argv) > 1 else "COM6"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 1000000

ser = serial.Serial(port, baudrate=baud, timeout=0.1)
time.sleep(0.1)
ser.reset_input_buffer()

# Send INST_READ for position of servo 1 (identical to old code)
addr, rlen = 56, 2  # ADDR_PRESENT_POSITION
params = [addr, rlen]
length = len(params) + 2
core = [1, length, 2] + params  # ID=1, INST_READ=2
chk = (~sum(core)) & 0xFF
packet = bytes([0xFF, 0xFF] + core + [chk])

print(f"Port: {port} @ {baud}")
print(f"Sending: {packet.hex()} ({len(packet)} bytes)")
ser.write(packet)
ser.flush()

# Read all response bytes
start = time.time()
packet_bytes = []
while (time.time() - start) < 0.5:
    if ser.in_waiting > 0:
        b = ser.read(1)
        if b:
            packet_bytes.append(b[0])

if len(packet_bytes) == 0:
    print("Result: zero bytes received")
else:
    print(f"Received {len(packet_bytes)} bytes: {bytes(packet_bytes).hex()}")
    # Parse
    if len(packet_bytes) >= 2 and packet_bytes[0] == 0xFF and packet_bytes[1] == 0xFF:
        print("Header: OK (0xFF 0xFF)")
    else:
        print("Header: WRONG or echo")

ser.close()
