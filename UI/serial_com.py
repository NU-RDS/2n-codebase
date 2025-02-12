import serial
import struct
import time

# Protocol constants (must match Teensy's definitions)
START_BYTE = 0xAA
END_BYTE = 0xBB
MSG_FEEDBACK = 0x01
FEEDBACK_PAYLOAD_LENGTH = 32
FEEDBACK_PACKET_SIZE = 37

def compute_checksum(msg_type, payload_len, payload):
    """
    Compute checksum as the modulo-256 sum of the message type, payload length, and payload bytes.
    """
    checksum = msg_type + payload_len
    for b in payload:
        checksum += b
    return checksum & 0xFF

def decode_packet(packet):
    """
    Given a packet (expected to be FEEDBACK_PACKET_SIZE bytes), validate it and decode its payload.
    
    Returns:
        (positions, velocities) tuple if valid, or None if packet is invalid.
    """
    if len(packet) != FEEDBACK_PACKET_SIZE:
        print("Invalid packet length.")
        return None

    # Verify start and end markers
    if packet[0] != START_BYTE or packet[-1] != END_BYTE:
        print("Start or end marker error.")
        return None

    msg_type = packet[1]
    payload_len = packet[2]
    
    # Verify the expected payload length
    if payload_len != FEEDBACK_PAYLOAD_LENGTH:
        print("Unexpected payload length:", payload_len)
        return None

    payload = packet[3:3 + payload_len]
    received_checksum = packet[3 + payload_len]
    computed_checksum = compute_checksum(msg_type, payload_len, payload)
    
    if computed_checksum != received_checksum:
        print("Checksum mismatch: computed {} != received {}".format(computed_checksum, received_checksum))
        return None

    # Decode 8 float32 values (little-endian): first 4 for positions, next 4 for velocities
    try:
        values = struct.unpack('<8f', payload)
    except struct.error as e:
        print("Unpack error:", e)
        return None

    positions = values[0:4]
    velocities = values[4:8]
    return positions, velocities

def main():
    # Change the port name as needed (e.g., 'COM3' on Windows or '/dev/ttyUSB0' on Linux)
    port = '/dev/ttyACM0'
    baud_rate = 115200

    try:
        ser = serial.Serial(port, baud_rate, timeout=1)
    except serial.SerialException as e:
        print("Error opening serial port:", e)
        return

    print("Serial port opened:", port)
    
    # Give the serial connection a moment to initialize
    time.sleep(2)

    while True:
        # Look for the start marker.
        # read(1) returns a single byte (as a bytes object) or empty bytes if timeout.
        byte = ser.read(1)
        if not byte:
            continue

        # Check if the byte is our start marker
        if byte[0] == START_BYTE:
            # Read the rest of the packet (FEEDBACK_PACKET_SIZE - 1 bytes)
            rest = ser.read(FEEDBACK_PACKET_SIZE - 1)
            if len(rest) != (FEEDBACK_PACKET_SIZE - 1):
                continue  # Incomplete packet; try again

            packet = byte + rest

            decoded = decode_packet(packet)
            if decoded is not None:
                positions, velocities = decoded
                print("Motor Positions: ", positions)
                print("Motor Velocities:", velocities)
            else:
                print("Failed to decode packet.")
        else:
            # If not the start marker, you might choose to ignore it or log it
            continue

if __name__ == '__main__':
    main()
