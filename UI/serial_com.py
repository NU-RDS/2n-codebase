import serial
import struct
import time
import threading
import csv  # Added for CSV file handling

# Protocol constants (must match Teensy's definitions)
START_BYTE = 0xAA
END_BYTE = 0xBB
MSG_FEEDBACK = 0x01
MSG_JOINT_COMMAND = 0x02
FEEDBACK_PAYLOAD_LENGTH = 8
FEEDBACK_PACKET_SIZE = 13
JOINT_COMMAND_PAYLOAD_LENGTH = 8
JOINT_COMMAND_PACKET_SIZE = 13

class MotorSerialInterface:
    def __init__(self, port='/dev/teensy4', baud_rate=115200, timeout=1):
        self.ser = serial.Serial(port, baud_rate, timeout=timeout)
        # Give the serial port a moment to initialize
        time.sleep(2)
        
        # Initialize motor state (positions) and joint commands.
        self._joint_states = (0.0, 0.0)
        self._joint_commands = [0.0, 0.0]

        # Locks to ensure thread-safe access.
        self._state_lock = threading.Lock()
        self._joint_command_lock = threading.Lock()

        # Event to signal thread shutdown.
        self._stop_event = threading.Event()

        # Start threads: one for reading feedback, one for sending joint commands.
        self._feedback_thread = threading.Thread(target=self._feedback_loop, daemon=True)
        self._joint_command_thread = threading.Thread(target=self._joint_command_loop, daemon=True)
        self._feedback_thread.start()
        self._joint_command_thread.start()

    def _compute_checksum(self, msg_type, payload_len, payload):
        """Compute checksum as the modulo-256 sum of the message type, payload length, and payload bytes."""
        checksum = msg_type + payload_len
        for b in payload:
            checksum += b
        return checksum & 0xFF

    def _decode_feedback_packet(self, packet):
        """
        Validate and decode a feedback packet.
        Returns a tuple (positions) if valid, or None otherwise.
        """
        if len(packet) != FEEDBACK_PACKET_SIZE:
            return None
        if packet[0] != START_BYTE or packet[-1] != END_BYTE:
            return None

        msg_type = packet[1]
        payload_len = packet[2]
        if payload_len != FEEDBACK_PAYLOAD_LENGTH:
            return None

        payload = packet[3:3 + payload_len]
        received_checksum = packet[3 + payload_len]
        computed_checksum = self._compute_checksum(msg_type, payload_len, payload)
        if computed_checksum != received_checksum:
            return None

        try:
            # Unpack 2 float32 values: 2 joint positions
            values = struct.unpack('<2f', payload)
        except struct.error:
            return None

        raw_positions = values[:]
        # Normalize angles to be within [-pi, pi].
        positions = tuple(self._pi2pi(angle) for angle in raw_positions)
        return positions

    def _encode_joint_command_packet(self, joint_commands):
        """
        Encode a joint command packet given 2 position values.
        Returns a bytes object.
        """
        packet = bytearray()
        packet.append(START_BYTE)
        packet.append(MSG_JOINT_COMMAND)
        packet.append(JOINT_COMMAND_PAYLOAD_LENGTH)
        payload = struct.pack('<2f', *joint_commands)
        packet.extend(payload)
        checksum = self._compute_checksum(MSG_JOINT_COMMAND, JOINT_COMMAND_PAYLOAD_LENGTH, payload)
        packet.append(checksum)
        packet.append(END_BYTE)
        return bytes(packet)

    def _feedback_loop(self):
        """Continuously reads from Serial and updates motor positions."""
        while not self._stop_event.is_set():
            # Look for the start marker.
            byte = self.ser.read(1)
            if not byte:
                continue
            if byte[0] == START_BYTE:
                rest = self.ser.read(FEEDBACK_PACKET_SIZE - 1)
                if len(rest) != (FEEDBACK_PACKET_SIZE - 1):
                    continue  # Incomplete packet; try again.
                packet = byte + rest
                decoded = self._decode_feedback_packet(packet)
                if decoded is not None:
                    with self._state_lock:
                        self._joint_states = decoded

    def _joint_command_loop(self):
        """Periodically sends the current joint commands over Serial at approximately 50 Hz."""
        while not self._stop_event.is_set():
            with self._joint_command_lock:
                current_joint_commands = self._joint_commands[:]
            packet = self._encode_joint_command_packet(current_joint_commands)
            self.ser.write(packet)

    def _pi2pi(self, angle):
        """
        Normalize angle to be within [-pi, pi].
        """
        return (angle + 3.14159265359) % (2 * 3.14159265359) - 3.14159265359

    def get_joint_states(self):
        """
        Return the latest motor states.
        Returns:
            (positions): tuple of 2 float values.
        """
        with self._state_lock:
            return self._joint_states

    def set_joint_positions(self, positions):
        """
        Update the joint commands to be sent.
        Parameters:
            positions: list or tuple of 2 float values.
        """
        if len(positions) != 2:
            raise ValueError("Expected 2 joint position command values")
        with self._joint_command_lock:
            self._joint_commands = list(positions)

    def close(self):
        """Stop the threads and close the serial port."""
        self._stop_event.set()
        self.ser.close()

# Example usage:
if __name__ == '__main__':
    msi = MotorSerialInterface(port='/dev/teensy4', baud_rate=115200)
    # Set an example joint command
    joint_commands = [0.0, 0.0]
    msi.set_joint_positions(joint_commands)

    # New feature: Record joint states for 10 seconds.
    recorded_data = []  # This will hold tuples of (timestamp, joint1, joint2)
    record_duration = 10  # seconds
    record_start = time.time()
    print("Recording joint states for 10 seconds...")
    while time.time() - record_start < record_duration:
        # Get the current joint states and timestamp
        msi.set_joint_positions(joint_commands)
        positions = msi.get_joint_states()
        current_time = time.time()
        recorded_data.append((current_time, positions[0], positions[1]))
        time.sleep(0.01)  # Sampling interval of 10ms

    # Save the recorded data to a CSV file.
    csv_filename = 'joint_states.csv'
    with open(csv_filename, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(['timestamp', 'joint1', 'joint2'])
        csv_writer.writerows(recorded_data)

    print(f"Joint state data recorded for {record_duration} seconds has been saved to '{csv_filename}'.")
    msi.close()
