import serial
import struct
import time
import threading

# Protocol constants (must match Teensy's definitions)
START_BYTE = 0xAA
END_BYTE = 0xBB

# Message type definitions
MSG_FEEDBACK      = 0x01  # Combined motor and joint state feedback
MSG_TORQUE        = 0x02  # Torque command
MSG_JOINT_DESIRED = 0x03  # Joint desired command (to be sent to Teensy)

# Packet length definitions for MSG_FEEDBACK now includes joint positions.
# 10 float32 values: 4 motor positions + 4 motor velocities + 2 joint positions.
FEEDBACK_PAYLOAD_LENGTH = 40   # 10 float32 * 4 bytes each
FEEDBACK_PACKET_SIZE    = 45   # 1 + 1 + 1 + 40 + 1 + 1

TORQUE_PAYLOAD_LENGTH   = 16   # 4 float32
TORQUE_PACKET_SIZE      = 21   # 1 + 1 + 1 + 16 + 1 + 1

JOINT_DESIRED_PAYLOAD_LENGTH = 8   # 2 float32
JOINT_DESIRED_PACKET_SIZE    = 13  # 1 + 1 + 1 + 8 + 1 + 1

class MotorSerialInterface:
    def __init__(self, port='/dev/ttyACM0', baud_rate=115200, timeout=1):
        self.ser = serial.Serial(port, baud_rate, timeout=timeout)
        # Give the serial port a moment to initialize
        time.sleep(2)
        
        # Initialize motor state (4 motor positions and velocities)
        self._motor_positions = (0.0, 0.0, 0.0, 0.0)
        self._motor_velocities = (0.0, 0.0, 0.0, 0.0)
        self._torque_commands = [0.0, 0.0, 0.0, 0.0]
        
        # New joint state and desired command (only two elements)
        self._joint_positions = (0.0, 0.0)
        self._joint_desired = [0.0, 0.0]

        # Locks to ensure thread-safe access
        self._state_lock = threading.Lock()         # For motor state (positions & velocities)
        self._joint_state_lock = threading.Lock()     # For joint positions
        self._torque_lock = threading.Lock()          # For torque command
        self._joint_desired_lock = threading.Lock()   # For joint desired command

        # Event to signal thread shutdown
        self._stop_event = threading.Event()

        # Start threads:
        # 1. Feedback thread (handles combined motor and joint state feedback)
        self._feedback_thread = threading.Thread(target=self._feedback_loop, daemon=True)
        self._feedback_thread.start()
        # 2. Torque command sending thread
        self._torque_thread = threading.Thread(target=self._torque_loop, daemon=True)
        self._torque_thread.start()
        # 3. Joint desired command sending thread
        self._joint_desired_thread = threading.Thread(target=self._joint_desired_loop, daemon=True)
        self._joint_desired_thread.start()

    def _compute_checksum(self, msg_type, payload_len, payload):
        """
        Compute checksum as the modulo-256 sum of the message type, payload length, and payload bytes.
        """
        checksum = msg_type + payload_len
        for b in payload:
            checksum += b
        return checksum & 0xFF

    def _decode_feedback_packet(self, packet):
        """
        Decode the combined feedback packet.
        The packet should contain 10 float32 values:
            - First 4: motor positions.
            - Next 4: motor velocities.
            - Last 2: joint positions.
        Returns:
            (motor_positions, motor_velocities, joint_positions) if valid, else None.
        """
        if len(packet) != FEEDBACK_PACKET_SIZE:
            return None
        if packet[0] != START_BYTE or packet[-1] != END_BYTE:
            return None

        msg_type = packet[1]
        payload_len = packet[2]
        if msg_type != MSG_FEEDBACK or payload_len != FEEDBACK_PAYLOAD_LENGTH:
            return None

        payload = packet[3:3 + payload_len]
        received_checksum = packet[3 + payload_len]
        computed_checksum = self._compute_checksum(msg_type, payload_len, payload)
        if computed_checksum != received_checksum:
            return None

        try:
            # Unpack 10 float32 values: 4 motor positions, 4 motor velocities, 2 joint positions.
            values = struct.unpack('<10f', payload)
        except struct.error:
            return None

        motor_positions = values[0:4]
        motor_velocities = values[4:8]
        joint_positions = values[8:10]
        return motor_positions, motor_velocities, joint_positions

    def _encode_torque_packet(self, torques):
        """
        Encode a torque command packet given 4 torque values.
        Returns a bytes object.
        """
        packet = bytearray()
        packet.append(START_BYTE)
        packet.append(MSG_TORQUE)
        packet.append(TORQUE_PAYLOAD_LENGTH)
        payload = struct.pack('<4f', *torques)
        packet.extend(payload)
        checksum = self._compute_checksum(MSG_TORQUE, TORQUE_PAYLOAD_LENGTH, payload)
        packet.append(checksum)
        packet.append(END_BYTE)
        return bytes(packet)
    
    def _encode_joint_desired_packet(self, joint_desired):
        """
        Encode a joint desired command packet given 2 joint target values.
        Returns a bytes object.
        """
        packet = bytearray()
        packet.append(START_BYTE)
        packet.append(MSG_JOINT_DESIRED)
        packet.append(JOINT_DESIRED_PAYLOAD_LENGTH)
        payload = struct.pack('<2f', *joint_desired)
        packet.extend(payload)
        checksum = self._compute_checksum(MSG_JOINT_DESIRED, JOINT_DESIRED_PAYLOAD_LENGTH, payload)
        packet.append(checksum)
        packet.append(END_BYTE)
        return bytes(packet)

    def _feedback_loop(self):
        """
        Continuously read from the serial port.
        Reads the header (first 3 bytes) to determine the packet size,
        then reads the remaining bytes and decodes the packet using _decode_feedback_packet.
        Updates motor state (positions & velocities) and joint positions accordingly.
        """
        while not self._stop_event.is_set():
            # Read first 3 bytes: start byte, message type, payload length.
            header = self.ser.read(3)
            if len(header) != 3:
                continue
            if header[0] != START_BYTE:
                continue  # Discard if start byte is not found

            msg_type = header[1]
            payload_len = header[2]

            # Only handle MSG_FEEDBACK now.
            if msg_type != MSG_FEEDBACK:
                continue

            full_packet_size = FEEDBACK_PACKET_SIZE

            # Already read 3 bytes; read the remaining bytes.
            rest = self.ser.read(full_packet_size - 3)
            if len(rest) != (full_packet_size - 3):
                continue  # Incomplete packet, skip

            packet = header + rest
            decoded = self._decode_feedback_packet(packet)
            if decoded is None:
                continue

            motor_positions, motor_velocities, joint_positions = decoded
            with self._state_lock:
                self._motor_positions = motor_positions
                self._motor_velocities = motor_velocities
            with self._joint_state_lock:
                self._joint_positions = joint_positions

    def _torque_loop(self):
        """
        Periodically send the current torque command over the serial port at approximately 50Hz.
        """
        while not self._stop_event.is_set():
            with self._torque_lock:
                current_torques = self._torque_commands[:]
            packet = self._encode_torque_packet(current_torques)
            self.ser.write(packet)
            time.sleep(0.02)

    def _joint_desired_loop(self):
        """
        Periodically send the current joint desired command over the serial port at approximately 50Hz.
        """
        while not self._stop_event.is_set():
            with self._joint_desired_lock:
                current_joint_desired = self._joint_desired[:]
            packet = self._encode_joint_desired_packet(current_joint_desired)
            self.ser.write(packet)
            time.sleep(0.02)

    def get_motor_states(self):
        """
        Return the latest motor state:
        (motor_positions, motor_velocities) as tuples of 4 floats each.
        """
        with self._state_lock:
            return self._motor_positions, self._motor_velocities

    def get_joint_position(self):
        """
        Return the latest joint state (joint positions), consisting of 2 float values.
        """
        with self._joint_state_lock:
            return self._joint_positions

    def set_motor_torques(self, torques):
        """
        Update the torque command to be sent.
        Parameters:
            torques: list or tuple of 4 float values.
        """
        if len(torques) != 4:
            raise ValueError("Expected 4 torque values")
        with self._torque_lock:
            self._torque_commands = list(torques)

    def set_joint_desired(self, joint_desired):
        """
        Update the joint desired command to be sent.
        Parameters:
            joint_desired: list or tuple of 2 float values.
        """
        if len(joint_desired) != 2:
            raise ValueError("Expected 2 joint desired values")
        with self._joint_desired_lock:
            self._joint_desired = list(joint_desired)

    def close(self):
        """
        Stop all threads and close the serial port.
        """
        self._stop_event.set()
        self._feedback_thread.join(timeout=1)
        self._torque_thread.join(timeout=1)
        self._joint_desired_thread.join(timeout=1)
        self.ser.close()

# Example usage:
if __name__ == '__main__':
    msi = MotorSerialInterface(port='/dev/teensy4', baud_rate=115200)
    torques = [0.034, 0.0, 0.0, 0.0]
    joint_desired = [1.0, 1.0]  # Example joint desired values (2 elements)
    now = time.time()
    try:
        while True:
            motor_positions, motor_velocities = msi.get_motor_states()
            joint_positions = msi.get_joint_position()
            print("Motor Positions:", motor_positions)
            print("Motor Velocities:", motor_velocities)
            print("Joint Positions:", joint_positions)
            # Every 0.5 seconds, toggle the first motor torque and first joint desired value.
            if time.time() - now > 0.5:
                torques[0] = -torques[0]
                msi.set_motor_torques(torques)
                joint_desired[0] = -joint_desired[0]
                msi.set_joint_desired(joint_desired)
                now = time.time()
    except KeyboardInterrupt:
        msi.close()
        print("Interface closed.")
