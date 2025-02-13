import serial
import struct
import time
import threading

# Protocol constants (must match Teensy's definitions)
START_BYTE = 0xAA
END_BYTE = 0xBB
MSG_FEEDBACK = 0x01
MSG_TORQUE = 0x02
FEEDBACK_PAYLOAD_LENGTH = 32
FEEDBACK_PACKET_SIZE = 37
TORQUE_PAYLOAD_LENGTH = 16
TORQUE_PACKET_SIZE = 21

class MotorSerialInterface:
    def __init__(self, port='/dev/ttyACM0', baud_rate=115200, timeout=1):
        self.ser = serial.Serial(port, baud_rate, timeout=timeout)
        # Give the serial port a moment to initialize
        time.sleep(2)
        
        # Initialize motor state (positions and velocities) and torque commands.
        self._motor_positions = (0.0, 0.0, 0.0, 0.0)
        self._motor_velocities = (0.0, 0.0, 0.0, 0.0)
        self._torque_commands = [0.0, 0.0, 0.0, 0.0]

        # Locks to ensure thread-safe access.
        self._state_lock = threading.Lock()
        self._torque_lock = threading.Lock()

        # Event to signal thread shutdown.
        self._stop_event = threading.Event()

        # Start threads: one for reading feedback, one for sending torque commands.
        self._feedback_thread = threading.Thread(target=self._feedback_loop, daemon=True)
        self._torque_thread = threading.Thread(target=self._torque_loop, daemon=True)
        self._feedback_thread.start()
        self._torque_thread.start()

    def _compute_checksum(self, msg_type, payload_len, payload):
        """Compute checksum as the modulo-256 sum of the message type, payload length, and payload bytes."""
        checksum = msg_type + payload_len
        for b in payload:
            checksum += b
        return checksum & 0xFF

    def _decode_feedback_packet(self, packet):
        """
        Validate and decode a feedback packet.
        Returns a tuple (positions, velocities) if valid, or None otherwise.
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
            # Unpack 8 float32 values: first 4 are positions, next 4 are velocities.
            values = struct.unpack('<8f', payload)
        except struct.error:
            return None

        positions = values[0:4]
        velocities = values[4:8]
        return positions, velocities

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

    def _feedback_loop(self):
        """Continuously reads from Serial and updates motor positions and velocities."""
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
                        self._motor_positions, self._motor_velocities = decoded

    def _torque_loop(self):
        """Periodically sends the current torque commands over Serial at approximately 50 Hz."""
        while not self._stop_event.is_set():
            with self._torque_lock:
                current_torques = self._torque_commands[:]
            packet = self._encode_torque_packet(current_torques)
            self.ser.write(packet)

    def get_motor_states(self):
        """
        Return the latest motor states.
        Returns:
            (positions, velocities): tuple of 4-element tuples of floats.
        """
        with self._state_lock:
            return self._motor_positions, self._motor_velocities

    def set_motor_torques(self, torques):
        """
        Update the torque commands to be sent.
        Parameters:
            torques: list or tuple of 4 float values.
        """
        if len(torques) != 4:
            raise ValueError("Expected 4 torque values")
        with self._torque_lock:
            self._torque_commands = list(torques)

    def close(self):
        """Stop the threads and close the serial port."""
        self._stop_event.set()
        self.ser.close()

# Example usage:
if __name__ == '__main__':
    msi = MotorSerialInterface(port='/dev/teensy4', baud_rate=115200)
    torques = [0.034, 0.0, 0.0, 0.0]
    now = time.time()
    try:
        while True:
            positions, velocities = msi.get_motor_states()
            print("Motor Positions:", positions)
            # print("Motor Velocities:", velocities)
            # For demonstration, toggle the first motor's torque every 1 seconds.
            if time.time() - now > 0.5:
                torques[0] = -torques[0]
                msi.set_motor_torques(torques)
                now = time.time()
    except KeyboardInterrupt:
        msi.close()
        print("Interface closed.")
