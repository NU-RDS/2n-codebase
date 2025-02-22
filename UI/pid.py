class PIDController:
    def __init__(self, p: float, i: float, d: float, window_size: int, max_input: float):
        """
        Initialize the PID controller with specified gains and integral window size.
        
        :param p: Proportional gain
        :param i: Integral gain
        :param d: Derivative gain
        :param window_size: Maximum number of past error terms to keep for the integral calculation
        """
        self.p_gain = p
        self.i_gain = i
        self.d_gain = d
        self.window_size = window_size
        self.max_input = max_input
        self._position_state = None  # Current motor position (to be updated)
        self._velocity_state = None  # Current motor velocity (to be updated)
        self._integral_errors = []  # List to store recent errors for integral term
        self._last_error = None  # Last error used to compute derivative term

    def update_state(self, current_position: float, current_velocity: float) -> None:
        """
        Update the current state (motor position).
        
        :param current_position: The current position of the motor.
        """
        self._position_state = current_position
        self._velocity_state = current_velocity

    def get_input(self, target_position: float) -> float:
        """
        Calculate and return the control input (torque) for the motor based on the PID formula.
        
        :param target_position: The desired target position of the motor.
        :return: The computed control input (torque).
        """
        if self._position_state is None:
            raise ValueError("Motor state not initialized. Please call update_state() with a valid position first.")
        
        # Calculate the error between the target and current state.
        error = target_position - self._position_state
        
        # Proportional term.
        p_term = self.p_gain * error
        
        # Integral term.
        # Append the current error to the list.
        self._integral_errors.append(error)
        # Keep the list size within the defined window.
        if len(self._integral_errors) > self.window_size:
            self._integral_errors.pop(0)
        i_term = self.i_gain * sum(self._integral_errors)
        
        # Derivative term.
        if self._last_error is None:
            d_term = 0  # No derivative action on the first call.
        else:
            d_term = - self.d_gain * (self._last_error - error)
        self._last_error = error

        # Clamp the result to the maximum input value.
        result = p_term + i_term + d_term
        if abs(result) > self.max_input:
            result = self.max_input * abs(result) / result
        else:
            result = result

        return result
