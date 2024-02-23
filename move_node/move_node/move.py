import numpy as np

class SimpleRobotMover:
    class _State:
        IDLING = 0
        ROTATING_TO_TARGET = 1
        MOVING = 2
        ROTATING_FINAL = 3

    _v = 0.1  # Linear velocity
    _omega = 0.6 # Angular velocity
    _distance_tolerance = 0.1
    _angle_tolerance = 0.1  

    def __init__(self):
        """
        Initializes the SimpleRobotMover.

        Attributes:
            _target_pose (list): Target pose [x, y, theta].
            _state (int): Current state of the robot.
        """
        self._target_pose = None
        self._state = self._State.IDLING

    def set_target_pose(self, x, y, theta):
        """
        Sets the target pose and transitions the robot to the ROTATING_TO_TARGET state.

        Args:
            x (float): x-coordinate of the target.
            y (float): y-coordinate of the target.
            theta (float): Orientation of the target.
        """
        try:
            self._target_pose = [float(x), float(y), float(theta)]
            self._state = self._State.ROTATING_TO_TARGET
        except ValueError:
            self._target_pose = None
            self._state = self._State.IDLING
    
    def cancel_target(self):
        """
        Cancels the target and transitions the robot to the IDLING state.
        """
        self._target_pose = None
        self._state = self._State.IDLING

    def get_velocity_command(self, current_x, current_y, current_theta):
        """
        Calculates velocity commands based on the current state and position.

        Args:
            current_x (float): Current x-coordinate of the robot.
            current_y (float): Current y-coordinate of the robot.
            current_theta (float): Current orientation of the robot.

        Returns:
            list or None: Velocity commands [linear_velocity, angular_velocity] or None if no commands.
        """
        if current_x is None or current_y is None or current_theta is None:
            return None
        
        if self._state == self._State.IDLING:
             # Wenn der Roboter in einem Ruhezustand ist, gibt es keine Geschwindigkeitsanweisungen
            return None
        
        if current_x == self._target_pose[0] and current_y == self._target_pose[1] and current_theta == self._target_pose[2]:
             # Wenn der Roboter das Ziel erreicht hat, gibt es keine Geschwindigkeitsanweisungen
            return None    
        
        elif self._state == self._State.ROTATING_TO_TARGET:
              # Überprüfe, ob der Roboter auf das Ziel ausgerichtet ist
            if self._is_oriented(current_x, current_y, current_theta):
                # Wechsle in den Zustand "MOVING", wenn der Roboter ausgerichtet ist
                self._state = self._State.MOVING
                return self._move_cmd()
            
             # Gib Rotationsanweisungen zurück, um auf das Ziel ausgerichtet zu sein
            return self._get_rotation_cmd(current_x, current_y, current_theta, self._target_pose)

        elif self._state == self._State.MOVING:
             # Überprüfe, ob das Ziel vor dem Roboter liegt
            if self._is_in_front(current_x, current_y, current_theta):
                # Wechsle in den Zustand "ROTATING_FINAL", wenn das Ziel vor dem Roboter liegt
                self._state = self._State.ROTATING_FINAL
                return self._get_rotation_cmd(current_x, current_y, current_theta, self._target_pose)
            
            # Gib Geschwindigkeitsanweisungen zum Vorwärtsbewegen zurück
            return self._move_cmd()

        elif self._state == self._State.ROTATING_FINAL:
              # Überprüfe, ob der Roboter auf das finale Ziel ausgerichtet ist
            if self._is_oriented(current_x, current_y, current_theta):
                 # Wechsle in den Zustand "IDLING", wenn der Roboter ausgerichtet ist
                self._state = self._State.IDLING
                return None
            
              # Gib Rotationsanweisungen zurück, um auf das finale Ziel ausgerichtet zu sein
            return self._get_rotation_cmd(current_x, current_y, current_theta, self._target_pose)

    def _get_rotation_cmd(self, current_x, current_y, current_theta, target_pose):
        delta_x = target_pose[0] - current_x
        delta_y = target_pose[1] - current_y
        target_theta = np.arctan2(delta_y, delta_x)
        delta_theta = target_theta - current_theta

        delta_theta = np.arctan2(np.sin(delta_theta), np.cos(delta_theta))

        delta_theta_target = target_pose[2] - current_theta
        delta_theta_target = np.arctan2(np.sin(delta_theta_target), np.cos(delta_theta_target))
        
        if target_pose[2] != 0:
            if np.abs(delta_theta_target) > self._angle_tolerance:
                omega = np.sign(delta_theta_target) * self._omega
                return [0.0, omega]
            
        if target_pose[2] == 0:
            if np.abs(delta_theta) > self._angle_tolerance:
                omega = np.sign(delta_theta) * self._omega
                return [0.0, omega]

        

    def _move_cmd(self):
        """
        Returns velocity commands for forward movement.

        Returns:
            list: Velocity commands [linear_velocity, angular_velocity].
        """
        return [self._v, 0.0]  # Move forward with linear velocity _v

    def _is_oriented(self, current_x, current_y, current_theta):
        """
        Checks if the robot is oriented towards the target orientation.

        Args:
            current_x (float): Current x-coordinate of the robot.
            current_y (float): Current y-coordinate of the robot.
            current_theta (float): Current orientation of the robot.

        Returns:
            bool: True if the robot is oriented, False otherwise.
        """
        delta_x = self._target_pose[0] - current_x
        delta_y = self._target_pose[1] - current_y
        target_theta = np.arctan2(delta_y, delta_x)
        delta_theta = target_theta - current_theta

        # Ensure delta_theta is in the range [-pi, pi]
        delta_theta = np.arctan2(np.sin(delta_theta), np.cos(delta_theta))

        return np.abs(delta_theta) <= self._angle_tolerance

    def _is_in_front(self, current_x, current_y, current_theta):
        """
        Checks if the target is located in front of the robot.

        Args:
            current_x (float): Current x-coordinate of the robot.
            current_y (float): Current y-coordinate of the robot.
            current_theta (float): Current orientation of the robot.

        Returns:
            bool: True if the target is in front of the robot, False otherwise.
        """
        # Berechne die relative Position des Ziels in Bezug auf den Roboter
        delta_x = self._target_pose[0] - current_x
        delta_y = self._target_pose[1] - current_y

        # Berechne den Winkel zwischen der Ausrichtung des Roboters und der Zielposition
        target_theta = np.arctan2(delta_y, delta_x)
        delta_theta = target_theta - current_theta

        # Stelle sicher, dass delta_theta im Bereich [-pi, pi] liegt
        delta_theta = np.arctan2(np.sin(delta_theta), np.cos(delta_theta))

        # Überprüfe, ob das Ziel im Frontalbereich des Roboters liegt (innerhalb des ±pi/2-Winkels)
        return np.abs(delta_theta) <= np.pi / 2
