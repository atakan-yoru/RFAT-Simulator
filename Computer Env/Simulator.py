import numpy as np
from scipy.spatial.transform import Rotation as R
# import imageio


class Tee:
    def __init__(self, *streams):
        self.streams = streams

    def write(self, data):
        for s in self.streams:
            s.write(data)
            s.flush()

    def flush(self):
        for s in self.streams:
            s.flush()

class PathUtils:
    @staticmethod
    def spherical_to_cartesian(sph):
        """
        Converts spherical coordinates [rho, azimuth, elevation] to Cartesian [x, y, z].
        Conventions:
          - ρ: radial distance
          - azimuth: angle in the xy–plane from the x–axis (in radians)
          - elevation: angle above the xy–plane (in radians)
        """
        rho, az, elev = sph
        x = rho * np.cos(elev) * np.cos(az)
        y = rho * np.cos(elev) * np.sin(az)
        z = rho * np.sin(elev)
        return np.array([x, y, z])

    @staticmethod
    def cartesian_to_spherical(cart):
        """
        Converts Cartesian coordinates [x, y, z] to spherical [rho, azimuth, elevation].
        """
        x, y, z = cart
        rho = np.linalg.norm(cart)
        az = np.arctan2(y, x)
        elev = np.arctan2(z, np.sqrt(x**2 + y**2))
        return np.array([rho, az, elev])


# --- Class to represent a single path segment ---

class PathSegment:
    def __init__(self, movement_type, start, end, dt=1.0,
                 linear_speed=None, angular_speed=None, rho_speed=None):
        """
        Parameters for a segment:
          - movement_type: "linear" or "spherical"
          
        For linear segments:
          - start, end: Cartesian coordinates [x, y, z]
          - linear_speed: distance moved per update
          
        For spherical segments:
          - start, end: spherical coordinates [ρ, azimuth, elevation] (angles in radians)
          - angular_speed: angular increment (radians per update) applied to both azimuth and elevation
          - rho_speed: radial speed (increment per update) for ρ
        """
        self.dt = dt
        self.movement_type = movement_type.lower()
        if self.movement_type == "linear":
            if start is None or end is None or linear_speed is None:
                raise ValueError("For linear movement, provide start, end, and linear_speed.")
            self.start = np.array(start, dtype=float)
            self.end = np.array(end, dtype=float)
            self.linear_speed = linear_speed * dt
            # Internal state always in Cartesian coordinates.
            self.current = self.start.copy()
        elif self.movement_type == "spherical":
            if start is None or end is None or angular_speed is None or rho_speed is None:
                raise ValueError("For spherical movement, provide start, end, angular_speed, and rho_speed.")
            self.start = np.array(start, dtype=float)  # [ρ, azimuth, elevation]
            self.end = np.array(end, dtype=float)
            self.angular_speed = angular_speed * dt
            self.rho_speed = rho_speed * dt
            # Internal state in spherical coordinates.
            self.current = self.start.copy()
        else:
            raise ValueError("Unknown movement type. Use 'linear' or 'spherical'.")

    def move(self):
        """Advances the current segment by one update step."""
        if self.movement_type == "linear":
            direction = self.end - self.current
            distance = np.linalg.norm(direction)
            if distance <= self.linear_speed:
                self.current = self.end.copy()
            else:
                self.current += (direction / distance) * self.linear_speed

        elif self.movement_type == "spherical":
            # Update angular components: indices 1 (azimuth) and 2 (elevation)
            for idx in [1, 2]:
                angle_diff = self.end[idx] - self.current[idx]
                # Normalize the difference to be within -pi to pi
                angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))
                if abs(angle_diff) <= self.angular_speed:
                    self.current[idx] = self.end[idx]
                else:
                    self.current[idx] += np.sign(angle_diff) * self.angular_speed

            # Update the radial component (index 0)
            rho_diff = self.end[0] - self.current[0]
            if abs(rho_diff) <= self.rho_speed:
                self.current[0] = self.end[0]
            else:
                self.current[0] += np.sign(rho_diff) * self.rho_speed

        else:
            raise ValueError("Invalid movement type in PathSegment.")

    def is_complete(self, tol=1e-6):
        """Checks if the segment has reached its end within a tolerance."""
        if self.movement_type == "linear":
            return np.linalg.norm(self.current - self.end) < tol
        elif self.movement_type == "spherical":
            return np.all(np.abs(self.current - self.end) < tol)

    def get_cartesian(self):
        """Returns the current position of the segment in Cartesian coordinates."""
        if self.movement_type == "linear":
            return self.current
        elif self.movement_type == "spherical":
            return PathUtils.spherical_to_cartesian(self.current)

# --- Target class that chains segments together ---

class Target:
    def __init__(self, segments):
        """
        Parameters:
          - segments: a list of PathSegment objects.
          
        This constructor also adjusts the start point of each segment (except the first)
        so that it matches the previous segment's end point, converting between coordinate
        systems if necessary.
        """
        if not segments:
            raise ValueError("At least one segment must be provided.")
        self.segments = segments
        self.current_segment_index = 0

        # Adjust boundaries: for each segment after the first, override its start
        # with the end of the previous segment, converting to the appropriate coordinate system.
        for i in range(1, len(self.segments)):
            prev = self.segments[i-1]
            curr = self.segments[i]
            # Get the end of the previous segment in Cartesian coordinates.
            if prev.movement_type == "linear":
                prev_end_cart = prev.end
            else:
                prev_end_cart = PathUtils.spherical_to_cartesian(prev.end)
            # Adjust current segment start based on its movement type.
            if curr.movement_type == "linear":
                curr.start = prev_end_cart
                curr.current = prev_end_cart.copy()
            elif curr.movement_type == "spherical":
                # Convert the Cartesian value to spherical.
                curr.start = PathUtils.cartesian_to_spherical(prev_end_cart)
                curr.current = curr.start.copy()
            else:
                raise ValueError("Unknown movement type in segment.")

        # Set overall position.
        self.position = self.segments[0].get_cartesian()
        self.history = [self.position.copy()] # Store history of positions


    def current_segment(self):
        return self.segments[self.current_segment_index]

    def move(self):
        """
        Moves the current segment one update step. If the segment completes,
        the Target transitions to the next segment (if any).
        """
        seg = self.current_segment()
        seg.move()
        if seg.is_complete():
            # Transition to the next segment if available.
            if self.current_segment_index < len(self.segments) - 1:
                self.current_segment_index += 1
                seg = self.current_segment()
        self.position = seg.get_cartesian()
        self.history.append(self.position.copy())

class OrientationUtils:
    @staticmethod
    def quat_to_vector(obj):
        """Converts quaternion orientation to a unit direction vector."""
        if hasattr(obj, 'orientation'):
            rotation = R.from_quat(obj.orientation)
            if hasattr(obj, 'length'): # Check if antenna has a length attribute
                return rotation.apply([0, 0, obj.length])  # Forward direction based on length
            return rotation.apply([0, 0, 1])  # Default forward direction
        else:
            raise ValueError("Object does not have an orientation attribute.")
    @staticmethod
    def get_local_azimuth_elevation_vectors(obj):
        """
        Returns the local azimuth (right) and elevation (up) unit vectors for a plane,
        computed from the plane's .orientation quaternion. This gives the effective 
        rotation axes relative to the standard (unrotated) configuration:
        
          Standard:
            - Forward (normal) = [0, 0, 1]
            - Azimuth (right)  = [1, 0, 0]
            - Elevation (up)   = [0, 1, 0]
        
        The returned vectors are the rotated versions of [1, 0, 0] and [0, 1, 0].
        """
        rotation = R.from_quat(obj.orientation)
        azimuth_vector = rotation.apply([1.0, 0.0, 0.0])
        elevation_vector = rotation.apply([0.0, 1.0, 0.0])
        azimuth_vector /= np.linalg.norm(azimuth_vector)
        elevation_vector /= np.linalg.norm(elevation_vector)
        return azimuth_vector, elevation_vector
    

class Antenna:
    def __init__(self, position=None, orientation=None, relative_position=None, relative_orientation=None, antenna_plane=None, length=1, beamwidth=60):
        if position is not None and orientation is not None:
            self.position = np.array(position)
            self.orientation = R.from_euler('xyz', orientation, degrees=True).as_quat()
        elif relative_position is not None and relative_orientation is not None and antenna_plane is not None:
            self.antenna_plane = antenna_plane
            self.relative_position = np.array(relative_position)
            self.relative_orientation = np.array(relative_orientation)
            self.position = antenna_plane.position + self.relative_position  # First apply translation
            rotation = R.from_quat(antenna_plane.orientation)  # Convert antenna plane orientation to rotation
            self.orientation = (rotation * R.from_rotvec(relative_orientation)).as_quat()  # Then apply rotation
        else:
            raise ValueError("Either provide global position and orientation or relative position and orientation with an antenna plane.")
        self.length = length
        self.tip_point = OrientationUtils.quat_to_vector(self) + self.position
        self.signal_strength = 0
        self.beamwidth = beamwidth

    def update_global_transform(self):
        """Updates global position and orientation based on antenna plane transformation."""
        rotation = R.from_quat(self.antenna_plane.orientation)
        self.position = self.antenna_plane.position + rotation.apply(self.relative_position)
        self.orientation = (rotation * R.from_rotvec(self.relative_orientation)).as_quat()
        self.tip_point = OrientationUtils.quat_to_vector(self) + self.position

    def compute_gain(antenna, target, beamwidth_deg):
        """
        Computes the gain factor for an antenna using a raised cosine power approximation.
        
        The gain pattern is approximated by:
            gain = cos(theta)^n,
        where theta is the error angle between the antenna's boresight and the direction
        to the target. The exponent n is computed such that the gain is 0.5 at theta = beamwidth/2.
        """
        # Get the antenna's boresight (forward) direction as a unit vector.
        boresight = OrientationUtils.quat_to_vector(antenna)
        boresight = boresight / np.linalg.norm(boresight)
        
        # Compute the unit vector from the antenna's position to the target.
        vec_to_target = target.position - antenna.position
        vec_to_target /= np.linalg.norm(vec_to_target)
        
        # Compute the angle between the boresight and the target direction.
        cos_theta = np.clip(np.dot(boresight, vec_to_target), -1.0, 1.0)
        theta = np.arccos(cos_theta)  # in radians
        error_angle_deg = np.degrees(theta)
        
        # Convert beamwidth to radians.
        beamwidth_rad = np.deg2rad(beamwidth_deg)
        n = np.log(0.5) / np.log(np.cos(beamwidth_rad / 2))
        gain = (cos_theta ** n) if abs(error_angle_deg) < 90 else 0.0

        # print(f"Gain: {gain:.5f}, Error Angle: {error_angle_deg:.2f}°")
        
        return error_angle_deg, gain

    def receive_signal(self, target):
        """Computes received signal strength based on inverse distance."""
        distance = np.linalg.norm(target.position  - self.tip_point)
        error_angle, gain = self.compute_gain(target, self.beamwidth)
        self.signal_strength = gain * 1 / distance if distance != 0 else 1
        return self.signal_strength

    def signal_vector(self):
        """Computes the signal strength vector using quaternion rotation."""
        return self.signal_strength * OrientationUtils.quat_to_vector(self)

class AntennaPlane:
    def __init__(self, position, orientation):
        self.position = np.array(position, dtype=np.float64)
        self.orientation = R.from_euler('xyz', orientation, degrees=True).as_quat()
        self.antennas = []
        self.origin = self.position
        self.current_azimuth = 0.0
        self.current_elevation = np.deg2rad(90.0)

    def add_antenna(self, antenna):
        """Adds an antenna to the plane."""
        self.antennas.append(antenna)

    def compute_normal(self):
        """Computes the normal to the antenna plane using quaternion transformations."""
        return np.array(OrientationUtils.quat_to_vector(self), dtype=np.float64)
    
    def set_initial_orientation(self, desired_orientation):
        """
        Sets the initial orientation of the antenna plane to a desired pose.
        Updates all attached antennas and local azimuth/elevation vectors.

        Parameters:
        - desired_orientation: A list of Euler angles [roll, pitch, yaw] in degrees.
        """
        # Update the plane's orientation
        self.orientation = R.from_euler('xyz', desired_orientation, degrees=True).as_quat()

        # Update the transformations for all attached antennas
        for antenna in self.antennas:
            antenna.update_global_transform()

        # Update local azimuth and elevation vectors
        azimuth_vector, elevation_vector = OrientationUtils.get_local_azimuth_elevation_vectors(self)
        print("Updated Azimuth Vector:", azimuth_vector)
        print("Updated Elevation Vector:", elevation_vector)

    def update_orientation(self, new_azimuth_angle, new_elevation_angle):
        """
        Updates the antenna plane's orientation using motor angles.
        
        Parameters:
        - new_azimuth_angle: The target azimuth angle (yaw) about the global Z-axis.
        - new_elevation_angle: The target elevation angle (pitch) about the plane's local azimuth axis.
        
        Instead of applying the absolute new angles, the function computes the differences 
        (delta angles) relative to the previously stored target angles and applies only those differences.
        """
        # Compute the difference between the new and current angles.
        delta_azimuth = new_azimuth_angle - self.current_azimuth
        delta_elevation = new_elevation_angle - self.current_elevation

        # Update the stored target angles.
        self.current_azimuth = new_azimuth_angle
        self.current_elevation = new_elevation_angle

        # Step 1: Create the relative azimuth (yaw) rotation about the global Z-axis.
        azimuth_rotation = R.from_euler('z', delta_azimuth, degrees=False)

        # Step 2: Get the current local azimuth and elevation unit vectors from the plane's orientation.
        local_azimuth_vector, _ = OrientationUtils.get_local_azimuth_elevation_vectors(self)
        
        # Check for degenerate case: if the plane's forward (normal) vector is nearly [0, 0, 1],
        # the computed local azimuth vector might be ambiguous.
        # In that case, rotate the standard [1, 0, 0] vector by the delta azimuth to get the effective azimuth direction.
        forward = OrientationUtils.quat_to_vector(self)
        if np.linalg.norm(forward - np.array([0.0, 0.0, 1.0])) < 1e-6:
            local_azimuth_vector = azimuth_rotation.apply(np.array([1.0, 0.0, 0.0]))

        # Step 3: Create the relative elevation (pitch) rotation about the computed azimuth vector.
        elevation_rotation = R.from_rotvec(-delta_elevation * local_azimuth_vector)

        # Step 4: Combine the relative rotations with the current orientation.
        new_rotation = azimuth_rotation * elevation_rotation * R.from_quat(self.orientation)
        
        # Update the antenna plane's orientation.
        self.orientation = new_rotation.as_quat()

        # Update the transformations for all attached antennas.
        for antenna in self.antennas:
            antenna.update_global_transform()

class DecisionSubsystem:
    def __init__(self, antenna_plane):
        self.antenna_plane = antenna_plane
        self.total_vector = None
        self.projection = None
        self.norm = None

    def update_antenna_plane(self, antenna_plane):
        """Updates the antenna plane reference."""
        self.antenna_plane = antenna_plane

    def compute_direction(self):
        """Estimates the target's direction based on signal vectors. 3D vector averaging is used."""
        self.total_vector = sum(antenna.signal_vector() for antenna in self.antenna_plane.antennas)
        return self.total_vector  # No normalization to keep raw values
        
    def compute_direction_projection(self):
        """Projects the estimated direction onto the antenna plane without normalizing."""
        estimated_direction = self.compute_direction()
        normal = self.antenna_plane.compute_normal()
        
        # Project the estimated direction onto the plane
        self.projection = estimated_direction - np.dot(estimated_direction, normal) * normal

        return self.projection  # Returning the raw projection without normalization

    def decompose_direction(self):
        """
        Decomposes the projected direction into azimuth (right-left) and elevation (up-down) 
        components based on the antenna plane's local coordinate axes.
        
        Returns:
        - azimuth_magnitude: The component in the right-left direction (along the plane's local azimuth vector).
        - elevation_magnitude: The component in the up-down direction (along the plane's local elevation vector).
        """
        if self.projection is None:
            self.compute_direction_projection()  # Ensure projection is calculated

        # Retrieve the local azimuth (right) and elevation (up) unit vectors from the antenna plane's orientation.
        azimuth_vector, elevation_vector = OrientationUtils.get_local_azimuth_elevation_vectors(self.antenna_plane)
        print("Azimuth Vector: ", azimuth_vector)
        print("Elevation Vector: ", elevation_vector)

        # Compute the signal components along the local azimuth and elevation axes.

        azimuth_magnitude = np.dot(self.projection, azimuth_vector)
        elevation_magnitude = np.dot(self.projection, elevation_vector)

        print("Azimuth Magnitude: ", azimuth_magnitude)
        print("Elevation Magnitude: ", elevation_magnitude)

        return azimuth_magnitude, elevation_magnitude

class PIDController:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.previous_error = 0
        self.integral = 0

    def compute_control(self, error):
        """Computes the control signal using PID logic."""

        self.integral += error 
        derivative = error - self.previous_error
        self.previous_error = error
        control_signal = self.kp * error + self.ki * self.integral + self.kd * derivative
        return control_signal if abs(control_signal) > 1e-6 else 0
    
class Motor:
    """Base class for motors with PID control"""
    def __init__(self, angle=0 ,w=0, controller=None):
        self.angle =  angle # Current position
        self.w = w # Current velocity
        self.controller = controller

    def wrap_angle(self, angle):
        return angle % (2 * np.pi)    

    def update_angle(self, new_angle):
        self.angle = self.wrap_angle(new_angle)
        
    def attach_controller(self, controller):
        self.controller = controller
    
    def attach_plane(self, plane):
        self.plane = plane
    
    def actuate(self, error, dt):
        """Applies control signal to the motor."""
        print("Error: ", error)
        print("Angle: ", np.degrees(self.angle))
        if self.controller:    
            control_signal = self.controller.compute_control(error)
            print("Control Signal: ", control_signal)
            self.w = control_signal
        updated_angle = self.angle + self.w * dt
        print("Updated Angle: ", np.degrees(updated_angle))
        self.update_angle(updated_angle)
        print("Angle: ", np.degrees(self.angle))
        return self.angle


class ActuationSubsystem:
    def __init__(self, antenna_plane, decision_system):
        """
        Initializes the Actuation Subsystem with independent PID controllers
        for azimuth (DC Motor) and elevation (Servo Motor).

        Parameters:
        - azimuth_kp, azimuth_ki, azimuth_kd: PID gains for azimuth (horizontal rotation).
        - elevation_kp, elevation_ki, elevation_kd: PID gains for elevation (vertical tilt).
        """
        self.antenna_plane = antenna_plane
        self.decision_system = decision_system

        # Independent PID controllers for azimuth and elevation
        self.azimuth_motor = Motor(angle=0, controller=PIDController(kp=1, ki=0.1, kd=0))
        self.elevation_motor = Motor(angle=np.pi/2, controller=PIDController(kp=1, ki=0.1, kd=0))
        print(f"Azimuth Motor: Angle = {np.degrees(self.azimuth_motor.angle):.2f}, Elev Motor: Angle = {np.degrees(self.elevation_motor.angle):.2f}")

    def compute_error_angle(self, target):
        """
        Computes the error angle between the antenna plane's normal and
        the vector pointing towards the target.
        """
        normal = self.antenna_plane.compute_normal()
        direction_to_target = target.position - self.antenna_plane.origin
        direction_to_target /= np.linalg.norm(direction_to_target)  # Normalize

        # Compute angle difference
        cos_theta = np.clip(np.dot(normal, direction_to_target), -1.0, 1.0)
        error_angle = np.degrees(np.arccos(cos_theta))
        return error_angle

    def actuate_system(self, dt=1):
        """
        Uses independent PID control for azimuth (DC motor) and elevation (Servo motor).
        """
        self.decision_system.compute_direction_projection()
        azimuth_error, elevation_error = self.decision_system.decompose_direction()
        print(f"Azimuth Error: {np.degrees(azimuth_error):.2f}, Elevation Error: {np.degrees(elevation_error):.2f}")
        # Apply PID control to motors
        self.azimuth_motor.actuate(azimuth_error, dt=dt)
        self.elevation_motor.actuate(elevation_error, dt=dt)
        print(f"Azimuth Motor: Angle = {np.degrees(self.azimuth_motor.angle):.2f}, Elev Motor: Angle = {np.degrees(self.elevation_motor.angle):.2f}")
        self.antenna_plane.update_orientation(self.azimuth_motor.angle, self.elevation_motor.angle)

        
