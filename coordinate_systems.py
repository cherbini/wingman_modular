import numpy as np

class CoordinateSystems:
    def __init__(self, camera_intrinsics, tag_size, servo_mounting):
        # Camera intrinsic parameters (focal length, principal point, etc.)
        self.camera_intrinsics = camera_intrinsics

        # Actual size of the AprilTags (width and height)
        self.tag_size = tag_size

        # Orientation and mounting of the servos relative to the camera
        self.servo_mounting = np.array(servo_mounting)

        # Define scale factors for servo movement
        self.servo_scale_x = 10# Adjust as needed
        self.servo_scale_y = 10# Adjust as needed

    def image_to_realworld(self, tag_center, camera_distance):
        fx, fy = self.camera_intrinsics['focal_length']
        cx, cy = self.camera_intrinsics['principal_point']

        # Convert image coordinates to real-world coordinates
        # based on camera intrinsic parameters
        x = (tag_center[0] - cx) * self.tag_size[0] / fx * camera_distance
        y = (tag_center[1] - cy) * self.tag_size[1] / fy * camera_distance
        return np.array([x, y])

    def realworld_to_servo(self, realworld_coords):
        # Map real-world coordinates to servo coordinate system
        # based on servo mounting and orientation
        servo_coords = np.dot(self.servo_mounting, realworld_coords)
        return servo_coords

    def image_to_servo(self, tag_center, camera_distance):
        # Convert image coordinates to servo coordinates
        realworld_coords = self.image_to_realworld(tag_center, camera_distance)
        servo_coords = self.realworld_to_servo(realworld_coords)

        # Scale the deviation by the scale factors
        servo_coords[0] *= self.servo_scale_x
        servo_coords[1] *= self.servo_scale_y

        return servo_coords

# Usage example
if __name__ == "__main__":
    # Sample camera intrinsic parameters
    camera_intrinsics = {
        'focal_length': (.52,.52),
        'principal_point': (360,360)
    }

    # Sample tag size (width, height) in meters
    tag_size = (0.1, 0.1)

    # Sample servo mounting matrix (identity matrix for this example)
    servo_mounting = np.eye(2)

    # Create CoordinateSystems instance
    coordinate_systems = CoordinateSystems(camera_intrinsics, tag_size, servo_mounting)

    # Sample tag center in image coordinates (pixels) and camera distance (meters)
    tag_center = (400, 300)
    camera_distance = 2.0

    # Convert image coordinates to servo coordinates
    servo_coords = coordinate_systems.image_to_servo(tag_center, camera_distance)
    print("Servo coordinates:", servo_coords)

