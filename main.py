from motion_tracker import MotionTracker
from math import cos, sin

def main():
    # Define camera_intrinsics (focal length, principal point, etc.)
    camera_intrinsics = {
    'focal_length': (600,600),
    'principal_point': (360,360),
    #'distortion_coefficients': (k1, k2, p1, p2, k3)
    }

    # Define the actual size of the AprilTags (in meters)
    tag_size = (0.1,0.1)

    # Define the servo mounting configuration (e.g., offset, orientation, etc.)
    theta = 0.1
    servo_mounting = [[cos(theta), -sin(theta)],  # Transformation matrix
                  [sin(theta), cos(theta)]]

    camera_distance = 1.0

    motion_tracker = MotionTracker(camera_intrinsics, tag_size, servo_mounting, camera_distance)
    motion_tracker.run()

if __name__ == '__main__':
    main()

