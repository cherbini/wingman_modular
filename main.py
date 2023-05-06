from motion_tracker import MotionTracker
from math import cos, sin

def main():
    # Define camera_intrinsics (focal length, principal point, etc.)
    nnPath = "models/yolo-v4-tiny-tf_openvino_2021.4_6shave.blob"  # Set the correct path to the YOLO model blob file
    #nnPath = "models/yolo-v3-tiny-tf_openvino_2021.4_6shave.blob"  # Set the correct path to the YOLO model blob file
        # Create a motion_tracker instance to access the video frame dimensions

    # Given values
    focal_length_mm = 4.74
    pixel_size_mm = 0.0008
    
    # Calculate focal length in pixels for both x and y dimensions
    focal_length_x_pixels = focal_length_mm / pixel_size_mm
    focal_length_y_pixels = focal_length_mm / pixel_size_mm
    
    camera_intrinsics = {
        'focal_length': (focal_length_x_pixels, focal_length_y_pixels),
        'principal_point': (213,213),
        #'distortion_coefficients': (k1, k2, p1, p2, k3)
    }
    # Define the actual size of the AprilTags (in meters)
    tag_size = (0.1,0.1)

    # Define the servo mounting configuration (e.g., offset, orientation, etc.)
    theta = 1
    servo_mounting = [[cos(theta), -sin(theta)],  # Transformation matrix
                  [sin(theta), cos(theta)]]

    camera_distance = 1.0

    motion_tracker = MotionTracker(camera_intrinsics, tag_size, servo_mounting, camera_distance, nnPath)
    motion_tracker.run()

if __name__ == '__main__':
    main()

