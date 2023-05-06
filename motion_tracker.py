#!/usr/bin/env python3
from pathlib import Path
import numpy as np
import cv2
import psutil
import depthai as dai
import time
from dynamixel_control import DynamixelController
from coordinate_systems import CoordinateSystems
from kalman_tracker import KalmanTracker
from utils import (
    get_roi_area,
    limit_lead_distance,
    get_interception_point,
    get_center,
    process_motion_data
)

labelMap = ["person"]

class MotionTracker:
    #WINDOW_WIDTH = 416  # Width of the video frame
    #WINDOW_HEIGHT = 416  # Height of the video frame
    syncNN = True

    def __init__(self, camera_intrinsics, tag_size, servo_mounting, camera_distance, nnPathInput, archive_size=5):
        import sys
        nnPath = str((Path(__file__).parent / Path('models/yolo-v4-tiny-tf_openvino_2021.4_6shave.blob')).resolve().absolute())
        if 1 < len(sys.argv):
            arg = sys.argv[1]
            if arg == "yolo3":
                nnPath = str((Path(__file__).parent / Path('models/yolo-v3-tiny-tf_openvino_2021.4_6shave.blob')).resolve().absolute())
            elif arg == "yolo4":
                nnPath = str((Path(__file__).parent / Path('models/yolo-v4-tiny-tf_openvino_2021.4_6shave.blob')).resolve().absolute())
            else:
                nnPath = arg
        else:
            print("Using Tiny YoloV4 model. If you wish to use Tiny YOLOv3, call 'tiny_yolo.py yolo3'")
        
        if not Path(nnPath).exists():
            import sys
            raise FileNotFoundError(f'Required file/s not found, please run "{sys.executable} install_requirements.py"')
        
        # tiny yolo v4 label texts
        labelMap = [ "person" ]
        self.prev_rois = []
        self.decay_time = 2
        self.decay_info = []
        self.interception_points = []
        self.frame_number = 0
        self.WINDOW_WIDTH = 416
        self.WINDOW_HEIGHT = 416
        self.archive_size = 5
        self.archive = []
        self.kf = KalmanTracker()
        self.camera_intrinsics = camera_intrinsics
        self.tag_size = tag_size
        self.servo_mounting = servo_mounting
        self.archive_size = archive_size
        self.coordinate_systems = CoordinateSystems(camera_intrinsics, tag_size, servo_mounting)
        self.no_detection_counter = 0  # Counter for consecutive frames with no detections
        self.return_threshold = 3 * 40  # Threshold for returning to starting position (3 seconds * 40 FPS)
        self.starting_pan_position = 2048  # Starting position for PAN servo (adjust as needed)
        self.starting_tilt_position = 2048  # Starting position for TILT servo (adjust as needed)


        # Initialize DynamixelController (device port, baud rate, pan servo ID, tilt servo ID)
        self.dynamixel_controller = DynamixelController("/dev/ttyDXL", 1000000, 1, 2) 
        self.camera_distance = camera_distance # Initialize camera_distance attribute 
        self.dynamixel_controller.servo_test() 
        self.nnPath = nnPathInput 
        self.pipeline, self.device = self.setup_pipeline(nnPath)
        self.prev_frame = None
        self.prev_roi = None
        self.lead_time = 5.0
        self.kalman_tracker = KalmanTracker()

    def setup_pipeline(self, nnPath, confidence_threshold=0.2):
        p = dai.Pipeline()
        p.setOpenVINOVersion(dai.OpenVINO.VERSION_2021_4)
    
        camRgb = p.create(dai.node.ColorCamera)
        detectionNetwork = p.create(dai.node.YoloDetectionNetwork)
        xoutRgb = p.create(dai.node.XLinkOut)
        nnOut = p.create(dai.node.XLinkOut)
    
        xoutRgb.setStreamName("rgb")
        nnOut.setStreamName("nn")
    
        # Properties
        camRgb.setPreviewSize(416, 416)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        camRgb.setFps(40)
    
        # Network specific settings
        detectionNetwork.setNumClasses(80)
        detectionNetwork.setCoordinateSize(4)
        detectionNetwork.setAnchors([10])
        detectionNetwork.setAnchorMasks({"side26": [1, 2, 3], "side13": [3, 4, 5]})
        detectionNetwork.setIouThreshold(0.2)
        detectionNetwork.setBlobPath(nnPath)
        detectionNetwork.setNumInferenceThreads(2)
        detectionNetwork.input.setBlocking(False)
    
        # Linking
        camRgb.preview.link(detectionNetwork.input)
        if self.syncNN:
            detectionNetwork.passthrough.link(xoutRgb.input)
        else:
            camRgb.preview.link(xoutRgb.input)
    
        detectionNetwork.out.link(nnOut.input)
    
        device = dai.Device(p)
        return p, device
    
    def get_frame(self, data, shape):
        diff = np.array(data.getFirstLayerFp16()).reshape(shape)
        colorize = cv2.normalize(diff, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
        return cv2.applyColorMap(colorize, cv2.COLORMAP_JET)

    def process_frame(self, rois, video_frame, current_time):
        # Filter out invalid ROIs
        valid_rois = []
        for roi in rois:
            x, y, w, h = roi
            if w > 0 and h > 0 and x >= 0 and y >= 0 and x + w < self.WINDOW_WIDTH and y + h < self.WINDOW_HEIGHT:
                valid_rois.append(roi)
        rois = valid_rois

        # If no ROIs are detected, skip processing
        if len(rois) == 0:
            self.no_detection_counter += 1
            if self.no_detection_counter >= self.return_threshold:
                # Set the servos to return to their starting positions
                self.dynamixel_controller.set_goal_position(self.dynamixel_controller.PAN_SERVO_ID, self.starting_pan_position)
                self.dynamixel_controller.set_goal_position(self.dynamixel_controller.TILT_SERVO_ID, self.starting_tilt_position)
            return
        else:
            self.no_detection_counter = 0

        # Sort the ROIs by area in descending order and take the largest one (primary ROI)
        rois.sort(key=lambda roi: roi[2] * roi[3], reverse=True)
        largest_roi = rois[0]
    
        # Draw a green rectangle around the primary detected ROI
        x, y, w, h = largest_roi
        cv2.rectangle(video_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                # Get the center of the current ROI
        center_x, center_y = get_center(x, y, w, h)

        # Correct Kalman filter based on current measurement
        measurement = np.array([center_x, center_y], dtype=np.float32)
        corrected_state = self.kalman_tracker.correct(measurement)

        # Predict the next state using the Kalman filter
        predicted_state = self.kalman_tracker.predict()

        # Get the current positions of the PAN and TILT servos
        pan_present_position, tilt_present_position = self.dynamixel_controller.get_present_position()

        # Calculate the interception point using the predicted position from the Kalman filter
        interception_point_pan, interception_point_tilt = get_interception_point(
            pan_present_position, tilt_present_position, predicted_state[2], predicted_state[3], self.lead_time
        )

        # Set the new goal position for the PAN and TILT servos
        self.dynamixel_controller.set_goal_position(self.dynamixel_controller.PAN_SERVO_ID, int(interception_point_pan))
        self.dynamixel_controller.set_goal_position(self.dynamixel_controller.TILT_SERVO_ID, int(interception_point_tilt))

        # Draw the yellow detection point (center of primary ROI)
        cv2.circle(video_frame, (int(center_x), int(center_y)), 5, (0, 255, 255), -1)

        # Draw the green dot for the interception point
        cv2.circle(video_frame, (int(interception_point_pan), int(tilt_present_position - interception_point_tilt)), 5, (0, 255, 0), -1)

        # Update previous ROI
        self.prev_roi = largest_roi
    
        # Store the current video frame for the next iteration
        if video_frame is not None:
            self.prev_frame = video_frame.copy()
        else:
            self.prev_frame = None 

    def run(self):
        qNn = self.device.getOutputQueue(name="nn", maxSize=4, blocking=False)
        qCam = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    
        video_frame = None  # Initialize video_frame to None
    
        # Create window and sliders for Kalman filter
        cv2.namedWindow("Kalman Filter Settings")
        cv2.createTrackbar("Process Noise Cov", "Kalman Filter Settings", 1, 100, lambda x: None)
        cv2.createTrackbar("Measurement Noise Cov", "Kalman Filter Settings", 1, 100, lambda x: None)
        cv2.createTrackbar("Error Cov Post", "Kalman Filter Settings", 1, 100, lambda x: None)
    
        # Divider
        cv2.createTrackbar("----- Divider ------", "Kalman Filter Settings", 0, 1, lambda x: None)
    
        # Slider for network confidence threshold
        cv2.createTrackbar("Confidence Threshold", "Kalman Filter Settings", 20, 100, lambda x: None)
        # Divider
        cv2.createTrackbar("----- Divider ------", "Kalman Filter Settings", 0, 1, lambda x: None)
        cv2.createTrackbar("Threshold", "Kalman Filter Settings", 80, 100, lambda x: None)  # Initial value 0.8 (scaled by 100)
        cv2.createTrackbar("Min Side Length", "Kalman Filter Settings", 25, 100, lambda x: None)

    
        while True:
            inRgb = qCam.tryGet()
            inDet = qNn.tryGet()
    
            # Update Kalman filter parameters based on slider positions
            process_noise = cv2.getTrackbarPos("Process Noise Cov", "Kalman Filter Settings") / 10
            measurement_noise = cv2.getTrackbarPos("Measurement Noise Cov", "Kalman Filter Settings") / 10
            error_cov_post = cv2.getTrackbarPos("Error Cov Post", "Kalman Filter Settings") / 10
            self.kf.update_parameters(process_noise, measurement_noise, error_cov_post)
    
            # Update confidence threshold based on slider position
            confidence_threshold = cv2.getTrackbarPos("Confidence Threshold", "Kalman Filter Settings") / 100
    
            if inRgb is not None:
                video_frame = inRgb.getCvFrame()
                video_frame = cv2.flip(video_frame, -1)  # Flip horizontally
    
            rois = []  # Initialize empty list for ROIs
            if inDet is not None:
                # Filter detections based on updated threshold
                detections = [det for det in inDet.detections if det.confidence >= confidence_threshold]

                # Convert relative detection coordinates to image coordinates
                rois = []
                for detection in detections:
                    if detection.label == 0:

                        if np.isnan(detection.xmin) or np.isnan(detection.ymin) or np.isnan(detection.xmax) or np.isnan(detection.ymax) or \
                           np.isinf(detection.xmin) or np.isinf(detection.ymin) or np.isinf(detection.xmax) or np.isinf(detection.ymax):
                            continue
                        print("Relative Coordinates (xmin, ymin, xmax, ymax):", detection.xmin, detection.ymin, detection.xmax, detection.ymax)

                        xmin = int(detection.xmin * self.WINDOW_WIDTH)
                        ymin = int(detection.ymin * self.WINDOW_HEIGHT)
                        xmax = int(detection.xmax * self.WINDOW_WIDTH)
                        ymax = int(detection.ymax * self.WINDOW_HEIGHT)

                        # Adjust for flipped frame
                        flipped_xmin = self.WINDOW_WIDTH - xmax
                        flipped_xmax = self.WINDOW_WIDTH - xmin

                        rois.append((flipped_xmin, ymin, flipped_xmax - flipped_xmin, ymax - ymin))

                print("Calculated ROIs:", rois)

                self.process_frame(rois, video_frame, time.time())

    
            if video_frame is not None:  # Check whether video_frame is not None before calling cv2.imshow
                cv2.imshow("Color", video_frame)
                cv2.moveWindow("Color", self.WINDOW_WIDTH , 0)  # Move inside the if condition
    
            self.frame_number += 1
    
            key = cv2.waitKey(1)
            if key == 27 or key == ord("q"):
                break
    
        cv2.destroyAllWindows()
        self.device.close()
        self.dynamixel_controller.close()

def main():
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

    camera_distance = 2.0

    motion_tracker = MotionTracker(camera_intrinsics, tag_size, servo_mounting, camera_distance, nnPath)
    motion_tracker.run()

if __name__ == '__main__':
    main()

