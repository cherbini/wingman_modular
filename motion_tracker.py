import numpy as np
import cv2
import depthai as dai
import time
from dynamixel_control import DynamixelController
from coordinate_systems import CoordinateSystems
from kalman_tracker import KalmanTracker
from utils import (
    get_roi_area,
    limit_lead_distance,
    get_interception_point,
    extract_regions_of_interest,
    get_center,
    process_motion_data
)

class MotionTracker:
    WINDOW_WIDTH = 720  # Width of the video frame
    WINDOW_HEIGHT = 720  # Height of the video frame

    def __init__(self, camera_intrinsics, tag_size, servo_mounting, camera_distance, archive_size=5):
        self.prev_rois = []
        self.decay_time = 3
        self.decay_info = []
        self.interception_points = []
        self.frame_number = 0
        self.WINDOW_WIDTH = 720
        self.WINDOW_HEIGHT = 720
        self.archive_size = 5
        self.archive = []
        self.kf = KalmanTracker()
        self.pipeline, self.device = self.setup_pipeline()
        self.camera_intrinsics = camera_intrinsics
        self.tag_size = tag_size
        self.servo_mounting = servo_mounting
        self.archive_size = archive_size
        self.coordinate_systems = CoordinateSystems(camera_intrinsics, tag_size, servo_mounting)

        # Initialize DynamixelController (device port, baud rate, pan servo ID, tilt servo ID)
        self.dynamixel_controller = DynamixelController("/dev/ttyUSB0", 1000000, 1, 2)
        self.camera_distance = camera_distance  # Initialize camera_distance attribute




    def setup_pipeline(self):
        p = dai.Pipeline()
        p.setOpenVINOVersion(dai.OpenVINO.VERSION_2021_4)

        camRgb = p.create(dai.node.ColorCamera)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setVideoSize(720, 720)
        camRgb.setPreviewSize(720, 720)
        camRgb.setInterleaved(False)

        nn = p.create(dai.node.NeuralNetwork)
        nn.setBlobPath("models/diff_openvino_2021.4_6shave.blob")

        script = p.create(dai.node.Script)
        camRgb.preview.link(script.inputs['in'])
        script.setScript("""
        old = node.io['in'].get()
        while True:
            frame = node.io['in'].get()
            node.io['img1'].send(old)
            node.io['img2'].send(frame)
            old = frame
        """)
        script.outputs['img1'].link(nn.inputs['img1'])
        script.outputs['img2'].link(nn.inputs['img2'])

        nn_xout = p.create(dai.node.XLinkOut)
        nn_xout.setStreamName("nn")
        nn.out.link(nn_xout.input)

        rgb_xout = p.create(dai.node.XLinkOut)
        rgb_xout.setStreamName("rgb")
        camRgb.video.link(rgb_xout.input)

        device = dai.Device(p)
        return p, device

    def get_frame(self, data, shape):
        diff = np.array(data.getFirstLayerFp16()).reshape(shape)
        colorize = cv2.normalize(diff, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
        return cv2.applyColorMap(colorize, cv2.COLORMAP_JET)

    def process_frame(self, diff_frame, video_frame, current_time):
        rois = extract_regions_of_interest(diff_frame)

        # Update the archive
        self.archive.append(rois)
        if len(self.archive) > self.archive_size:
            self.archive.pop(0)
    
        n_largest_rois = 3  # Change this value to consider more or fewer top ROIs
    
        # Estimate the center of the larger object
        larger_object_center = np.zeros(2)
        roi_count = 0
        for frame_rois in self.archive:
            sorted_rois = sorted(frame_rois, key=get_roi_area, reverse=True)[:n_largest_rois]
            for x, y, w, h in sorted_rois:
                larger_object_center += np.array([x + w / 2, y + h / 2])
                roi_count += 1

        # Draw the yellow detection point
        if roi_count > 0:
            larger_object_center /= roi_count
            cv2.circle(video_frame, tuple(larger_object_center.astype(int)), 5, (0, 255, 255), -1)
        
    
        # Process regions of interest and update decay_info
        self.decay_info = [(roi, *process_motion_data(prev_roi, roi, 720, 720), current_time) for prev_roi, roi in zip(self.prev_rois, rois) if prev_roi is not None]
        
        for roi, azimuth, velocity, direction, roi_time in self.decay_info:
            x, y, w, h = roi
            center = get_center(x, y, w, h)

            lead_time = 1  # Change this value to adjust the distance between the yellow and green dots
    
            # Get the interception point and limit its distance from the yellow dot
            interception_x, interception_y = get_interception_point(larger_object_center[0], larger_object_center[1], velocity, direction, lead_time)
            interception_x, interception_y = limit_lead_distance(larger_object_center[0], larger_object_center[1], interception_x, interception_y)
    
            # Update the interception point in the interception_points list
            self.interception_points.append((self.frame_number, interception_x, interception_y))
    
            # Draw the green dot for the interception point
            cv2.circle(video_frame, (int(interception_x), int(interception_y)), 5, (0, 255, 0), -1)
    
            # Calculate deviation from center of frame
            frame_center = np.array([self.WINDOW_WIDTH / 2, self.WINDOW_HEIGHT / 2])
            deviation = larger_object_center - frame_center
            
            # Convert deviation to servo coordinates
            servo_deviation = self.coordinate_systems.image_to_servo(deviation, self.camera_distance)

            
            # Update servo goal positions based on deviation
            pan_position = self.dynamixel_controller.get_present_position(self.dynamixel_controller.PAN_SERVO_ID)
            tilt_position = self.dynamixel_controller.get_present_position(self.dynamixel_controller.TILT_SERVO_ID)

            # Convert the values to integers
            new_pan_position = int(pan_position + servo_deviation[0])
            new_tilt_position = int(tilt_position + servo_deviation[1])
        
            self.dynamixel_controller.set_goal_position(self.dynamixel_controller.PAN_SERVO_ID, new_pan_position)
            self.dynamixel_controller.set_goal_position(self.dynamixel_controller.TILT_SERVO_ID, new_tilt_position)

        self.prev_rois = rois

    def servo_test(self):
        # Get the current position of the pan servo
        initial_position = self.dynamixel_controller.get_present_position(self.dynamixel_controller.PAN_SERVO_ID)
        print("servo test initial position" + str(initial_position))
    
        # Convert +/- 10 degrees to position values (ticks) based on the servo's resolution
        # The conversion factor depends on the servo model (e.g., 4096 ticks per 360 degrees for MX-28)
        ticks_per_degree = 4096 / 360
        offset = int(10 * ticks_per_degree)
    
        # Move the pan servo +10 degrees
        self.dynamixel_controller.set_goal_position(self.dynamixel_controller.PAN_SERVO_ID, initial_position + offset)
        time.sleep(1)  # Wait for 1 second
    
        # Move the pan servo -10 degrees
        self.dynamixel_controller.set_goal_position(self.dynamixel_controller.PAN_SERVO_ID, initial_position - offset)
        time.sleep(1)  # Wait for 1 second
    
        # Return the pan servo to its initial position
        self.dynamixel_controller.set_goal_position(self.dynamixel_controller.PAN_SERVO_ID, initial_position)
        time.sleep(1)  # Wait for 1 second

    def run(self):
        self.servo_test()
        qNn = self.device.getOutputQueue(name="nn", maxSize=4, blocking=False)
        qCam = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    
        while True:
            diff_frame = self.get_frame(qNn.get(), (720, 720))
            video_frame = qCam.get().getCvFrame()
            current_time = time.time()
    
            self.process_frame(diff_frame, video_frame, current_time)
    
            # Flip images horizontally
            diff_frame = cv2.flip(diff_frame, 1)
            video_frame = cv2.flip(video_frame, 1)
    
            cv2.imshow("Diff", diff_frame)
            cv2.imshow("Color", video_frame)
    
            # Move windows
            cv2.moveWindow("Diff", 0, 0)
            cv2.moveWindow("Color", self.WINDOW_WIDTH , 0)
    
            self.frame_number += 1
    
            key = cv2.waitKey(1)
            if key == 27 or key == ord("q"):
                break
    
        cv2.destroyAllWindows()
        self.device.close()
        self.dynamixel_controller.close()

if __name__ == "__main__":
    motion_tracker = MotionTracker()
    motion_tracker.run()

