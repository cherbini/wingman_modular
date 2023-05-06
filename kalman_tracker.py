import cv2
import numpy as np

class KalmanTracker:
    def __init__(self):
        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.transitionMatrix = np.array([[1, 0, 1, 0],
                                             [0, 1, 0, 1],
                                             [0, 0, 1, 0],
                                             [0, 0, 0, 1]], np.float32)
        self.kf.measurementMatrix = np.array([[1, 0, 0, 0],
                                              [0, 1, 0, 0]], np.float32)

        # Initialize default values
        self.update_parameters(10, 1, 10)

    def update_parameters(self, process_noise_scale, measurement_noise_scale, error_covariance_scale):
        # Adjust processNoiseCov values (Q)
        self.kf.processNoiseCov = np.array([[1, 0, 0, 0],
                                            [0, 1, 0, 0],
                                            [0, 0, 1, 0],
                                            [0, 0, 0, 1]], np.float32) * process_noise_scale * 1e-3

        # Adjust measurementNoiseCov values (R)
        self.kf.measurementNoiseCov = np.array([[1, 0],
                                                [0, 1]], np.float32) * measurement_noise_scale * 1e-4

        # Adjust initial errorCovPost values (P)
        self.kf.errorCovPost = np.eye(4, dtype=np.float32) * error_covariance_scale * 1e-2

    def predict(self):
        return self.kf.predict()

    def correct(self, measurement):
        return self.kf.correct(measurement)

