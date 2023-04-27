import numpy as np
import cv2

class KalmanTracker:
    def __init__(self):
        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.transitionMatrix = np.array([[1, 0, 1, 0],
                                             [0, 1, 0, 1],
                                             [0, 0, 1, 0],
                                             [0, 0, 0, 1]], np.float32)
        self.kf.measurementMatrix = np.array([[1, 0, 0, 0],
                                              [0, 1, 0, 0]], np.float32)
        self.kf.processNoiseCov = np.array([[1, 0, 0, 0],
                                            [0, 1, 0, 0],
                                            [0, 0, 1, 0],
                                            [0, 0, 0, 1]], np.float32) * 1e-2
        self.kf.measurementNoiseCov = np.array([[1, 0],
                                                [0, 1]], np.float32) * 1e-4
        self.kf.errorCovPost = np.eye(4, dtype=np.float32)
        self.kf.statePost = np.zeros(4, dtype=np.float32)

    def predict(self):
        """
        Predicts the next state of the object being tracked.
        Returns: The predicted state as a numpy array.
        """
        return self.kf.predict()

    def correct(self, measurement):
        """
        Corrects the predicted state based on the actual measurement.
        Args:
            measurement: A numpy array representing the actual measurement.
        Returns: The corrected state as a numpy array.
        """
        return self.kf.correct(measurement)

