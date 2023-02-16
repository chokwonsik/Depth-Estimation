import cv2
import numpy as np
import time
import rospy
from yolov3_trt_ros.msg import BoundingBox, BoundingBoxes

class homography:
    def __init__(self):
        rospy.init_node("ppb_estimation")
        self.sub = rospy.Subscriber(
            "/yolov3_trt_ros/detections", BoundingBoxes, self.det_callback
        )
        self.bboxes = None
        self.get_homography()

    def det_callback(self, data):
        for bbox in data.bounding_boxes:
            obj_id = bbox.id
            obj_bbox = [bbox.xmin, bbox.xmax, bbox.ymin, bbox.ymax]
            score = bbox.probability
            estimated_distance = self.estimate_distance(obj_bbox)

    def get_rtvec(self):
        self.camera_matrix = np.array(
            [
                [357.906682, 0.000000, 332.457325],
                [0.000000, 356.170785, 253.010996],
                [0.000000, 0.000000, 1.000000]
            ], dtype=np.float64
        )

        self.distcoeffs = np.array(
            [-0.274053, 0.049790, -0.001539, -0.003057, 0.000000], dtype=np.float64
        )

        self.retval, self.rvec, self.tvec = cv2.solvePnP(
            self.object_points,
            self.image_points,
            self.camera_matrix,
            distCoeffs=self.distcoeffs,
            useExtrinsicGuess=True,
            flags=cv2.SOLVEPNP_EPNP,
        )
        return self.rvec, self.tvec

    def get_homography(self):
        self.image_points = np.array(
            [
                [140.15, 474.2],
                [476.905, 472.035],
                [230.96, 350.87],
                [400.662, 350.936],
            ],
            dtype=np.float32,
        )
        self.object_points = np.array(
            [
                [2.75, 9.65, -3],
                [2.75, -9.65, -3],
                [22.0, 9.65, -3],
                [22.0, -9.65, -3],
            ],
            dtype=np.float32,
        )

        self.homo_object_point = np.array(
            [
                self.object_points[:, 2],
                -self.object_points[:, 1],
                np.ones(len(self.object_points)),
            ]
        ).T

        self.homography, _ = cv2.findHomography(
            self.image_points, self.homo_object_point
        )
        return self.homography

    def estimate_distance(self, bbox):
        cx = bbox[0] + bbox[1]
        bottom = bbox[2]
        bottom_center_ptn = [cx, bottom, 1]
        return np.dot(self.homography, bottom_center_ptn)