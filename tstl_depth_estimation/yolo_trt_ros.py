#!/usr/bin/env python

import time
import numpy as np
import math
import cv2
import rospy

from std_msgs.msg import String
from yolov3_trt_ros.msg import BoundingBox, BoundingBoxes

from cv_bridge import CvBridge
from sensor_msgs.msg import Image as Imageros

trt_msg = BoundingBoxes()
bbox_list = []
distance_list = []
xycar_image = np.empty(shape=[0])

def callback(data):
    global obj_id
    global bbox_list

    bbox_list = []
    for bbox in data.bounding_boxes:
        obj_id = bbox.id
        x_min = bbox.xmin * 640 / 352
        x_max = bbox.xmax * 640 / 352
        y_min = bbox.ymin * 480 / 352
        y_max = bbox.ymax * 480 / 352

        print(x_min, x_max, y_min, y_max)

        bbox_list.append([obj_id, x_min, x_max, y_min, y_max])

# focal length finder function
def focal_length(measured_distance, real_width, width_in_rf_image):
    focal_length_value = (width_in_rf_image * measured_distance) / real_width
    return focal_length_value

# distance estimation function
def distance_finder(focal_length, real_tstl_width, face_width_in_frame):
    distance = (real_tstl_width * focal_length) / face_width_in_frame
    return distance

def calculate_distance(tstl_list, xycar_image):
    distance_list = []
    x_y_dist_list = []
    for tstl in tstl_list:
        id_tstl = tstl[0]
        x1 = tstl[1]
        x2 = tstl[2]
        y1 = tstl[3]
        y2 = tstl[4]
        center_x = int((x1 + x2) / 2)
        center_x_float = (x1 + x2) / 2
        if center_x_float > 320:
            delta_x = center_x_float - 320
        else:
            delta_x = 320 - center_x_float
        center_y = int((y1 + y2) / 2)

        tstl_width_in_frame = x2 - x1

        if tstl_width_in_frame != 0:
            Distance = distance_finder(focal_length_found, KNOWN_WIDTH, tstl_width_in_frame)
            Distance = Distance - 15
            azimuth = delta_x / 320 * 140 / 2 * math.pi / 180 # degree to radian
            if center_x_float > 320:
                x_dist = Distance * math.sin(azimuth)
            else:
                x_dist = -(Distance * math.sin(azimuth))
            y_dist = Distance * math.cos(azimuth)

            x_y_dist_list.append([x_dist, y_dist, Distance])

            distance_list.append(Distance)
            # Drwaing Text on the screen
            cv2.circle(xycar_image, (center_x, center_y), 3, (GREEN), -1)
            cv2.putText(xycar_image, str(Distance), (int(x1), int(y1) + 10), fonts, 0.5, (WHITE), 2)
            cv2.putText(
                xycar_image, str(round(Distance, 4)) + "CM", (x1, y1 - 20), fonts, 0.5, (WHITE), 1
            )
            cv2.putText(
                xycar_image, "x =" + str(round(x_dist, 3)) + ", y =" + str(round(y_dist, 3)), (x1, y1), fonts, 0.5, (RED), 1
            )
    return distance_list


def img_callback(data_img):
    global xycar_image
    img = np.frombuffer(data_img.data, dtype=np.uint8).reshape(data_img.height, data_img.width, -1)
    xycar_image = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    xycar_msg = data_img

rospy.init_node("PPB_EST")
rospy.Subscriber('/yolov3_trt_ros/detections', BoundingBoxes, callback, queue_size=1)
image_sub = rospy.Subscriber("/usb_cam/image_raw", Imageros, img_callback)

# distance from camera to object(face) measured
KNOWN_DISTANCE = 45.0 + 15.0
# width of face in the real world or Object Plane
KNOWN_WIDTH = 19.5  # centimeter
ref_image_face_width = 103.4
# Colors
GREEN = (0, 255, 0)
RED = (0, 0, 255)
WHITE = (255, 255, 255)
fonts = cv2.FONT_HERSHEY_COMPLEX

focal_length_found = focal_length(KNOWN_DISTANCE, KNOWN_WIDTH, ref_image_face_width)

rate = rospy.Rate(10)

while not rospy.is_shutdown():
    if xycar_image.shape[0] == 0:
        continue
            
            #if self.show_img:

    distance_list = calculate_distance(bbox_list, xycar_image)
    loc = Locations()
    try:
        for i in range(len(distance_list)):
            print("test", distance_list[i])
    except:
        pass
    cv2.imshow("show_trt",xycar_image)
    
    cv2.waitKey(1)
    rate.sleep()
