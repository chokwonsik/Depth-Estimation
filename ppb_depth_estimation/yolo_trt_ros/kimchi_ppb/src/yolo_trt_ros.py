#!/usr/bin/env python

import time
import numpy as np
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
    """
    focal_length(KNOWN_DISTANCE, KNOWN_WIDTH, ref_image_face_width)
    This Function Calculate the Focal Length(distance between lens to CMOS sensor), it is simple constant we can find by using
    MEASURED_DISTACE, REAL_WIDTH(Actual width of object) and WIDTH_OF_OBJECT_IN_IMAGE
    :param1 Measure_Distance(int): It is distance measured from object to the Camera while Capturing Reference image
    :param2 Real_Width(int): It is Actual width of object, in real world (like My face width is = 14.3 centimeters)
    :param3 Width_In_Image(int): It is object width in the frame /image in our case in the reference image(found by Face detector)
    :retrun focal_length(Float):"""
    focal_length_value = (width_in_rf_image * measured_distance) / real_width
    return focal_length_value

# distance estimation function
def distance_finder(focal_length, real_tstl_width, face_width_in_frame):
    """
    This Function simply Estimates the distance between object and camera using arguments(focal_length, Actual_object_width, Object_width_in_the_image)
    :param1 focal_length(float): return by the focal_length_Finder function
    :param2 Real_Width(int): It is Actual width of object, in real world (like My face width is = 5.7 Inches)
    :param3 object_Width_Frame(int): width of object in the image(frame in our case, using Video feed)
    :return Distance(float) : distance Estimated
    """
    distance = (real_tstl_width * focal_length) / face_width_in_frame
    return distance

def calculate_distance(tstl_list, xycar_image):
    distance_list = []
    for tstl in tstl_list:
        id_tstl = tstl[0]
        x1 = tstl[1]
        x2 = tstl[2]
        y1 = tstl[3]
        y2 = tstl[4]
        center_x = int((x1 + x2) / 2)
        center_y = int((y1 + y2) / 2)

        tstl_width_in_frame = x2 - x1

        if tstl_width_in_frame != 0:
            Distance = distance_finder(focal_length_found, KNOWN_WIDTH, tstl_width_in_frame)
            Distance = Distance - 15
            distance_list.append(Distance)
            # Drwaing Text on the screen
            cv2.circle(xycar_image, (center_x, center_y), 3, (GREEN), -1)
            cv2.putText(xycar_image, str(Distance), (int(x1), int(y1) + 10), fonts, 0.5, (WHITE), 2)
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
KNOWN_WIDTH = 4.0  # centimeter
ref_image_face_width = 23.5
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

    distance_list = calculate_distance(bbox_list, xycar_image)
    try:
        for i in range(len(distance_list)):
            print("test", distance_list[i])
    except:
        pass
    cv2.imshow("show_trt",xycar_image)
    
    cv2.waitKey(1)
    rate.sleep()
