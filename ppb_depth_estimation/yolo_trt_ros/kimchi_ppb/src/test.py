#!/usr/bin/env python3

import cv2
import json
import os
import numpy as np
import matplotlib.pyplot as plt
import calibration_parser
import rospy

from yolov3_trt_ros.msg import BoundingBoxes, BoundingBox

trt_msg = BoundingBoxes()
bbox_list = []

def callback(data) :
    global obj_id
    global bbox_list

    bbox_list = []
    for bbox in data.bounding_boxes:
        x_min = bbox.xmin
        x_max = bbox.xmax
        y_min = bbox.ymin
        y_max = bbox.ymax

        center_x = (x_max + x_min) / 2 
        center_y = (y_max + y_min) / 2 
        bbox_list.append([center_x, center_y, 1])

rospy.init_node("PPB_EST")
rospy.Subscriber('/yolov3_trt_ros/detections', BoundingBoxes, callback, queue_size=1)
# pub = rospy.Publisher('xycar_motor',xycar_motor,queue_size=1)


def initial_condition(image):
    image_points = np.array([
        # [80,350],
        # [261,344],
        # [126, 260],
        # [220,260],
        # [180,265],
        # [167,257],
        [95,192],
        [255,198],
        [233,187],
        [132,177]
    ], dtype=np.float32)

    object_points = np.array([
        # [2.75, 9.65, -3],
        # [2.75, -9.65, -3],
        # [22.0, 9.65, -3],
        # [22.0, -9.65, -3],
        # [19.3, -4.125, -3],
        # [22.0, 1.375, -3],
        [90, 45, -3],
        [90,-45,-3],
        [135,-45,-3],
        [180,45,-3]
    ], dtype=np.float32)

    homo_object_point = object_points[:,0:2]
    _, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, distCoeffs=None, useExtrinsicGuess=True, flags=cv2.SOLVEPNP_EPNP)
    image = cv2.drawFrameAxes(image, camera_matrix, None, rvec, tvec, 1, 5)
    # cv2.imshow('cli', image)
    # cv2.waitKey(0)
    homography, _ = cv2.findHomography(image_points, homo_object_point)
    return homography

def calculate_distance(new_image_points, homography, DATA_SIZE=4):
    distance = []
    ### [[200,200], [100,100]]
    ###[200,200,1]
    # print(new_image_points)

    for image_points in new_image_points:
        image_points = np.array(image_points, dtype = np.float32)
        print(image_points)
        # print(homography)
        estimation_distance = np.dot(homography, image_points)
        x = estimation_distance[0]
        y = estimation_distance[1]
        print(x,y)
        dist = ((x**2 + y**2)**0.5)*0.01
        distance.append(round(dist,2))
    return distance

if __name__ == "__main__":
    calibration_json_filepath = "/home/nvidia/xycar_ws/src/kimchi_ppb/image/cologne_000065_000019_camera.json"
    camera_matrix = calibration_parser.read_json_file(calibration_json_filepath)
    image = cv2.imread("/home/nvidia/xycar_ws/src/kimchi_ppb/image/image1.png", cv2.IMREAD_ANYCOLOR)
    homography = initial_condition(image)


    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        distance = calculate_distance(bbox_list, homography, DATA_SIZE=4)
        for i in range(len(distance)):
            print(distance[i])
        rate.sleep()

    


    
    # # (u, v) -> (u, v, 1)


    # for image_point in appned_image_points:
    #     # estimation point(object_point) -> homography * src(image_point)
    #     estimation_distance = np.dot(homography, [244.9,371.8,1])

    #     x = estimation_distance[0]
    #     y = estimation_distance[1]
    #     z = estimation_distance[2]
    #     print('distance')
    #     print(x/z, y/z)
    #     dist = ((x**2 + y**2)**0.5)*0.01
    #     print('distance: {}'.format(round(dist,2)) + 'm')

    # # 여기서 중요한 점
    # #호모그래피는 평면과 평면 사이의 변환을 의미한다.
