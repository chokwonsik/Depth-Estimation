#!/usr/bin/env python3

import cv2
import json
import os
import numpy as np
import matplotlib.pyplot as plt

def initial_condition():
    image_points = np.array([
        [140.15, 474.2],
        [476.905, 472.035],
        [230.96, 350.87],
        [400.662, 350.936],
    ], dtype=np.float32)

    object_points = np.array([
        [2.75, 9.65, -3],
        [2.75, -9.65, -3],
        [22.0, 9.65, -3],
        [22.0, -9.65, -3],
    ], dtype=np.float32)

    homo_object_point = object_points[:,0:2]
    _, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, distCoeffs=None, useExtrinsicGuess=True, flags=cv2.SOLVEPNP_EPNP).
    image = cv2.drawFrameAxes(image, camera_matrix, None, rvec, tvec, 1, 5)
    homography, _ = cv2.findHomography(image_points, homo_object_point)
    return homography

def calculate_distance(new_image_points, homography, DATA_SIZE=4):
    distance = []
    for image_points in new_image_points:
        image_points = np.append(image_points.reshape(4, 2), np.ones([1, DATA_SIZE]).T, axis=1)
        estimation_distance = np.dot(homography, image_points)
        x = estimation_distance[0]
        y = estimation_distance[1]
        dist = ((x**2 + y**2)**0.5)*0.01
        distance.append(round(dist,2))
    return distance

if __name__ == "__main__":
    calibration_json_filepath = "/home/nvidia/xycar_ws/src/kimchi_ppb/image/cologne_000065_000019_camera.json"
    camera_matrix = calibration_parser.read_json_file(calibration_json_filepath)
    image = cv2.imread(os.path.join("image", "image1.png"), cv2.IMREAD_ANYCOLOR)
    homography = initial_condition()

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
