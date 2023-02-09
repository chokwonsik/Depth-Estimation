import cv2
import json
import os
import numpy as np

import matplotlib.pyplot as plt

import calibration_parser


if __name__ == "__main__":
    calibration_json_filepath = os.path.join("image", "cologne_000065_000019_camera.json")
    camera_matrix = calibration_parser.read_json_file(calibration_json_filepath)
    image = cv2.imread(os.path.join("image", "image1.png"), cv2.IMREAD_ANYCOLOR)
    # cv2.imshow('image', image)
    # plt.imshow(image)
    # plt.show()
    # cv2.waitKey(0)


    # # extrinsic -> homography src, dst
    # # prior dst -> image coordinate
    # # present dst -> vehicle coordinate (=camera coordinate)

    # # lane (inner) width -> 2.5m, lane width -> 0.25m
    # # lane lenght -> 2.0m
    # # lane interval -> 2.0m

    """
    Extrinsic Calibration for Ground Plane
    [0, 1]
    140.15, 474.2 -> 2.75, 9.65, 3
    476.905, 472.035 -> 2.75, -9.65, 3

    [2, 3]
    230.96, 350.87 -> 22.0, 9.65, 3
    400.662, 350.936-> 22.0, -9.65, 3
    """
    image_points = np.array([
        [140.15, 474.2],
        [476.905, 472.035],
        [230.96, 350.87],
        [400.662, 350.936],
    ], dtype=np.float32)

    # # X Y Z, X -> down, Z -> forward, Y -> Right
    # # 실측이 중요하다.
    object_points = np.array([
        [2.75, 9.65, -3],
        [2.75, -9.65, -3],
        [22.0, 9.65, -3],
        [22.0, -9.65, -3],
    ], dtype=np.float32)

    DATA_SIZE = 4
    homo_object_point = object_points[:,0:2]

    # object point
    # X: forward, Y: left, Z: 1

    # 지면에 대해서 위치와 자세 추정이 가능하다면,
    # 임의의 포인트를 생성하여 이미지에 투영할수있다.
    retval, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, distCoeffs=None, useExtrinsicGuess=True, flags=cv2.SOLVEPNP_EPNP)

    # # 잘 맞지 않는다.
    # # 왜냐하면, 이미지 좌표와 실제 오브젝트와의 관계가 부정확하기 때문
    # # 실제 측정을 통해 개선이 가능하다.
    image = cv2.drawFrameAxes(image, camera_matrix, None, rvec, tvec, 1, 5)
    # cv2.imshow('image', image)
    # cv2.waitKey(0)

    proj_image_points, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, None)

    homography, _ = cv2.findHomography(image_points, homo_object_point)
    print(proj_image_points.shape)
    
    # # (u, v) -> (u, v, 1)
    appned_image_points = np.append(image_points.reshape(4, 2), np.ones([1, DATA_SIZE]).T, axis=1)
    print(homography.shape)
    for image_point in appned_image_points:
        # estimation point(object_point) -> homography * src(image_point)
        estimation_distance = np.dot(homography, [244.9,371.8,1])

        x = estimation_distance[0]
        y = estimation_distance[1]
        z = estimation_distance[2]
        print('distance')
        print(x/z, y/z, z/z)

    # # 여기서 중요한 점
    # #호모그래피는 평면과 평면 사이의 변환을 의미한다.
