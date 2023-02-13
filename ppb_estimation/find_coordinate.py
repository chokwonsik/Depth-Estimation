import cv2
import numpy as np
import matplotlib.pyplot as plt

def mouse_handler(event, x, y, flags, img_src):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"x, y = {x, y}")
        cv2.circle(img_src, (x, y), 3, (0, 0, 255), -1)
        cv2.imshow('image', img_src)

if __name__ == "__main__":
    img_src = cv2.imread('images/frame0022.jpg')
    img_src = cv2.resize(img_src, (640, 480), interpolation=cv2.INTER_CUBIC)
    
    camera_matrix = np.array([
                                [357.906682, 0.00000,    332.457325],
                                [0.00000,    356.170785, 253.010996],
                                [0.00000,    0.00000,    1.00000],
                                ], dtype=np.float64)
    dist_coeff = np.array([-0.274053, 0.049790, -0.001539, -0.003057, 0.00000], dtype=np.float64)

    # mapx, mapy = cv2.initUndistortRectifyMap(camera_matrix, dist_coeff, None, None, (img_src.shape[1], img_src.shape[0]), 5)
    # img_src = cv2.remap(img_src, mapx, mapy, cv2.INTER_LINEAR)

    cv2.imshow('image',img_src)
    plt_img = cv2.cvtColor(img_src, cv2.COLOR_BGR2RGB)
    plt.imshow(plt_img)
    plt.show()
    
    cv2.setMouseCallback("image", mouse_handler, img_src)
    
    cv2.waitKey(0)
    cv2.destroyAllWindows()


# import cv2
# import json
# import os
# import numpy as np
# import matplotlib.pyplot as plt
# import calibration_parser
# import math


# def initial_condition(image):
#     image_points = np.array([
#         [140.15, 474.2],
#         [476.905, 472.035],
#         [230.96, 350.87],
#         [400.662, 350.936],
#     ], dtype=np.float32)

#     object_points = np.array([
#         [2.75, 9.65, -3],
#         [2.75, -9.65, -3],
#         [22.0, 9.65, -3],
#         [22.0, -9.65, -3],
#     ], dtype=np.float32)

#     homo_object_point = object_points[:,0:2]
#     _, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, distCoeffs=None, useExtrinsicGuess=True, flags=cv2.SOLVEPNP_EPNP)
#     image = cv2.drawFrameAxes(image, camera_matrix, None, rvec, tvec, 5, 5)
#     cv2.imshow("image", image)
#     cv2.waitKey(0)
#     homography, _ = cv2.findHomography(image_points, homo_object_point)
#     return homography

# def calculate_distance(new_image_points, homography, DATA_SIZE=4):
#     distance = []
#     for image_points in new_image_points:
#         image_points = np.append(image_points.reshape(4, 2), np.ones([1, DATA_SIZE]).T, axis=1)
#         estimation_distance = np.dot(homography, image_points)
#         x = estimation_distance[0]
#         y = estimation_distance[1]
#         z = estimation_distance[2]
#         print(f"x/z = {x/z}, y/z = {y/z}, z = {z/z}")
#         dist = math.sqrt((x/z)**2 + (y/z)**2 + (z/z)**2)
#         # dist = ((x**2 + y**2)**0.5)*0.01
#         distance.append(dist)
#     return distance

# if __name__ == "__main__":
#     calibration_json_filepath = os.path.join("calibration.json")
#     camera_matrix = calibration_parser.read_json_file(calibration_json_filepath)
#     image = cv2.imread(os.path.join("images", "img2.jpg"), cv2.IMREAD_ANYCOLOR)
#     homography = initial_condition(image)
#     img_point = (400.662, 350.936)
#     img_point = np.array([img_point[0],img_point[1], 1], dtype=np.float32)
#     estimation = np.dot(homography, img_point)
#     x,y,z = estimation[0] ,estimation[1],estimation[2]
#     print(f"x/z = {x}, y/z = {y}, z = {z}")
#     distance = math.sqrt((x)**2 + (y)**2 + (z)**2)
#     print(f"distance = {distance}")