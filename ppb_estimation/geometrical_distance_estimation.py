import math
import cv2
import numpy as np
import os, json

def focal_length_distanc_list(pingpong_list, image):
    global CAMERA_HEIGHT
    fov_x = 80
    CAMERA_HEIGHT = 0.15
    # camera_matrix
    calibration_jsonfile_path = os.path.join("./calibration.json")
    with open(calibration_jsonfile_path, 'r') as calibration_file:
        calibration_info = json.load(calibration_file)
        intrinsic = calibration_info['intrinsic']
        fx, fy, cx, cy = intrinsic['fx'], intrinsic['fy'], intrinsic['cx'], intrinsic['cy']
        camera_matrix = np.array([
                                [fx,      0.00000,      cx],
                                [0.00000,      fy,      cy],
                                [0.00000, 0.00000, 1.00000],
                                ], dtype=np.float64)
        dist_coeff = np.array([-0.274053, 0.049790, -0.001539, -0.003057, 0.00000], dtype=np.float64)

    print(f"camera_matrix = {camera_matrix}")
    print(f"dist_coeff = {dist_coeff}")
        
    mapx, mapy = cv2.initUndistortRectifyMap(camera_matrix, dist_coeff, None, None, (img.shape[1], img.shape[0]), 5)
    undistort_img = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)

    pingpong_distance = []
    '''
    pingpong list format: xyxy
    '''

    for pingpong in pingpong_list:
        x1 = pingpong[1]
        y1 = pingpong[2]
        x2 = pingpong[3]
        y2 = pingpong[4]
        print(x1, y1, x2, y2)

        # Nomalized Image Plane
        y_norm = (y2 - camera_matrix[1][2]) / (camera_matrix[1][1])
        print(f"y_norm = {y_norm}")
        distance = 1 * CAMERA_HEIGHT / y_norm
        print(f"distance = {distance}")
        # x_angle = fov_x * ((float(x1+x2)) / 2 - camera_matrix[0][2]) / 480
        # print(f"x_angle = {x_angle}")
        # x_distance = y_distance * math.tan(x_angle)
        # y_distance = y_distance - 16

        # d = math.sqrt(y_distance**2 + x_distance**2)

        # pingpong_distance.append(
        #     list(map(int, [x_distance, y_distance])))
        # print(f"dist = {pingpong_distance}")
    return undistort_img, pingpong_distance

if __name__ == "__main__":
    img_file_path = "./images/frame0022.jpg"
    img = cv2.imread(img_file_path)
    
    pingpong_list = [
                    # [0, 291, 264, 316, 289],
                    #  [0, 407.8, 225.3, 417.4, 234.2],
                     [0, 301, 282, 325, 303],
                    #  [0, 421, 239, 430, 249]
                    ]
    
    undistort_img, pingpong_distance = focal_length_distanc_list(pingpong_list, img)