import os, json
import cv2
import math
import numpy as np
import matplotlib.pyplot as plt



def estimation_distance(img, detected_pingpongs):
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
    
    # mapx, mapy = cv2.initUndistortRectifyMap(camera_matrix, dist_coeff, None, None, (img.shape[1], img.shape[0]), 5)
    # undistort_img = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)

    img_points = np.array([
                            [145.5, 475.0],
                            [476.905, 472.035],
                            [228.71, 350.49],
                            [400.662, 350.936],
                            [223.40, 359.43],
                        ], dtype=np.float32)

    obj_points = np.array([
                            [-3.0, -9.65, 2.75],
                            [-3.0, 9.65, 2.75],
                            [-3.0, -9.65, 22.0],
                            [-3.0, 9.65, 22.0],
                            [-3.0, -9.65, 19.25],
                        ], dtype=np.float32)

    data_size = len(img_points)

    homo_object_point = np.append(obj_points[:,2:3], obj_points[:,1:2],axis=1)
    homo_object_point = np.append(homo_object_point, np.ones([1,data_size]).T, axis=1)

    if visualize:
        _, rvec, tvec = cv2.solvePnP(obj_points, img_points, camera_matrix, distCoeffs=None, useExtrinsicGuess=True, flags=cv2.SOLVEPNP_EPNP)
        img = cv2.drawFrameAxes(img, camera_matrix, distCoeffs=None, rvec=rvec, tvec=tvec, length=5, thickness=3)

        proj_image_points, _ = cv2.projectPoints(obj_points, rvec, tvec, camera_matrix, None)
        for proj_image_point, img_point in zip(proj_image_points, img_points):
            print(img_point, proj_image_point, "\n")
    homography, _ = cv2.findHomography(img_points, homo_object_point)

    return img, homography

def calc_coord(detected_pingpongs):
    bottom_center_list = []
    for pingpong in detected_pingpongs:
        id_pingpong = pingpong[0]
        xmin = pingpong[1]
        ymin = pingpong[2]
        xmax = pingpong[3]
        ymax = pingpong[4]
        
        center_x = int((xmin + xmax) / 2)
        center_y = int((ymin + ymax) / 2)
        
        bottom_center_list.append([id_pingpong, center_x, ymax])
        
        if visualize:
            cv2.circle(img, (center_x, center_y), 1, (0, 0, 255), -1)
            cv2.rectangle(img, (xmin, ymin), (xmax, ymax), (255,255,0), 1)
            
    return bottom_center_list


if __name__ =="__main__":
    # visualize
    visualize = True
    
    img = cv2.imread("./images/frame0022.jpg", cv2.IMREAD_ANYCOLOR) # cls -> 0 yellow
    detected_pingpongs = [[0, 302, 279, 326, 304],
                          [0, 419, 239, 429, 248],
                          [0, 237, 234, 245, 241]] # [cls, xmin, ymin, xmax, ymax]
    
    # img = cv2.imread("./images/frame0016.jpg", cv2.IMREAD_ANYCOLOR) # cls -> 1 white
    # detected_pingpongs = [[1, 87, 272, 109, 290],
    #                       [1, 315, 238, 326, 249],
    #                       [1, 458, 250, 470, 263]] # [cls, xmin, ymin, xmax, ymax]
    
    bottom_center_list = calc_coord(detected_pingpongs) # [cls, center_x, ymax]
    
    img_result, homography = estimation_distance(img, detected_pingpongs)
    for bottom_center in bottom_center_list:
        img_point = np.array([bottom_center[1],bottom_center[2], 1], dtype=np.float32)

        estimation = np.dot(homography, img_point)
        y, x, z = estimation[0] ,estimation[1],estimation[2]
        distance = math.sqrt((x/z)**2 + (y/z)**2 + (z/z)**2)
        if visualize:
            cv2.putText(img_result, "distance :" + str(round(distance,2)) + "cm", (bottom_center[1],bottom_center[2]+10), 2, 0.4, (50, 250, 50), 1)
            cv2.putText(img_result, "x :" + str(round(x/z,2)) + " y :" + str(round(y/z,2)), (bottom_center[1],bottom_center[2]+20), 2, 0.4, (255, 255, 255), 1)
        print(f"cls={bottom_center[0]}, distance {round(distance,2)}cm")
        print(f"x/z = {x/z}, y/z = {y/z}, z = {z/z} \n")
    cv2.imshow("img", img_result)
    cv2.imwrite('./estimation_result/homography_cls_' + str(bottom_center[0]) + "_result.jpg", img_result)
    cv2.waitKey(0)

    print(f"homography matrix \n{homography}\n")
    