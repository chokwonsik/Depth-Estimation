import os, json
import cv2
import math
import numpy as np
import matplotlib.pyplot as plt

visualize = True

def estimation_distance():
    ######## step 2. intrinsic, distort coefficients information
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
    
    img_file_path = "./images/frame0022.jpg"
    img = cv2.imread(img_file_path)

    ######## step 3. undistort
    mapx, mapy = cv2.initUndistortRectifyMap(camera_matrix, dist_coeff, None, None, (img.shape[1], img.shape[0]), 5)
    undistort_img = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)

    img_points = np.array([
                        [145.5, 475.0],
                        [476.905, 472.035],
                        [228.71, 350.49],
                        [400.662, 350.936],
                        [223.40, 359.43],
                        ], dtype=np.float32)


    obj_points = np.array([
                        [-3.0, -9.65, 2.75], # z, y, x
                        [-3.0, 9.65, 2.75],
                        [-3.0, -9.65, 22.0],
                        [-3.0, 9.65, 22.0],
                        [-3.0, -9.65, 19.25],
                        ], dtype=np.float32)

    data_size = len(img_points)

    homo_object_point = np.append(obj_points[:,2:3], obj_points[:,1:2],axis=1)
    homo_object_point = np.append(homo_object_point, np.ones([1,data_size]).T, axis=1)

    if visualize:
        ######## get rotation , translation vector using obj points and img points
        _, rvec, tvec = cv2.solvePnP(obj_points, img_points, camera_matrix, distCoeffs=None, useExtrinsicGuess=True, flags=cv2.SOLVEPNP_EPNP)
        undistort_img = cv2.drawFrameAxes(undistort_img, camera_matrix, distCoeffs=None, rvec=rvec, tvec=tvec, length=5, thickness=3)
        ######## image points, object points 의 pair 쌍이 맞는지 재투영을 통해 확인
        proj_image_points, _ = cv2.projectPoints(obj_points, rvec, tvec, camera_matrix, None)
        for proj_image_point, img_point in zip(proj_image_points, img_points):
            print(img_point, proj_image_point, "\n")

        ######## 카메라 축 확인
        img_un = cv2.cvtColor(undistort_img, cv2.COLOR_BGR2RGB)
        plt.imshow(img_un)
        plt.title("undistort_image")
        plt.show()

    homography, _ = cv2.findHomography(img_points, homo_object_point)

    print(f"data_size {data_size}")

    return undistort_img, homography

if __name__ =="__main__":
    img = cv2.imread("./images/frame0022.jpg", cv2.IMREAD_ANYCOLOR)

    center_point = (242,243)
    # white
    # 123 (100,292), (468,264), (322,251)
    # yellow
    # 134 (314,304), (425,248), (242,243)
    img_, homography = estimation_distance()
    img_point = np.array([center_point[0],center_point[1], 1], dtype=np.float32)

    ######## inference
    estimation = np.dot(homography, img_point)
    x,y,z = estimation[0] ,estimation[1],estimation[2]
    print(f"x/z = {x/z}, y/z = {y/z}, z = {z/z}")
    distance = math.sqrt((x/z)**2 + (y/z)**2 + (z/z)**2)

    ######## visualize
    # cv2.rectangle(img, (top_left), (bottom_right), (255,255,0), 2)
    cv2.circle(img, center_point, 3, (0, 255, 0), 3)
    # cv2.putText(img, "distance : "+str(round(distance,2)) + "cm", top_left, 2, 0.5, (10,250,10),1)
    cv2.putText(img, "distance : "+str(round(distance,2)) + "cm", center_point, 2, 0.5, (10,250,10),1)
    cv2.imshow("img", img)
    cv2.waitKey()

    print(f"homography matrix \n{homography}\n")    
    print(f"distance {round(distance,2)}cm")