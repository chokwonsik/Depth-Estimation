import cv2
import numpy as np
import math
import matplotlib.pyplot as plt

# focal length finder function
def focal_length(measured_distance, real_width, width_in_rf_image):
    focal_length_value = (width_in_rf_image * measured_distance) / real_width
    return focal_length_value

# distance estimation function
def distance_finder(focal_length, real_face_width, face_width_in_frame):
    distance = (real_face_width * focal_length) / face_width_in_frame
    return distance

def calc_coord(detected_pingpongs):
    pingpongs_data_list = []
    for pingpong in detected_pingpongs:
        id_tstl = pingpong[0]
        xmin = int(pingpong[1])
        ymin = int(pingpong[2])
        xmax = int(pingpong[3])
        ymax = int(pingpong[4])
        
        width_x = pingpong[3] - pingpong[1]
        
        center_x = int((xmin + xmax) / 2)
        center_y = int((ymin + ymax) / 2)
        
        if center_x > 320:
            delta_x = center_x - 320
        else:
            delta_x = 320 - center_x
        
        pingpongs_data_list.append([id_tstl, center_x, center_y, width_x, delta_x])
        
        if visualize:
            cv2.circle(img, (center_x, center_y), 1, (0, 0, 255), -1)
            cv2.rectangle(img, (xmin, ymin), (xmax, ymax), (255,255,0), 1)
            
    return pingpongs_data_list

if __name__ == "__main__":
    # visualize
    visualize = True
    
    # distance from camera to object(tstl) measured
    KNOWN_DISTANCE = 45.0 + 15.0  # centimeter
    # width of face in the real world or Object Plane
    KNOWN_WIDTH = 19.5  # centimeter

    # ['left', 'right', 'stop', 'crosswalk', 'uturn', 'traffic_light']
    # img = cv2.imread("./images/frame_1.jpg", cv2.IMREAD_ANYCOLOR) # frame_1.jpg
    # tstl_list = [
    #             [0, 359.4, 196.0, 416.2, 253.4],
    #             [1, 245.4, 198.0, 289.0, 238.8]] # [cls, xmin, ymin, xmax, ymax]
    
    # img = cv2.imread("./images/frame_2.jpg", cv2.IMREAD_ANYCOLOR) # frame_2.jpg
    # tstl_list = [
    #             [0, 287.8, 193.2, 346.9, 252.0],
    #             [1, 246.5, 198.8, 286.8, 239.7]] # [cls, xmin, ymin, xmax, ymax]
    
    # img = cv2.imread("./images/frame_3.jpg", cv2.IMREAD_ANYCOLOR) # frame_3.jpg
    # tstl_list = [
    #             [1, 190, 203.5, 218.8, 234.6],
    #             [0, 333.4, 212.2, 353.4, 233.7]] # [cls, xmin, ymin, xmax, ymax]
    
    # img = cv2.imread("./images/frame_4.jpg", cv2.IMREAD_ANYCOLOR) # frame_4.jpg -> 1
    # tstl_list = [
    #             [1, 88.0, 176.2, 149.8, 258.3],
    #             [0, 165.8, 189.9, 216.4, 242.6], 
    #             [0, 229.0, 202.6, 259.8, 233.0],
    #             [0, 299.7, 197.1, 343.4, 238.1],
    #             [3, 378.9, 201.5, 411.2, 236.7],
    #             [2, 428.1, 192.1, 479.8, 250.1],
    #             [0, 482.3, 180.5, 554.3, 267.3]
    #             ] # [cls, xmin, ymin, xmax, ymax]
    
    img = cv2.imread("./images/frame_5.jpg", cv2.IMREAD_ANYCOLOR) # frame_5.jpg -> 2
    tstl_list = [
                [1, 274.3, 163.1, 375.1, 260.0],
                [0, 394.6, 197.4, 434.4, 238.2],
                [3, 444.8, 203.0, 473.4, 237.0],
                [2, 529.9, 200.4, 567.7, 250.0]] # [cls, xmin, ymin, xmax, ymax]
    
    ref_image_tstl_width = 103.4
    focal_length_found = focal_length(KNOWN_DISTANCE, KNOWN_WIDTH, ref_image_tstl_width)
    print(f"focal_length_found = {focal_length_found}")
    
    tstl_data_list = calc_coord(tstl_list) # [id_pingpong, center_x, center_y, width_x, delta_x]
    
    for tstl_data in tstl_data_list:
        center_x = tstl_data[1]
        center_y = tstl_data[2]
        tstl_width_in_frame = tstl_data[3]
        delta_x = tstl_data[4]
        
        if tstl_width_in_frame != 0:
            Distance = distance_finder(focal_length_found, KNOWN_WIDTH, tstl_width_in_frame)
            Distance = Distance - 15
            print(f"delta_x = {delta_x}")
            
            azimuth = delta_x / 320 * 130 / 2 * math.pi / 180 # degree to radian
            if center_x > 320:
                x_dist = Distance * math.sin(azimuth)
            else:
                x_dist = -(Distance * math.sin(azimuth))
            y_dist = Distance * math.cos(azimuth)
            
            print(f"azimuth = {azimuth}")
            cv2.putText(
                img, f"dist :{round(Distance,2)}cm", (center_x, center_y - 40), 2, 0.3, (50, 250, 50), 1
            )
            cv2.putText(
                img, f"x :{round(x_dist, 2)}, y :{round(y_dist, 2)}", (center_x, center_y - 30), 2, 0.3, (255, 255, 255), 1
            )
            print(f"x = {x_dist}, y = {y_dist}, dist = {Distance}\n")
            
    cv2.imshow("img", img)
    cv2.imwrite('./estimation_result/geometrical_' + str(2) + "_result.jpg", img)
    cv2.waitKey(0)
