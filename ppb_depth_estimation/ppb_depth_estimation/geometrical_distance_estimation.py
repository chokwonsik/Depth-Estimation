import cv2
import numpy as np
import math
import matplotlib.pyplot as plt

# distance from camera to object(face) measured
KNOWN_DISTANCE = 45.0 + 15.0  # centimeter
# width of face in the real world or Object Plane
KNOWN_WIDTH = 4.0  # centimeter
fonts = cv2.FONT_HERSHEY_COMPLEX

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
        id_pingpong = pingpong[0]
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
        
        pingpongs_data_list.append([id_pingpong, center_x, center_y, width_x, delta_x])
        
        if visualize:
            cv2.circle(img, (center_x, center_y), 1, (0, 0, 255), -1)
            cv2.rectangle(img, (xmin, ymin), (xmax, ymax), (255,255,0), 1)
            
    return pingpongs_data_list

if __name__ == "__main__":
    # visualize
    visualize = True
    
    # img = cv2.imread("./images/frame0022.jpg", cv2.IMREAD_ANYCOLOR) # cls -> 0 yellow
    # detected_pingpongs = [[0, 303.0, 279.3, 327.1, 302.2],
                        #   [0, 419.3, 239.9, 429., 248.8],
                        #   [0, 238.2, 235.4, 245.2, 241.1]] # [cls, xmin, ymin, xmax, ymax]
    
    img = cv2.imread("./images/frame0016.jpg", cv2.IMREAD_ANYCOLOR) # cls -> 1 white
    detected_pingpongs = [[1, 88.0, 272.3, 108.5, 292.6],
                          [1, 316.3, 239.2, 327.1, 249.4],
                          [1, 458.7, 250.0, 470.7, 262.1]] # [cls, xmin, ymin, xmax, ymax]
    
    ref_image_ppb_width = 23.5
    focal_length_found = focal_length(KNOWN_DISTANCE, KNOWN_WIDTH, ref_image_ppb_width)
    print(f"focal_length_found = {focal_length_found}")
    
    pingpongs_data_list = calc_coord(detected_pingpongs) # [id_pingpong, center_x, center_y, width_x, delta_x]
    
    for pingpong_data in pingpongs_data_list:
        center_x = pingpong_data[1]
        center_y = pingpong_data[2]
        ppb_width_in_frame = pingpong_data[3]
        delta_x = pingpong_data[4]
        
        if ppb_width_in_frame != 0:
            Distance = distance_finder(focal_length_found, KNOWN_WIDTH, ppb_width_in_frame)
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
                img, f"distance = {round(Distance,2)} cm", (center_x, center_y + 15), 2, 0.4, (50, 250, 50), 1
            )
            cv2.putText(
                img, f"x = {round(x_dist, 2)}, y = {round(y_dist, 2)}", (center_x, center_y + 25), 2, 0.4, (255, 255, 255), 1
            )
            print(f"x = {x_dist}, y = {y_dist}, dist = {Distance}\n")
            
    cv2.imshow("img", img)
    cv2.imwrite('./estimation_result/geometrical_cls_' + str(pingpong_data[0]) + "_result.jpg", img)
    cv2.waitKey(0)
