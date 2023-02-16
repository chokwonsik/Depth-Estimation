# import cv2

# # variables
# # distance from camera to object(face) measured
# KNOWN_DISTANCE = 47.7  # centimeter
# # width of face in the real world or Object Plane
# KNOWN_WIDTH = 4.0  # centimeter
# # Colors
# GREEN = (0, 255, 0)
# RED = (0, 0, 255)
# WHITE = (255, 255, 255)
# fonts = cv2.FONT_HERSHEY_COMPLEX
# cap = cv2.VideoCapture(1)

# # face detector object
# # face_detector = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")///


# # focal length finder function
# def focal_length(measured_distance, real_width, width_in_rf_image):
#     """
#     This Function Calculate the Focal Length(distance between lens to CMOS sensor), it is simple constant we can find by using
#     MEASURED_DISTACE, REAL_WIDTH(Actual width of object) and WIDTH_OF_OBJECT_IN_IMAGE
#     :param1 Measure_Distance(int): It is distance measured from object to the Camera while Capturing Reference image
#     :param2 Real_Width(int): It is Actual width of object, in real world (like My face width is = 14.3 centimeters)
#     :param3 Width_In_Image(int): It is object width in the frame /image in our case in the reference image(found by Face detector)
#     :retrun focal_length(Float):"""
#     focal_length_value = (width_in_rf_image * measured_distance) / real_width
#     return focal_length_value


# # distance estimation function
# def distance_finder(focal_length, real_face_width, face_width_in_frame):
#     """
#     This Function simply Estimates the distance between object and camera using arguments(focal_length, Actual_object_width, Object_width_in_the_image)
#     :param1 focal_length(float): return by the focal_length_Finder function
#     :param2 Real_Width(int): It is Actual width of object, in real world (like My face width is = 5.7 Inches)
#     :param3 object_Width_Frame(int): width of object in the image(frame in our case, using Video feed)
#     :return Distance(float) : distance Estimated
#     """
#     distance = (real_face_width * focal_length) / face_width_in_frame
#     return distance

# # reading reference image from directory
# ref_image = cv2.imread("./data/frame0022.jpg")

# # 291, 265, 313, 291
# ref_image_face_width = 29
# # ref_image_face_width = 313 - 291
# focal_length_found = focal_length(KNOWN_DISTANCE, KNOWN_WIDTH, ref_image_face_width)
# print(f"focal_length_found = {focal_length_found}")
# # focal_length_found = 356
# cv2.imshow("ref_image", ref_image)

# while True:
#     # _, frame = cap.read()
#     frame = cv2.imread("./data/undist_img.jpg")

#     # calling face_data function
#     # face_width_in_frame = 4.6
#     # face_width_in_frame = 7.6
#     face_width_in_frame = 10
#     # face_width_in_frame = 29

#     # face_width_in_frame = 313 - 291
#     # finding the distance by calling function Distance
#     if face_width_in_frame != 0:
#         Distance = distance_finder(focal_length_found, KNOWN_WIDTH, face_width_in_frame)
#         # Drwaing Text on the screen
#         cv2.putText(
#             frame, f"Distance = {round(Distance,2)} CM", (50, 50), fonts, 1, (WHITE), 2
#         )
#     cv2.imshow("frame", frame)
#     if cv2.waitKey(1) == ord("q"):
#         break
# cap.release()
# cv2.destroyAllWindows()


import cv2
import math
import numpy as np

# focal length finder function
def focal_length(measured_distance, real_width, width_in_rf_image):
    focal_length_value = (width_in_rf_image * measured_distance) / real_width
    return focal_length_value

# distance estimation function
def distance_finder(focal_length, real_tstl_width, face_width_in_frame):
    distance = (real_tstl_width * focal_length) / face_width_in_frame
    return distance

# visualization
def visualize_location(visualization, x_y_dist_list):
    if visualization == False:
        return
    else:
        world_board = np.zeros([600, 600, 3],dtype=np.uint8)
        world_board.fill(127)
        
        for x_y_dist in x_y_dist_list:
            if x_y_dist[0] < 0:
                cv2.circle(world_board, (int(x_y_dist[0]) + 300, 600 - int(x_y_dist[1])), 5, RED, -1)
                cv2.putText(world_board, f"{round(x_y_dist[0], 3)}, {round(x_y_dist[1], 3)}, {round(x_y_dist[2], 3)}cm",
                (int(x_y_dist[0]) + 300, 585 - int(x_y_dist[1])), fonts, 0.4, (RED), 1)
            else:
                cv2.circle(world_board, (int(x_y_dist[0]) + 300, 600 - int(x_y_dist[1])), 5, GREEN, cv2.FILLED)
                cv2.putText(world_board, f"{round(x_y_dist[0], 3)}, {round(x_y_dist[1], 3)}, {round(x_y_dist[2], 3)}cm",
                (int(x_y_dist[0]) + 300, 585 - int(x_y_dist[1])), fonts, 0.4, (GREEN), 1)

        cv2.imshow("world_board", world_board)


if __name__ == "__main__":
    # distance from camera to object(face) measured
    KNOWN_DISTANCE = 60  # 타일 하나 + 카메라 거리 centimeter
    # width of face in the real world or Object Plane
    KNOWN_WIDTH = 19.5  # centimeter
    ref_image_face_width = 103.4 # 카메라로부터 60cm 떨어진 위치에서의 사진 픽셀 width
    # Colors
    GREEN = (0, 255, 0)
    RED = (0, 0, 255)
    WHITE = (255, 255, 255)
    fonts = cv2.FONT_HERSHEY_COMPLEX

    # visualization
    visualization = False

    focal_length_found = focal_length(KNOWN_DISTANCE, KNOWN_WIDTH, ref_image_face_width)
    print(f"focal_length_found = {focal_length_found}")

    # reading reference image from directory
    ref_image = cv2.imread("./images/frame_test.jpg")

    tstl_list = [
        [0, 190, 204, 217, 234], # [cls, x1, y1, x2, y2]
        [0, 331, 213, 353, 232],
        ]
    
    x_y_dist_list = []

    for tstl in tstl_list:
        id_tstl = tstl[0]
        x1 = tstl[1]
        y1 = tstl[2]
        x2 = tstl[3]
        y2 = tstl[4]
        print(x1, y1, x2, y2)
        center_x = int((x1 + x2) / 2)
        center_x_float = (x1 + x2) / 2
        if center_x_float > 320:
            delta_x = center_x_float - 320
        else:
            delta_x = 320 - center_x_float
        center_y = int((y1 + y2) / 2)

        # face_width_in_frame = 40
        tstl_width_in_frame = x2 - x1
        # face_width_in_frame = 103.4

        # face_width_in_frame = 29
        # face_width_in_frame = 22

        # finding the distance by calling function Distance
        if tstl_width_in_frame != 0:
            Distance = distance_finder(focal_length_found, KNOWN_WIDTH, tstl_width_in_frame)
            Distance = Distance - 15
            azimuth = delta_x / 320 * 120 / 2 * math.pi / 180 # degree to radian
            print(f"azimuth = {azimuth}")
            if center_x_float > 320:
                x_dist = Distance * math.sin(azimuth)
            else:
                x_dist = -(Distance * math.sin(azimuth))
            y_dist = Distance * math.cos(azimuth)

            print(f"y_dist, x_dist = {y_dist}, {x_dist}")

            x_y_dist_list.append([x_dist, y_dist, Distance])

            # Drwaing Text on the screen
            cv2.circle(ref_image, (center_x, center_y), 3, (GREEN), -1)
            cv2.putText(
                ref_image, f"dist = {round(Distance, 4)} CM", (x1, y1 - 20), fonts, 0.5, (WHITE), 1
            )
            cv2.putText(
                ref_image, f"x = {round(x_dist, 3)}, y = {round(y_dist, 3)}", (x1, y1), fonts, 0.5, (RED), 1
            )
    print(x_y_dist_list)
    visualize_location(visualization, x_y_dist_list)

    cv2.imshow("ref_image", ref_image)
    cv2.waitKey(0)