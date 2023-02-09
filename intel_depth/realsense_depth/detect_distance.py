import cv2
import pyrealsense2
from realsense_depth import *

def show_distance(event, x, y, args, params):
    global point
    point = (x, y)

# Initialize Camera Intel Realsense
dc = DepthCamera()

ret, depth_frame, color_frame = dc.get_frame()
cv2.namedWindow("depth_frame")
cv2.setMouseCallback("depth_frame", show_distance, depth_frame)

while True:
    key = cv2.waitKey(0)

    ret, depth_frame, color_frame = dc.get_frame()
    cv2.imshow("depth_frame", depth_frame)

    cv2.circle(color_frame, point, 4, (0, 255, 0))
    distance = depth_frame[point[1], point[0]]

    cv2.putText(color_frame, "({0}, {1}) ".format(point[0], point[1]) + "{}mm".format(distance),
                (point[0], point[1] - 20), cv2.FONT_HERSHEY_PLAIN, 0.75, (0, 255, 0), 2)

    cv2.imshow("color_frame", color_frame)
    key = cv2.waitKey(0)
    if key == 27:
        break
cv2.destroyAllWindows()

