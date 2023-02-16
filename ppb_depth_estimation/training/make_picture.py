import cv2
import numpy as np

capture = cv2.VideoCapture(1)
capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

n = 1

""" camera_matrix
[[357.906682, 0.000000, 332.457325],
[0.000000, 356.170785, 253.010996],
[0.000000, 0.000000, 1.000000]]
"""

"""dist_coeffs
[-0.274053 0.049790 -0.001539 -0.003057 0.000000]
"""
camera_matrix = np.array([[357.906682, 0.000000, 332.457325],
                [0.000000, 356.170785, 253.010996],
                [0.000000, 0.000000, 1.000000]], dtype=np.float64)

dist_coeffs = np.array([-0.274053, 0.049790, -0.001539, -0.003057, 0.000000], dtype=np.float64)

while True:
    ret, frame = capture.read()
    frame_undist = cv2.undistort(frame, camera_matrix, dist_coeffs, None)
    cv2.imshow("VideoFrame", frame_undist)
    key = cv2.waitKey(1)
    if key == ord('a'):
        cv2.imwrite('C:/study_git/depth_estimation/Depth-Estimation/extrinsic/pictures/extrinsic_pic'+str(n)+'.png', frame_undist)
        # C:\study_git\depth_estimation\Depth-Estimation\extrinsic
        n += 1

    elif key == ord('q'):
        break

capture.release()
cv2.destroyAllWindows()