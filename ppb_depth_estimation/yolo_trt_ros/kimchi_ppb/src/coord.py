import cv2

cap = cv2.VideoCapture(0)
if cap.isOpened():

    while True:
        ret, img = cap.read()
        cv2.imshow('img', img)
        if cv2.waitKey(1) != -1:
            break
cap.release()
cv2.destroyAllWindows()
