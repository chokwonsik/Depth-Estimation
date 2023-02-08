import cv2

capture = cv2.VideoCapture(1)
capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

n = 1

while True:
    ret, frame = capture.read()
    cv2.imshow("VideoFrame", frame)
    key = cv2.waitKey(1)
    if key == ord('a'):
        # cv2.imwrite('C:/study_git/make_kimchi/Let-s_make_kimchi/picture_/left/left'+str(n)+'.png', frame)
        # cv2.imwrite('C:/study_git/make_kimchi/Let-s_make_kimchi/picture_/right/right'+str(n)+'.png', frame)
        # cv2.imwrite('C:/study_git/make_kimchi/Let-s_make_kimchi/picture_/stop/stop'+str(n)+'.png', frame)
        # cv2.imwrite('C:/study_git/make_kimchi/Let-s_make_kimchi/picture_/crosswalk/crosswalk'+str(n)+'.png', frame)
        cv2.imwrite('C:/study_git/make_kimchi/Let-s_make_kimchi/picture_/traffic_light/traffic_light'+str(n)+'.png', frame)
        n += 1

    elif key == ord('q'):
        break

capture.release()
cv2.destroyAllWindows()