import cv2
video_capture = cv2.VideoCapture(cv2.CAP_V4L2)
video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)


while True:
    result, video_frame = video_capture.read() # read frames from camera
    if result is False:
        break
    cv2.imshow("USB Camera Test", video_frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

video_capture.release()
cv2.destroyAllWindows()
