import cv2

cam = cv2.VideoCapture(0)
while True:
    ret, frame = cam.read()
    resizedFrame = cv2.resize(frame, (1080, 810))
    cv2.imshow('frame', resizedFrame)

    if cv2.waitKey(33) & 0xFF == ord('q'):
        break

cam.release
cv2.destroyAllWindows