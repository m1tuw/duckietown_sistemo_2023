import numpy as np
import cv2 as cv

a = cv.aruco
dic = a.getPredefinedDictionary(cv.aruco.DICT_7X7_100)
par = a.DetectorParameters_create()


cap = cv.VideoCapture(0)
if not cap.isOpened():
 print("Cannot open camera")
 exit()
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    # Our operations on the frame come here
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    (corners, imID, rejected) = a.detectMarkers(gray, dic, parameters = par)
    if type(imID) != type(None):
        for i in range(len(imID)):
            cv.putText(gray, imID[i].astype(str)[0], corners[i][0][0].astype(int), cv.FONT_HERSHEY_PLAIN, 5, (255,255, 255))

    # Display the resulting frame
    cv.imshow('frame', gray)
    if cv.waitKey(1) == ord('q'):
        break
# When everything done, release the capture
cap.release()
cv.destroyAllWindows()
