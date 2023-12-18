import numpy as np
import cv2 as cv

aruco = cv.aruco
dic = aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_100)
par = aruco.DetectorParameters_create()
calib_data = np.load("./calib_data/MultiMatrix.npz")

cap = cv.VideoCapture(0)
if not cap.isOpened():
 print("Cannot open camera")
 exit()

# Inicia un loop que revisa frame a frame el video
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    # Our operations on the frame come here
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    (corners, imID, rejected) = aruco.detectMarkers(gray, dic, parameters = par)


    if imID is not None:
       aruco.drawDetectedMarkers(frame, corners)
       #camMatrix, distCoef, rVector, tVector
       rvec, tvec, _objPoints = aruco.estimatePoseSingleMarkers(corners, 100, calib_data["camMatrix"], calib_data["distCoef"])
       cv.drawFrameAxes(frame, calib_data["camMatrix"], calib_data["distCoef"], rvec, tvec, 100)
    # Display the resulting frame
    cv.imshow('frame', frame)
    if cv.waitKey(1) == ord('q'):
        break
# When everything done, release the capture
cap.release()
cv.destroyAllWindows()