import numpy as np
import cv2 as cv
import glob



################ FIND CHESSBOARD CORNERS - OBJECT POINTS AND IMAGE POINTS #############################

chessboardSize = (8,6) #width*height
frameSize = (640,480)



# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)


# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)

size_of_chessboard_squares_mm = 25
objp = objp * size_of_chessboard_squares_mm


# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

num_img=0
images = glob.glob('*.jpg')

for image in images:
	img = cv.imread(image)
	gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
		 
		# Find the chess board corners
	ret, corners = cv.findChessboardCorners(img, chessboardSize, flags=cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_FAST_CHECK+
																				  cv.CALIB_CB_NORMALIZE_IMAGE+cv.CALIB_CB_FILTER_QUADS)
		# If found, add object points, image points (after refining them)
	if ret == True:
		objpoints.append(objp)
		corners2 = cv.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
		imgpoints.append(corners)

		# Draw and display the corners
		cv.drawChessboardCorners(img, chessboardSize, corners2, ret)
		num_img+=1
		cv.imshow('img', img)
		cv.waitKey(0)
	else:
		print("somethings wrong")

    img = cv.imread(image)
    # norm= np.zeros((800,800))
    # norm_img= cv.normalize(img, norm,0,255,cv.NORM_MINMAX)
    # img = cv.resize(img, (640, 480))
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    cv.imshow("img",gray)
    cv.waitKey(0)

    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(img, chessboardSize, flags=cv.CALIB_CB_ADAPTIVE_THRESH + cv.CALIB_CB_FAST_CHECK+
                                                                    cv.CALIB_CB_NORMALIZE_IMAGE+cv.CALIB_CB_FILTER_QUADS)
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray, corners, (14,14), (-1,-1), criteria)
        imgpoints.append(corners)

        # Draw and display the corners
        cv.drawChessboardCorners(img, chessboardSize, corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(0)
    else:
        print("somethings wrong")

cv.destroyAllWindows()
print('calibrating_camera')

############# CALIBRATION #######################################################

ret, cameraMatrix, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, frameSize, None, None)

print('/n ret is:', ret)
print('/n camera matrix is:', cameraMatrix)
print('/n distortion is:', dist)


