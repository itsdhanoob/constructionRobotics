from apriltag import apriltag
import numpy as np
import cv2 as cv 
import glob 

images= glob.glob("*.png")

for image in images:
	img= cv.imread(image, cv.IMREAD_GRAYSCALE)
# apriltag detection
	detector= apriltag('tagStandard41h12')
	result= detector.detect(img)
	print(result)

# pose estimation

cam_mat = [[566.17993276,   0.,        318.86306433],
				[  0.,         567.08593332, 212.10183606],
				[  0. ,          0.,           1.,        ]]
				
				
cam_matrix = np.asarray(cam_mat)
				
				
dist_mat = [[-1.26091809e+01, -7.63465264e+01, -1.56168174e-02,  5.30016919e-03,
   1.03834540e+03, -1.28762768e+01, -7.05881320e+01,  1.00726917e+03,
   0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
   0.00000000e+00,  0.00000000e+00]]

				
dist_matrix= np.asarray(dist_mat)				
				
# 2d points from apriltag ccw		
			
image_point = ([[221.46931458, 320.22940063],
				[416.48123169, 327.65234375],
				[417.38134766, 143.02896118],
				[233.71627808, 127.21188354]])
				
image_points= np.asarray(image_point)
				
# 3d real world points alwyas from center of tag

object_point = [[-0.05 , 0.05 , 0],
				[0.05, 0.05, 0],
				[0.05, -0.05, 0],
				[-0.05, -0.05, 0]]
				
object_points= np.asarray(object_point)

# cv pose estimations

ret, rvecs, tvecs = cv.solvePnP(object_points, image_points, cam_matrix, dist_matrix, flags=cv.SOLVEPNP_IPPE_SQUARE)


print("rotation is:",rvecs)
print("/n trans is:", tvecs)


						
						
