from apriltag import apriltag
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

cam_matrix = [[566.17993276,   0.,        318.86306433],
				[  0.,         567.08593332, 212.10183606],
				[  0. ,          0.,           1.,        ]]
				
dist_matrix = ([[-1.26091809e+01, -7.63465264e+01, -1.56168174e-02,  5.30016919e-03,
					[1.03834540e+03, [-1.28762768e+01, -7.05881320e+01,1.00726917e+03,
					[0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
						[0.00000000e+00,  0.00000000e+00,]]]]]])
				
# 2d points from apriltag ccw		
			
image_points = ([[221.46931458, 320.22940063],
				[416.48123169, 327.65234375],
				[417.38134766, 143.02896118],
				[233.71627808, 127.21188354]])
				
				
# 3d real world points alwyas from center of tag

object_points = [[-0.05 , -0.05 , 0],
				[0.05, -0.05, 0],
				[0.05, 0.05, 0],
				[-0.05, 0.05, 0]]

# cv pose estimations

ret, rvecs, tvecs = cv.solvePnP(object_points, image_points, cam_matrix, dist_matrix, flags=SOLVEPNP_IPPE_SQUARE)


print("rotation is:",rvecs)
print("/n trans is:", tvecs)


						
						
