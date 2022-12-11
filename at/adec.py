import apriltag
import cv2 as cv 
import glob 

images= glob.glob("*.png")

for image in images:
	img= cv.imread(image)
	gray= cv.cvtColor(img, cv.COLOR_BGR2GRAY)
	
	
	# options = apriltag.FamilyDetectorOptions(families='tag36h11',
                                 # border=1,
                                 # nthreads=4,
                                 # quad_decimate=1.0,
                                 # quad_blur=0.0,
                                 # refine_edges=True,
                                 # refine_decode=False,
                                 # refine_pose=False,
                                 # debug=False,
                                 # quad_contours=True)
	
	
	detector= apriltag.Detector(apriltag.DetectorOptions(families ="tagStandard41h12 tagCustom48h12"))
	result= detector.detect(img)
	cv.imshow("dected_image",result)
	cv.waitKey(0)
	
	img=+1
