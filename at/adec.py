from apriltag import apriltag
import cv2 as cv 
import glob 

#imagepath = 'home/group4/constructionRobotics/at/Image_0.png'
images= glob.glob("*.png")

for image in images:
	img= cv.imread(image, cv.IMREAD_GRAYSCALE)
	#print(img)
	#gray= cv.cvtColor(img, cv.COLOR_BGR2GRAY)
		
		
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
		
		#options = apriltag.DetectorOptions(families ="tagStandard41h12 tagCustom48h12")
		#detector= apriltag.Detector(options)
	detector= apriltag('tagStandard41h12')
	result= detector.detect(img)
	print(result)
	
	t_size= .10
	
	c_param=[[566.17993276,   0.,        318.86306433],
			[  0.,         567.08593332, 212.10183606],
			[  0. ,          0.,           1.,        ]]
	
	
	# pose= estimate_tag_pose(img, estimate_tag_pose=True, tag_size=t_size)
	# print(pose)
	# detector= apriltag('tagStandard41h12')
	# detections = detector.detect(img, estimate_tag_pose=True, camera_params=c_param,tag_size=t_size)
	# print(detections)
	
	
	
	
