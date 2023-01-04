#!usr/bin/python

from apriltag import apriltag
import threading
import numpy as np
import cv2 as cv
import time
import csv
import pandas as pd

cam_mat = [[566.17993276,   0.,        318.86306433],
				[  0.,         567.08593332, 212.10183606],
				[  0. ,          0.,           1.,        ]]
				
				
cam_matrix = np.asarray(cam_mat)
				
				
dist_mat = [[-1.26091809e+01, -7.63465264e+01, -1.56168174e-02,  5.30016919e-03,
   1.03834540e+03, -1.28762768e+01, -7.05881320e+01,  1.00726917e+03,
   0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
   0.00000000e+00,  0.00000000e+00]]

				
dist_matrix= np.asarray(dist_mat)	


class AprilTag:

    def __init__(self, frame=None):
        self.frame = frame
        self.stopped = False
        self.tvector=np.array([0,0,0])
        self.rvector=np.array([0,0,0])
        
    

    
    def start(self):
        threading.Thread(target=self.localize, args=()).start()
        return self

    def localize(self):
        # implement your function here!
        global cam_matrix
        global dist_matrix
        
      

        # with open('/home/group4/ege_ConsRob/constructionRobotics/data.csv', 'w') as csv_file:
                        
                # fieldnames=['rvecs', 'tvecs']
                
                # writer=csv.DictWriter(csv_file, fieldnames=fieldnames)
                # info={"rvecs":self.rvector,"tvecs":self.tvector}
                
        while True:
            if self.frame is None:
                print("No tags detected.")
            
            else:
                #image=self.frame
                img= cv.cvtColor(self.frame, cv.COLOR_BGR2GRAY)
                # apriltag detection
                detector= apriltag('tagStandard41h12')
                time.sleep(1)
                result= detector.detect(img)
                
                if len(result) > 0:
                    for r in result:
                        a =(r["lb-rb-rt-lt"])
                    
                            
                        image_points = np.asarray(a)
                        
                                        
                                        
                        # 3d real world points always from center of tag

                        object_point = [[-0.05 , 0.05 , 0],
                                        [0.05, 0.05, 0],
                                        [0.05, -0.05, 0],
                                        [-0.05, -0.05, 0]]
                                        
                        object_points= np.asarray(object_point)

                        # cv pose estimations

                        ret, rvecs, tvecs = cv.solvePnP(object_points, image_points, cam_matrix, dist_matrix, flags=cv.SOLVEPNP_IPPE_SQUARE)
                        
                        
                        # # print(type(tvecs))
                        self.tvector=tvecs
                        self.rvector=rvecs
                        print("rotation is:",self.rvector)
                        print("\n trans is:", self.tvector)
                        
                        
                        #----------------------------------------------------------------------------------------------------------------
                        # np.savetxt("/home/group4/ege_ConsRob/constructionRobotics/data.csv",self.tvector, delimiter=",")
                        pd.DataFrame(tvecs).to_csv("/home/group4/ege_ConsRob/constructionRobotics/trans.csv", mode='a', header=["Translational Vector"], index=None)
                        pd.DataFrame(rvecs).to_csv("/home/group4/ege_ConsRob/constructionRobotics/rot.csv", mode='a', header=["Rotational Vector"], index=None)
                        
                        #----------------------------------------------------------------------------------------------------------------
                                
                        # with open('data.csv', 'w') as csv_file:
                                        
                                # fieldnames=['rvecs', 'tvecs']
                                
                                # writer=csv.DictWriter(csv_file, fieldnames=fieldnames)
                                
                                # info={
                                        # "rvecs":rvecs,
                                        # "tvecs":tvecs
                                        # }
                                # writer.writerow(info)
                               
                        
                elif len(result)<=0:
                        print("nothing to detect")
        pass
    
    def stop(self):
        self.stopped = True
