#!usr/bin/python

from apriltag import apriltag
import threading

class AprilTag:

    def __init__(self, frame=None):
        self.frame = frame
        self.stopped = False
    
    def start(self):
        threading.Thread(target=self.localize, args=()).start()
        return self

    def localize(self):
        # implement your function here!
        pass
    
    def stop(self):
        self.stopped = True
