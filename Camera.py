#!usr/bin/python
import cv2
import threading
import time

class VideoGet:
    # Class that continuously gets frames from a VideoCapture object
    # with a dedicated thread.

    def __init__(self, src=0):
        self.stream = cv2.VideoCapture(src)
        (self.grabbed, self.frame) = self.stream.read()
        self.stopped = False

    def start(self):
        threading.Thread(target=self.get, args=()).start()
        return self

    def get(self):
        while not self.stopped:
            if not self.grabbed:
                self.stop()
                print("Can't grab an image. Stopping camera thread.\n")
            else:
                (self.grabbed, self.frame) = self.stream.read()

    def stop(self):
        self.stopped = True


class VideoShow:
    # Class that continuously shows a frame using a dedicated thread.

    def __init__(self, frame=None):
        self.frame = frame
        self.stopped = False

    def start(self):
        cv2.startWindowThread()
        threading.Thread(target=self.show, args=()).start()
        return self

    def show(self):
        while not self.stopped:
            cv2.imshow("mIDOG_Cam", self.frame)

    def stop(self):
        self.stopped = True
        # close mIDOG_Cam
        time.sleep(0.2)
        cv2.destroyAllWindows()