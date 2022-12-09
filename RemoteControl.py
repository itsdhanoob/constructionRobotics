#!usr/bin/python
import threading
import time
import sys
sys.path.append("/home/group4/OpenCat/serialMaster/") # set the path to where Opencat is located on your Pi
from ardSerial import *

class MovementCommand:
    # Class that manages the movement commands and sends them to the NyBoard

    def __init__(self):
        self.stopped = False
        self.command = ""  # can contain gaits and skills
        self.key = 255  # the last key that was pressed for remote control
        self.goodPorts = {}
        connectPort(self.goodPorts) # for connecting to the NyBoard

    def start(self, keyPressed):
        # start the thread
        threading.Thread(target=keepCheckingPort, args=(goodPorts,)).start()
        threading.Thread(target=self.remoteControl, args=(keyPressed,)).start()
        return self

    def rest(self):
        self.command = "d" # lay down
        send(self.goodPorts, [self.command, 1])

    def remoteControl(self, keyPressed):
        # remote control for the mIDOG with the keyboard

        skill = ""  # skills are executed only once and not continouos like gaits
        direction = ""  # direction for gaits
        gait = "wk" # default gait is walk, gyro should be turned off when using walk
        key = self.key  # input for remoteControl
        
        time.sleep(2)

        while not self.stopped:
            keyPressed.wait(0.001)  # Event for new keyboard input with timeout
            if keyPressed.is_set() == True:
                # a key was pressed
                keyPressed.clear()  # reset the flag
                key = self.key

                # select skill or gait
                if key == " ":  # spacebar
                    skill = "balance"  # to stop the mIDOG
                elif key == "e":
                    gait = "wk"  # walk
                elif key == "r":
                    gait = "cr"  # crawl
                elif key == "t":
                    gait = "tr"  # trot
                elif key == "w":
                    direction = "F" # forward
                elif key == "a":
                    direction = "L" # left
                elif key == "d":
                    direction = "R" # right
                elif key == "s":
                    direction = "B" # back
                elif key == "q":
                    skill = "rc"  # recovery
                elif key == "f":
                    skill = "hi"  # hi sequence
                elif key == "g":
                    skill = "g"  # turn gyro on/off
                elif key == "q":
                    skill = "rest"  # lay down
                elif key == "x":
                    skill = "sit" # sit
                elif key == "c":
                    skill = "c"  # calibration mode
                elif key == "v":
                    skill = "pee" # pee

                # delete direction if skill was selected
                if skill != "":
                    direction == ""

                # compose command string which is readable by the NyBoard
                if skill == "rest":
                    self.command = "d"
                elif skill == "g" or skill == "c":
                    self.command = skill
                elif skill != "":
                    self.command = "k" + skill
                elif direction != "":
                    if direction == "B":
                        self.command = "kbk"
                    else:
                        self.command = "k" + gait + direction

                # if a gait is selected, but no direction
                if skill == "" and (gait == "cr" or gait == "wk" or gait == "tr") and direction == "":
                    # these gaits need a direction
                    print("Need a direction for gait " + gait)
                else:
                    # print and send command
                    print("Executing command: " + self.command)
                    send(self.goodPorts, [self.command, "\n"])

                # sleep for a short time
                time.sleep(0.1)
                
                # delete skill
                skill = ""

    def stop(self):
        # shut down servos of the mIDOG
        self.rest()
        time.sleep(1)
        # stop thread
        self.stopped = True
        # close serial communication with the NyBoard
        time.sleep(0.2)
        closeAllSerial(self.goodPorts)
