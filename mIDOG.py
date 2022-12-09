#!usr/bin/python
import time
import threading
import sys
import select
import tty
import termios
import yaml
import os

import Camera
import RemoteControl
import AprilTagLocalization
import cv2

# list of all classes with an active thread
threads=[]

def printHelp(settings):
    print("\nHelp display:\n")
    print("Settings:")
    if settings["videostream"] == True:
        print("\tVideostream: enabled")
    else:
        print("\tVideostream: disabled")
    if settings["remoteControl"] == True:
        print("\tRemote control: enabled")
        print("\t\tSelect a gait with 'e', 'r' or 't' and a direction")
        print("\t\twith 'w', 'a', 's' and 'd'. Turn gyro on/off with 'g'.")
        print("\t\tThe most stable gait is walk (selected by default")
        print("\t\tor by pressing 'e') with the gyro turned off.")
        print("\t\tPress 'q' to recover after falling over.")
        print("\t\tPress the spacebar for balancing and 'x' to sit.")
    else:
        print("\tRemote control: disabled")
    if settings["aprilTagLocalization"] == True:
        print("\tLocalizing AprilTags: enabled")
    else:
        print("\tLocalizing AprilTags: disabled")
    print("Settings can be changed in settings.yaml.")
    print("\nPress 'Esc' for exit.\n")

def main():
    
    # Welcome message
    print("Hello, I am mIDOG!")
    print("Press 'Esc' for exit and 'h' for help.\n")

    # read settings.yaml file
    with open("settings.yaml", "r") as file:
        settings = yaml.safe_load(file)
    if settings["videostream"] == True or settings["aprilTagLocalization"] == True:
        # get the videostream from Pi camera
        video_getter = Camera.VideoGet(0)
        threads.append(video_getter)
        video_getter.start()
        if settings["videostream"] == True:
            video_shower = Camera.VideoShow(video_getter.frame)
            threads.append(video_shower)
            video_shower.start()
        if settings["aprilTagLocalization"] == True:
            # Videostream with two different threads
            localizer = AprilTagLocalization.AprilTag()
            threads.append(localizer)
            localizer.start()
 
    if settings["remoteControl"] == True:
        # Event if a key was pressed for remote control
        keyPressed = threading.Event()
        # start Thread for sending movement commands
        movement_commander = RemoteControl.MovementCommand() 
        threads.append(movement_commander)
        movement_commander.start(keyPressed)

    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        key = ""
  
        while True:
            
            if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
                key = sys.stdin.read(1)
                if key == "\x1b":         # \x1b is ESC
                    print("\nShutting down\n")
                    break
                    
                elif key == "p":
                    printHelp(settings)
                    while True: 
                        path= ('/home/group4/construction-robotics-ws-2022_23/cc')
                        for i in path:
                        image= cv2.imwrite(path+"/Image_{}.png".format(img_num),video_getter.frame)
                        img_num++
                    
                    
                elif key == "h":
                    printHelp(settings)
            if settings["remoteControl"] == True:
                # send keyboard input to remote control if key is not empty
                if key != "":
                    movement_commander.key = key
                    keyPressed.set()
                    key = "" # reset key
            
            if settings["videostream"] == True:
                # show new frame
                video_shower.frame = video_getter.frame
            if settings["aprilTagLocalization"] == True:
                # show new frame
                localizer.frame = video_getter.frame
                
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    # stop all threads
    for thread in threads:
        thread.stop()
    time.sleep(2)

if __name__ == "__main__":
    main()
