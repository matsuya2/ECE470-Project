"""camera_stand controller."""
from controller import Robot
import numpy as np
import time 



# time in [ms] of a simulation step
TIME_STEP = 32
# create the Robot instance.
robot = Robot()



camera= robot.getCamera("camera")
camera.enable(3)
camera.recognitionEnable(3)
objects= camera.getRecognitionNumberOfObjects
camera.getRecognitionObjects
#Image= camera.getImage
#Red= camera.imageGetRed(Image,camera.getWidth(),5,10)
#print(Image)
    








while robot.step(TIME_STEP) != -1: # repeats every TIME_STEP millisecond of simulation

    
    pass
    

    
    