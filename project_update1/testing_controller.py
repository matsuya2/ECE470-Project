"""testing_controller controller."""
from controller import Robot
import numpy as np
import time 

# time in [ms] of a simulation step
TIME_STEP = 32

# create the Robot instance.
robot = Robot()

# initialize gripper motors
f1j1_motor = robot.getMotor('finger_1_joint_1')
f2j1_motor = robot.getMotor('finger_2_joint_1')
fmj1_motor = robot.getMotor('finger_middle_joint_1')

#initialize gripper position
f1j1_motor.setPosition(0)
f2j1_motor.setPosition(0)
fmj1_motor.setPosition(0)

#close gripper 
f1j1_motor.setPosition(np.pi/4)
f2j1_motor.setPosition(np.pi/4)
fmj1_motor.setPosition(np.pi/4)


#initialize arm joint motors
elbow_joint_motor = robot.getMotor("elbow_joint")
shoulder_lift_joint_motor = robot.getMotor("shoulder_lift_joint")
shoulder_pan_joint_motor = robot.getMotor("shoulder_pan_joint")
wrist_1_joint_motor = robot.getMotor("wrist_1_joint")
wrist_2_joint_motor = robot.getMotor("wrist_2_joint")
wrist_3_joint_motor = robot.getMotor("wrist_3_joint")

#set initial arm joint positions
elbow_joint_motor.setPosition(0)
shoulder_lift_joint_motor.setPosition(0) #-np.pi/2
shoulder_pan_joint_motor.setPosition(0)
wrist_1_joint_motor.setPosition(0)
wrist_2_joint_motor.setPosition(0)
wrist_3_joint_motor.setPosition(0)

#initialize arm joint position sensors
elbow_joint_sensor = robot.getPositionSensor("elbow_joint_sensor")
shoulder_lift_joint_sensor = robot.getPositionSensor("shoulder_lift_joint_sensor")
shoulder_pan_joint_sensor = robot.getPositionSensor("shoulder_pan_joint_sensor")
wrist_1_joint_sensor = robot.getPositionSensor("wrist_1_joint_sensor")
wrist_2_joint_sensor = robot.getPositionSensor("wrist_2_joint_sensor")
wrist_3_joint_sensor = robot.getPositionSensor("wrist_3_joint_sensor")



#enabling sensors (each sensor needs to be enabled and you need to specify an update delay)
elbow_joint_sensor.enable(TIME_STEP)
shoulder_lift_joint_sensor.enable(TIME_STEP)
shoulder_pan_joint_sensor.enable(TIME_STEP)
wrist_1_joint_sensor.enable(TIME_STEP)
wrist_2_joint_sensor.enable(TIME_STEP)
wrist_3_joint_sensor.enable(TIME_STEP)


dist_sensor = robot.getDistanceSensor("distance sensor")
dist_sensor.enable(TIME_STEP)

# cam_sensor = robot.getCamera("camera")
# cam_sensor.enable(TIME_STEP)
# img = cam_sensor.saveImage('test_img.png',50)
# print(img)

k=0
while robot.step(TIME_STEP) != -1:

    # dist_sensor_val = dist_sensor.getValue()
    # print(dist_sensor_val)

    print("The elbow_joint_sensor reads: %.2f radian" %elbow_joint_sensor.getValue())
    print("The shoulder_lift_joint_sensor reads: %.2f radian" %shoulder_lift_joint_sensor.getValue())
    print("The shoulder_pan_joint_sensor reads: %.2f radian" %shoulder_pan_joint_sensor.getValue())
    print("The wrist_1_joint_sensor reads: %.2f radian" %wrist_1_joint_sensor.getValue())
    print("The wrist_2_joint_sensor reads: %.2f radian" %wrist_2_joint_sensor.getValue())
    print("The wrist_3_joint_sensor reads: %.2f radian" %wrist_3_joint_sensor.getValue())
    print("")
    
    if k == 50:
        shoulder_lift_joint_motor.setPosition(-np.pi/2)
    k+=1
    
    # pass