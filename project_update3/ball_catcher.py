"""ball_catcher controller."""
from controller import Robot
import numpy as np
import time 
import core as mr #functions from modern robotics 
import custom_module as cm #functions we created 


# time in [ms] of a simulation step
TIME_STEP = 32
# create the Robot instance.
robot = Robot()


# all joint angles are in radians
# Only sensors need to be explicitly enabled 

################################################# FUNCTIONS ########################################
# close gripper 
def close_gripper():
    joint_ang = np.pi/5
    f1j1_motor.setPosition(joint_ang)
    f2j1_motor.setPosition(joint_ang)
    fmj1_motor.setPosition(joint_ang)
    return None

# open gripper 
def open_gripper():
    joint_ang = 0
    f1j1_motor.setPosition(joint_ang)
    f2j1_motor.setPosition(joint_ang)
    fmj1_motor.setPosition(joint_ang)
    return None
    
    
# initialize everything needed for gripper to work
def init_gripper():
    global f1j1_motor
    global f2j1_motor
    global fmj1_motor
    
    # Each device/actuator needs to be identified as seen below. 
    # initialize gripper motors (identifies a motor or device in the code)
    f1j1_motor = robot.getMotor('finger_1_joint_1')
    f2j1_motor = robot.getMotor('finger_2_joint_1')
    fmj1_motor = robot.getMotor('finger_middle_joint_1')
    
    #setting rotational acceleration of joints
    f1j1_motor.setAcceleration(1500000)
    f2j1_motor.setAcceleration(1500000)
    fmj1_motor.setAcceleration(1500000)
    
    #setting rotational velocity of joints
    f1j1_motor.setVelocity(100)
    f2j1_motor.setVelocity(100)
    fmj1_motor.setVelocity(100)
    
    
    #set initial gripper joint positions
    f1j1_motor.setPosition(0)
    f2j1_motor.setPosition(0)
    fmj1_motor.setPosition(0)
    
    return None

# initialize everything needed for ur3 robot arm to work
def init_ur3():
    global j1_motor
    global j2_motor
    global j3_motor
    global j4_motor
    global j5_motor
    global j6_motor
    
    global j1_sensor
    global j2_sensor
    global j3_sensor
    global j4_sensor
    global j5_sensor
    global j6_sensor
    
    
    #initialize ur3 joint motors
    #j1 to j6 ordered from base joint to wrist joint
    j1_motor = robot.getMotor("shoulder_pan_joint")
    j2_motor = robot.getMotor("shoulder_lift_joint")
    j3_motor = robot.getMotor("elbow_joint")
    j4_motor = robot.getMotor("wrist_1_joint")
    j5_motor = robot.getMotor("wrist_2_joint")
    j6_motor = robot.getMotor("wrist_3_joint")
    
    #set initial arm joint positions
    j1_motor.setPosition(0)
    j2_motor.setPosition(0) #-np.pi/2
    j3_motor.setPosition(0)
    j4_motor.setPosition(0)
    j5_motor.setPosition(0)
    j6_motor.setPosition(0)
    
    #initialize ur3 joint angle sensors
    j1_sensor = robot.getPositionSensor("shoulder_pan_joint_sensor")
    j2_sensor = robot.getPositionSensor("shoulder_lift_joint_sensor")
    j3_sensor = robot.getPositionSensor("elbow_joint_sensor")
    j4_sensor = robot.getPositionSensor("wrist_1_joint_sensor")
    j5_sensor = robot.getPositionSensor("wrist_2_joint_sensor")
    j6_sensor = robot.getPositionSensor("wrist_3_joint_sensor")
    
    # enabling sensors (each sensor needs to be enabled and you need to specify an update delay)
    j1_sensor.enable(TIME_STEP)
    j2_sensor.enable(TIME_STEP)
    j3_sensor.enable(TIME_STEP)
    j4_sensor.enable(TIME_STEP)
    j5_sensor.enable(TIME_STEP)
    j6_sensor.enable(TIME_STEP)
    return None
 
def set_joints(theta):
   j1_motor.setPosition(theta[0])
   j2_motor.setPosition(theta[1])
   j3_motor.setPosition(theta[2])
   j4_motor.setPosition(theta[3])
   j5_motor.setPosition(theta[4])
   j6_motor.setPosition(theta[5])
   return None
################################################# FUNCTIONS ########################################



init_gripper()
init_ur3()





# [0.457, 0.202, 0.25] this is the end-effector location at home position




# homogeneous transformation matrix expressing the position and orientation of the end-effector frame (at zero/home configuration) in the base frame 
M = np.array([ 
    [-1, 0,  0, 0.457],
    [ 0, 1,  0, 0.202],
    [ 0, 0, -1, 0.069],
    [ 0, 0,  0,     1] 
    ])




# each columm is a screw axis (in BASE FRAME)
# first column corresponds to screw axis closest to base frame
q1 = np.transpose(np.array([ [0, 0, 0] ]))
q2 = np.transpose(np.array([ [0, 0, 0.152] ]))
q3 = np.transpose(np.array([ [0.244, 0, 0.152] ]))
q4 = np.transpose(np.array([ [0.457, 0, 0.152] ]))
q5 = np.transpose(np.array([ [0.457, 0.110, 0] ]))
q6 = np.transpose(np.array([ [0.457, 0, 0.69] ]))

w1 = np.transpose(np.array([ [0, 0, 1] ]))
w2 = np.transpose(np.array([ [0, 1, 0] ]))
w3 = np.transpose(np.array([ [0, 1, 0] ]))
w4 = np.transpose(np.array([ [0, 1, 0] ]))
w5 = np.transpose(np.array([ [0, 0, -1] ]))
w6 = np.transpose(np.array([ [0, 1, 0] ]))

v1 = np.cross(-w1, q1, axis=0)
v2 = np.cross(-w2, q2, axis=0)
v3 = np.cross(-w3, q3, axis=0)
v4 = np.cross(-w4, q4, axis=0)
v5 = np.cross(-w5, q5, axis=0)
v6 = np.cross(-w6, q6, axis=0)

screw_matrix = np.array(np.bmat([
    [w1, w2, w3, w4, w5, w6],
    [v1, v2, v3, v4, v5, v6]
]))


theta0= np.zeros(6) #initial joint angles   (MUST BE SHAPE (6,))

#Desired orientation and position of end-effector
Rsd = np.array([ 
    [-1, 0, 0], 
    [0,  1, 0], 
    [0, 0, -1] 
    ]) # desired orientation of end-effector frame expressed in base frame
# Psd = np.array([ [0.457*(1)],[0.202*(1)],[0.25*(2) ] ]) # desired position of end-effector frame expressed in base frame (columm vector)
Psd = np.array([ [-0.26],[0.3],[0.403 ] ])
Tsd = np.array(np.bmat([ 
    [Rsd, Psd], 
    [np.array([[0, 0, 0, 1]])]
    ]))

# tolerances
ew = 0.01
ev = 0.01

Xstart = M
Xend = Tsd
Tf = 15
N = 100
method = 5
traj = mr.ScrewTrajectory(Xstart,Xend,Tf,N,method)


k = 0
j = 0
while robot.step(TIME_STEP) != -1: # repeats every TIME_STEP millisecond of simulation
    if k%100 == 0:
        theta0 = cm.random_theta(theta0,-50,50) # theta0 in radians (initializing random angle with range for each joint)
        theta_new, status = mr.IKinSpace(screw_matrix , M, Tsd, theta0, ew, ev) #theta_new in degrees
        print(status)
        theta_new = np.deg2rad(theta_new)
        set_joints(theta_new)
    
    k += 1
   
    
    

   


    
    
    
    pass
    

    
    