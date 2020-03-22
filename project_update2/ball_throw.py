"""ball_throw controller."""
from controller import Robot
import numpy as np
import time 
from custom_module import inv_kin
from scipy.linalg import logm, expm






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

################################################# FUNCTIONS ########################################


init_gripper()
init_ur3()



Rsd = np.array([ 
    [-1, 0, 0], 
    [0, 1, 0], 
    [0, 0, -1] 
    ]) # desired orientation of end-effector frame expressed in base frame
Psd = np.transpose(np.array([ [0.457*(0.8), 0.202*(1.4), 0.25] ])) # desired position of end-effector frame expressed in base frame (columm vector)
ew = 0.1
ev = 0.001



Tsd = np.bmat([ 
    [Rsd, Psd], 
    [np.array([[0, 0, 0, 1]])]
    ])
M = np.array([ 
    [-1, 0, 0, 0.457],
    [0, 1, 0, 0.202],
    [0, 0, -1, 0.069],
    [0, 0, 0, 1] 
    ])
# each columm is a screw axis (in body frame)
# first column corresponds to screw axis closest to base frame
q1 = np.transpose(np.array([ [0.457, -0.202, 0] ]))
q2 = np.transpose(np.array([ [0.457, 0, -0.083] ]))
q3 = np.transpose(np.array([ [0.213, 0, -0.083] ]))
q4 = np.transpose(np.array([ [0, 0, -0.083] ]))
q5 = np.transpose(np.array([ [0, -0.082, 0] ]))
q6 = np.transpose(np.array([ [0, 0, 0] ]))

w1 = np.transpose(np.array([ [0, 0, -1] ]))
w2 = np.transpose(np.array([ [0, 1, 0] ]))
w3 = np.transpose(np.array([ [0, 1, 0] ]))
w4 = np.transpose(np.array([ [0, 1, 0] ]))
w5 = np.transpose(np.array([ [0, 0, 1] ]))
w6 = np.transpose(np.array([ [0, 1, 0] ]))

v1 = np.cross(-w1, q1, axis=0)
v2 = np.cross(-w2, q2, axis=0)
v3 = np.cross(-w3, q3, axis=0)
v4 = np.cross(-w4, q4, axis=0)
v5 = np.cross(-w5, q5, axis=0)
v6 = np.cross(-w6, q6, axis=0)

beta_screw_matrix = np.bmat([
    [w1, w2, w3, w4, w5, w6],
    [v1, v2, v3, v4, v5, v6]
])

beta_screw_skew_matrices = np.zeros((4,4,6)) #3d array of skewed beta screw matrices
for i in range(beta_screw_matrix.shape[1]):
    w = beta_screw_matrix[0:3,i]
    w_skew = np.array([
        [0, -w[2], w[1]],
        [w[2], 0, -w[0]],
        [-w[1], w[0], 0]
    ])
    v = np.reshape(beta_screw_matrix[3:,i],(-1,1))
    beta_screw_skew_matrices[:,:,i] = np.bmat([
        [w_skew, v],
        [np.array([[0, 0, 0, 0]])]
    ])

theta_old = np.zeros((6,1))



theta_new = inv_kin(beta_screw_skew_matrices, beta_screw_matrix, M, Tsd, theta_old, ew, ev)





j1_motor.setPosition(theta_new[0,0])
j2_motor.setPosition(theta_new[1,0])
j3_motor.setPosition(theta_new[2,0])
j4_motor.setPosition(theta_new[3,0])
j5_motor.setPosition(theta_new[4,0])
j6_motor.setPosition(theta_new[5,0])









# dist_sensor = robot.getDistanceSensor("distance sensor")
# dist_sensor.enable(TIME_STEP)

# cam_sensor = robot.getCamera("camera")
# cam_sensor.enable(TIME_STEP)
# img = cam_sensor.saveImage('test_img.png',50)
# print(img)

k = 0
while robot.step(TIME_STEP) != -1: # repeats every TIME_STEP millisecond of simulation

    if k == 20:
        Rsd = np.array([ 
        [0, 1, 0], 
        [1, 0, 0], 
        [0, 0, -1] 
        ]) # desired orientation of end-effector frame expressed in base frame
        Psd = np.transpose(np.array([ [0.457*(0.8), -0.202*(1.4), 0.25*(1.3)] ])) # desired position of end-effector frame expressed in base frame (columm vector)
        Tsd = np.bmat([ 
            [Rsd, Psd], 
            [np.array([[0, 0, 0, 1]])]
            ])
    
        theta_new = inv_kin(beta_screw_skew_matrices, beta_screw_matrix, M, Tsd, theta_new, ew, ev)

        j1_motor.setPosition(theta_new[0,0])
        j2_motor.setPosition(theta_new[1,0])
        j3_motor.setPosition(theta_new[2,0])
        j4_motor.setPosition(theta_new[3,0])
        j5_motor.setPosition(theta_new[4,0])
        j6_motor.setPosition(theta_new[5,0])

    # dist_sensor_val = dist_sensor.getValue()
    # print(dist_sensor_val)
    #set ur3 joint positions to throw
    # if k == 25:
        # j5_motor.setPosition(np.pi/2)
        # j6_motor.setPosition(np.pi/2)
    # elif k == 50:
        # j2_motor.setAcceleration(1500000)
        # j2_motor.setVelocity(3.14)
        # j2_motor.setPosition(-np.pi)
    # elif k == 55:
        # open_gripper()
    

    # if np.abs(j2_sensor.getValue()-(-np.pi)) < 1:
        # open_gripper()
    
    k += 1
    
    pass
    
    