import numpy as np
from scipy.linalg import logm, expm
#when you index out a vector from a 2d numpy array, it comes back as a 1d vector 
#must use virtualenv consistent with webot (python 2.7.16, numpy version 1.8, etc)

Rsd = np.array([ 
    [-1, 0, 0], 
    [0, 1, 0], 
    [0, 0, -1] 
    ]) # desired orientation of end-effector frame expressed in base frame
Psd = np.transpose(np.array([ [0.457, 0.202, 0.25] ])) # desired position of end-effector frame expressed in base frame (columm vector)
ew = 0.1
ev = 0.01

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


def adj(T):
    R = T[0:3,0:3]
    p = T[0:3,3]
    p_skew = np.array([ 
        [0, -p[2], p[1]],
        [p[2], 0, -p[0]],
        [-p[1], p[0], 0]
    ])
    Ad = np.bmat([
        [R, np.zeros(R.shape)],
        [p_skew.dot(R), R]
    ])
    return Ad


def inv_kin(beta_screw_skew_matrices, beta_screw_matrix, M, Tsd, theta_old, ew, ev):
    b1_skew = beta_screw_skew_matrices[:,:,0]
    b2_skew = beta_screw_skew_matrices[:,:,1]
    b3_skew = beta_screw_skew_matrices[:,:,2]
    b4_skew = beta_screw_skew_matrices[:,:,3]
    b5_skew = beta_screw_skew_matrices[:,:,4]
    b6_skew = beta_screw_skew_matrices[:,:,5]

    b1 = np.reshape(beta_screw_matrix[:,0],(-1,1))
    b2 = np.reshape(beta_screw_matrix[:,1],(-1,1))
    b3 = np.reshape(beta_screw_matrix[:,2],(-1,1))
    b4 = np.reshape(beta_screw_matrix[:,3],(-1,1))
    b5 = np.reshape(beta_screw_matrix[:,4],(-1,1))
    b6 = np.reshape(beta_screw_matrix[:,5],(-1,1))

    vb = np.array([
        [10], 
        [10], 
        [10]
        ])
    #can use magnitude of vb and/or wb as constraint
    while np.linalg.norm(vb) > ev:
        Tsb = M.dot(expm(b1_skew*theta_old[0,0]))\
        .dot(expm(b2_skew*theta_old[1,0]))\
        .dot(expm(b3_skew*theta_old[2,0]))\
        .dot(expm(b4_skew*theta_old[3,0]))\
        .dot(expm(b5_skew*theta_old[4,0]))\
        .dot(expm(b6_skew*theta_old[5,0]))
        Tsb_inv = np.linalg.inv(Tsb)
        Vb_skew = logm(Tsb_inv.dot(Tsd))
        vb = np.reshape(Vb_skew[0:3,3],(-1,1))
        wb = np.transpose(np.array([ [Vb_skew[2,1], Vb_skew[0,2], Vb_skew[1,0]] ]))
        Vb_vec = np.bmat([
            [wb],
            [vb]
        ])
 
        Jb = np.bmat([
            [
            adj(expm(-b6_skew*theta_old[5,0]).dot(expm(-b5_skew*theta_old[4,0])).dot(expm(-b4_skew*theta_old[3,0])).dot(expm(-b3_skew*theta_old[2,0])).dot(expm(-b2_skew*theta_old[1,0]))).dot(b1),
            adj(expm(-b6_skew*theta_old[5,0]).dot(expm(-b5_skew*theta_old[4,0])).dot(expm(-b4_skew*theta_old[3,0])).dot(expm(-b3_skew*theta_old[2,0]))).dot(b2),
            adj(expm(-b6_skew*theta_old[5,0]).dot(expm(-b5_skew*theta_old[4,0])).dot(expm(-b4_skew*theta_old[3,0]))).dot(b3),
            adj(expm(-b6_skew*theta_old[5,0]).dot(expm(-b5_skew*theta_old[4,0]))).dot(b4),
            adj(expm(-b6_skew*theta_old[5,0])).dot(b5),
            b6
            ]
        ])
        theta_old = theta_old + np.linalg.pinv(Jb).dot(Vb_vec)
    
    theta_new = np.real(theta_old) 
    return theta_new
        

# I THINK THETA IS EXPLODING BECAUSE THETA_OLD IS NOT CLOSE ENOUGH TO DESIRED THETA