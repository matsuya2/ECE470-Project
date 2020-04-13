import numpy as np
#when you index out a vector from a 2d numpy array, it comes back as a 1d vector 
#must use virtualenv consistent with webot (python 2.7.16, numpy version 1.8, etc)



def random_theta(theta0,start_range, end_range):
    theta0[1] = np.random.randint(start_range,end_range)
    theta0[2] = np.random.randint(start_range,end_range)
    theta0[3] = np.random.randint(start_range,end_range)
    theta0[4] = np.random.randint(start_range,end_range)
    theta0[5] = np.random.randint(start_range,end_range)
    return np.deg2rad(theta0) #output is in radian