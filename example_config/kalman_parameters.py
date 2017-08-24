import numpy as np

### Define kalman filter properties ########
# TODO include dt? in case using different camera rates? somewhere downstream?
phi = np.matrix([                    [1, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                                     [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                                     [0, 0, 1, 1, 0, 0, 0, 0, 0, 0],
                                     [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                                     [0, 0, 0, 0, 1, 1, 0, 0, 0, 0],
                                     [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                                     [0, 0, 0, 0, 0, 0, 1, 1, 0, 0],
                                     [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                                     [0, 0, 0, 0, 0, 0, 0, 0, 1, 1],
                                     [0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])

H   = np.matrix([                    [1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                     [0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                                     [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                                     [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                                     [0, 0, 0, 0, 0, 0, 0, 0, 1, 0]])

# TODO how much is this contributing to the problem?
# should P0 be initialized w/ position of current contour and 0 velocity?
# maybe (max_expected_area + min_area) / 2 area, etc?
P0  = 10 * np.eye(10)
Q   = 50 * np.matrix(np.eye(10))
#R   = 1*np.matrix(np.eye(5))
R   = np.matrix([[0.39229913, 0.11372618, -0.12277696, -0.03180835, 0],
                 [0.11372618, 0.10118509, -0.03190704, -0.02905728, 0],
                 [-0.12277696, -0.03190704, 0.18215241, 0.08535093, 0],
                 [-0.03180835, -0.02905728, 0.08535093, 0.07658138, 0],
                 [0, 0, 0, 0, 1]])


gamma  = None
gammaW = None

max_covariance = 1000
max_velocity = 150

# TODO what exactly is this doing? name doesn't seem to reflect its function
# seems like it might just be masking irrelevant state variables?
association_matrix = np.matrix([[1,1,0,0,0]], dtype=float).T
association_matrix /= np.linalg.norm(association_matrix)
