#!/usr/bin/env python3
import rospy
import math
import sympy
import numpy as np
from nav_msgs.msg import Odometry



STATE_SIZE = 3  # State size [x,y,yaw]
landmark_indexes=0
landmarks=[]


def ekf_slam(xEst, PEst, u, z):
    """
    Performs an iteration of EKF SLAM from the available information.

    :param xEst: the belief in last position
    :param PEst: the uncertainty in last position
    :param u:    the control function applied to the last position
    :param z:    measurements at this step
    :returns:    the next estimated position and associated covariance
    """

    # Predict
    xEst, PEst = predict(xEst, PEst, u)
    initP = np.eye(2)

    # Update
    for measurement in z:
        H_f,Psi_f,difference_f,K=data_association(xEst,PEst,measurement,z)
        xEst, PEst = update(xEst, PEst, measurement,H_f,Psi_f,difference_f,K)
        return xEst, PEst

def predict(xEst, PEst, u):
    """
    Performs the prediction step of EKF SLAM

    :param xEst: nx1 state vector
    :param PEst: nxn covariance matrix
    :param u:    2x1 control vector
    :returns:    predicted state vector, predicted covariance, jacobian of control vector, transition fx
    """
    
    delta_t=0.01
    x=xEst[0]+u[0]*np.cos(xEst[2])*delta_t
    y=xEst[1]+u[0]*np.sin(xEst[2])*delta_t
    theta=xEst[2]+u[1]*delta_t
    if (theta > np.pi):
        theta -= 2 * np.pi
    elif (theta < -np.pi):
        theta += 2 * np.pi

    xEst[0]=x
    xEst[1]=y
    xEst[2]=theta

    G = np.identity(3 + 2 * len(landmark_indexes))
    G[0][2] = - u[0] * delta_t * np.sin(xEst[2])
    G[1][2] = u[0] * delta_t * np.cos(xEst[2])

    PEst=G.dot(PEst).dot(G.T)
    
    #M = np.array([[0.5**2, 0], [0, 0.5**2]])

    #PEst = F.T @ PEst @ F + V @ M @ V.T
    return xEst, PEst

def data_association(xEst,PEst,measurement,z):
    # measurement=[x,y,range,bearing]

    # Get current robot state, measurement
    x_t=xEst[0]
    y_t=xEst[1]
    theta_t=xEst[2]
    range_t=measurement[2]
    bearing_t=measurement[3]


    landmark_expected_x=measurement[0]
    landmark_expected_y=measurement[1]

    min_distance = 1e16

    for i in range(1,landmark_indexes+1):

        # Get current landmark estimate
        x_l=xEst[2*i+1]
        y_l= xEst[2*i+2]
        delta_x = abs(x_l - x_t)
        delta_y = abs(y_l - y_t)
        q = delta_x ** 2 + delta_y ** 2
        range_expected = np.sqrt(q)
        bearing_expected = np.arctan2(delta_y, delta_x) - theta_t

        # Compute Jacobian H of Measurement Model
        # Jacobian: H = d h(x_t, x_l) / d (x_t, x_l)
        #        1 0 0  0 ...... 0   0 0   0 ...... 0
        #        0 1 0  0 ...... 0   0 0   0 ...... 0
        # F_x =  0 0 1  0 ...... 0   0 0   0 ...... 0
        #        0 0 0  0 ...... 0   1 0   0 ...... 0
        #        0 0 0  0 ...... 0   0 1   0 ...... 0
        #          (2*landmark_idx - 2)
        #          -delta_x/√q  -delta_y/√q  0  delta_x/√q  delta_y/√q
        # H_low =   delta_y/q   -delta_x/q  -1  -delta_y/q  delta_x/q
        #               0            0       0       0          0
        # H = H_low x F_x
        F_x = np.zeros((5, 3 + 2 * len(landmark_indexes)))
        F_x[0][0] = 1.0
        F_x[1][1] = 1.0
        F_x[2][2] = 1.0
        F_x[3][2 * i + 1] = 1.0
        F_x[4][2 * i + 2] = 1.0
        H_1 = np.array([-delta_x/np.sqrt(q), -delta_y/np.sqrt(q), 0, delta_x/np.sqrt(q), delta_y/np.sqrt(q)])
        H_2 = np.array([delta_y/q, -delta_x/q, -1, -delta_y/q, delta_x/q])
        H_3 = np.array([0, 0, 0, 0, 0])
        H = np.array([H_1, H_2, H_3]).dot(F_x)

        # Compute Mahalanobis distance
        Psi = H.dot(PEst).dot(H.T)
        difference = np.array([range_t - range_expected, bearing_t - bearing_expected, 0])
        Pi = difference.T.dot(np.linalg.inv(Psi)).dot(difference)

        # Get landmark information with least distance
        if Pi < min_distance:
            min_distance = Pi
            # Values for measurement update
            H_f = H
            Psi_f = Psi
            difference_f = difference
        
        return H_f,Psi_f,difference_f,K

def update(xEst, PEst, z, H_f,Psi_f, difference_f,K):
    """
    Performs the update step of EKF SLAM

    :param xEst:  nx1 the predicted pose of the system and the pose of the landmarks
    :param PEst:  nxn the predicted covariance
    :param z:     the measurements read at new position
    :param initP: 2x2 an identity matrix acting as the initial covariance
    :returns:     the updated state and covariance for the system
    """
    K = PEst.dot(H_f.T).dot(np.linalg.inv(Psi_f))
    innovation = K.dot(difference_f)
    xEst[0]=xEst[0]+innovation
    xEst[1]=xEst[1]+innovation
    xEst[2]=xEst[2]+innovation
    
    # Update covariance
    PEst = (np.identity(3 + 2 * len(landmark_indexes)) - K.dot(H_f)).dot(PEst)

    #xEst[2] = pi_2_pi(xEst[2])
    return xEst, PEst


def callback(data):
	global car_coordinate 
	car_coordinate=[0,0]
	car_coordinate[0]=data.pose.pose.position.x
	car_coordinate[1]=data.pose.pose.position.y
	print("x=",car_coordinate[0])
	print("y=",car_coordinate[1])
	

def start():
    # State Vector [x y yaw ]
    xEst = np.zeros((STATE_SIZE, 1))
    PEst = 1e-6 * np.full((3 + 2 * len(landmark_indexes), 3 + 2 * len(landmark_indexes)), 1)
    for i in range(3, 3 + 2 * len(landmark_indexes)):
        PEst[i][i] = 1e6
    xEst, PEst = ekf_slam(xEst, PEst, u)

    






if __name__ == '__main__':
	print("EKF_SLAM")
	rospy.init_node('ekf_slam',anonymous = True)
	
	rospy.spin()