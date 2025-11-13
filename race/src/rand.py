#!/usr/bin/env python3
import rospy
import math
import numpy as np
from rosgraph_msgs.msg import Clock
from race.msg import perception
from race.msg import ekf
from race.msg import perception
from datetime import datetime
from race.msg import slam
from geometry_msgs.msg import PoseStamped
from numpy.linalg import LinAlgError 
from race.msg import final_coordinates
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from race.msg import covariance


pub = rospy.Publisher('/final_coordinates', final_coordinates, queue_size=100)
cov_pub = rospy.Publisher('/covariance_ellipse', covariance, queue_size=1)


STATE_SIZE = 3  # State size [x,y,yaw]
prev_m=0
prev_s=0
prev_ns=0
delta_t=0
bt=True
cones=[]
global landmark_indexes
landmark_indexes=37
landmarks=[]
num_landmarks=0
prev_yaw=[0]
global u
u=[0.48,0]
global z
z=[]
global xEst
global PEst
prev_steer=0
steer=0
p=False
o=True
xEst = np.zeros((STATE_SIZE, 1))
H_f=0
sigma=[]
difference_f=0

PEst = 1e-15 * np.full((3 + 2 * landmark_indexes, 3 + 2 * landmark_indexes), 1)
for i in range(3, 3 + 2 * landmark_indexes):
    PEst[i][i] = 4              # 1e-15                   #-1

difference_f=0
H_f = np.zeros((3, 3 + 2 * landmark_indexes))
Psi_f = np.zeros((3, 3))
sutta=[0,0]
Q = np.diagflat(np.array([0.08, 0.08, 0.08]))**2

def ekf_slam():
    """
    Performs an iteration of EKF SLAM from the available information.

    :param xEst: the belief in last position
    :param PEst: the uncertainty in last position
    :param u:    the control function applied to the last position
    :param z:    measurements at this step
    :returns:    the next estimated position and associated covariance
    """

    global xEst,PEst,z
    #print("hello")
    # Predict
    predict()
    
    # Update
    ###print("z",z)
    for measurement in z:
        print("===============")
        #H_f,Psi_f,difference_f,K=data_association(xEst,PEst,measurement,z)
        data_association(measurement)
    
   
    msg=final_coordinates()
    msg.x=xEst[0][0]
    msg.y=xEst[1][0]
    pub.publish(msg)
    # print("after")
    # print(xEst)


    #print("x=",xEst)
    #print("p=",PEst)
    



def predict():
    """
    Performs the prediction step of EKF SLAM

    :param xEst: nx1 state vector
    :param PEst: nxn covariance matrix
    :param u:    2x1 control vector
    """
    global xEst,PEst,delta_t, landmark_indexes,sigma
    
    # print("starting")
    # print(xEst)

    #print(PEst)
    
    print("------------------------------------------------------------------------")
    # print("before",xEst)
    # print(PEst)
    theta=u[1]+xEst[2][0]
    xEst[2][0]=theta
    x=xEst[0][0]+u[0]*np.cos(xEst[2][0])*delta_t
    y=xEst[1][0]+u[0]*np.sin(xEst[2][0])*delta_t
    # x=xEst[0][0]+0.2*np.cos(xEst[2][0])
    #y=xEst[1][0]+0.2*np.sin(xEst[2][0])
    '''
    if (theta > np.pi):
        theta -= 2 * np.pi

    elif (theta < -np.pi):
        theta += 2 * np.pi
    '''
    xEst[0][0]=x
    xEst[1][0]=y
    # print("after",xEst)
    

    G = np.identity(3 + 2 * (landmark_indexes))
    G[0][2] = - u[0] * delta_t * np.sin(xEst[2][0])
    G[1][2] = u[0] * delta_t * np.cos(xEst[2][0])
    #G[0][2] = -0.2* np.sin(xEst[2][0])
    #G[1][2] = 0.2 * np.cos(xEst[2][0])

    sigma=np.dot(np.dot(G,PEst),(np.transpose(G)))
    # PEst=sigma
    
    #print(PEst)
    # print("---------------------------------------")
    # print("before")
    # print(xEst)
    # print(PEst)
    
    #M = np.array([[0.5**2, 0], [0, 0.5**2]])

    #PEst = F.T @ PEst @ F + V @ M @ V.T

    
   


def data_association(measurement):
    # print("landmarks",landmarks)
    global  car_coordinate, xEst, PEst,Q,Psi_f,difference_f,H_f,sigma
    # measurement=[x,y,range,bearing]
    ###print("measurement=",measurement)
    # Get current robot state, measurement
    x_t=xEst[0][0]
    y_t=xEst[1][0]
    theta_t=xEst[2][0]
    range_t=measurement[2]
    bearing_t=measurement[3]
    
    landmark_expected_x=measurement[0]
    landmark_expected_y=measurement[1]
    
    msg=covariance()
    total=[]
    
    min_distance = 1e16
    count=0
    c=3
    ###print("landmarks=",landmarks)
    for i in landmarks:
        count+=1
        # Get current landmark estimate
        # x_l=x_t+ range_t*math.cos(bearing_t)                #car_coordinate[0]+ range_t*math.cos(bearing_t)
        # y_l=y_t+ range_t*math.sin(bearing_t)                #car_coordinate[1]+ range_t*math.sin(bearing_t)
        x_l=i[0]
        y_l=i[1]

        delta_x = abs(x_l - x_t)
        delta_y = abs(y_l - y_t)
        q = delta_x ** 2 + delta_y ** 2
        range_expected = np.sqrt(q)
        bearing_expected = np.arctan2(delta_y, delta_x) 
        # print(bearing_t,bearing_expected)

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
        F_x = np.zeros((5, 3 + 2 * (landmark_indexes)))
        F_x[0][0] = 1.0
        F_x[1][1] = 1.0
        F_x[2][2] = 1.0
        F_x[3][2 * count + 1] = 1.0
        F_x[4][2 * count + 2] = 1.0
        H_1 = np.array([-delta_x/np.sqrt(q), -delta_y/np.sqrt(q), 0, delta_x/np.sqrt(q), delta_y/np.sqrt(q)])
        H_2 = np.array([delta_y/q, -delta_x/q, -1, -delta_y/q, delta_x/q])
        H_3 = np.array([0, 0, 0, 0, 0])
        H = np.array([H_1, H_2, H_3]).dot(F_x)

        # Compute Mahalanobis distance
        Psi = np.dot(np.dot(H,sigma),np.transpose(H))+ Q
        # Psi += np.eye(Psi.shape[0]) * 1e-6
        # difference =  np.array([range_t-range_expected,bearing_t-bearing_expected,0])
        difference = np.array([range_expected-range_t,bearing_expected-bearing_t, 0])
        Pi = difference.T.dot(np.linalg.inv(Psi)).dot(difference)
        ###print("pi=",Pi)
        total.append(i[0])
        total.append(i[1])
        ###print("PEst[c][c]",PEst[c][c])
        ###print("PEst[c+1][c+1]",PEst[c+1][c+1])
        total.append(math.sqrt(PEst[c][c]))
        total.append(math.sqrt(PEst[c+1][c+1]))
        c+=2
      
        # Get landmark information with least distance
        if Pi < min_distance:
            ###print("final=",i)
            min_distance = Pi
            # Values for measurement update
            H_f = H
            Psi_f = Psi
            difference_f = difference

    msg.c=total
    cov_pub.publish(msg) 
    update()     

def update():
    """
    Performs the update step of EKF SLAM

    :param xEst:  nx1 the predicted pose of the system and the pose of the landmarks
    :param PEst:  nxn the predicted covariance
    :param z:     the measurements read at new position
    :param initP: 2x2 an identity matrix ac
    ting as the initial covariance
    :returns:     the updated state and covariance for the system
    """
    global xEst, PEst, H_f,Psi_f, difference_f, landmark_indexes
    
    ###print("Psi_f=",Psi_f)
    ###print("det=",np.linalg.det(Psi_f))
    K = np.dot(np.dot(sigma,H_f.T),np.linalg.inv(Psi_f))      #PEst.dot(H_f.T).dot(np.linalg.inv(Psi_f)) 
    innovation = np.dot(K,difference_f)
    # print("innovation=",innovation[0],innovation[1],innovation[2])
    # if(abs(innovation[0])>1 or abs(innovation[1])>1 ):
    #     return
        
    xEst[0][0]+=innovation[0]*(1)
    xEst[1][0]+=innovation[1]*(1)
    xEst[2][0]+=innovation[2]*(1)
    # print("before",PEst)
    # Update covariance
    PEst = np.dot((np.identity(3 + 2 * (landmark_indexes)) - np.dot(K,H_f)),sigma)
    # print("after=",PEst)

    

    #xEst[2] = pi_2_pi(xEst[2])
    '''
    if not isinstance(Psi_f, np.ndarray):
        rospy.logerr("Psi_f is not properly initialized.")
        return
    # Regularize Psi_f to avoid singularity
    regularization_factor = 1e-6
    Psi_f += regularization_factor * np.eye(Psi_f.shape[0])
    
    # Compute Kalman gain
    K = PEst.dot(H_f.T).dot(np.linalg.inv(Psi_f))
    
    # Compute innovation
    innovation = K.dot(difference_f)
    innovation = innovation.flatten()
    
    if len(innovation) < 3:
        print(f"Unexpected innovation shape: {innovation.shape}")
        return
    
    
    # Update state estimate
    xEst[0][0] += innovation[0]
    xEst[1][0] += innovation[1]
    xEst[2][0] += innovation[2]
    
    # Update covariance estimate
    PEst = (np.eye(3 + 2 * landmark_indexes) - K.dot(H_f)).dot(PEst)
    xEst[2][0] = (xEst[2][0] + np.pi) % (2 * np.pi) - np.pi
    '''
def real(data):
    
    global car_coordinate , sutta, bt, delta_t, prev_m, prev_s, prev_ns, prev_steer,prev_yaw,p,o
    car_coordinate=[0,0]
    car_coordinate[0]=data.pose.position.x
    car_coordinate[1]=data.pose.position.y
    #print("real coordinates=",car_coordinate)
    '''
    diff=math.sqrt(((abs(car_coordinate[1]-sutta[1]))**2) + ((abs(car_coordinate[0]-sutta[0]))**2))
    if(diff>0.5):
        print("difference=",diff)
        #rospy.Subscriber("/controltoekf", ekf, call)
        print("sutta")
        sutta=car_coordinate

    '''
    global roll, pitch, yaw
    orientation_q = data.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    if(o):
        print("o")
        xEst[0][0]=car_coordinate[0]
        xEst[1][0]=car_coordinate[1]
        xEst[2][0]=yaw
        o=False


    


def back(da):
    ###print("data=",da)
    #print(1)
    # global cones
    global xEst,PEst,delta_t, z,num_landmarks, landmarks
    global car_coordinate , sutta, bt, delta_t, prev_m, prev_s, prev_ns, prev_steer,prev_yaw,p,o
    l=int(len(da.oned_list)/4)
    d=(np.array(da.oned_list).reshape((l, 4)))
    ###print("d=",d)
    cones=d.tolist()
    ###print("cones=",cones)
    z=cones
    ###print("updating z",z)
    for i in z:
        t=True
        
        for j in landmarks:
            d=math.sqrt(((abs(i[1]-j[1]))**2) + ((abs(i[0]-j[0]))**2))
            if(d<1):
                t=False
                break
        if(t):
            landmarks.append(i)
            num_landmarks+=1
    

    
   
    if(bt and p):
        print("bt")
        prev_yaw[0]=yaw
        a=datetime.now().time()
        prev_m=a.minute
        prev_s=a.second
        prev_ns=a.microsecond
        bt=False

    a=datetime.now().time()
    b=(a.minute)
    c=(a.second)
    d=(a.microsecond)
    delta_t=(((b-prev_m)*60)+(c-prev_s)+((d-prev_ns)/1000000))

    if(delta_t>0.008 and p):
       
        u[0]=0.49
        u[1]=yaw-prev_yaw[0]
        prev_yaw[0]=yaw
        
        prev_m=b
        prev_s=c
        prev_ns=d
        ###print("++++++++++++++++++++++++++++++++++++++++++")
        ekf_slam()

    
    # print(landmarks)
    # print("size=",len(landmarks))
    # print("num=",num_landmarks)


def call(data):
    
    # u[0]=data.controls[0]
    # u[1]=data.controls[1]

    # global car_coordinate , sutta, b, delta_t, prev_m, prev_s, prev_ns, prev_steer
    # if(b):
    #     a=datetime.now().time()
    #     prev_m=a.minute
    #     prev_s=a.second
    #     prev_ns=a.microsecond
    #     b=False
    
       
    
    
    # diff=math.sqrt(((abs(car_coordinate[1]-sutta[1]))**2) + ((abs(car_coordinate[0]-sutta[0]))**2))
    # if(diff>0.2):
    #     #print("difference=",diff)
    #     #rospy.Subscriber("/controltoekf", ekf, call)
    #     #print("sutta")
    #     sutta=car_coordinate

    #     a=datetime.now().time()
    #     b=(a.minute)
    #     c=(a.second)
    #     d=(a.microsecond)
    #     delta_t=(((b-prev_m)*60)+(c-prev_s)+((d-prev_ns)/1000000))
    #     #print("delta=",delta_t)
    # #if(delta_t>0.1):
    # #    delta_t=0.001
    #     prev_m=b
    #     prev_s=c
    #     prev_ns=d
        
    #     ekf_slam()

    # #calculating delta time
    
    global p
    p=True
    


if __name__ == '__main__':
    print("Hokuyo LIDAR node started")
    rospy.init_node('rand',anonymous = True)
    
    rospy.Subscriber("/perception_to_slam",perception,back)
    rospy.Subscriber("/gt_pose",PoseStamped, real)
    #rospy.Subscriber("/slam_to_distfinder",slam,real_coordinates)
    rospy.Subscriber("/controltoekf", ekf, call)
    
    #rospy.Subscriber("/slam_to_distfinder",slam,c)
    
    
    rospy.spin()
