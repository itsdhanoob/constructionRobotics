# #!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Float64
from fav_msgs.msg import RangeMeasurementArray
from nav_msgs.msg import Odometry
import numpy as np
from dynamic_reconfigure.server import Server
from localization.cfg import Kalman_CovariancesConfig
import threading



np.random.seed(1)  # for repeatability


class KalmanFilter():
    

    
    def __init__(self):
        rospy.init_node("kalman_filter")
        # rospy.Subscriber("/bluerov/ranges", RangeMeasurementArray, self.callbackRanges)
        # rospy.Subscriber("/bluerov/ground_truth/state", Odometry, self.callback_Waypoint)
        # rospy.Subscriber("least_square_estimation", Odometry, self.estimation_callback)
        rospy.Subscriber("smoothed_pressure", Float64, self.pressure_callback)
        self.kalman_pub=rospy.Publisher("estimated_depth",Float64, queue_size=1)
        
        self.atmospheric_pressure = 100582 # 101300  # in Pa
        self.dim_state = 2
        self.dim_measurement = 1
        self.dt = 0.3  # updating at 10 hz
        self.pressure_true=0 #initialize real pressure data
        self.OFFSET = 0.13
        

       
        # Initial guess covariance
        # ---------------------------------------------------------------------------------------------------------------------------------------
        self.depth_initial_std_dev = 0.5  # initial depth standard deviation
        self.depth_velocity_initial_std_dev = 0.5   # initial velocity standard deviation
        self.P0 = np.array([[self.depth_initial_std_dev ** 2, 0], [0, self.depth_velocity_initial_std_dev ** 2]])  # covariance of initial guess
        # ---------------------------------------------------------------------------------------------------------------------------------------
        # Process noise covariance
        # ---------------------------------------------------------------------------------------------------------------------------------------
        self.depth_std_dev = 0.00005  # depth standard deviation,           set:0.00005
        self.depth_velocity_std_dev = 0.05  # velocity standard deviation,  set:0.005
        self.Q = np.diag([self.depth_std_dev ** 2, self.depth_velocity_std_dev ** 2]) # Process noise covariance
        # ---------------------------------------------------------------------------------------------------------------------------------------
        # Measurement noise covariance
        # ---------------------------------------------------------------------------------------------------------------------------------------
        self.pressure_std_dev_true = 100  # Pascal, default: 100
        self.pressure_std_dev = 100000  # in Pa, default: 100, set:50000
        self.R = np.array([self.pressure_std_dev ** 2]).reshape((-1, 1)) # Measurement noise covariance
       
        # .reshape above turns converts a matrix's arrangement to the specified rows and columns. In this scenario -1 is the completing rows, 1 is the specified amount of columns.
        # ---------------------------------------------------------------------------------------------------------------------------------------
        
        self.drop_measurement_probability = 0.7  # default: 0.5
        self.x0_est = np.array([[-0.5], [0]])  #state matrix initial values, [position,velocity]
        # self.location_position=np.array([[0,0,0]]).reshape((-1,1))

        
    # Dynamic Reconfigure
        self.data_lock = threading.RLock()
        self.dyn_server = Server(Kalman_CovariancesConfig, self.on_dyn_reconfigure)

    def on_dyn_reconfigure(self, config, level):
        with self.data_lock:
            self.depth_std_dev = config["depth_std_dev"]
            self.depth_velocity_std_dev = config["depth_velocity_std_dev"]
            self.pressure_std_dev = config["pressure_std_dev"]

            rospy.loginfo("self.depth_std_dev is %s", self.depth_std_dev)
            rospy.loginfo("self.depth_velocity_std_dev is %s", self.depth_velocity_std_dev)
            rospy.loginfo("self.pressure_std_dev is %s", self.pressure_std_dev)
        self.Q = np.diag([self.depth_std_dev ** 2, self.depth_velocity_std_dev ** 2]) # Process noise covariance
        self.R = np.array([self.pressure_std_dev ** 2]).reshape((-1, 1)) # Measurement noise covariance
        return config
    
        #-- CALLBACK FUNCTIONS --
        # ---------------------------------------------------------------------------------------------------------------------------------------  
    def estimation_callback(self, data):
        self.location_position = np.array([[data.pose.pose.position.x], [data.pose.pose.position.y], [data.pose.pose.position.z]])
        # rospy.loginfo("position z is: %s",self.location_position[2][0])
        self.location_orientation = data.pose.pose.orientation
    
    def pressure_callback(self, data):
        self.pressure_true= data.data
        # ---------------------------------------------------------------------------------------------------------------------------------------

    def update_func(self,depth):
        return -(depth- self.OFFSET) * 1.0e4 + self.atmospheric_pressure # (depth-0.15)
    

    def get_update_jacobian(self):
        return np.array([-1.0e4, 0.0]).reshape((1, -1))
    

    def predict_func(self,dt, x_est):
        return np.array([x_est[0,0] + dt * x_est[1,0], x_est[1,0]]).reshape((-1,1)) # [x+dt*x,xdot]
    

    def get_predict_jacobian(self,dt):
        return np.array([[1, dt], [0, 1]])
        

    def predict(self,dt, x_est, P):
        #- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        #-Calculates the new estimated state and the new estimation covariance.-
        #- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        x_est_new = self.predict_func(dt, x_est)
        predict_jacobian = self.get_predict_jacobian(dt)
        P_new = np.matmul(np.matmul(predict_jacobian, P), predict_jacobian.transpose()) + self.Q
        return x_est_new, P_new
    

    def update(self,p_true, x_est, P):
        
        update_jacobian = self.get_update_jacobian()
        
        
        p_est = self.update_func(x_est[0, 0]) # ++++++++++++++++++++++++++++++++++++++++++++++++ # estimated pressure
        
        # innovation
        y = np.asarray(p_true - p_est).reshape((-1,1))# +++++++++++++++++++++++++++++++++++++++ # update of pressure
        
        # compute Kalman gain
        S = np.matmul(np.matmul(update_jacobian, P), update_jacobian.transpose()) + self.R #++++++++ # innovation covariance
        K = np.matmul(np.matmul(P, update_jacobian.transpose()), np.linalg.inv(S))
        # print("K is: %s",K)
        # print("y is: %s", y)
        # print("p_true is: %s",p_true)
        # print("p_est is: %s", p_est)
        # update state
        x_est_new = x_est + np.matmul(K, y)
        
        # update covariance
        P_new = np.matmul(np.eye(self.dim_state) - np.matmul(K, update_jacobian), P)
        return x_est_new, P_new
    
    
    def run(self):
        rate = rospy.Rate(50.0)

        x_est = self.x0_est
        P = self.P0
        t_last = 0
        t_now=0
    
        
        while not rospy.is_shutdown():

            ## Process step - propagating estimated state
            dt_real =  t_now - t_last  # compute actual time difference since last prediction
            t_last = t_now
            x_est_next, P_next = self.predict(dt_real, x_est, P)

            ## If we get a measurement, the measurement is updated
            if (np.random.uniform() > self.drop_measurement_probability):

                p_true = self.pressure_true
                # rospy.loginfo("p_true is: %s", p_true)

                ## Measurement update 
                x_est_next, P_next = self.update(p_true, x_est_next, P_next)

                

            x_est, P = x_est_next, P_next
            self.kalman_pub.publish(x_est[0][0])
            t_now=rospy.get_time()
            rate.sleep()


def main():

    node = KalmanFilter()
    node.run()

if __name__ == "__main__":
    
    main()








