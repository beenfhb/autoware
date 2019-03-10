#!/usr/bin/env python
import numpy as np
import rospy
import rosnode
import cvxopt
import cvxopt.solvers
from tf.transformations import euler_from_quaternion
from autoware_msgs.msg import Lane, LaneArray
from utils import *


class waypoint_qp_speed_replanner:
    def __init__(self):
        self.__app_name = "waypoint_qp_speed_replanner"
        self._a_max = rospy.get_param('~a_max', 2.0)
        self._s_max = rospy.get_param('~s_max', 3.0)
        self._v_max = rospy.get_param('~v_max', 10.0)
        self._latacc_max = rospy.get_param('~latacc_max', 0.5)
        self._tire_angvel_max = rospy.get_param('~tire_angvel_max', 0.1)
        self._tire_angvel_thr = rospy.get_param('~tire_angvel_thr', 2.0)
        self._vel_min_for_tire = rospy.get_param('~vel_min_for_tire', 2.0)
        self._wheelbase = 2.9
        self._max_iter = rospy.get_param('~max_iter', 20)
        self._lane_pub = {}
        self._lane_pub['with_decision'] = rospy.Publisher(
             "/based/lane_waypoints_array", LaneArray, queue_size=10)
        self._lane_pub['without_decision'] = rospy.Publisher(
             "lane_waypoints_array", LaneArray, queue_size=10)
        self._lane_sub = rospy.Subscriber("/based/lane_waypoints_raw",
                                          LaneArray, self.lane_array_callback)
        self._lane_array = LaneArray()
        self._replanning_mode = True
    
    def is_decision_maker(self):
        return rosnode.rosnode_ping("/decision_maker",
                                    max_count=1, verbose=False)

    def lane_array_callback(self, data):   
        self._lane_array = data
        self.lane_array_publisher()
    
    def lane_array_publisher(self):
        array = self._lane_array
        if(self._replanning_mode):
            self.replan(array)
        if(self.is_decision_maker()):
            self._lane_pub['with_decision'].publish(array)
        else:
            self._lane_pub['without_decision'].publish(array)

    def replan(self, lane_array):
        cvxopt.solvers.options['abstol'] = 1e-15
        cvxopt.solvers.options['reltol'] = 1e-15
        cvxopt.solvers.options['feastol'] = 1e-15
        for lane in lane_array.lanes:
            l = len(lane.waypoints)
            v = np.array(map(lambda wp: wp.twist.twist.linear.x,  
                         lane.waypoints))
            orientation_list = lambda o: [o.x, o.y, o.z, o.w]
            yaw = np.array(map(lambda wp: 
                           euler_from_quaternion(
                           orientation_list(wp.pose.pose.orientation))[2],
                           lane.waypoints))
            curvature = calc_waypoints_curvature(lane)
            waypoints_dist = np.array([dist(lane.waypoints[i],
                                      lane.waypoints[i+1]) 
                                      for i in range(l-1)])
            acc_orig = v[1:l] * np.array(v[1:l] - v[0:l-1]) / waypoints_dist
            jerk_orig = (v[1:l-1] * v[1:l-1]) * \
                        np.array(v[2:l] - 2.0 * v[1:l-1] + v[0:l-2]) / \
                        (np.power(waypoints_dist[1:l-1], 2))
            latacc_orig = curvature * v.reshape((l, 1)) * v.reshape((l, 1))
            tire_angvel_orig = v[1:l-1] * (yaw[2:l] - 2.0 * yaw[1:l-1] + yaw[0:l-2]) / \
                               np.power(waypoints_dist[1:l-1], 2) * self._wheelbase
            A = np.zeros((2, l))
            A[0, 0] = 1
            A[1, l-1] = 1
            b = np.array([v[0], v[l-1]])
            for j in range(self._max_iter):
                # velocity constraint
                G_vel = np.eye(l)
                G_vel = np.vstack((G_vel, -G_vel))
                h_vel_max = np.zeros((l, 1))
                
                for i in range(l):
                    k = max(np.abs(curvature[i]), 0.0001)
                    h_vel_max[i] = min((self._latacc_max / k) ** 0.5,  
                                       self._v_max)
                
                tire_angvel_tmp = np.zeros((l, 1))
                for i in range(1, l-1):
                    tire_angvel_tmp[i] = tire_angvel_orig[i-1]
                tire_angvel_tmp[0] = tire_angvel_orig[0]
                tire_angvel_tmp[-1] = tire_angvel_orig[-1]

                h_vel_min = np.zeros((l, 1))
                for i in range(l):
                    if abs(tire_angvel_tmp[i]) > self._tire_angvel_thr:
                        h_vel_min[i] = -self._vel_min_for_tire
                h_vel = np.vstack((h_vel_max, h_vel_min))
                G = G_vel
                h = h_vel
                # acceleration constraint
                G_acc = np.zeros((l-1, l))
                for i in range(l-1):
                    G_acc[i, i] = -v[i] / waypoints_dist[i]
                    G_acc[i, i+1] = v[i] / waypoints_dist[i]
                G_acc = np.vstack((G_acc, -G_acc))
                h_acc = np.ones((l-1, 1)) * self._a_max
                h_acc = np.vstack((h_acc, h_acc))
                G = np.vstack((G, G_acc))
                h = np.vstack((h, h_acc))
                # jerk constraint
                G_jerk = np.zeros((l-2, l))
                for i in range(l-2):
                    G_jerk[i, i] = (v[i+1] / waypoints_dist[i+1])**2
                    G_jerk[i, i+1] = -2.0 * ((v[i+1] / waypoints_dist[i+1])**2)
                    G_jerk[i, i+2] = (v[i+1] / waypoints_dist[i+1])**2
                G_jerk = np.vstack((G_jerk, -G_jerk))
                h_jerk = np.ones((l-2, 1)) * self._s_max
                h_jerk = np.vstack((h_jerk, h_jerk))
                G = np.vstack((G, G_jerk))
                h = np.vstack((h, h_jerk))
                # minimize squared error from original velocity
                P = np.eye(l)
                q = -np.array(v)
                cvxopt.solvers.options['show_progress'] = False
                print("start cvxopt")
                sol = cvxopt.solvers.qp(cvxopt.matrix(P), cvxopt.matrix(q), 
                                        G=cvxopt.matrix(G), h=cvxopt.matrix(h), 
                                        A=cvxopt.matrix(A), b=cvxopt.matrix(b))
                print("end cvxopt")
                v_replan = np.array(sol['x'])
                cost = sol["primal objective"] + np.dot(v, v)
                for i, waypoint in enumerate(lane.waypoints):
                    waypoint.twist.twist.linear.x = v_replan[i]
        return lane_array
if __name__ == '__main__':
    try:
        rospy.init_node("waypoint_qp_speed_replanner")
        wqsr = waypoint_qp_speed_replanner()
        rospy.spin()
    except rospy.ROSInternalException: 
        pass



 