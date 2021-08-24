#!/usr/bin/env python

# from pickle import NONE
import rospy, math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from threading import Lock

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry



class ControlPlot:
    def __init__(self, control_topic, plot_states=False, state_topic=None):

        self.initialized = False
        self.plot_states = plot_states
        self.dim_states = 0
        self.dim_controls = 0
        self.x_fig = plt.Figure()
        self.x_axes = []
        self.u_fig = plt.Figure()
        self.u_axes = []
        self.tx = []
        self.x = []
        self.tu = []
        self.u = []
        self.n_plot = 50
        self.mutex = Lock()
        
        self.sub_controller = rospy.Subscriber(control_topic, TwistStamped, self.update_ctrl, queue_size = 1)
        rospy.loginfo("[MPCPlot] Plotting control results published on '%s'.",control_topic)
        
        if self.plot_states:
            self.sub_state      = rospy.Subscriber(state_topic, Odometry, self.update_state, queue_size =1)
            rospy.loginfo("[MPCPlot] Plotting state published on '%s'.",state_topic)


    def update_ctrl(self, data):
        self.mutex.acquire()
        # if self.tu == []:
        #     self.tu = np.array(data.header.stamp.secs + data.header.stamp.nsecs/1000000000.)
        #     self.u  = np.array([data.twist.linear.x, data.twist.angular.z])
        # else:
        #     np.insert(self.tu, data.header.stamp.secs + data.header.stamp.nsecs/1000000000.)
        #     np.insert
        self.tu.append(data.header.stamp.secs + data.header.stamp.nsecs/1000000000.)
        self.u.append([data.twist.linear.x, data.twist.angular.z])
        self.mutex.release()

    def update_state(self, data):
        # Read data
        self.mutex.acquire()
        if self.plot_states:
            pass
        self.mutex.release()
        
    def initializePlotWindows(self):
        if self.plot_states:
            self.x_fig, self.x_axes = plt.subplots(self.dim_states, sharex=True)
            self.x_axes[0].set_title('States')
            for idx, ax in enumerate(self.x_axes):
                ax.set_ylabel("x" + str(idx))
            self.x_axes[-1].set_xlabel('Time [s]')

        self.u_fig, self.u_axes = plt.subplots(2, sharex=True)
        self.u_axes[0].set_title('Controls')
        for idx, ax in enumerate(self.u_axes):
            ax.set_ylabel("u_" + str(idx))
            self.u_axes[-1].set_xlabel('Time [s]')
        plt.ion()
        plt.show()

    
    def plot(self):
        # We recreate the plot every time, not fast, but okay for now....
        self.mutex.acquire()
        if self.plot_states:
            for idx, ax in enumerate(self.x_axes):
                ax.cla()
                ax.grid()
                ax.plot(self.tx, self.x[:,idx])
                ax.get_yaxis().set_major_formatter(ticker.FuncFormatter(lambda x, p: "%.2f" % x))
                ax.set_ylabel("x" + str(idx))
            self.x_axes[0].set_title('States')
            self.x_axes[-1].set_xlabel('Time [s]')
            self.x_fig.canvas.draw()

        u_array = np.array(self.u)
        t_array = np.array(self.tu)
        for idx, ax in enumerate(self.u_axes):
            ax.cla()
            ax.grid()
            if u_array.shape[0] < self.n_plot:
                ax.step(t_array, u_array[:, idx], where='post')
            else:
                ax.step(t_array[-self.n_plot:], u_array[-self.n_plot:, idx], where='post')
            if idx == 0:
                ax.set_ylim([-0.1, 1])
            else:
                ax.set_ylim([-1.5, 1.5])

            ax.get_yaxis().set_major_formatter(ticker.FuncFormatter(lambda x, p: "%.2f" % x))
            
            ax.set_ylabel("u_" + str(idx))
        self.u_axes[0].set_title('Controls')
        self.u_axes[-1].set_xlabel('Time [s]')
        self.u_fig.canvas.draw()
        self.mutex.release()
        
    def start(self, rate):
        r = rospy.Rate(rate) # define rate here
        while not rospy.is_shutdown():
            if not self.initialized:
                self.initializePlotWindows()
                self.initialized = True
            if self.initialized:
                self.plot()
        r.sleep()


if __name__ == '__main__': 
    try:
    
        rospy.init_node("control_plot", anonymous=True) 

        topic_name = "/cmd_vel_stmp"
        topic_name = rospy.get_param('~control_topic', topic_name)

        plot_states = rospy.get_param('~plot_states', False)

        result_plotter = ControlPlot(topic_name)
        rate = 5
        rate = rospy.get_param('~plot_rate', rate)

        result_plotter.start(rate)
    except rospy.ROSInterruptException:
        pass