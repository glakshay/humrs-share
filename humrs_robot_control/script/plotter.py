#!/usr/bin/env python
import rospy
import time
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from geometry_msgs.msg import Vector3
from gazebo_msgs.msg import ModelStates


class DataPlotter:

    def __init__(self):
        self.counter = 0
        self.start = time.time()
        self.fig, self.ax = plt.subplots()
        self.x_position_line, = plt.plot([], [], 'r-')
        self.y_position_line, = plt.plot([], [], 'g-')
        self.z_position_line, = plt.plot([], [], 'b-')
        self.x_twist_line, = plt.plot([], [], 'r*')
        self.y_twist_line, = plt.plot([], [], 'g*')
        self.z_twist_line, = plt.plot([], [], 'b*')
        self.time = []
        self.x_position = []
        self.y_position = []
        self.z_position = []
        self.x_twist = []
        self.y_twist = []
        self.z_twist = []

        # rospy.Subscriber("/rexrov/current_velocity", Vector3, self.plot)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.append_data)

    def plot_init(self):
        self.ax.set_xlim(0, 10)
        self.ax.set_ylim(-5, 5)

    def plot(self, data):
        if self.counter % 10 == 0 and len(data.name) > 1 and data.name == "rexrov":
            print "pose", data.pose[1]
            print "twist", data.twist[1]
            print (time.time() - self.start, data.pose[1].position.x)
            plt.plot(time.time() - self.start, data.pose[1].position.x, 'r.')
            plt.plot(time.time() - self.start, data.pose[1].position.y, 'g.')
            plt.plot(time.time() - self.start, data.pose[1].position.z, 'b.')
            plt.axis("equal")
            plt.draw()
            plt.pause(0.000000001)
        
        self.counter += 1

    def append_data(self, data):
        if self.counter % 20 == 0 and len(data.name) > 1 and data.name[1] == "rexrov":
            self.time.append(time.time() - self.start)
            self.x_position.append(data.pose[1].position.x)
            self.y_position.append(data.pose[1].position.y)
            self.z_position.append(data.pose[1].position.z)
            self.x_twist.append(data.twist[1].linear.z)
            self.y_twist.append(data.twist[1].linear.z)
            self.z_twist.append(data.twist[1].linear.z)
        self.counter += 1

    def update_plot(self, frame):
        if len(self.time) > 0:
            self.x_position_line.set_data(self.time, self.x_position)
            self.y_position_line.set_data(self.time, self.y_position)
            self.z_position_line.set_data(self.time, self.z_position)
            self.x_twist_line.set_data(self.time, self.x_twist)
            self.y_twist_line.set_data(self.time, self.y_twist)
            self.z_twist_line.set_data(self.time, self.z_twist)

            if self.time[-1] > 10:
                self.ax.set_xlim(self.time[-1]-10, self.time[-1])
        

if __name__ == "__main__":
    rospy.init_node("Plotter")
    dp = DataPlotter()
    rospy.loginfo("Plotter node activated.")
    # rospy.spin()
    ani = FuncAnimation(dp.fig, dp.update_plot, init_func=dp.plot_init, interval=200)
    plt.show(block=True)
    rospy.loginfo("Data logger exited gracefully.")
