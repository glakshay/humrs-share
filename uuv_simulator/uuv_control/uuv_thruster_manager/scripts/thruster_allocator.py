#!/usr/bin/env python
from __future__ import print_function
import numpy
import rospy
import tf
import tf.transformations as trans
from os.path import isdir, join
from copy import deepcopy
import yaml
import tf2_ros
from uuv_thrusters import ThrusterManager
from geometry_msgs.msg import Wrench, WrenchStamped, Pose
from uuv_thruster_manager.msg import custom
from uuv_thruster_manager.srv import *


class ThrusterAllocatorNode(ThrusterManager):
    """The thruster allocator node allows a client node 
    to command the thrusters.
    """

    def __init__(self):
        """Class constructor."""
        #print("before")
        ThrusterManager.__init__(self)
        #print("after")
        self.last_update = rospy.Time.now()

        # Subscriber to the wrench to be applied on the UUV
        self.input_sub = rospy.Subscriber('thruster_manager/input',
                                          Wrench, self.input_callback)

        #new subscriber 
        self.input_subsriber = rospy.Subscriber('thruster_manager/tam_input',
                                          custom, self.input_callback_fn)

        # To deliver the wrench input with an option to use another body frame
        # (options: base_link and base_link_ned), use the wrench stamped
        # message
        self.input_stamped_sub = rospy.Subscriber(
            'thruster_manager/input_stamped', WrenchStamped,
            self.input_stamped_callback)
        self.thruster_info_service = rospy.Service(
            'thruster_manager/get_thrusters_info', ThrusterManagerInfo,
            self.get_thruster_info)
        self.curve_calc_service = rospy.Service(
            'thruster_manager/get_thruster_curve', GetThrusterCurve,
            self.get_thruster_curve)
        self.set_thruster_manager_config_service = rospy.Service(
            'thruster_manager/set_config', SetThrusterManagerConfig,
            self.set_config)
        self.get_thruster_manager_config_service = rospy.Service(
            'thruster_manager/get_config', GetThrusterManagerConfig,
            self.get_config)

        rate = rospy.Rate(self.config['update_rate'])
        while not rospy.is_shutdown():
            if self.config['timeout'] > 0:
                # If a timeout is set, zero the outputs to the thrusters if
                # there is no command signal for the length of timeout
                if rospy.Time.now() - self.last_update > self.config['timeout']:
                    print('Turning thrusters off - inactive for too long')
                    if self.thrust is not None:
                        self.thrust.fill(0)
                        self.command_thrusters()
            rate.sleep()

    def get_thruster_info(self, request):
        """Return service callback with thruster information."""
        return ThrusterManagerInfoResponse(
            self.n_thrusters,
            self.configuration_matrix.flatten().tolist(),
            self.namespace + self.config['base_link'])

    def get_thruster_curve(self, request):
        """Return service callback for computation of thruster curve."""
        if self.n_thrusters == 0:
            return GetThrusterCurveResponse([], [])
        # TODO Get thruster index, for the case the vehicle has different
        # models
        input_values, thrust_values = self.thrusters[0].get_curve(
            request.min, request.max, request.n_points)
        return GetThrusterCurveResponse(input_values, thrust_values)

    def set_config(self, request):
        old_config = deepcopy(self.config)
        self.ready = False
        self.config['base_link'] = request.base_link
        self.config['thruster_frame_base'] = request.thruster_frame_base
        self.config['thruster_topic_prefix'] = request.thruster_topic_prefix
        self.config['thruster_topic_suffix'] = request.thruster_topic_suffix
        self.config['timeout'] = request.timeout
        print('New configuration:\n')
        for key in self.config:
            print(key, '=', self.config[key])
        if not self.update_tam(recalculate=True):
            print('Configuration parameters are invalid, going back to old configuration...')
            self.config = old_config
            self.update_tam(recalculate=True)
        return SetThrusterManagerConfigResponse(True)

    def get_config(self, request):
        return GetThrusterManagerConfigResponse(
            self.namespace,
            self.config['base_link'],
            self.config['thruster_frame_base'],
            self.config['thruster_topic_prefix'],
            self.config['thruster_topic_suffix'],
            self.config['timeout'],
            self.config['max_thrust'],
            self.n_thrusters,
            self.configuration_matrix.flatten().tolist())

    def input_callback(self, msg):
        """
        Callback to the subscriber that receiver the wrench to be applied on
        UUV's BODY frame.
        @param msg Wrench message
        """
        if not self.ready:
            return

        force = numpy.array((msg.force.x, msg.force.y, msg.force.z))
        torque = numpy.array((msg.torque.x, msg.torque.y, msg.torque.z))

        # This mode assumes that the wrench is given wrt thruster manager
        # configured base_link reference
        self.publish_thrust_forces(force, torque)

        self.last_update = rospy.Time.now()

    def input_callback_fn(self, msg):
        """
        Callback to the subscriber that receiver the wrench to be applied on
        UUV's BODY frame.
        @param msg Pose message
        """
        if not self.ready:
            return
        pose = numpy.array([(msg.thruster1_pose.position.x, msg.thruster1_pose.position.y, msg.thruster1_pose.position.z),(msg.thruster2_pose.position.x, msg.thruster2_pose.position.y, msg.thruster2_pose.position.z),(msg.thruster3_pose.position.x, msg.thruster3_pose.position.y, msg.thruster3_pose.position.z),(msg.thruster4_pose.position.x, msg.thruster4_pose.position.y, msg.thruster4_pose.position.z), (msg.thruster5_pose.position.x, msg.thruster5_pose.position.y, msg.thruster5_pose.position.z)])
        quat = numpy.array([(msg.thruster1_pose.orientation.x, msg.thruster1_pose.orientation.y, msg.thruster1_pose.orientation.z, msg.thruster1_pose.orientation.w),(msg.thruster2_pose.orientation.x, msg.thruster2_pose.orientation.y, msg.thruster2_pose.orientation.z, msg.thruster2_pose.orientation.w),(msg.thruster3_pose.orientation.x, msg.thruster3_pose.orientation.y, msg.thruster3_pose.orientation.z, msg.thruster3_pose.orientation.w),(msg.thruster4_pose.orientation.x, msg.thruster4_pose.orientation.y, msg.thruster4_pose.orientation.z, msg.thruster4_pose.orientation.w),(msg.thruster5_pose.orientation.x, msg.thruster5_pose.orientation.y, msg.thruster5_pose.orientation.z, msg.thruster5_pose.orientation.w)])
        #pose = numpy.array([(msg.thruster1_pose.position.x, msg.thruster1_pose.position.y, msg.thruster1_pose.position.z),(msg.thruster2_pose.position.x, msg.thruster2_pose.position.y, msg.thruster2_pose.position.z),(msg.thruster3_pose.position.x, msg.thruster3_pose.position.y, msg.thruster3_pose.position.z),(msg.thruster3_pose.position.x, msg.thruster3_pose.position.y, msg.thruster3_pose.position.z)])
        #quat = numpy.array([(msg.thruster1_pose.orientation.x, msg.thruster1_pose.orientation.y, msg.thruster1_pose.orientation.z, msg.thruster1_pose.orientation.w),(msg.thruster2_pose.orientation.x, msg.thruster2_pose.orientation.y, msg.thruster2_pose.orientation.z, msg.thruster2_pose.orientation.w),(msg.thruster3_pose.orientation.x, msg.thruster3_pose.orientation.y, msg.thruster3_pose.orientation.z, msg.thruster3_pose.orientation.w)])
        #pose = numpy.array((msg.position.x, msg.position.y, msg.position.z))
        #quat = numpy.array((msg.thruster1_pose.orientation.x, msg.thruster1_pose.orientation.y, msg.thruster1_pose.orientation.z, msg.thruster1_pose.orientation.w))

        # This mode assumes that the wrench is given wrt thruster manager
        # configured base_link reference
        self.dynamic_thruster_input(pose, quat)

        self.last_update = rospy.Time.now()

    def input_stamped_callback(self, msg):
        """
        Callback to the subscriber that receiver the stamped wrench to be
        applied on UUV's BODY frame.
        @param msg Stamped wrench message
        """
        if not self.ready:
            return

        force = numpy.array(
            (msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z))
        torque = numpy.array(
            (msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z))

        # Send the frame ID for the requested wrench
        self.publish_thrust_forces(force, torque, msg.header.frame_id.split('/')[-1])
        self.last_update = rospy.Time.now()


if __name__ == '__main__':
    rospy.init_node('thruster_allocator')

    try:
        node = ThrusterAllocatorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('ThrusterAllocatorNode::Exception')
    print('Leaving ThrusterAllocatorNode')
