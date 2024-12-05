#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Jaeyoung Lim"
__contact__ = "jalim@ethz.ch"

from re import M
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import TrajectorySetpoint
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
import numpy as np
import sys


def vector2PoseMsg(frame_id, position, attitude):
    pose_message = PoseStamped()
    # msg.header.stamp = Clock().now().nanoseconds / 1000
    pose_message.header.frame_id=frame_id
    pose_message.pose.orientation.w = attitude[0]
    pose_message.pose.orientation.x = attitude[1]
    pose_message.pose.orientation.y = attitude[2]
    pose_message.pose.orientation.z = attitude[3]
    pose_message.pose.position.x = position[0]
    pose_message.pose.position.y = position[1]
    pose_message.pose.position.z = position[2]

    return pose_message

class PX4Visualizer(Node):

    def __init__(self, namespace=''):
        super().__init__('px4_visualizer')
        self.namespace = namespace

        # configure subscritpions
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.attitude_subsbcription = self.create_subscription(
            VehicleAttitude,
            f'/{self.namespace}/fmu/out/vehicle_attitude',
            self.vehicle_attitude_callback,
            qos_profile)
        
        self.local_position_subscription = self.create_subscription(
            VehicleLocalPosition,
            f'/{self.namespace}/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            qos_profile)
        
        self.setpoint_subscription = self.create_subscription(
            TrajectorySetpoint,
            f'/{self.namespace}/fmu/in/trajectory_setpoint',
            self.trajectory_setpoint_callback,
            qos_profile)

        self.vehicle_pose_publisher = self.create_publisher(PoseStamped, f'/{self.namespace}/px4_visualizer/vehicle_pose', 10)
        self.vehicle_velocity_publisher = self.create_publisher(Marker, f'/{self.namespace}/px4_visualizer/vehicle_velocity', 10)
        self.vehicle_path_publisher = self.create_publisher(Path, f'/{self.namespace}/px4_visualizer/vehicle_path', 10)
        self.setpoint_path_publisher = self.create_publisher(Path, f'/{self.namespace}/px4_visualizer/setpoint_path', 10)

        self.vehicle_attitude = np.array([1.0, 0.0, 0.0, 0.0])
        self.vehicle_local_position = np.array([0.0, 0.0, 0.0])
        self.vehicle_local_velocity = np.array([0.0, 0.0, 0.0])
        self.setpoint_position = np.array([0.0, 0.0, 0.0])
        self.vehicle_path_message = Path()
        self.setpoint_path_message = Path()
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

    def vehicle_attitude_callback(self, msg):
        # TODO: handle NED->ENU transformation 
        self.vehicle_attitude[0] = msg.q[0]
        self.vehicle_attitude[1] = msg.q[1]
        self.vehicle_attitude[2] = -msg.q[2]
        self.vehicle_attitude[3] = -msg.q[3]

    def vehicle_local_position_callback(self, msg):
        # TODO: handle NED->ENU transformation 
        self.vehicle_local_position[0] = msg.x
        self.vehicle_local_position[1] = -msg.y
        self.vehicle_local_position[2] = -msg.z
        self.vehicle_local_velocity[0] = msg.vx
        self.vehicle_local_velocity[1] = -msg.vy
        self.vehicle_local_velocity[2] = -msg.vz

    def trajectory_setpoint_callback(self, msg):
        self.setpoint_position[0] = msg.position[0]
        self.setpoint_position[1] = -msg.position[1]
        self.setpoint_position[2] = -msg.position[2]

    def create_arrow_marker(self, id, tail, vector):
        msg = Marker()
        msg.action = Marker.ADD
        msg.header.frame_id = 'map'
        # msg.header.stamp = Clock().now().nanoseconds / 1000
        msg.ns = 'arrow'
        msg.id = id
        msg.type = Marker.ARROW
        msg.scale.x = 0.1
        msg.scale.y = 0.2
        msg.scale.z = 0.0
        msg.color.r = 0.5
        msg.color.g = 0.5
        msg.color.b = 0.0
        msg.color.a = 1.0
        dt = 0.3
        tail_point = Point()
        tail_point.x = tail[0]
        tail_point.y = tail[1]
        tail_point.z = tail[2]
        head_point = Point()
        head_point.x = tail[0] + dt * vector[0]
        head_point.y = tail[1] + dt * vector[1]
        head_point.z = tail[2] + dt * vector[2]
        msg.points = [tail_point, head_point]

        return msg

    def cmdloop_callback(self):
        vehicle_pose_message = vector2PoseMsg('map', self.vehicle_local_position, self.vehicle_attitude)
        self.vehicle_pose_publisher.publish(vehicle_pose_message)

        # publish time history of the vehicle path
        self.vehicle_path_message.header = vehicle_pose_message.header
        self.vehicle_path_message.poses.append(vehicle_pose_message) 
        self.vehicle_path_publisher.publish(self.vehicle_path_message)

        # publish time history of the vehicle path
        setpoint_pose_message = vector2PoseMsg('map', self.setpoint_position, self.vehicle_attitude)
        self.setpoint_path_message.header = setpoint_pose_message.header
        self.setpoint_path_message.poses.append(setpoint_pose_message)
        self.setpoint_path_publisher.publish(self.setpoint_path_message)

        # publish arrow markers for velocity
        velocity_msg = self.create_arrow_marker(1, self.vehicle_local_position, self.vehicle_local_velocity)
        self.vehicle_velocity_publisher.publish(velocity_msg)

def main(args=None):
    rclpy.init(args=args)

    namespace = sys.argv[1] if len(sys.argv) > 1 else 'default_namespace'
    px4_visualizer = PX4Visualizer(namespace)

    rclpy.spin(px4_visualizer)

    px4_visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()