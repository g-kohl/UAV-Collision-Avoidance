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

__author__ = "Braden Wagstaff"
__contact__ = "braden@arkelectron.com"

from geometry_msgs.msg import Twist, Vector3
import math
import numpy as np
from px4_msgs.msg import ObstacleDistance, OffboardControlMode, TrajectorySetpoint, VehicleAttitude, VehicleCommand, VehicleOdometry, VehicleStatus, VehicleLandDetected
import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Bool
from .teleport import teleport_uav
import sys


DISTANCE_COLLISION = 50
DISTANCE_TOOCLOSE = 150
DISTANCE_SAFE = 1500
MAX_SPEED_LIMIT = 500

DISTANCE_VARSPACE = DISTANCE_SAFE - DISTANCE_TOOCLOSE
SPEED_LIMIT_MULTIPLIER = MAX_SPEED_LIMIT / DISTANCE_VARSPACE

HALF_PI = 1.570796325

DANGER_ZONE_RADIUS = 2

MIN_SPEED = 0.0
MAX_SPEED = 1.0
MAX_YAW_SPEED = 0.5

MISSION_TOLERANCE = 0.25
ANGLE_TOLERANCE = 5


class OffboardControl(Node):

    def __init__(self, namespace, uav_number, mission_mode, mission_steps, spawn_configuration):
        super().__init__('minimal_publisher')

        self.namespace = namespace
        self.uav_id = int(namespace[-1]) + 1
        self.uav_number = uav_number

        self.mission_mode = mission_mode
        self.mission_steps = mission_steps
        self.mission_index = 0
        self.last_position = {
            'N': 0.0,
            'E': 0.0,
            'D': 0.0,
            'yaw': 0.0 
        }

        self.spawn_configuration = spawn_configuration
        self.spawn_position = get_spawn_position(self.spawn_configuration, int(self.namespace[-1]) - 1, self.uav_number)

        self.arrived = True
        self.landed = False
        self.collision_detected = False

        self.cmdloop_control = 0

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        # create subscriptions
        self.status_subscription = self.create_subscription(
            VehicleStatus,
            f'/{self.namespace}/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile
        )

        self.offboard_velocity_subscription = self.create_subscription(
            Twist,
            f'/{self.namespace}/offboard_velocity_cmd',
            self.offboard_velocity_callback,
            qos_profile
        )

        self.attitude_subscription = self.create_subscription(
            VehicleAttitude,
            f'/{self.namespace}/fmu/out/vehicle_attitude',
            self.attitude_callback,
            qos_profile
        )

        # subscription to receive the position of every UAV
        self.uav_positions = {}
        self.position_subscriptions = []

        for i in range(1, self.uav_number+1):
            uav_namespace = f'px4_{i}'
            topic_name = f'/{uav_namespace}/fmu/out/vehicle_odometry'

            self.uav_positions[uav_namespace] = None

            subscription = self.create_subscription(
                VehicleOdometry,
                topic_name,
                lambda msg, ns=uav_namespace: self.odometry_callback(msg, ns),
                qos_profile
            )

            self.position_subscriptions.append(subscription)

        self.obstacle_distance_subscription = self.create_subscription(
            ObstacleDistance,
            f'/{self.namespace}/fmu/out/obstacle_distance',
            self.obstacle_distance_callback,
            qos_profile
        )

        self.my_bool_subscription = self.create_subscription(
            Bool,
            f'/{self.namespace}/arm_message',
            self.arm_message_callback,
            qos_profile
        )

        # create publishers
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, f'/{self.namespace}/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_velocity = self.create_publisher(Twist, f'/{self.namespace}/fmu/in/setpoint_velocity/cmd_vel_unstamped', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, f'/{self.namespace}/fmu/in/trajectory_setpoint', qos_profile)
        self.publisher_vehicle_command = self.create_publisher(VehicleCommand, f'/{self.namespace}/fmu/in/vehicle_command', 10)

        self.arm_publisher = self.create_publisher(Bool, f'/{self.namespace}/arm_message', qos_profile)

        # create callback function for the arm timer (period is arbitrary, just should be more than 2Hz)
        arm_timer_period = .1 # seconds
        self.arm_timer_ = self.create_timer(arm_timer_period, self.arm_timer_callback)

        # create callback function for the command loop (period is arbitrary, just should be more than 2Hz)
        # commands in cmdloop_callback won't be executed if the vehicle is not in offboard mode
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_ARMED
        self.velocity = Vector3()
        self.yaw = 0.0 # yaw value we send as command
        self.true_yaw = 0.0 # current yaw value of drone
        self.offboard_mode = False
        self.flight_check = False
        self.my_control = 0
        self.arm_message = False
        self.failsafe = False

        # states with corresponding callback functions that run once when state switches
        self.states = {
            "IDLE": self.state_init,
            "ARMING": self.state_arming,
            "TAKEOFF": self.state_takeoff,
            "LOITER": self.state_loiter,
            "OFFBOARD": self.state_offboard
        }

        self.current_state = "IDLE"
        self.last_state = self.current_state

    # callback function that arms, takes off, and switches to offboard mode (implements a finite state machine)
    def arm_timer_callback(self):
        match self.current_state:
            case "IDLE":
                if(self.flight_check and self.arm_message == True):
                    self.current_state = "ARMING"
                    self.get_logger().info(f"Arming")

            case "ARMING":
                if(not(self.flight_check)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Arming, Flight Check Failed")
                elif(self.arm_state == VehicleStatus.ARMING_STATE_ARMED and self.my_control > 10):
                    self.current_state = "TAKEOFF"
                    self.get_logger().info(f"Arming, Takeoff")

                self.arm() # send arm command

            case "TAKEOFF":
                if(not(self.flight_check)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Takeoff, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF):
                    self.current_state = "LOITER"
                    self.get_logger().info(f"Takeoff, Loiter")

                self.arm() # send arm command
                self.take_off() # send takeoff command

            # wait in this state while taking off, and the moment VehicleStatus switches to Loiter state it will switch to offboard
            case "LOITER":
                if(not(self.flight_check)):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Loiter, Flight Check Failed")
                elif(self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER):
                    self.current_state = "OFFBOARD"
                    self.get_logger().info(f"Loiter, Offboard")

                self.arm()

            case "OFFBOARD":
                if(not(self.flight_check) or self.arm_state == VehicleStatus.ARMING_STATE_DISARMED or self.failsafe == True):
                    self.current_state = "IDLE"
                    self.get_logger().info(f"Offboard, Flight Check Failed")

                self.state_offboard()

        if(self.arm_state != VehicleStatus.ARMING_STATE_ARMED):
            self.arm_message = False

        if (self.last_state != self.current_state):
            self.last_state = self.current_state
            self.get_logger().info(self.current_state)

        self.my_control += 1


    def state_init(self):
        self.my_control = 0


    def state_arming(self):
        self.my_control = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command send")


    def state_takeoff(self):
        self.my_control = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1 = 1.0, param7=5.0) # param7 is altitude in meters
        self.get_logger().info("Takeoff command send")


    def state_loiter(self):
        self.my_control = 0
        self.get_logger().info("Loiter Status")


    def state_offboard(self):
        self.my_control = 0
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
        self.offboard_mode = True

    # arms the vehicle
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command send")

    # takes off the vehicle to a user specified altitude (meters)
    def take_off(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1 = 1.0, param7=5.0) # param7 is altitude in meters
        self.get_logger().info("Takeoff command send")

    # publishes command to /fmu/in/vehicle_command
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7 # altitude value in takeoff command
        msg.command = command # command ID
        msg.target_system = self.uav_id # system which should execute the command
        msg.target_component = 1 # component which should execute the command, 0 for all components
        msg.source_system = 1 # system sending the command
        msg.source_component = 1 # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.publisher_vehicle_command.publish(msg)

    # receives and sets vehicle status values
    def vehicle_status_callback(self, msg):
        if (msg.nav_state != self.nav_state):
            self.get_logger().info(f"NAV_STATUS: {msg.nav_state}")

        if (msg.arming_state != self.arm_state):
            self.get_logger().info(f"ARM STATUS: {msg.arming_state}")

        if (msg.failsafe != self.failsafe):
            self.get_logger().info(f"FAILSAFE: {msg.failsafe}")

        if (msg.pre_flight_checks_pass != self.flight_check):
            self.get_logger().info(f"Flight check: {msg.pre_flight_checks_pass}")

        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flight_check = msg.pre_flight_checks_pass

    # receives Twist commands from Teleop and converts FRD -> FLU
    def offboard_velocity_callback(self, msg):
        self.velocity.x = -msg.linear.y
        self.velocity.y = msg.linear.x

        self.velocity.z = -msg.linear.z # Z (FLU) is -Z (FRD)

        self.yaw = msg.angular.z # conversion for angular z is done in the attitude_callback function(it's the '-' in front of self.true_yaw)

    # receives current trajectory values from drone and grabs the yaw value of the orientation
    def attitude_callback(self, msg):
        orientation_q = msg.q

        # true_yaw is the drones current yaw value
        self.true_yaw = -(np.arctan2(2.0*(orientation_q[3]*orientation_q[0] + orientation_q[1]*orientation_q[2]),
                                  1.0 - 2.0*(orientation_q[0]*orientation_q[0] + orientation_q[1]*orientation_q[1])))
        
    # receives odometry values for each UAV
    def odometry_callback(self, msg, namespace):
        """
        Odometry message fields:
            timestamp: int
            timestamp_sample: int
            pose_frame: int
            position: float[3]
            q: float[4]
            velocity_frame: int
            velocity: float[3]
            angular_velocity: float[3]
            position_variance: float[3]
            orientation_variance: float[3]
            velocity_variance: float[3]
        """

        spawn = get_spawn_position(self.spawn_configuration, int(namespace[-1]) - 1, self.uav_number)
        self.uav_positions[namespace] = (msg.position[0]+spawn[0], msg.position[1]+spawn[1], msg.position[2])
        self.odometry = msg

    # receives obstacle distances values
    def obstacle_distance_callback(self, msg):
        """
        Obstacle distance message fields:
            timestamp: int
            frame: int
            sensor_type: int
            distances: int[]
            increment: float
            min_distance: int
            max_distance: int
            angle_offset: float
        """

        self.obstacle_distance = msg

    def arm_message_callback(self, msg):
        self.arm_message = msg.data
        self.get_logger().info(f"Arm Message: {self.arm_message}")

    # publishes offboard control modes and velocity as trajectory setpoints
    def cmdloop_callback(self):
        if self.current_state == "IDLE" and self.mission_mode: ### TENTAR MANIPULAR ODOMETRIA DO PX4
            arm_message = Bool()
            arm_message.data = True
            self.arm_publisher.publish(arm_message)

        if self.offboard_mode:
            self.cmdloop_control += 1

            # publish offboard control modes
            offboard_message = OffboardControlMode()
            offboard_message.timestamp = int(Clock().now().nanoseconds / 1000)
            offboard_message.position = False
            offboard_message.velocity = True
            offboard_message.acceleration = False
            self.publisher_offboard_mode.publish(offboard_message)

            # compute velocity in the direction of the current mission step
            if self.mission_mode:
                if self.arrived:
                    self.last_position['N'] = self.odometry.position[0]
                    self.last_position['E'] = self.odometry.position[1]
                    self.last_position['D'] = self.odometry.position[2]
                    self.last_position['yaw'] = math.degrees(self.true_yaw) + 90

                    self.next_step = self.mission_steps[self.mission_index]
                    self.arrived = False

                if self.mission_index < len(self.mission_steps):
                    if self.next_step.startswith("go"):
                        self.arrived = self.go_to_destiny(self.next_step)
                    elif self.next_step.startswith("turn"):
                        self.arrived = self.turn_to_destiny(self.next_step)

                    if self.arrived:
                        self.mission_index += 1

                if self.mission_index >= len(self.mission_steps): # end of mission
                    self.end_episode()

            horizontal_safe_velocity = self.avoid_obstacles()
            safe_velocity = self.avoid_uav(horizontal_safe_velocity)

            # compute velocity in the world frame
            cos_yaw = np.cos(self.true_yaw)
            sin_yaw = np.sin(self.true_yaw)
            velocity_world_x = (safe_velocity.x * cos_yaw - safe_velocity.y * sin_yaw)
            velocity_world_y = (safe_velocity.x * sin_yaw + safe_velocity.y * cos_yaw)

            # create and publish TrajectorySetpoint message with NaN values for position and acceleration
            trajectory_message = TrajectorySetpoint()
            trajectory_message.timestamp = int(Clock().now().nanoseconds / 1000)
            trajectory_message.velocity[0] = velocity_world_x
            trajectory_message.velocity[1] = velocity_world_y
            trajectory_message.velocity[2] = safe_velocity.z            
            trajectory_message.position[0] = float('nan')
            trajectory_message.position[1] = float('nan')
            trajectory_message.position[2] = float('nan')
            trajectory_message.acceleration[0] = float('nan')
            trajectory_message.acceleration[1] = float('nan')
            trajectory_message.acceleration[2] = float('nan')
            trajectory_message.yaw = float('nan')
            trajectory_message.yawspeed = self.yaw
            trajectory_message.yawspeed = self.yaw

            self.publisher_trajectory.publish(trajectory_message)


    def go_to_destiny(self, next_step):
        coordinates = conversion_XYZ_NED(get_coordinates(next_step))

        error_N = (self.last_position['N'] + coordinates[0]) - self.odometry.position[0]
        error_E = (self.last_position['E'] + coordinates[1]) - self.odometry.position[1]

        cos_yaw = np.cos(self.true_yaw)
        sin_yaw = np.sin(self.true_yaw)

        body_error_x = error_N * cos_yaw + error_E * sin_yaw
        body_error_y = -error_N * sin_yaw + error_E * cos_yaw

        if abs(body_error_x) > MISSION_TOLERANCE:
            self.velocity.x = MAX_SPEED if body_error_x > 0 else -MAX_SPEED
        else:
            self.velocity.x = MIN_SPEED

        if abs(body_error_y) > MISSION_TOLERANCE:
            self.velocity.y = MAX_SPEED if body_error_y > 0 else -MAX_SPEED
        else:
            self.velocity.y = MIN_SPEED

        error_D = (self.last_position['D'] + coordinates[2]) - self.odometry.position[2]
        if abs(error_D) > MISSION_TOLERANCE:
            self.velocity.z = MAX_SPEED if error_D > 0 else -MAX_SPEED
        else:
            self.velocity.z = MIN_SPEED

        return abs(error_N) < MISSION_TOLERANCE and abs(error_E) < MISSION_TOLERANCE and abs(error_D) < MISSION_TOLERANCE
        

    def turn_to_destiny(self, next_step):
        turn = float(next_step.strip("turn:"))
        current_direction = math.degrees(self.true_yaw) + 90
        error = (self.last_position['yaw'] + turn) - current_direction

        if abs(error) > ANGLE_TOLERANCE:
            self.yaw = MAX_YAW_SPEED if error > 0 else -MAX_YAW_SPEED
        else:
            self.yaw = MIN_SPEED

        return abs(error) < ANGLE_TOLERANCE
                

    def avoid_obstacles(self):
        velocity = Vector2(-self.velocity.x, -self.velocity.y) * 100
        horizontal_speed = np.sqrt(velocity.x**2 + velocity.y**2)
        velocity_direction = np.arctan2(velocity.y, velocity.x)
        increment_direction = deg2rad(self.obstacle_distance.increment)
        offset_direction = deg2rad(self.obstacle_distance.angle_offset)

        # for each distance measurement (in cm)
        for i, distance in enumerate(self.obstacle_distance.distances):
            # if the distance is higher than the safe distance, ignore it
            # if the distance is higher than the maximum distance, it acts like a flag
            if distance > DISTANCE_SAFE:
                if distance == 65535: # 65535 marks the end of the valid data
                    break

                continue # anything that is greater than the safe distance is ignored

            if distance < DISTANCE_COLLISION:
                self.collision_detected = True
                self.end_episode()

            ray_direction = i * increment_direction + offset_direction # direction of the ray relative to the drone's front
            ray_velocity_angle = ray_direction - velocity_direction # ray-velocity angle (angle with the drone's velocity)

            # ignore if the drone is not going toward that direction at all
            if (ray_velocity_angle < -HALF_PI or ray_velocity_angle > HALF_PI):
                continue

            direction_speed_limit = (distance - DISTANCE_TOOCLOSE) * SPEED_LIMIT_MULTIPLIER # individual speed limit for the direction in question
            projected_speed = horizontal_speed * np.cos(ray_velocity_angle) # projected speed of the drone in the direction of the ray
            exceeding_speed = projected_speed - direction_speed_limit # speed over the speed limit in the direction

            if exceeding_speed <= 0:
                continue # if the projected speed is below the speed limit, it does not need to be adjusted

            # the speed is not safe
            # set the speed in that direction to the speed limit
            direction_vector = Vector2(np.cos(ray_direction), np.sin(ray_direction)) # unit vector in the direction of the ray
            exceeding_velocity = direction_vector * exceeding_speed

            # adjust velocity
            velocity -= exceeding_velocity
            horizontal_speed = np.sqrt(velocity.x**2 + velocity.y**2)
            velocity_direction = np.arctan2(velocity.y, velocity.x)

        # update velocity
        return velocity * -0.01
    

    def avoid_uav(self, horizontal_velocity):
        velocity = Vector3()
        velocity.x = horizontal_velocity.x
        velocity.y = horizontal_velocity.y
        velocity.z = self.velocity.z

        return velocity


    def end_episode(self):
        self.arrived = False

        if self.current_state != "IDLE":
            self.velocity.x = MIN_SPEED
            self.velocity.y = MIN_SPEED
            self.velocity.z = MAX_SPEED
            self.yaw = MIN_SPEED
        else:
            self.mission_mode = True
            self.mission_index = 0
            self.arrived = True
            self.collision_detected = False
            self.offboard_mode = False

            teleport_uav()

        ##########

        # self.arrived = False
        # self.cmdloop_control = 0

        # while self.cmdloop_control < 1000000:
        #     self.cmdloop_control += 1

        #     self.velocity.x = MIN_SPEED
        #     self.velocity.y = MIN_SPEED
        #     self.yaw = MIN_SPEED

        #     print(self.cmdloop_control)

        # self.cmdloop_control = 0

        # self.mission_mode = True
        # self.mission_index = 0
        # self.arrived = True
        # self.collision_detected = False
        # self.offboard_mode = False

        # teleport_uav()




class Vector2:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def __add__(self, other):
        return Vector2(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vector2(self.x - other.x, self.y - other.y)

    def __mul__(self, other: float):
        return Vector2(self.x * other, self.y * other)

    def __div__(self, other: float):
        return Vector2(self.x / other, self.y / other)


# auxiliar functions

def get_coordinates(step):
    coordinates = step.strip("go:").split(",")
    coordinates = [float(c) for c in coordinates]

    return coordinates


def conversion_XYZ_NED(coordinates):
    return (-coordinates[0], coordinates[1], -coordinates[2])


def deg2rad(deg: float):
    return deg * math.pi / 180.0


def is_in_danger_zone(reference_uav, intruder_uav):
    if np.sqrt((reference_uav[0] - intruder_uav[0])**2 + (reference_uav[1] - intruder_uav[1])**2 + (reference_uav[2] - intruder_uav[2])**2) < DANGER_ZONE_RADIUS:
        return True
    
    return False


def get_spawn_position(spawn_configuration, uav_instance, uav_number):
    if spawn_configuration == 'l': # UAVs in a line fomation
        coord_x = uav_instance
        coord_y = 0
    elif spawn_configuration == 's': # UAVs in a square formation
        width = math.ceil(math.sqrt(uav_number))
        coord_x = uav_instance % width
        coord_y = uav_instance // width

    return [coord_x*3, coord_y*3]


def main(args=None):
    rclpy.init(args=args)

    namespace = sys.argv[1]
    uav_number = int(sys.argv[2])
    mission_mode = True if sys.argv[3] == 'true' else False
    mission_steps = sys.argv[4].strip("\n").split(";")
    spawn_configuration = sys.argv[5]

    offboard_control = OffboardControl(namespace, uav_number, mission_mode, mission_steps, spawn_configuration)

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()