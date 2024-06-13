import rclpy
from rclpy.node import Node

import px4_msgs.msg as px4_msgs
import std_srvs.srv as std_srvs

import glassy_msgs.msg as glassy_msgs


# allowed libraries
import numpy as np


class GlassyEval(Node):
    def __init__(self):
        """
        Class
        """
        super().__init__('glassy_eval')

        # initialize all the variables to be used
        self.yaw_rate = 0.0
        self.surge = 0.0
        self.sway = 0.0
        self.yaw = 0.0

        self.initial_yaw = 0.0

        self.x = 0.0
        self.y = 0.0

        self.initial_x = 0.0
        self.initial_y = 0.0

        self.is_active = False

        # used to calculate the score... ( you may test different coeficients to see how they affect the score, and different max_speeds)

        self.max_velocity = 3.0
        self.total_dist_coef = 1.0
        self.cross_dist_coef = 1.0
        self.vel_coef = 10.0


        #auxiliary variables
        self.time_prev = 0.0
        self.cross_track_distance = 0.0
        self.velocity_above_max = 0.0





        # create subscriber for the state of the vehicle
        self.state_subscriber_ = self.create_subscription(glassy_msgs.State, 'glassy/state', self.state_subscription_callback, 1)

        # create subscriber for the mission status
        self.mission_status_subscriber_ = self.create_subscription(glassy_msgs.MissionInfo, 'glassy/mission_status', self.mission_status_subscription_callback, 1)


        # create timer
    
        self.time_of_last_mission_status_msg_received = None



    def state_subscription_callback(self, msg):
        """
        Receives the state message and updates the local variables that store the state of the vehicle.
        """

        # Get the yaw and yaw rate (most important for the challenge)
        self.yaw = msg.yaw
        self.yaw_rate = msg.yaw_rate

        self.surge = msg.v_body[0]
        self.sway = msg.v_body[1]

        self.x = msg.p_ned[0]
        self.y = msg.p_ned[1]

        current_time = self.get_clock().now().nanoseconds/1e9
        dt = (current_time - self.time_prev)

        # calculate the integral of the cross track distance
        self.cross_track_distance += np.abs(( np.cos(self.initial_yaw) * (self.initial_y-self.y) - np.sin(self.initial_yaw) * (self.initial_x - self.x)))**2 * dt

        # calculate the integral of the velocity above the max velocity
        self.velocity_above_max += np.maximum(np.sqrt(self.surge**2 + self.sway**2) - self.max_velocity, 0)**2 * dt


        self.time_prev = current_time

        




    def mission_status_subscription_callback(self, msg):
        """
        Checks whether the mission is active or not.
        """
        if self.is_active:
            if msg.mission_mode != glassy_msgs.MissionInfo.SUMMER_CHALLENGE:
                self.is_active = False
                get_projection_on_line = np.dot([self.x - self.initial_x, self.y - self.initial_y], [np.cos(self.initial_yaw), np.sin(self.initial_yaw)])

                with open("~/.tt/tt.txt", "a") as myfile:
                    myfile.write('MISSION SCORE ------------------------------------------------------------------------------\n')
                    myfile.write('ALONG TRACK DISTANCE (larger is better): ' + str(get_projection_on_line)+'\n')
                    myfile.write('CROSS TRACK DISTANCE SQUARED INTEGRAL (lower is better): ' + str(self.cross_track_distance)+'\n')
                    myfile.write('VELOCITY OVER MAX SQUARED INTEGRAL (lower is better): ' + str(self.velocity_above_max)+'\n')
                    myfile.write('ALONG TRACK DISTANCE SCORE: ' + str(get_projection_on_line * self.total_dist_coef)+'\n')
                    myfile.write('CROSS TRACK DISTANCE INTEGRAL SCORE: ' + str(- self.cross_dist_coef * self.cross_track_distance)+'\n')
                    myfile.write('VELOCITY OVER MAX INTEGRAL SCORE: ' + str(- self.vel_coef * self.velocity_above_max)+'\n')
                    myfile.write('TOTAL SCORE: ' + str(get_projection_on_line * self.total_dist_coef - self.cross_dist_coef * self.cross_track_distance - self.vel_coef * self.velocity_above_max)+'\n')
                    myfile.write('MISSION SCORE ------------------------------------------------------------------------------'+'\n')


        else:
            if msg.mission_mode == glassy_msgs.MissionInfo.SUMMER_CHALLENGE:
                self.is_active = True

                # reset the initial mission values
                # HERE YOU CAN ADD SLIGHT OFFSETS TO THE INITIAL VALUES, TO MAKE THE CHALLENGE MORE INTERESTING/COMPLICATED, AND TEST YOUR CONTROLLER BETTER
                # Ex: self.initial_yaw = self.initial_yaw + 0.1 (around 5º offset)
                self.initial_x = self.x
                self.initial_y = self.y
                self.initial_yaw = self.yaw

                # these are used for evaluation purposes
                self.time_prev = self.get_clock().now().nanoseconds/1e9
                self.cross_track_distance = 0.0
                self.velocity_above_max = 0.0
        


def main(args=None):
    rclpy.init(args=args)

    test = GlassyEval()

    rclpy.spin(test)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    test.destroy_node()
    rclpy.shutdown()