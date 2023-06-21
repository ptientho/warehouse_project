#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from geometry_msgs.msg import Polygon, Point32, Twist
from attach_shelf_msg.srv import GoToLoading
from std_msgs.msg import Empty
from rclpy.parameter import Parameter, ParameterType
import rclpy
from rclpy.node import Node

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Loading position defined in map frame
shelf_positions = {
    "shelf_A": [5.76276, -0.0164632],
    }

# Shipping destination for picked products

shipping_destinations = {
    "shipping_position": [0.518168, -2.94348],
    "pallet_jack7": [-0.073, -8.497],
    "conveyer_432": [6.217, 2.153],
    "frieght_bay_3": [-6.349, 9.147]}


'''
Basic item picking demo. In this demonstration, the expectation
is that a person is waiting at the item shelf to put the item on the robot
and at the pallet jack to remove it
(probably with a button for 'got item, robot go do next task').
'''


def main():
    
    ####################
    request_item_location = 'shelf_A'
    request_destination = 'shipping_position'
    ####################
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Define Vars~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    rclpy.init()
    main = Node('nav2_app_node')
    
    navigator = BasicNavigator()
    #define publisher for footprint in local map
    polygon_pub_local = main.create_publisher(Polygon,'/local_costmap/footprint',10)
    polygon_pub_global = main.create_publisher(Polygon,'/global_costmap/footprint',10)
    #define a publisher for velocity
    vel_pub = main.create_publisher(Twist,'/robot/cmd_vel',10)
    #define attach_shelf client
    attach_shelf_client = main.create_client(GoToLoading,'/approach_shelf')
    #define elevator up/down
    lift_up_pub = main.create_publisher(Empty,'/elevator_up',10)
    lift_down_pub = main.create_publisher(Empty,'/elevator_down',10)
    msg = Empty()
    lift_down_pub.publish(msg)

    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Initial Pose~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.010746
    initial_pose.pose.position.y = -0.0113578
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to activate fully
    navigator.waitUntilNav2Active()

    shelf_item_pose = PoseStamped()
    shelf_item_pose.header.frame_id = 'map'
    shelf_item_pose.header.stamp = navigator.get_clock().now().to_msg()
    shelf_item_pose.pose.position.x = shelf_positions[request_item_location][0]
    shelf_item_pose.pose.position.y = shelf_positions[request_item_location][1]
    shelf_item_pose.pose.orientation.z = -0.790661
    shelf_item_pose.pose.orientation.w = 0.612254
    print('Received request for item picking at ' + request_item_location + '.')
    navigator.goToPose(shelf_item_pose)


    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~Send Feedback~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival at ' + request_item_location +
                  ' for worker: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Reached Loading Position for ' + request_item_location)
        #call attach_shelf service. how to continue after service call?
        while not attach_shelf_client.wait_for_service(timeout_sec=5.0):
            main.get_logger().info('service not available... waiting')

        #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        main.get_logger().info('begin request....') 
        request_msg = GoToLoading.Request()
        request_msg.attach_to_shelf = True

        #wait the task to complete and get the result
        result_future = attach_shelf_client.call_async(request_msg)
        rclpy.spin_until_future_complete(main,result_future)
        result = result_future.result()

        if (result.complete == True):
            main.get_logger().info('attach shelf completed... moving the shelf to shipping position')
            
            #elevator up
            lift_up_pub.publish(msg)

            footprint = Polygon()
            point1 = Point32()
            point2 = Point32()
            point3 = Point32()
            point4 = Point32()

            point1.x = 0.5
            point1.y = 0.4

            point2.x = 0.5
            point2.y = -0.4

            point3.x = -0.5
            point3.y = -0.4

            point4.x = -0.5
            point4.y = 0.4
            
            #reverse movement
            vel = Twist()
            vel.linear.x = -0.2
            vel_pub.publish(vel)
            time.sleep(12.0)
            vel.linear.x = 0.0
            vel_pub.publish(vel)
            
            #rotate 90 deg
            vel.angular.z = -0.2
            vel_pub.publish(vel)
            time.sleep(16.0)
            vel.angular.z = 0.0
            vel_pub.publish(vel)

            footprint.points = [point1,point2,point3,point4]
            polygon_pub_local.publish(footprint)
            polygon_pub_global.publish(footprint)
            main.get_logger().info('footprint size changed')
            #~~~~~~~~~~~~~~~~~~~~~~~~~~~~Send shipping position~~~~~~~~~~~~~~~~~~~~~
            shipping_destination = PoseStamped()
            shipping_destination.header.frame_id = 'map'
            shipping_destination.header.stamp = navigator.get_clock().now().to_msg()
            shipping_destination.pose.position.x = shipping_destinations[request_destination][0]
            shipping_destination.pose.position.y = shipping_destinations[request_destination][1]
            shipping_destination.pose.orientation.z = 0.713029
            shipping_destination.pose.orientation.w = 0.701135
            navigator.goToPose(shipping_destination)

            #~~~~~~~~~~~~~~~~~~~~~~~~~~~~Send Feedback~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            i = 0
            while not navigator.isTaskComplete():
                i = i + 1
                feedback = navigator.getFeedback()
                if feedback and i % 5 == 0:
                    print('Estimated time of arrival at ' + request_destination  +
                            ' for worker: ' + '{0:.0f}'.format(
                                Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                            + ' seconds.')
            
            result = navigator.getResult()

            if result == TaskResult.SUCCEEDED:
                print('Reached ' + request_destination)

                #elevator down
                lift_down_pub.publish(msg)
                #go out of the shelf
                vel.linear.x = 0.3
                vel_pub.publish(vel)
                time.sleep(5.0)
                vel.linear.x = 0.0
                vel_pub.publish(vel)

                #set robot_radius
                '''
                parameter1 = Parameter(

                    "local_costmap.local_costmap.robot_radius",Parameter.Type.DOUBLE,
                    0.25
                )
                parameter2 = Parameter(
                    "global_costmap.global_costmap.robot_radius",Parameter.Type.DOUBLE,
                    0.25
                )
                
                main.set_parameters([parameter1,parameter2])
                '''
                #navigate to initial pose
                navigator.goToPose(initial_pose)

                #~~~~~~~~~~~~~~~~~~~~~~~~~~~~Send Feedback~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                i = 0
                while not navigator.isTaskComplete():
                    i = i + 1
                    feedback = navigator.getFeedback()
                    if feedback and i % 5 == 0:
                        print('Estimated time of arrival at ' + 'initial position'  +
                                ' for worker: ' + '{0:.0f}'.format(
                                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                                + ' seconds.')
                
                result = navigator.getResult()
                if (result == TaskResult.SUCCEEDED):
                    print('Reached home. Task completed.')
                elif result == TaskResult.CANCELED:
                    print('Task ' +
                    ' was canceled. Stopping the robot...')
                    vel.linear.x = 0.0
                    vel_pub.publish(vel)
                elif result == TaskResult.FAILED:
                    print('Task failed!')
                    exit(-1)

            elif result == TaskResult.CANCELED:
                print('Task at ' + request_destination +
                    ' was canceled. Returning to staging point...')
                initial_pose.header.stamp = navigator.get_clock().now().to_msg()
                navigator.goToPose(initial_pose)

            elif result == TaskResult.FAILED:
                print('Task at ' + request_destination + ' failed!')
                exit(-1)

    elif result == TaskResult.CANCELED:
        print('Task at ' + request_item_location +
              ' was canceled. Returning to staging point...')
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)

    elif result == TaskResult.FAILED:
        print('Task at ' + request_item_location + ' failed!')
        exit(-1)

    while not navigator.isTaskComplete():
        pass

    exit(0)


if __name__ == '__main__':
    main()

