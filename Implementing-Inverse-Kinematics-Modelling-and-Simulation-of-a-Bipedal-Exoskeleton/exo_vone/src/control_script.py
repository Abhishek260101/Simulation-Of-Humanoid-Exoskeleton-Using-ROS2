#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import math
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Quaternion
from std_msgs.msg import  MultiArrayLayout, MultiArrayDimension, Float64MultiArray
from sensor_msgs.msg import JointState

from tf_transformations import euler_from_quaternion
import numpy as np


import sympy as sp
import time

# global dx, dy, yaw, 
global θrad, αrad, a, d
global θ1, θ2, θ3

θrad, αrad, a, d = sp.symbols('θdeg αdeg a d')
θ1, θ2, θ3 = sp.symbols('θ1 θ2 θ3')

# Created a function to print the Transformation Matrix of each joint wrt previous joint

def Transform_matrix(θrad, αrad, a, d):
    return sp.Matrix([
        [sp.cos(θrad), -sp.sin(θrad) * sp.cos(αrad), sp.sin(θrad) * sp.sin(αrad), a * sp.cos(θrad)],
        [sp.sin(θrad), sp.cos(θrad) * sp.cos(αrad), -sp.cos(θrad) * sp.sin(αrad), a * sp.sin(θrad)],
        [0, sp.sin(αrad), sp.cos(αrad), d],
        [0, 0, 0, 1]
    ])

#Writing all the original transformation matrices of each joint wrt the previous joints for RIGHT LEG

# def T1(θ1_):
#     return Transform_matrix((θ1_ + sp.pi/2), 0, 30, -163.4)

# def T2(θ2_):
#     return Transform_matrix(θ2_, - sp.pi, -522.4, 0)

# def T3(θ3_):
#     return Transform_matrix(θ3_, - sp.pi/2, -460.5, 0)

def T1(θ1_):
    return Transform_matrix((θ1_ + sp.pi/2), - sp.pi/2, 0, 30)

def T2(θ2_):
    return Transform_matrix((θ2_ + sp.pi/2), 0, 522.4, 163.4)

def T3(θ3_):
    return Transform_matrix(θ3_, 0, 460.5, 0)

def main(args=None):
        
        # Declared some initial angles in order to avoid singularity while calculating the Jacobian inverse
        q = sp.Matrix([0.0001, 0, 0])

        #Printing all the original transformation matrices of each joint wrt the base
        T20 = T1(θ1) * T2(θ2)
        T30 = T1(θ1) * T2(θ2) * T3(θ3)

        # From the final transformation matrix, extracting the first three members of the final matrix
        # to get the position of the end effector wrt base
        P6 = T30[:3, -1]

        # Declaring all the partial derivatives of the Position of end effector wrt the joint angles
        dP_dθ1 = P6.diff(θ1)
        dP_dθ2 = P6.diff(θ2)
        dP_dθ3 = P6.diff(θ3)

        # Obtaining the axis vector Z for each joint
        Z1 = T1(θ1)[:3, 2]
        Z2 = T20[:3, 2]
        Z3 = T30[:3, 2]

        # Jacobian matrix using the seconf method
        J = sp.Matrix([
            [dP_dθ1, dP_dθ2, dP_dθ3],
            [Z1, Z2, Z3]
        ])

        # Printing out the Jacobian Matrix
        # sp.pprint(J)
        # J0 = J.subs({
        #     θ1: 0, θ2: 0, θ3: 0
        # })
        # sp.pprint(J0)

        rclpy.init(args=args)

        node1 = rclpy.create_node('vel_pos_publisher_node1')
        joint_position_pub1 =   node1.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        joint_state_pub1 =   node1.create_publisher(JointState, '/joint_states', 10)
        
        node2 = rclpy.create_node('vel_pos_publisher_node2')
        joint_position_pub2 =   node2.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        joint_state_pub2 =   node2.create_publisher(JointState, '/joint_states', 10)

        node1.get_logger().info("The node started spinning")

        while rclpy.ok():
                 
            node1.get_logger().info("The node entered while loop")


      
# # LEFT Leg BACKWARD Motion
#             for i_val in range(0, 40):
#                 if float(q[2]) < 0.6 :
#                         # y = 952.9  * sp.sin((2 * np.pi / 40) * i_val)
#                         y_dot = 952.9 * sp.cos((2 * np.pi / 40) * i_val) * (2 * np.pi / 200)

#                         # z = - 952.9  * sp.cos((2 * np.pi / 40) * i_val)
#                         z_dot = 952.9 * sp.sin((2 * np.pi / 40) * i_val) * (2 * np.pi / 200)

#                     # Calculating the end effector velocity Matrix
#                         E = sp.Matrix([
#                             [0],
#                             [y_dot],
#                             [z_dot],
#                             [0],
#                             [0],
#                             [0],
#                         ])
                        
#                     # Substituting joint angles to obtian Jacobian and Jacobian Inverse and the Joint angular velocity matrix
#                         J0 = J.subs({
#                             θ1: q[0], θ2: q[1], θ3: q[2]
#                         })

#                         qdot = J0.pinv() * E

#                         time.sleep(1)
#                     # Obtained the joint angle in each iteration
#                         q += qdot
#                         node2.get_logger().info('The hip-joint angle is %f' % q[2])
#                     # Obtained the updated value of Transformation Matrix after rotation and extracted the end-effector position for the 
#                     # Transformation Matrix
#                         # T = T1(q[0]) * T2(q[1]) * T3(q[2])
#                         # P6 = T[:3, -1]


#                         joint_positions = Float64MultiArray(layout=MultiArrayLayout(
#                             dim=[MultiArrayDimension(label="joint_positions", size=2, stride=1)],
#                             data_offset=0
#                         ))        
#                         joint_positions.data = [-float(q[0]), -float(q[1]), -float(q[2]), 0.0, 0.0, 0.0]   
#                         joint_position_pub2.publish(joint_positions)

# # LEFT BACKWARD RIGHT FORWARD Motion
            for i_val in range(0, 40):
                if float(q[2]) < 0.5 :
                    # y = 952.9  * sp.sin((2 * np.pi / 40) * i_val)
                    y_dot = 952.9 * sp.cos((2 * np.pi / 40) * i_val) * (2 * np.pi / 80)

                    # z = - 952.9  * sp.cos((2 * np.pi / 40) * i_val)
                    z_dot = 952.9 * sp.sin((2 * np.pi / 40) * i_val) * (2 * np.pi / 80)

                # Calculating the end effector velocity Matrix
                    E = sp.Matrix([
                        [0],
                        [y_dot],
                        [z_dot],
                        [0],
                        [0],
                        [0],
                    ])
                    
                # Substituting joint angles to obtian Jacobian and Jacoian Inverse and the Joint angular velocity matrix
                    J0 = J.subs({
                        θ1: q[0], θ2: q[1], θ3: q[2]
                    })

                    qdot = J0.pinv() * E

                    time.sleep(0.08)
                # Obtained the joint angle in each iteration
                    q += qdot*(40/80)
                    node1.get_logger().info('The hip-joint angle is %f' % q[2])
                    node1.get_logger().info('1st loop')                    
                # Obtained the updated value of Transformation Matrix after rotation and extracted the end-effector position for the 
                # Transformation Matrix
                    # T = T1(q[0]) * T2(q[1]) * T3(q[2])
                    # P6 = T[:3, -1]

                    joint_positions = Float64MultiArray(layout=MultiArrayLayout(
                        dim=[MultiArrayDimension(label="joint_positions", size=2, stride=1)],
                        data_offset=0
                    ))        
                    # joint_positions.data = [float(q[2]), float(q[2]), float(q[2]), float(q[2]), float(q[2]), float(q[2])]   
                    joint_positions.data = [-float(q[2]), -float(q[1]), -float(q[2]), -float(q[2]), -float(q[2]), float(q[2])]
                    joint_position_pub1.publish(joint_positions)
        
            # joint_positions.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
            # joint_position_pub1.publish(joint_positions)
            node1.get_logger().info('Bringing in between')

# LEFT FORWARD RIGHT BACKWARD Motion
            for i_val in range(0, 40):
                if float(q[2]) >  0.2 :
                    # y = 952.9  * sp.sin((2 * np.pi / 40) * i_val)
                    y_dot = 952.9 * sp.cos((2 * np.pi / 40) * i_val) * (2 * np.pi / 80)

                    # z = - 952.9  * sp.cos((2 * np.pi / 40) * i_val)
                    z_dot = 952.9 * sp.sin((2 * np.pi / 40) * i_val) * (2 * np.pi / 80)

                # Calculating the end effector velocity Matrix
                    E = sp.Matrix([
                        [0],
                        [y_dot],
                        [z_dot],
                        [0],
                        [0],
                        [0],
                    ])
                    
                # Substituting joint angles to obtian Jacobian and Jacoian Inverse and the Joint angular velocity matrix
                    J0 = J.subs({
                        θ1: q[0], θ2: q[1], θ3: q[2]
                    })

                    qdot = J0.pinv() * E

                    time.sleep(0.08)
                # Obtained the joint angle in each iteration
                    q -= qdot*(40/80)
                    node1.get_logger().info('The hip-joint angle is %f' % q[2])
                    node1.get_logger().info('2nd loop')
                # Obtained the updated value of Transformation Matrix after rotation and extracted the end-effector position for the 
                # Transformation Matrix
                    # T = T1(q[0]) * T2(q[1]) * T3(q[2])
                    # P6 = T[:3, -1]


                    joint_positions = Float64MultiArray(layout=MultiArrayLayout(
                        dim=[MultiArrayDimension(label="joint_positions", size=2, stride=1)],
                        data_offset=0
                    ))        
                    # joint_positions.data = [-float(q[2]), -float(q[1]), -float(q[2]), float(q[2]), float(q[1]), -float(q[2])]   
                    joint_positions.data = [-float(q[2]), -float(q[1]), -float(q[2]), -float(q[2]), -float(q[2]), float(q[2])]
                    joint_position_pub1.publish(joint_positions)

            # joint_positions.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
            # joint_position_pub1.publish(joint_positions)
                
# LEFT FORWARD RIGHT BACKWARD Motion 3rd loop
            for i_val in range(0, 40):
                if float(q[2]) <  0.5 :
                    # y = 952.9  * sp.sin((2 * np.pi / 40) * i_val)
                    y_dot = 952.9 * sp.cos((2 * np.pi / 40) * i_val) * (2 * np.pi / 80)

                    # z = - 952.9  * sp.cos((2 * np.pi / 40) * i_val)
                    z_dot = 952.9 * sp.sin((2 * np.pi / 40) * i_val) * (2 * np.pi / 80)

                # Calculating the end effector velocity Matrix
                    E = sp.Matrix([
                        [0],
                        [y_dot],
                        [z_dot],
                        [0],
                        [0],
                        [0],
                    ])
                    
                # Substituting joint angles to obtian Jacobian and Jacoian Inverse and the Joint angular velocity matrix
                    J0 = J.subs({
                        θ1: q[0], θ2: q[1], θ3: q[2]
                    })

                    qdot = J0.pinv() * E

                    time.sleep(0.08)
                # Obtained the joint angle in each iteration
                    q += qdot*(40/80)
                    node1.get_logger().info('The hip-joint angle is %f' % q[2])
                    node1.get_logger().info('3rd loop')
                # Obtained the updated value of Transformation Matrix after rotation and extracted the end-effector position for the 
                # Transformation Matrix
                    # T = T1(q[0]) * T2(q[1]) * T3(q[2])
                    # P6 = T[:3, -1]


                    joint_positions = Float64MultiArray(layout=MultiArrayLayout(
                        dim=[MultiArrayDimension(label="joint_positions", size=2, stride=1)],
                        data_offset=0
                    ))        
                    # joint_positions.data = [-float(q[2]), -float(q[1]), -float(q[2]), float(q[2]), float(q[1]), -float(q[2])]   
                    joint_positions.data = [float(q[2]), float(q[1]), float(q[2]), float(q[2]), float(q[2]), -float(q[2])]
                    joint_position_pub1.publish(joint_positions)

            joint_positions.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
            joint_position_pub1.publish(joint_positions)
            node1.get_logger().info('Bringing in between 2')

# # LEFT BACKWARD RIGHT FORWARD Motion
            # for i_val in range(0, 40):
            #     if float(q[2]) > 0.5 :
            #         while float(q[2]) >  -0.5 :
            #             # y = 952.9  * sp.sin((2 * np.pi / 40) * i_val)
            #             y_dot = 952.9 * sp.cos((2 * np.pi / 40) * i_val) * (2 * np.pi / 80)

            #             # z = - 952.9  * sp.cos((2 * np.pi / 40) * i_val)
            #             z_dot = 952.9 * sp.sin((2 * np.pi / 40) * i_val) * (2 * np.pi / 80)

            #         # Calculating the end effector velocity Matrix
            #             E = sp.Matrix([
            #                 [0],
            #                 [y_dot],
            #                 [z_dot],
            #                 [0],
            #                 [0],
            #                 [0],
            #             ])
                        
            #         # Substituting joint angles to obtian Jacobian and Jacoian Inverse and the Joint angular velocity matrix
            #             J0 = J.subs({
            #                 θ1: q[0], θ2: q[1], θ3: q[2]
            #             })

            #             qdot = J0.pinv() * E

            #             time.sleep(0.08)
            #         # Obtained the joint angle in each iteration
            #             q += qdot*(40/80)
            #             node1.get_logger().info('The hip-joint angle is %f' % q[2])
            #             node1.get_logger().info('Entered fourth loop')
            #         # Obtained the updated value of Transformation Matrix after rotation and extracted the end-effector position for the 
            #         # Transformation Matrix
            #             # T = T1(q[0]) * T2(q[1]) * T3(q[2])
            #             # P6 = T[:3, -1]

            #             joint_positions = Float64MultiArray(layout=MultiArrayLayout(
            #                 dim=[MultiArrayDimension(label="joint_positions", size=2, stride=1)],
            #                 data_offset=0
            #             ))        
            #             # joint_positions.data = [float(q[2]), float(q[2]), float(q[2]), float(q[2]), float(q[2]), float(q[2])]   
            #             joint_positions.data = [-float(q[2]), -float(q[1]), -float(q[2]), -float(q[2]), -float(q[2]), float(q[2])]
            #             joint_position_pub1.publish(joint_positions)
        
            # # joint_positions.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
            # # joint_position_pub1.publish(joint_positions)
            # node1.get_logger().info('Bringing in between')

        node1.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()