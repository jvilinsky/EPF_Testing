#!/usr/bin/env python3

"""
A collision avoidance node that detects potential collision based on onboard
    multirange sensor measurements, provides nearest obstacle point coordinates,
    and provide collision avoidance maneuvers.

    2023 = Shyam Rauniyar (IASRL)
"""
import rclpy
from rclpy.node import Node

from crazyflie_interfaces.srv import CA
from crazyflie_interfaces.msg import CollDetect, WayPoint, Position, Hover, InFlight
from geometry_msgs.msg import PoseStamped
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from functools import partial
import numpy as np
import math

class DetectAndAvoid(Node):
    def __init__(self):
        super().__init__('detect_avoid',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,)
        # self.declare_parameter('ca_threshold1', 0.1)
        # self.declare_parameter('ca_threshold2', 0.2)
        # self.declare_parameter('avoidance_vel', 0.2)
        # self.declare_parameter('robot_prefix', '/cf2')

        self.ca_threshold1  = self.get_parameter('ca_threshold1').value
        self.ca_threshold2  = self.get_parameter('ca_threshold2').value
        self.avoidance_vel  = self.get_parameter('avoidance_vel').value
        self.ca_on  = self.get_parameter('ca_on').value
        robot_prefix  = self.get_parameter('robot_prefix').value
        self.uris = self.get_parameter('uris').value
        self.uav_uri = robot_prefix
        #self.get_logger().info('uris %s' % (self.uris))
        
        # Publishers
        self.publisher_collision = self.create_publisher(
            CollDetect, 
            robot_prefix + '/coll_detect', 
            10)

        # Subscriptions
        self.pose_sub = self.create_subscription(
            PoseStamped,
            robot_prefix + '/pose',
            self.pose_callback,
            10)
        qos_profile = QoSProfile(reliability =QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                deadline = Duration(seconds=0, nanoseconds=1e9/100.0))
        self.all_poses_sub = self.create_subscription(
            NamedPoseArray,
            '/poses',
            self.all_poses_callback,
            qos_profile
        )
        # self.scan_sub = self.create_subscription(
        #     LaserScan,
        #     robot_prefix + '/scan',
        #     self.scan_callback,
        #     10)
        self.scan_sub = self.create_subscription(
            Odometry,
            robot_prefix + '/odom',
            self.odom_callback,
            10)
        self.waypoint_sub = self.create_subscription(
            WayPoint,
            robot_prefix + '/waypoint',
            self.waypoint_callback,
            10)
        self.inflight_sub = self.create_subscription(
            InFlight,
            robot_prefix + '/inflight',
            self.inflight_callback,
            10)

        # Clients
        
        
        # Services
        self.ca_choice = self.epf_ca
        self.create_service(
            CA,
            robot_prefix + "/ca", 
            partial(self.ca_choice))

        # Initializations
        inf = float("inf")
        self.nearest_inertial = [inf] * 3
        self.nearest_relative = [inf] * 3
        self.nearest_dist = inf
        self.m1, self.c1 = self.vel_eq(self.avoidance_vel, 0.0, self.ca_threshold2, self.ca_threshold1) # Axial Slow down
        self.m2, self.c2 = self.vel_eq(0.0, self.avoidance_vel, self.ca_threshold2, self.ca_threshold1) # Lateral Speed up
        self.coll_check = False

        self.get_logger().info(f"Detect and Avoid set for {robot_prefix}"+
                               f" with ca_th1 = {self.ca_threshold1} m, ca_th2 = {self.ca_threshold2} m")
        
        # Initialize Crazyflie logging
        self.poses_dict = {}
        for key in self.uris: # Create a unique dictionary entry for each crazyflie
            self.poses_dict[key] = {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "roll": 0.0
            }

    def pose_callback(self, msg:PoseStamped):
        self.msg_pose = msg
        self.cf_position = msg.pose.position
        self.cf_inertial = [self.cf_position.x, self.cf_position.y, self.cf_position.z]

    def pose_log(self,uri,position):
        self.poses[uri] = position

    def all_poses_callback(self, msg:NamedPoseArray):
        #self.get_logger().info('msg %s' % (msg))

        self.poses = msg.poses
        for pose in self.poses:
            name = pose.name
            x = pose.pose.position.x
            y = pose.pose.position.y
            z = pose.pose.position.z
            quat = pose.pose.orientation
            pitch,yaw,roll=self.quaternion_to_euler(quat.x,quat.y,quat.z,quat.w)
            self.poses_dict[name]["x"] = x
            self.poses_dict[name]["y"] = y
            self.poses_dict[name]["z"] = z
            self.poses_dict[name]["pitch"] = pitch
            self.poses_dict[name]["yaw"] = yaw
            self.poses_dict[name]["roll"] = roll

        uri = self.uav_uri
        for pose in self.poses:
            uri2 = pose.name
            if uri2 != uri:
                distance = np.sqrt((self.poses_dict[uri2]["x"]-self.poses_dict[uri]["x"])**2+\
                                   (self.poses_dict[uri2]["y"]-self.poses_dict[uri]["y"])**2+\
                                    (self.poses_dict[uri2]["z"]-self.poses_dict[uri]["z"])**2)
                
                # self.get_logger().info(f"uri2_x={self.poses_dict[uri2]['x']},uri_x={self.poses_dict[uri]['x']}, "
                #                    f"uri2_y={self.poses_dict[uri2]['y']},uri_y={self.poses_dict[uri]['y']}, "
                #                    f"uri2_z={self.poses_dict[uri2]['z']},uri_z={self.poses_dict[uri]['z']}")
                
                # self.get_logger().info(f"{uri} distance = {distance}")
                
                if distance <= self.ca_threshold2:
                    self.coll_check = True
                    distance_coll = distance
            
        msg_collision = CollDetect()
        # msg_collision.nearest = distance_coll
        if self.coll_check:
            msg_collision.collision = True
        else:
            msg_collision.collision = False
        
        

        #self.get_logger().info('poses = %s' % (self.poses_dict))


    def waypoint_callback(self, msg:WayPoint):
        self.msg_wp = msg
        self.wp = msg.pose
        self.wp_inertial = [self.wp.x, self.wp.y, self.wp.z]
        self.wp_relative_array = self.inertial2relative(self.wp_inertial)
        self.wp_dist = self.dist2pos(self.wp_relative_array)

    def odom_callback(self, msg:Odometry):
        self.msg_odom = msg

    def inflight_callback(self, msg:InFlight):
        self.msg_inflight = msg

    def scan_callback(self, msg:LaserScan):
        self.msg_scan = msg
        ranges = list(msg.ranges)
        
        if self.msg_inflight.inflight and self.ca_on:
            nearest_range = min(ranges)
            self.nearest_range = max(nearest_range, self.msg_scan.range_min) # Avoid zero division error
            self.nearest_range_index = ranges.index(nearest_range) #determine which sensor is being used
            self.nearest_range_relative = self.range_relative(self.nearest_range, self.nearest_range_index)
            self.nearest_range_inertial = self.relative2inertial(self.nearest_range_relative)
                
            self.nearest_relative = self.inertial2relative(self.nearest_inertial)
            self.nearest_dist = self.dist2pos(self.nearest_relative) # Consider previous nearest
            self.nearest_dist = self.nearest_range # Only consider sensed nearest
            if self.nearest_range <= self.nearest_dist:
                # self.get_logger().warn(f"Nearest range: [{self.nearest_range}, {self.nearest_range_index}].")
                self.nearest_relative = self.nearest_range_relative
                self.nearest_inertial = self.nearest_range_inertial
                self.nearest_dist = self.nearest_range

            self.coll_check = self.nearest_dist <= self.ca_threshold2 # Collision check via nearest obs pt.

            msg_collision = CollDetect()
            msg_collision.nearest = self.nearest_relative
            if self.coll_check:
                self.get_logger().warn(f"Nearest collision at {self.nearest_dist}.")
                self.get_logger().warn(f"Obs inertially at: " +
                                       f"obs_x = {self.nearest_inertial[0]}, " +
                                       f"obs_y = {self.nearest_inertial[1]}, ")
                self.get_logger().info(f"CF position at: "+
                                       f"cf_x = {self.cf_inertial[0]}, " + 
                                       f"cf_y = {self.cf_inertial[1]}, " +
                                       f"cf_z = {self.cf_inertial[2]}")
                msg_collision.collision = True
            else:
                msg_collision.collision = False
            self.publisher_collision.publish(msg_collision)

    def epf_ca(self, request, response):
        # uri = self.uav_uri
        
        # ka = 0.3 
        # kr = 0.01 
        # ng = 2
        # do = self.ca_threshold2
        # alpha = 0.5
        # gam = np.rad2deg(45)

        # R_2 = np.array([[np.cos(gam),-np.sin(gam),0],
        #                 [np.sin(gam),np.cos(gam),0],
        #                 [0,0,1]])
        # R_3 = np.array([[np.cos(gam),0,np.sin(gam)],
        #                 [0,1,0],
        #                 [-np.sin(gam),0,np.cos(gam)]])
        # R_2_T = R_2.T
        # R_3_T = R_3.T

        # #distance from uav to goal:
        # d_Nr_Nrg = np.sqrt((self.wp.x-self.poses_dict[uri]["x"])**2 + \
        #                            (self.wp.y-self.poses_dict[uri]["y"])**2 + \
        #                             (self.wp.z-self.poses_dict[uri]["z"])**2)
        # #direction from uav to goal:
        # C = np.array([self.wp.x,self.wp.y,self.wp.z]) - \
        #     np.array([self.poses_dict[uri]["x"],self.poses_dict[uri]["y"],self.poses_dict[uri]["z"]])
        # dir_Nr_Nrg = C/np.linalg.norm(C)
        # #attractive potential field
        # fa = ka*d_Nr_Nrg*dir_Nr_Nrg

        # for pose in self.poses:
        #     uri2 = pose.name
        #     if uri2 != uri:
        #         #distance from uav to object:
        #         distance = np.sqrt((self.poses_dict[uri2]["x"]-self.poses_dict[uri]["x"])**2+\
        #                            (self.poses_dict[uri2]["y"]-self.poses_dict[uri]["y"])**2+\
        #                             (self.poses_dict[uri2]["z"]-self.poses_dict[uri]["z"])**2)
                
        #         if distance <= self.ca_threshold2:
        #             d_Nr_Nro = distance
        #             #direction from uav to object:
        #             C = np.array([self.poses_dict[uri2]["x"],self.poses_dict[uri2]["y"],self.poses_dict[uri2]["z"]]) - \
        #                 np.array([self.poses_dict[uri]["x"],self.poses_dict[uri]["y"],self.poses_dict[uri]["z"]])
        #             dir_Nr_Nro = C/np.linalg.norm(C)
        #             #uav position:
        #             Nr = np.array([self.poses_dict[uri]["x"],self.poses_dict[uri]["y"],self.poses_dict[uri]["z"]])
        #             #object position:
        #             Nro = np.array([self.poses_dict[uri2]["x"],self.poses_dict[uri2]["y"],self.poses_dict[uri2]["z"]])
        #             #creating e frame:
        #             vx = self.msg_odom.twist.twist.linear.x
        #             e1 = np.array([vx,0])
        #             P_h = np.arccos(np.dot(e1,))

        #             q_h = R_h*(Nro-Nr)
        #             q_v = R_v*(Nro-Nr)
        #             q_p = np.array([[alpha*q_h1],[alpha*q_h2],[(1-alpha)*q_v3]])
        #             q_hat = q_p/np.linalg.norm(q_p)
        #             fr_e = -kr*((d_Nr_Nrg**ng)/(d_Nr_Nro**2))*((1/d_Nr_Nro)-(1/do))*q_hat
        self.get_logger().info(f"EPF Engaged")
        

    def repel_ca(self, request, response):
        old_hover = request.old_hover
        new_hover = old_hover
        new_hover = self.set_hover_z(new_hover)
        vel_avoid = self.avoidance_vel
        new_hover.vx = 0.0
        new_hover.vy = 0.0
        new_hover.yaw_rate = 0.0

        if self.nearest_range_index == 0: # back
            new_hover.vx = vel_avoid
        if self.nearest_range_index == 1: # right
            new_hover.vy = vel_avoid
        if self.nearest_range_index == 2: # front
            new_hover.vx = -vel_avoid
        if self.nearest_range_index == 3: # left
            new_hover.vy = -vel_avoid

        response.new_hover = new_hover

        self.get_logger().info("Repel CA engaged.")
        return response

    def repel_ca2(self, request, response):
        old_hover = request.old_hover
        new_hover = old_hover
        new_hover = self.set_hover_z(new_hover)
        new_hover.vx = 0.0
        new_hover.vy = 0.0
        new_hover.yaw_rate = 0.0

        x = self.nearest_relative[0] # x axis, - is back, + is front
        y = self.nearest_relative[1] # y axis, - is right, + is left
        new_hover.vx = - self.avoidance_vel * float(np.sign(x))
        new_hover.vy = - self.avoidance_vel * float(np.sign(y))

        response.new_hover = new_hover

        self.get_logger().info("Repel CA2 engaged.")
        return response

    def smooth_ca(self, request, response): # Best performance till now with ca1 = 0.2 and ca2 = 0.4
        old_hover = request.old_hover
        new_hover = old_hover
        new_hover = self.set_hover_z(new_hover)

        axial_speed = self.smooth_vel_change(self.nearest_dist, self.m1, self.c1)
        lateral_speed = self.smooth_vel_change(self.nearest_dist, self.m2, self.c2)
        if self.nearest_range_index == 0: # back
            new_hover.vx = -axial_speed
            new_hover.vy = -lateral_speed
        if self.nearest_range_index == 1: # right
            new_hover.vx = lateral_speed
            new_hover.vy = -axial_speed
        if self.nearest_range_index == 2: # front
            new_hover.vx = axial_speed
            new_hover.vy = lateral_speed
        if self.nearest_range_index == 3: # left
            new_hover.vx = -lateral_speed
            new_hover.vy = axial_speed

        response.new_hover = new_hover

        self.get_logger().info("Smooth CA engaged.")
        return response

    def smooth_ca2(self, request, response):
        old_hover = request.old_hover
        new_hover = old_hover
        new_hover = self.set_hover_z(new_hover)

        axial_speed = self.smooth_vel_change(self.nearest_dist, self.m1, self.c1)
        lateral_speed = self.smooth_vel_change(self.nearest_dist, self.m2, self.c2)
        x = self.nearest_relative[0] # x axis, - is back, + is front
        y = self.nearest_relative[1] # y axis, - is right, + is left
        theta = math.atan2(y, x)

        # Method 4 - makes circles around the obstacle, performs well.
        new_hover.vx = self.avoidance_vel * math.cos(theta - math.pi/2)
        new_hover.vy = self.avoidance_vel * math.sin(theta - math.pi/2)

        response.new_hover = new_hover

        self.get_logger().info("Smooth CA2 engaged.")
        return response
    
    def apf_ca(self, request, response): # best settings in comments, worked well when nearest not stored
        old_hover = request.old_hover
        new_hover = old_hover
        new_hover = self.set_hover_z(new_hover)
        x = self.nearest_relative[0] # x axis, - is back, + is front
        y = self.nearest_relative[1] # y axis, - is right, + is left
        rho_h = math.atan2(abs(y), x)

        ka = 0.3 # 0.3
        kr = 0.01 # 0.01
        ng = 2
        n = 2
        do = self.ca_threshold2 # with ca1 = 0.2 and ca2 = 0.4
        dr = max(min(self.nearest_dist, do), self.ca_threshold1)
        da = self.wp_dist
        unit_direction_nearest = np.array(self.nearest_relative) / dr
        unit_direction_wp = np.array(self.wp_relative_array) / da
        
        gamma = math.pi / 4
        alpha = 1

        R_1 = np.array([[np.cos(gamma),-np.sin(gamma), 0],
                        [np.sin(gamma), np.cos(gamma), 0],
                        [0            , 0            , 1]])
        
        R_2 = np.array([[ np.cos(gamma), 0, np.sin(gamma)],
                        [ 0            , 1,             0],
                        [-np.sin(gamma), 0, np.cos(gamma)]])

        if rho_h >= 0:
            R_h = R_1
            R_v = np.transpose(R_2)
        else:
            R_h = np.transpose(R_1)
            R_v = R_2

        p_h = np.matmul(np.array(self.nearest_relative), R_h)
        p_v = np.matmul(np.array(self.nearest_relative), R_v)
        p = np.array([alpha * p_h[0], 
                      alpha * p_h[1], 
                      (1-alpha) * p_v[2]])
        unit_p = p / max(min(self.dist2pos(p), self.nearest_dist), -self.nearest_dist)
        
        fa = ka * da * unit_direction_wp
        fr_o = -(1/2) * kr * (da**ng / dr**2) * n * (((1/dr) - (1/do))**(n-1)) * unit_p
        fr_g = -(1/2) * kr * ng * da**(ng-1)  * (((1/dr) - (1/do))**n) * unit_direction_wp
        fr = fr_o + fr_g
        fc = fa + fr

        new_hover.vx = max(min(fc[0], self.avoidance_vel), -self.avoidance_vel)
        new_hover.vy = max(min(fc[1], self.avoidance_vel), -self.avoidance_vel)

        response.new_hover = new_hover

        self.get_logger().info(f"fc_x = {fc[0]}, fc_y = {fc[1]}")
        self.get_logger().info("APF CA engaged.")
        return response
    
    def set_hover_z(self, msg_hover: Hover):
        try:
            msg_hover.z_distance = self.msg_wp.pose.z
        except Exception as e:
            msg_hover.z_distance = self.cf_position.z

        return msg_hover

    def range_relative(self, rangex, index):
        if index == 0: # back
            range_relative = [-rangex, 0.0, 0.0]
        elif index == 1: # right
            range_relative = [0.0, -rangex, 0.0]
        elif index == 2: # front
            range_relative = [rangex, 0.0, 0.0]
        elif index == 3: # left
            range_relative = [0.0, rangex, 0.0]

        return range_relative
    
    def compare_nearest(self, pos1, pos2): # Overwrites the nearest obs in memory
        pos1_rel_array = self.inertial2relative(pos1)
        pos2_rel_array = self.inertial2relative(pos2)
        pos1_reldist = self.dist2pos(pos1_rel_array)
        pos2_reldist = self.dist2pos(pos2_rel_array)

        if pos1_reldist < pos2_reldist:
            return pos1, pos1_rel_array, pos1_reldist
        else:
            return pos2, pos2_rel_array, pos1_reldist

    def dist2pos(self, pos):
        pos_reldist = np.linalg.norm(pos)

        return float(pos_reldist)
    
    def smooth_vel_change(self, dist, m, c):
        v = m * dist + c

        return v
    
    def vel_eq(self, v1, v2, x1, x2):
        m = (v2 - v1) / (x2 - x1)
        c = v1 - m * x1

        return m, c
    
    def inertial2relative(self, pos): # assuming yaw is at zero
        pos_rel_array = np.array([pos[i] - self.cf_inertial[i] for i in range(len(self.cf_inertial))])
        pos_rel = Position()
        pos_rel.x = pos_rel_array[0]
        pos_rel.y = pos_rel_array[1]
        pos_rel.z = pos_rel_array[2]
        pos_rel.yaw = math.atan2(pos_rel.y, pos_rel.z)

        return list(pos_rel_array)

    def relative2inertial(self, pos_rel): # assuming yaw is at zero
        pos_inertial_array = np.array([pos_rel[i] + self.cf_inertial[i] for i in range(len(self.cf_inertial))])
        pos_inertial = Position()
        pos_inertial.x = pos_inertial_array[0]
        pos_inertial.y = pos_inertial_array[1]
        pos_inertial.z = pos_inertial_array[2]
        pos_inertial.yaw = math.atan2(pos_inertial.y, pos_inertial.z)

        return list(pos_inertial_array)
    
    def quaternion_to_euler(self,x, y, z, w):

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if np.abs(sinp) >= 1:
            pitch = np.sign(sinp) * np.pi / 2  # Use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return pitch, yaw, roll





def main(args=None):
    rclpy.init(args=args)

    detect_avoid = DetectAndAvoid()

    rclpy.spin(detect_avoid)

    detect_avoid.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
