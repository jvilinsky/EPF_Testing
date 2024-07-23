#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from crazyflie_interfaces.msg import SCCAwp
from crazyflie_interfaces.srv import Shape

class SwarmControl(Node):
    def __init__(self):    
        super().__init__('swarm_control',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        # Turn ROS parameters into a dictionary
        self._ros_parameters = self._param_to_dict(self._parameters)
        print(self._ros_parameters)
        robot_prefix = self._ros_parameters["robot_prefix"]
        self.init_pos = self._ros_parameters["initial_position"]

        # Publishers
        self.publisher_SCCAwp = self.create_publisher(
            SCCAwp,
            robot_prefix + '/SCCA_wp',
            10
        )

        #Services
        self.create_service(
            Shape,
            "shape_select",
            self.shape_selector_callback
        )

        # Initialization
        # self.init_pos = list()
        # if init_pos:
        #     for key in init_pos:
        #         key_fix = int(key)
        #         print("SCCA_Key =",key)
        #         ip = init_pos[key_fix]
        #         self.init_pos.append(ip)

        self.x_pos = self.init_pos[0]
        self.y_pos = self.init_pos[1]
        self.z_pos = 0.5
        self.yaw = 0
        self.abs_rel = 0

        self.shape_selected = False

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.msg_SCCA = SCCAwp()
        # if self.shape_selected == False:
        #     self.msg_SCCA.x_pos = float(self.x_pos)
        #     self.msg_SCCA.y_pos = float(self.y_pos)
        #     self.msg_SCCA.z_pos = float(self.z_pos)
        #     self.msg_SCCA.yaw = float(0)
        #     self.msg_SCCA.abs_rel = int(0)    

        #     self.publisher_SCCAwp.publish(self.msg_SCCA)
        
        # elif self.shape_selected == True:
        #     self.msg_SCCA.x_pos = float(self.x_pos)
        #     self.msg_SCCA.y_pos = float(self.y_pos)
        #     self.msg_SCCA.z_pos = float(self.z_pos)
        #     self.msg_SCCA.yaw = float(self.yaw)
        #     self.msg_SCCA.abs_rel = int(self.abs_rel)

        #     self.publisher_SCCAwp.publish(self.msg_SCCA)   

        self.msg_SCCA.x_pos = float(self.x_pos)
        self.msg_SCCA.y_pos = float(self.y_pos)
        self.msg_SCCA.z_pos = float(self.z_pos)
        self.msg_SCCA.yaw = float(self.yaw)
        self.msg_SCCA.abs_rel = int(self.abs_rel)

        self.publisher_SCCAwp.publish(self.msg_SCCA)  

    def _param_to_dict(self, param_ros):
        """
        Turn ROS 2 parameters from the node into a dict
        """
        tree = {}
        for item in param_ros:
            t = tree
            for part in item.split('.'):
                if part == item.split('.')[-1]:
                    t = t.setdefault(part, param_ros[item].value)
                else:
                    t = t.setdefault(part, {})
        return tree

    def shape_selector_callback(self,request,response):
        self.shape = request.shape
        robot_prefix = self._ros_parameters["robot_prefix"]
        if self.shape == "Takeoff(FiveCF)":
            self.get_logger().info(f"Requesting shape {self.shape}...")
            self.shape_selected=True 
            if robot_prefix == 'cf5':
                self.x_pos = 0; self.y_pos = -0.5; self.z_pos = 1.3208; self.yaw = 0; self.abs_rel = 0
            elif robot_prefix == 'cf6':
                self.x_pos = -0.5; self.y_pos = 0; self.z_pos = 0.6604; self.yaw = 0; self.abs_rel = 0
            elif robot_prefix == 'cf7':
                self.x_pos = 0; self.y_pos = 0; self.z_pos = 0.6604; self.yaw = 0; self.abs_rel = 0
            elif robot_prefix == 'cf8':
                self.x_pos = 0.5; self.y_pos = 0; self.z_pos = 0.6604; self.yaw = 0; self.abs_rel = 0
            elif robot_prefix == 'cf9':
                self.x_pos = 0; self.y_pos = 0.5; self.z_pos = 1.3208; self.yaw = 0; self.abs_rel = 0
        elif self.shape == "U(FiveCF)": 
            self.get_logger().info(f"Requesting shape {self.shape}...")
            self.shape_selected=True
            if robot_prefix == 'cf5':
                self.x_pos = -0.4572; self.y_pos = 0.4; self.z_pos = 1.3208; self.yaw = 0; self.abs_rel = 0
            elif robot_prefix == 'cf6':
                self.x_pos = -0.4572; self.y_pos = 0.4; self.z_pos = 0.6604; self.yaw = 0; self.abs_rel = 0
            elif robot_prefix == 'cf7':
                self.x_pos = 0; self.y_pos = 0.4; self.z_pos = 0.6604; self.yaw = 0; self.abs_rel = 0
            elif robot_prefix == 'cf8':
                self.x_pos = 0.4572; self.y_pos = 0.4; self.z_pos = 0.6604; self.yaw = 0; self.abs_rel = 0
            elif robot_prefix == 'cf9':
                self.x_pos = 0.4572; self.y_pos = 0.4; self.z_pos = 1.3208; self.yaw = 0; self.abs_rel = 0
        elif self.shape == "Transition(FiveCF)": 
            self.get_logger().info(f"Requesting shape {self.shape}...")
            self.shape_selected=True
            if robot_prefix == 'cf5':
                self.x_pos = 0.4572; self.y_pos = 1.0; self.z_pos = 1.3208; self.yaw = 0; self.abs_rel = 0
            elif robot_prefix == 'cf6':
                self.x_pos = 0; self.y_pos = 0.4; self.z_pos = 1.3208; self.yaw = 0; self.abs_rel = 0
            elif robot_prefix == 'cf7':
                self.x_pos = -0.4572; self.y_pos = 1.0; self.z_pos = 0.9906; self.yaw = 0; self.abs_rel = 0
            elif robot_prefix == 'cf8':
                self.x_pos = -0.4572; self.y_pos = -0.2; self.z_pos = 0.6604; self.yaw = 0; self.abs_rel = 0
            elif robot_prefix == 'cf9':
                self.x_pos = 0.4572; self.y_pos = -0.2; self.z_pos = 0.9906; self.yaw = 0; self.abs_rel = 0
        elif self.shape == "C(FiveCF)": 
            self.get_logger().info(f"Requesting shape {self.shape}...")
            self.shape_selected=True
            if robot_prefix == 'cf5':
                self.x_pos = 0.23; self.y_pos = 0.4; self.z_pos = 1.3208; self.yaw = 0; self.abs_rel = 0
            elif robot_prefix == 'cf6':
                self.x_pos = -0.4572; self.y_pos = 0.4; self.z_pos = 1.3208; self.yaw = 0; self.abs_rel = 0
            elif robot_prefix == 'cf7':
                self.x_pos = -0.4572; self.y_pos = 0.4; self.z_pos = 0.9906; self.yaw = 0; self.abs_rel = 0
            elif robot_prefix == 'cf8':
                self.x_pos = -0.4572; self.y_pos = 0.4; self.z_pos = 0.6604; self.yaw = 0; self.abs_rel = 0
            elif robot_prefix == 'cf9':
                self.x_pos = 0.23; self.y_pos = 0.4; self.z_pos = 0.6604; self.yaw = 0; self.abs_rel = 0
        elif self.shape == "C(Alternative)(FiveCF)": 
            self.get_logger().info(f"Requesting shape {self.shape}...")
            self.shape_selected=True
            if robot_prefix == 'cf5':
                self.x_pos = -0.4572; self.y_pos = 0.4; self.z_pos = 1.3208; self.yaw = 0; self.abs_rel = 0
            elif robot_prefix == 'cf6':
                self.x_pos = -0.4572; self.y_pos = 0.4; self.z_pos = 0.6604; self.yaw = 0; self.abs_rel = 0
            elif robot_prefix == 'cf7':
                self.x_pos = -0.4572; self.y_pos = 0.4; self.z_pos = 0.9906; self.yaw = 0; self.abs_rel = 0
            elif robot_prefix == 'cf8':
                self.x_pos = 0.23; self.y_pos = 0.4; self.z_pos = 0.6604; self.yaw = 0; self.abs_rel = 0
            elif robot_prefix == 'cf9':
                self.x_pos = 0.23; self.y_pos = 0.4; self.z_pos = 1.3208; self.yaw = 0; self.abs_rel = 0
        elif self.shape == "U": 
            self.get_logger().info(f"Requesting shape {self.shape}...")
            self.shape_selected=True
            if robot_prefix == 'cf5':
                self.x_pos = 0; self.y_pos = 0.4572; self.z_pos = 1.1557; self.yaw = 0; self.abs_rel = 0
            elif robot_prefix == 'cf6':
                self.x_pos = 0; self.y_pos = 0.4; self.z_pos = 0.6604; self.yaw = 0; self.abs_rel = 0
            elif robot_prefix == 'cf7':
                self.x_pos = -0.4572; self.y_pos = 0.4; self.z_pos = 1.1557; self.yaw = 0; self.abs_rel = 0
        elif self.shape == "C": 
            self.get_logger().info(f"Requesting shape {self.shape}...")
            self.shape_selected=True
            if robot_prefix == 'cf5':
                self.x_pos = 0; self.y_pos = 0.4; self.z_pos = 1.3208; self.yaw = 0; self.abs_rel = 0
            elif robot_prefix == 'cf6':
                self.x_pos = 0; self.y_pos = 0.4; self.z_pos = 0.6604; self.yaw = 0; self.abs_rel = 0
            elif robot_prefix == 'cf7':
                self.x_pos = -0.4572; self.y_pos = 0.4; self.z_pos = 0.9906; self.yaw = 0; self.abs_rel = 0
        elif self.shape == "Landing": 
            self.get_logger().info(f"Requesting shape {self.shape}...")
            self.shape_selected=True
            if robot_prefix == 'cf5':
                self.x_pos = 0; self.y_pos = 0; self.z_pos = 0.5; self.yaw = 0; self.abs_rel = 0
            elif robot_prefix == 'cf6':
                self.x_pos = 0; self.y_pos = 0.5; self.z_pos = 0.5; self.yaw = 0; self.abs_rel = 0
            elif robot_prefix == 'cf7':
                self.x_pos = 0; self.y_pos = 1.0; self.z_pos = 0.5; self.yaw = 0; self.abs_rel = 0
        elif self.shape == 'line':
            self.get_logger().info(f"Requesting shape {self.shape}...")
            self.shape_selected=True
            if robot_prefix == 'cf8':
                self.x_pos = 0; self.y_pos = 1.5; self.z_pos = 0.5; self.yaw = 0; self.abs_rel = 0
            elif robot_prefix == 'cf9':
                self.x_pos = 0; self.y_pos = -0.5; self.z_pos = 0.5; self.yaw = 0; self.abs_rel = 0
        #Waypoint sequence for testing
        elif self.shape == 'Seq_1':
            self.get_logger().info(f"Requesting shape {self.shape}...")
            self.shape_selected=True
            if robot_prefix == 'cf6':
                self.x_pos = 0.4572; self.y_pos = 0.8; self.z_pos = 0.5; self.yaw = 0; self.abs_rel = 0
        elif self.shape == 'Seq_2':
            self.get_logger().info(f"Requesting shape {self.shape}...")
            self.shape_selected=True
            if robot_prefix == 'cf6':
                self.x_pos = -0.4572; self.y_pos = 0.8; self.z_pos = 1.0; self.yaw = 0; self.abs_rel = 0
        elif self.shape == 'Seq_3':
            self.get_logger().info(f"Requesting shape {self.shape}...")
            self.shape_selected=True
            if robot_prefix == 'cf6':
                self.x_pos = -0.4572; self.y_pos = -0.4; self.z_pos = 0.5; self.yaw = 0; self.abs_rel = 0
        elif self.shape == 'Seq_4':
            self.get_logger().info(f"Requesting shape {self.shape}...")
            self.shape_selected=True
            if robot_prefix == 'cf6':
                self.x_pos = 0.4572; self.y_pos = 0.4; self.z_pos = 0.5; self.yaw = 0; self.abs_rel = 0
        elif self.shape == 'Seq_5':
            self.get_logger().info(f"Requesting shape {self.shape}...")
            self.shape_selected=True
            if robot_prefix == 'cf6':
                self.x_pos = 0.0; self.y_pos = 0.0; self.z_pos = 0.5; self.yaw = 0; self.abs_rel = 0
        #Sequence for collision avoidance testing
        elif self.shape == 'CA_seq_1':
            self.get_logger().info(f"Requesting shape {self.shape}...")
            self.shape_selected=True
            if robot_prefix == 'cf5':
                self.x_pos = 0.0; self.y_pos = 0.3; self.z_pos = 0.5; self.yaw = 0; self.abs_rel = 0
            if robot_prefix == 'cf6':
                self.x_pos = 0.0; self.y_pos = -0.3; self.z_pos = 0.5; self.yaw = 0; self.abs_rel = 0
        elif self.shape == 'CA_seq_2':
            self.get_logger().info(f"Requesting shape {self.shape}...")
            self.shape_selected=True
            if robot_prefix == 'cf5':
                self.x_pos = 0.0; self.y_pos = 0.5; self.z_pos = 0.5; self.yaw = 0; self.abs_rel = 0
            if robot_prefix == 'cf6':
                self.x_pos = 0.0; self.y_pos = -0.5; self.z_pos = 0.5; self.yaw = 0; self.abs_rel = 0
        #Template for swarm control
        elif self.shape == "Template (Dont Use)": 
            self.get_logger().info(f"Requesting shape {self.shape}...")
            self.shape_selected=True
            if robot_prefix == 'cf5':
                self.x_pos = 0; self.y_pos = 0; self.z_pos = 0; self.yaw = 0; self.abs_rel = 0
            elif robot_prefix == 'cf6':
                self.x_pos = 0; self.y_pos = 0; self.z_pos = 0; self.yaw = 0; self.abs_rel = 0
            elif robot_prefix == 'cf7':
                self.x_pos = 0; self.y_pos = 0; self.z_pos = 0; self.yaw = 0; self.abs_rel = 0
            elif robot_prefix == 'cf8':
                self.x_pos = 0; self.y_pos = 0; self.z_pos = 0; self.yaw = 0; self.abs_rel = 0
            elif robot_prefix == 'cf9':
                self.x_pos = 0; self.y_pos = 0; self.z_pos = 0; self.yaw = 0; self.abs_rel = 0
        else:
            self.get_logger().info("======>Requested shape does not exist please select valid shape<======")

        return response



def main(args=None):
    rclpy.init(args=args)

    swarm_control = SwarmControl()

    rclpy.spin(swarm_control)

    swarm_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

