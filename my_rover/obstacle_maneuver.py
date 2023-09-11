import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist



class ObstacleAvoidingBot(Node):
    def __init__(self):
        super().__init__('Go_to_position_node') ## name of the node
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 40)
        self.subscription=self.create_subscription(LaserScan,'/scan',self.get_scan_values,40)
        #periodic call
        timer_period = 0.2;self.timer = self.create_timer(timer_period, self.send_cmd_vel)
        ## Initializing Global values for linear velocity
        self.linear_vel = 0.22 
        ## dividing the area of laser scan (720/120)
        self.regions={'front_right':0.0,'front':0.0,'front_left':0.0, 'rear_left':0.0, 'back':0.0, 'rear_right':0.0}
        ## creating a message object to fit new velocities and publish them
        self.velocity=Twist()


    ## Subscriber Callback function 
    def get_scan_values(self,scan_data):
        ## We have 360 data points and we are dividing them in 3 regions
        ## we say if there is something in the region get the smallest value
        self.regions = {
           'front_right': min(scan_data.ranges[0:60]),
            'front': min(scan_data.ranges[60:120]),
            'front_left': min(scan_data.ranges[120:180]),
            'rear_left': min(scan_data.ranges[180:240]),
            'back': min(scan_data.ranges[240:300]),
            'rear_right': min(scan_data.ranges[300:360])
        }        
        print(self.regions['front_right']," / ",self.regions['front']," / ",self.regions['front_left'], " / ",self.regions['rear_left'], " / ",self.regions['back'], " / ",self.regions['rear_right'])

  
    ## Callback Publisher of velocities called every 0.2 seconds
    def send_cmd_vel(self):
        ## angular and linear velocities are set into object self.velcity
        ## setting the linear velocity to be fixed and robot will keep on moving
        self.velocity.linear.x=self.linear_vel
        ## cases to make the robot change its angular velocity
    
        front_right_value = self.regions['front_right']
        front_value = self.regions['front']
        front_left_value = self.regions['front_left']
        rear_left_value = self.regions['rear_left']
        back_value = self.regions['back']
        rear_right_value = self.regions['rear_right']

        if all(value > 2 for value in [front_left_value, front_value, front_right_value, rear_right_value, back_value, rear_left_value]):
            self.velocity.angular.z = 0.0  # Area is total clear, move forward
            print("Forward")
        
        elif all(value > 2 for value in [front_value, front_right_value, rear_left_value, back_value, rear_right_value]) and front_left_value < 2:
            self.velocity.angular.z = 1.57  
            print("object in front_left, Moving front_right")
        
        elif all(value > 2 for value in [front_left_value, front_value, rear_right_value, rear_left_value, back_value]) and front_right_value < 2:
            self.velocity.angular.z = -1.57 
            print("object in front_right,Moving front_left")
        
        elif all(value > 2 for value in [front_value, rear_right_value, rear_left_value, back_value]) and front_right_value < 2 and front_left_value < 2:
            self.velocity.angular.z = 0.0  
            print("object in front_right and front_left ,Moving forward")
        
        elif all(value > 2 for value in [rear_right_value, rear_left_value, back_value]) and front_right_value < 2 and front_left_value < 2 and front_value < 2:
            self.velocity.angular.z = 3.14  
            print("object in front_right, front_left and front, going reverse")

        elif all(value > 2 for value in [rear_right_value]) and rear_left_value < 2 and front_left_value < 2 and front_value < 2 and front_right_value < 2 and back_value < 2:
            self.velocity.angular.z = 1.57  
            print("object in front_left, front, front_right, rear_left and back, Moving rear_right")

        elif all(value > 2 for value in [rear_left_value]) and rear_right_value < 2 and front_left_value < 2 and front_value < 2 and front_right_value < 2 and back_value < 2:
            self.velocity.angular.z = -1.57  
            print("object in front_left, front, front_right, rear_right and back, Moving rear_left")

        elif all(value > 2 for value in [back_value]) and rear_right_value < 2 and front_left_value < 2 and front_value < 2 and front_right_value < 2 and rear_left_value < 2:
            self.velocity.angular.z = 3.14 
            print("object in front_left, front, front_right, rear_right and rear_left_value, going reverse")

        elif all(value > 2 for value in [front_value]) and rear_right_value < 2 and front_left_value < 2 and back_value < 2 and front_right_value < 2 and rear_left_value < 2:
            self.velocity.angular.z = 0.0 
            print("object in front_left, back, front_right, rear_right and rear_left_value, going straight")

        # elif all(value < 2 for value in [front_left_value, front_value, front_right_value, rear_right_value, back_value, rear_left_value]):
        #     self.velocity.angular.z = 0.0  # Area is total clear, move forward
        #     print("robot stopped")

        # elif all(value > 2 for value in [front_left_value, front_value, front_right_value, rear_left_value, back_value]) and rear_left_value > 2:
        #     self.velocity.angular.z = -1.57  # Object on rear_left, take a right turn
        #     print("Moving rear_right")
        
        # elif all(value > 2 for value in [front_left_value, front_value, front_right_value, rear_right_value, back_value]) and back_value > 2:
        #     self.velocity.angular.z = 3.14  # Object ahead, take a full turn
        #     print("Reverse")
        
        # elif front_left_value < 2 and front_value < 2 and front_right_value < 2:
        #     self.velocity.angular.z = 1.57  # Object on front_left, front, front_right - taking right
        #     print("Moving right")
        
        # elif rear_left_value < 2 and rear_right_value < 2 and back_value < 2:
        #     self.velocity.angular.z = -1.57  # Object on rear_left, rear_right, back - taking left
        #     print("Moving left")
        
        else:
            #self.velocity.angular.z = 0.0  # Stop if none of the above conditions are met
            print("Robot stopped")       
        
        ## lets publish the complete velocity
        self.publisher.publish(self.velocity)

def main(args=None):
    rclpy.init(args=args)
    oab=ObstacleAvoidingBot()
    rclpy.spin(oab)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import Twist

# class ObstacleAvoidingBot(Node):
#     def __init__(self):
#         super().__init__('Go_to_position_node')  # Name of the node
#         # Publisher
#         self.publisher = self.create_publisher(Twist, '/cmd_vel', 40)
#         # Subscriber
#         self.subscription = self.create_subscription(
#             LaserScan, '/scan', self.get_scan_values, 40)
#         # Periodic publisher call
#         timer_period = 0.2
#         self.timer = self.create_timer(timer_period, self.send_cmd_vel)
#         # Initializing Global values
#         # Given a value for VELOCITY
#         self.linear_vel = 0.22
#         # Creating a message object to set new velocities and publish them
#         self.velocity = Twist()
#         # Initializing regions as empty lists
#         self.regions = {'right': [], 'mid': [], 'left': []}

#     # Subscriber Callback function
#     def get_scan_values(self, scan_data):
#         # We have 360 data points and we are dividing them into 3 regions
#         # We find the minimum value in each region
#         self.regions = {
#             'right': int(min(min(scan_data.ranges[0:120]), 100)),
#             'mid':   int(min(min(scan_data.ranges[120:240]),100)),
#             'left':  int(min(min(scan_data.ranges[240:360]),100)),
#         }
#         print(
#             f"Left: {self.regions['left']} / Mid: {self.regions['mid']} / Right: {self.regions['right']}")

#     # Callback Publisher of velocities called every 0.2 seconds
#     def send_cmd_vel(self):
#         # Angular and linear velocities are set into the self.velocity object
#         # Setting the linear velocity to be fixed, so the robot will keep moving
#         self.velocity.linear.x = self.linear_vel

#         left_values = self.regions['left']
#         mid_values = self.regions['mid']
#         right_values = self.regions['right']

#         if all(value > 4 for value in left_values + mid_values + right_values):
#             self.velocity.angular.z = 0.0  # Area is total clear, move forward
#             print("Forward")
#         elif all(value > 4 for value in left_values + mid_values) and any(value < 4 for value in right_values):
#             self.velocity.angular.z = 1.57  # Object on the right, take a left turn
#             print("Right")
#         elif any(value < 4 for value in left_values) and all(value > 4 for value in mid_values + right_values):
#             self.velocity.angular.z = -1.57  # Object on the left, take a right turn
#             print("Left")
#         elif all(value < 4 for value in left_values + mid_values + right_values):
#             self.velocity.angular.z = 3.14  # Object ahead, take a full turn
#             print("Reverse")
#         else:
#             print("Some other conditions are required to be programmed")

#         # Publish the complete velocity
#         self.publisher.publish(self.velocity)

# def main(args=None):
#     rclpy.init(args=args)
#     oab = ObstacleAvoidingBot()
#     rclpy.spin(oab)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
