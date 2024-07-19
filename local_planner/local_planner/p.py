import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from my_robot_interfaces.msg import AgentInfo

class DirectionSubscriber(Node):

    def __init__(self):
        super().__init__('agent_info_subscriber')
        self.create_subscription(AgentInfo,'/agent_info',self.listener_callback,10)
        self.create_subscription(Odometry, "/odom", self.pose_callback, 10)
        self.vel_command = self.create_publisher(Twist, "/cmd_vel", 10)
        self.start_X=None
        self.start_Y=None
        self.Received_Start_Position=False
        self.direction=None
        self.moving=True
        
        self.desired_distance=0
        self.tolerance_distance = 0.1

        self.reference_pose = None
        self.prev_error_distance = 0.0
        self.int_error_distance = 0.0
        
        self.orientation = "north"
        self.rotate_list=[]
        self.prev_direction = None
   
    def listener_callback(self, msg : AgentInfo):
        
        x=int(msg.pose.position.x)
        y=int(msg.pose.position.y)
        
        if self.start_X ==None and self.start_Y ==None:
            self.Received_Start_Position=True
            
        if self.Received_Start_Position==True:
            self.start_X=x
            self.start_Y=y
            self.get_logger().info(f'Start Position -> ({self.start_X},{self.start_X})')   
            self.Received_Start_Position=False
         
        if  self.start_X !=None and self.start_Y !=None: 
            direction_x = x-self.start_X 
            direction_y = y-self.start_Y
            
            if direction_y > 0 :
                self.direction="north"
                self.desired_distance+=1
                self.get_logger().info("NORTH")
            elif direction_y < 0 :
                self.direction="south"
                self.get_logger().info("SOUTH")   
            elif direction_x > 0 :
                self.direction="east"
                self.get_logger().info("EAST")
            elif direction_x < 0 :
                self.direction="west"
                self.get_logger().info("WEST")
                
            self.start_X =x
            self.start_Y =y
        
        if self.prev_direction != self.direction and self.prev_direction != None:
            match self.orientation:
                case "north":
                    if self.direction == "east":
                        self.orientation="east"
                        self.rotate_list.append("right")
                        
                    elif self.direction == "west":
                        self.orientation="west"
                        self.rotate_list.append("left")
                        
                case "east":
                    if self.direction == "north":
                        self.orientation="north"
                        self.rotate_list.append("left")
                        
                    elif self.direction == "south":
                        self.orientation="south"
                        self.rotate_list.append("right")
                        
                case "west":
                    if self.direction == "north":
                        self.orientation="north"
                        self.rotate_list.append("right")
                        
                    elif self.direction == "south":
                        self.orientation="south"
                        self.rotate_list.append("left")
        
        if self.desired_distance==0 and len(self.rotate_list) != 0:
            self.get_logger().info(f"Rotating {self.rotate_list[0]}")
            self.rotate_bot(self.rotate_list.pop(0)) 
        
        self.prev_direction = self.direction

    def rotate_bot(self, rotate_direction):
        self.current_rotate_angle = 0.0
        self.target_angle = math.pi / 2
        self.rotate_ang_vel = 0.8 if rotate_direction == "left" else -0.8
        self.set_up_timer()

    def rot_command(self):

        rot_msg = Twist()
        rot_msg.angular.z = self.rotate_ang_vel
        self.current_rotate_angle += self.rotate_ang_vel * 0.01

        if abs(self.current_rotate_angle) >= self.target_angle:
            rot_msg.angular.z = 0.0
            self.rot_complete = True
            self.get_logger().info(f"Rotation complete")
            self.get_logger().info(f"Current Orientation: {self.orientation}")
            self.current_rotate_angle = 0.0
            self.stop_timer()
            self.needs_rotation=False

        self.vel_command.publish(rot_msg) 
        
    def pose_callback(self, msg: Odometry):
        
        if self.moving:   
            if self.reference_pose is None:
                if self.direction=="north":
                    self.reference_pose = msg.pose.pose.position.x
                elif self.direction=="east" or self.direction=="west" :
                    self.reference_pose = msg.pose.pose.position.y
                return
                        
            if self.direction=="north":
                current_postion = msg.pose.pose.position.x
            elif (self.direction=="east" or self.direction=="west") and self.orientation=="east":
                current_postion = msg.pose.pose.position.x

            relative_current_postion = current_postion - self.reference_pose
            error_distance = self.desired_distance - abs(relative_current_postion)
                
            if abs(error_distance) < self.tolerance_distance:
                self.desired_distance=0
                self.moving=False
                self.stop_movement()
                return
            self.pidcontroller(error_distance)
        
    def pidcontroller(self, error_distance):
        
        kp_d = 0.2
        ki_d = 0.0002
        kd_d = 0.08

        self.int_error_distance += error_distance
        der_error_distance = error_distance - self.prev_error_distance

        if abs(error_distance) >= self.tolerance_distance:
            lin_velocity = kp_d * abs(error_distance) + ki_d * self.int_error_distance + kd_d * der_error_distance
        else:
            lin_velocity = 0.0
            self.int_error_distance = 0.0

        self.prev_error_distance = error_distance
        self.vel_command_pub(lin_velocity)
        
    def vel_command_pub(self, lin_velocity):
        pid_msg = Twist()
        if self.direction == "south":
            pid_msg.linear.x = -lin_velocity 
        elif (self.orientation == "east" and self.direction == "west") or (self.orientation == "west" and self.direction == "east"):
            pid_msg.linear.x = -lin_velocity
        else:
            pid_msg.linear.x = lin_velocity
        self.vel_command.publish(pid_msg)
        
    def stop_movement(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.vel_command.publish(stop_msg)   
        
    def set_up_timer(self):
        self.timer = self.create_timer(0.01, lambda: self.rot_command()) 
    
    def stop_timer(self):
        self.timer.cancel()
        
def main(args=None):
    rclpy.init(args=args)

    Direction_subscriber = DirectionSubscriber()

    rclpy.spin(Direction_subscriber)

    Direction_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()