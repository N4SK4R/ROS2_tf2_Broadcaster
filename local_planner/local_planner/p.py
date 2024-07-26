import rclpy
import math
import sys
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from my_robot_interfaces.msg import AgentInfo
from std_msgs.msg._int32 import Int32
from tf_transformations import euler_from_quaternion

class DirectionSubscriber(Node):

    def __init__(self,agent_name):
        super().__init__('agent_info_subscriber')
        self.create_subscription(AgentInfo,'/agent_info',self.listener_callback,10)
        self.create_subscription(Odometry, agent_name+"/odom", self.pose_callback, 10)
        self.vel_command = self.create_publisher(Twist, agent_name+"/cmd_vel", 10)
        self.elevator=self.create_publisher(Int32,"/elevator/target_position",10)
        
        self.agent_name=agent_name
        self.start_X=None
        self.start_Y=None
        self.direction=None
        self.moving=False
        self.valid=False
        self.check_orientaion=True
        
        self.direction_distance_list=[]
        self.x_distance=0
        self.y_distance=0
        self.tolerance_distance = 0.1
        self.tolerance_theta = 0.0001

        self.reference_pose = None
        self.prev_error_distance = 0.0
        self.sum_error_distance  = 0.0
        
        self.orientation = "north"
        self.reference_theta  = 0.0
        self.sum_error_theta  = 0.0
        self.prev_error_theta = 0.0
        self.rotate_list=[]
        self.prev_direction = None
        

    def listener_callback(self, msg : AgentInfo):
        
        if self.agent_name!=msg.serial_id:
            return
        
        x=int(msg.pose.position.x)
        y=int(msg.pose.position.y)
            
        if self.start_X ==None and self.start_Y ==None:
            self.start_X=x
            self.start_Y=y
            self.get_logger().info(f'Start Position -> ({self.start_X},{self.start_Y})') 
                        
        if  self.start_X !=None and self.start_Y !=None: 
            direction_x = x-self.start_X 
            direction_y = y-self.start_Y
            
            if direction_y > 0 :
                self.direction="north"
                self.x_distance+=1.01
                self.append_insert(self.x_distance,self.direction)
                self.get_logger().info(self.direction)
            elif direction_y < 0 :
                self.direction="south"
                self.x_distance+=1.01
                self.append_insert(self.x_distance,self.direction)
                self.get_logger().info(self.direction)   
            elif direction_x > 0 :
                self.direction="east"
                self.y_distance+=1.01
                self.append_insert(self.y_distance,self.direction)
                self.get_logger().info(self.direction)
            elif direction_x < 0 :
                self.direction="west"
                self.y_distance+=1.01
                self.append_insert(self.y_distance,self.direction)
                self.get_logger().info(self.direction)
                
            self.start_X =x
            self.start_Y =y
        
        if self.check_orientaion==True and self.direction!=None:
            
            if (self.direction=="south" and self.orientation=="north") or self.direction=="west" and self.orientation=="east":
                self.check_orientaion=False
                self.valid=True
                
            elif self.orientation != self.direction:
                self.get_logger().info("Needs Rotation Before Moving !")
                match self.orientation:
                    case "north":
                        if self.direction == "east":
                            self.rotate_list.append(("right","east"))
                            
                        elif self.direction == "west":
                            self.rotate_list.append(("left","west"))
                            
                    case "east":
                        if self.direction == "north" or self.direction == "south":
                            self.rotate_list.append(("left","north"))        
                            
                    case "west":
                        if self.direction == "north" or self.direction == "south":
                            self.rotate_list.append(("right","north"))
                            
                if len(self.rotate_list) != 0:            
                    self.get_logger().info(f"Rotating {self.rotate_list[0][0]}")
                    self.rotate_bot(self.rotate_list[0][0],self.rotate_list[0][1]) 
                    self.rotate_list.pop(0)
                    self.check_orientaion=False
                    self.valid=True
            else:
                self.check_orientaion=False
                self.valid=True
 
        if self.prev_direction != self.direction and self.prev_direction != None and self.direction !=None:
            match self.orientation:
                case "north":
                    if self.direction == "east":
                        self.rotate_list.append(("right","east"))
                        
                    elif self.direction == "west":
                        self.rotate_list.append(("left","west"))
                        
                case "east":
                    if self.direction == "north" or self.direction == "south":
                        self.rotate_list.append(("left","north"))        
                        
                case "west":
                    if self.direction == "north" or self.direction == "south":
                        self.rotate_list.append(("right","north"))            
        
        if len(self.direction_distance_list) != 0:
            if self.direction_distance_list[0][0]==0:  
                if len(self.rotate_list) != 0:
                    self.get_logger().info(f"Rotating {self.rotate_list[0][0]}")
                    self.rotate_bot(self.rotate_list[0][0],self.rotate_list[0][1]) 
                    self.rotate_list.pop(0)
                    self.direction_distance_list.pop(0)
                else:
                    self.direction_distance_list.pop(0)
                    self.x_distance=0
                    self.y_distance=0
                    self.direction=None
                    self.check_orientaion=True
                    self.get_logger().info("Reached!!")
                    if msg.serial_id=="agent_1":
                        up=Int32()
                        up.data=100
                        self.elevator.publish(up)                  
        self.prev_direction = self.direction
            
    def rotate_bot(self, rotate_direction,new_orientation):

        if rotate_direction == "left":
            self.reference_theta += math.pi / 2
        else:
            self.reference_theta -= math.pi / 2
            
        self.orientation=new_orientation
        self.get_logger().info(f"New Orientation ={self.orientation}")
        self.valid=True
                
    def pose_callback(self, msg: Odometry):
        
        if self.valid:
                 
            orientation = msg.pose.pose.orientation
            quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
            (roll, pitch, yaw) = euler_from_quaternion(quaternion)
            error_theta= -(yaw - self.reference_theta)
            error_distance=None   
            
            if abs(error_theta) < self.tolerance_theta:
                self.moving=True  
                
            if self.moving and len(self.direction_distance_list) != 0:   
                
                if self.reference_pose is None:
                    if self.direction_distance_list[0][1]=="north" or self.direction_distance_list[0][1]=="south":
                        self.reference_pose = msg.pose.pose.position.x
                    elif self.direction_distance_list[0][1]=="east" or self.direction_distance_list[0][1]=="west" :
                        self.reference_pose = msg.pose.pose.position.y
                    return
                            
                if self.direction_distance_list[0][1]=="north" or self.direction_distance_list[0][1]=="south":
                    current_postion = msg.pose.pose.position.x
                elif self.direction_distance_list[0][1]=="east" or self.direction_distance_list[0][1]=="west":
                    current_postion = msg.pose.pose.position.y

                relative_current_postion = current_postion - self.reference_pose
                error_distance = self.direction_distance_list[0][0] - abs(relative_current_postion)
                    
                if abs(error_distance) < self.tolerance_distance:
                    self.direction_distance_list[0][0]=0
                    self.reference_pose=None
                    self.moving=False
                    self.valid=False
                    self.stop_movement()
                    return
                
            self.pidcontroller(error_distance,error_theta)
    
    def pidcontroller(self, error_distance,error_theta):
        
        kp = 0.2
        ki = 0.0002
        kd = 0.08

        if error_distance != None:
            self.sum_error_distance += error_distance
            change_in_error_distance = error_distance - self.prev_error_distance

            if abs(error_distance) >= self.tolerance_distance:
                linear_velocity = kp * abs(error_distance) + ki * self.sum_error_distance + kd * change_in_error_distance
            else:
                linear_velocity = 0.0
                self.sum_error_distance = 0.0
                
            self.prev_error_distance = error_distance
        else:
            linear_velocity=0.0
            
        #-------------Rotation------------
        self.sum_error_theta += error_theta
        change_in_error_theta = error_theta - self.prev_error_theta

        if abs(error_theta) >= self.tolerance_theta:
            angular_velocity = 0.8 * error_theta + 0.005 * self.sum_error_theta + 0.4 * change_in_error_theta
        else:
            angular_velocity = 0.0
            self.int_error_theta = 0.0

        self.prev_error_theta = error_theta

            
        self.vel_command_pub(linear_velocity,angular_velocity)
        
    def vel_command_pub(self, linear_velocity,angular_velocity):
        pid_msg = Twist()
        if len(self.direction_distance_list)!=0 and self.moving:
            if self.direction_distance_list[0][1] == "south":
                pid_msg.linear.x = -linear_velocity 
            elif (self.orientation == "east" and self.direction_distance_list[0][1] == "west") or (self.orientation == "west" and self.direction_distance_list[0][1] == "east"):
                pid_msg.linear.x = -linear_velocity
            else:
                pid_msg.linear.x = linear_velocity

        pid_msg.angular.z = angular_velocity
        self.vel_command.publish(pid_msg)
        
    def stop_movement(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.vel_command.publish(stop_msg)   
        
    def append_insert(self,distance,direction):
        for path in self.direction_distance_list:
            if path[1]==direction:
                path[0]=distance
                return
            
        self.direction_distance_list.append([distance,direction])
        
def main(args=None):
    rclpy.init(args=args)
    Direction_subscriber = DirectionSubscriber(sys.argv[1])

    rclpy.spin(Direction_subscriber)
    Direction_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()