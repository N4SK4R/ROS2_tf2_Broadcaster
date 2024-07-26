from my_robot_interfaces.srv import UpdateGoal
from nav_msgs.msg import Odometry
from std_msgs.msg._int32 import Int32
import rclpy
from rclpy.node import Node

class Test(Node):

    def __init__(self):
        super().__init__('test')
        self.cli1 = self.create_client(UpdateGoal, '/agent_1/update_goal')
        self.cli2 = self.create_client(UpdateGoal, '/agent_2/update_goal')
        self.cli4 = self.create_client(UpdateGoal, '/agent_4/update_goal')
        self.create_subscription(Odometry, "/agent_1/odom", self.pose_callback_for_agent1, 10)
        self.create_subscription(Odometry, "/agent_2/odom", self.pose_callback_for_agent2, 10)
        self.elevator=self.create_publisher(Int32,"/elevator/target_position",10)
        self.req = UpdateGoal.Request()
        self.current = 0
        self.target  = 2.5
        self.Reached1 =True
        self.down =True

    def send_request(self,x,y):
        self.req._goal_pose.position.x = x
        self.req._goal_pose.position.y = y
        self.cli1.call_async(self.req)

    def pose_callback_for_agent1(self, msg : Odometry):
        
        if abs(int(msg.pose.pose.position.x))==5 and abs(int(msg.pose.pose.position.y))==5 and abs(int(msg.pose.pose.position.z))==0 and self.Reached1:
            self.Reached1=False
            self.get_logger().info("new goal") 
            self.timer = self.create_timer(0.01, lambda: self.wait())           
        
        if abs(int(msg.pose.pose.position.x))==0 and abs(int(msg.pose.pose.position.y))==8 and abs(int(msg.pose.pose.position.z))>0 and self.Reached1:
            self.get_logger().info("move agent 2")
            self.get_logger().info("move agent 4")
            self.Reached1=False
            self.req._goal_pose.position.x = 5.0
            self.req._goal_pose.position.y = 5.0
            self.cli2.call_async(self.req)
            self.req._goal_pose.position.x = 5.0
            self.req._goal_pose.position.y = 0.0
            self.cli4.call_async(self.req)
            
             
    def pose_callback_for_agent2(self, msg : Odometry):
        if abs(int(msg.pose.pose.position.x))==5 and abs(int(msg.pose.pose.position.y))==5 and abs(int(msg.pose.pose.position.z))==1 and self.down:
            self.down=False
            self.get_logger().info("down") 
            self.timer = self.create_timer(0.01, lambda: self.wait2()) 
              
                
    def wait(self):
        self.current += 0.4 * 0.01

        if self.current >= self.target:
            self.current = 0
            self.Reached1=True
            self.timer.cancel()    
            self.send_request(8.0,0.0)
    
    def wait2(self):
        self.current += 1.5 * 0.01

        if self.current >= self.target:
            self.current = 0
            self.timer.cancel()    
            down=Int32()
            down.data=0
            self.elevator.publish(down)
    
def main(args=None):
    rclpy.init(args=args)

    client = Test()
    client.send_request(5.0,5.0)
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()