import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from turtlesim.srv import SetPen
from turtlesim.msg import Pose



class Bot(Node):
    def __init__(self, goal_cords: dict | None = None, obstacles: list | None = None):
        super().__init__('Bot')

        self.publisher = self.create_publisher(Twist, '/bot/cmd_vel', 10)

        self.subscription = self.create_subscription(
            Pose, '/bot/pose', self.pose_callback, 10
        )
        self.pen_client = self.create_client(SetPen, '/bot/set_pen')
        self.pen_client.wait_for_service()
        pen_req = SetPen.Request()
        pen_req.off = 0
        self.pen_client.call_async(pen_req)
        


        self.done = False
        self.first = True

        if goal_cords is None:
            goal_cords = {'x': 5.5, 'y': 5.5}
        if obstacles is None:
            obstacles = []

        self.goal_x = goal_cords['x']
        self.goal_y = goal_cords['y']
        self.obstacles = obstacles

        self.get_logger().info(
            f'Bot received goal ({self.goal_x:.2f}, {self.goal_y:.2f}) and {len(self.obstacles)} obstacles'
        )


    def pose_callback(self, msg: Pose):
        if self.done:
            self.destroy_node()
            rclpy.shutdown()

        cmd = Twist()

        dx = msg.x - self.goal_x
        dy = msg.y - self.goal_y            
        distance = math.sqrt(dx*dx + dy*dy)

        closest_obstacle = None
        closest_distance = float('inf')

        for obstacle in self.obstacles:
            dx_obs = msg.x - obstacle[0]
            dy_obs = msg.y - obstacle[1]
            distance_obs = math.sqrt(dx_obs * dx_obs + dy_obs * dy_obs)

            if distance_obs < closest_distance:
                closest_distance = distance_obs
                closest_obstacle = obstacle

        if closest_obstacle is not None and closest_distance < closest_obstacle[2] + 0.75:
            dx_obs = msg.x - closest_obstacle[0]
            dy_obs = msg.y - closest_obstacle[1]
            angle_to_obstacle = math.atan2(dy_obs, dx_obs)
            angle_away_from_obstacle = angle_to_obstacle + math.pi
            angle_error = math.atan2(
                math.sin(angle_away_from_obstacle - msg.theta),
                math.cos(angle_away_from_obstacle - msg.theta),
            )

            cmd.angular.z = 2.0 * angle_error
            cmd.linear.x = -0.5 if abs(angle_error) < 0.5 else 0.0
            self.publisher.publish(cmd)
            return
        
        if distance <= 0.2:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.publisher.publish(cmd) # Skicka stopp-kommandot
            self.get_logger().info(f'Goal reached! (x = {self.goal_x}, y = {self.goal_y})')
            self.done = True
            return


        
        angle_to_goal = math.atan2(self.goal_y - msg.y, self.goal_x - msg.x)
        angle =  angle_to_goal - msg.theta
        angle = math.atan2(math.sin(angle), math.cos(angle))

        if abs(angle) > 0.05:
            cmd.angular.z = 2 *  angle
            self.publisher.publish(cmd)
        else:
            cmd.linear.x = distance
            self.publisher.publish(cmd)

            
    
            
        
        
            



def main():
    rclpy.init()
    rclpy.spin(Bot())
    rclpy.shutdown()