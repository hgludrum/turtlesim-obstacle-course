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
        pen_req.r = 255
        pen_req.g = 255
        pen_req.b = 255
        pen_req.width = 3
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

        self.orbiting = False
        self.orbit_start_angle = None
        self.orbit_obstacle = None 


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
                closest_obstacle = obstacle
                closest_distance = distance_obs

        if self.orbiting and self.orbit_obstacle is not None and self.orbit_start_angle is not None and closest_obstacle is not None:
            goal_dot = (
                (msg.x - self.orbit_obstacle[0]) * (self.goal_x - self.orbit_obstacle[0]) +
                (msg.y - self.orbit_obstacle[1]) * (self.goal_y - self.orbit_obstacle[1])
            )
            current_angle = math.atan2(msg.y - self.orbit_obstacle[1], msg.x - self.orbit_obstacle[0])
            angle_traveled = abs(math.atan2(
                math.sin(current_angle - self.orbit_start_angle),
                math.cos(current_angle - self.orbit_start_angle)
            ))

            if angle_traveled > math.pi / 3 and goal_dot > 0 and closest_distance > (closest_obstacle[2] + 1.2):
                self.orbiting = False
                self.orbit_obstacle = None

        if closest_obstacle is not None and closest_distance < (closest_obstacle[2] + 0.75):
            self.avoid_obstacle(msg=msg, cmd=cmd, closest_obstacle=closest_obstacle)
            return
               

        
        if distance <= 0.2:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.publisher.publish(cmd) 
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

            
    
            
    def avoid_obstacle(self, msg, cmd, closest_obstacle):
        ox, oy, radius = closest_obstacle


        if not self.orbiting or self.orbit_obstacle != (ox, oy):
            self.orbiting = True
            self.orbit_obstacle = (ox, oy)
            self.orbit_start_angle = math.atan2(msg.y - oy, msg.x - ox)

        dx = msg.x - ox
        dy = msg.y - oy

        distance = math.sqrt(dx*dx + dy*dy)
        
        normalizedx = dx / distance
        normalizedy = dy / distance

        tx = normalizedy
        ty = -normalizedx


        distance_obs = math.sqrt(dx * dx + dy * dy)


        gx = self.goal_x - msg.x
        gy = self.goal_y - msg.y
        gdist = math.sqrt(gx*gx + gy*gy) or 1.0
        gx, gy = gx / gdist, gy / gdist

        danger = max(0.0, 1.0 - (distance_obs - radius) / 1.5)
        steer_x = danger * tx + (1 - danger) * gx
        steer_y = danger * ty + (1 - danger) * gy

        target_angle = math.atan2(steer_y, steer_x)
        angle_error = math.atan2(
            math.sin(target_angle - msg.theta),
            math.cos(target_angle - msg.theta)
        )
        cmd.angular.z = 2.5 * angle_error
        cmd.linear.x = 0.5 if abs(angle_error) < 0.5 else 0.0
        self.publisher.publish(cmd)
        
            



def main():
    rclpy.init()
    rclpy.spin(Bot())
    rclpy.shutdown()