import random
import math

import rclpy
from rclpy.node import Node


from turtlesim.srv import Spawn, Kill
from obstacle_course.obstacle import Obstacle

class ObstacleCourseGenerator(Node):
    def __init__(self):
        super().__init__('obstacle_course_generator')

        # Bot
        self.client = self.create_client(Spawn, 'spawn')
        self.kill_client = self.create_client(Kill, 'kill')


        self.spawn_cords = {'x': 0.0, 'y': 0.0}
        self.goal_cords = {'x': 0.0, 'y': 0.0}

        self.obstacles = []

        if not self.client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('Spawn service is not available.')
            return

        if not self.kill_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('Kill service is not available.')
            return

        self.remove_default_turtle()

        self.start()
        self.goal()


        for i in range(5):
            
            x = 0.0
            y = 0.0
            distance_spawn = 0.0
            distance_goal = 0.0
           
            while distance_spawn < 2.0 or distance_goal < 2.0:
                x=random.uniform(1.0, 10.0)
                y=random.uniform(1.0, 10.0)
                distance_spawn = math.sqrt(((x - self.spawn_cords['x']) ** 2) + ((y - self.spawn_cords['y']) ** 2))
                distance_goal = math.sqrt(((x - self.goal_cords['x']) ** 2) + ((y - self.goal_cords['y']) ** 2))       



            obstacle = Obstacle(x,y , theta=0.0, radius=1.5)

            self.obstacles.append(obstacle)
           
            self.spawn_obstacle(obstacle, i)

        self.get_logger().info(
            f"Goal set to ({self.goal_cords['x']:.2f}, {self.goal_cords['y']:.2f})"
        )



    
    def spawn_obstacle(self, obstacle: Obstacle, index: int):
        req = Spawn.Request()
        req.x = obstacle.x
        req.y = obstacle.y
        req.theta = obstacle.theta
        req.name = f'obstacle_{index}'
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        response = future.result()

        if response is None:
            self.get_logger().warning(f'Failed to spawn {req.name}')
        else:
            self.get_logger().info(f'Spawned {response.name} at ({req.x:.2f}, {req.y:.2f})')


    def remove_default_turtle(self):
        req = Kill.Request()
        req.name = 'turtle1'

        future = self.kill_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.exception() is None:
            self.get_logger().info('Removed turtle1')
        else:
            # turtle1 may already be gone if this is not the first run.
            self.get_logger().warning(f'Could not remove turtle1: {future.exception()}')

    def start(self):
        req = Spawn.Request()

        x = random.uniform(1.0, 10.0)
        y = random.uniform(1.0, 10.0)

        self.spawn_cords['x'] = x
        self.spawn_cords['y'] = y

        req.name = 'bot'
        req.x = x
        req.y = y
        req.theta = random.uniform(0.0, 360.0)


        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.result() is None:
            self.get_logger().error('Failed to spawn bot')
        else:
            self.get_logger().info(f'Bot spawned at ({req.x:.2f}, {req.y:.2f})')

        
    
    def goal(self):
        self.goal_cords['x'] = random.uniform(1.0, 10.0)
        self.goal_cords['y'] = random.uniform(1.0, 10.0)

    def get_transfer_data(self):
        obstacle_positions = []
        for obs in self.obstacles:
            obstacle_positions.append((obs.x, obs.y, obs.radius))

        return {
            'goal': {'x': self.goal_cords['x'], 'y': self.goal_cords['y']},
            'obstacles': obstacle_positions,
        }


def main():
    rclpy.init()
    node = ObstacleCourseGenerator()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()