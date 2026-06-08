from obstacle_course.bot import Bot
from obstacle_course.obstacle_course_generator import ObstacleCourseGenerator
import rclpy

def main():
    rclpy.init()

    generator = ObstacleCourseGenerator()
    data = generator.get_transfer_data()
    bot = Bot(goal_cords=data['goal'], obstacles=data['obstacles'])

    rclpy.spin(bot)

    bot.destroy_node()
    generator.destroy_node()
    rclpy.shutdown()