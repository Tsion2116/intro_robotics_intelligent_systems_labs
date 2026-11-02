import math
import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist

from lab_interfaces.action import DriveDistance
from my_robot_interfaces.action import Rotate


class SquareDriver(Node):

    def __init__(self):
        super().__init__('square_driver')

        self.declare_parameter('side_length', 0.5)

        self.drive_client = ActionClient(
            self, DriveDistance, '/drive_distance'
        )

        self.rotate_client = ActionClient(
            self, Rotate, '/rotate'
        )

        self.cmd_vel_publisher = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

    def drive_feedback(self, msg):
        self.get_logger().info(
            f"Travelled: {msg.feedback.travelled:.2f} m"
        )

    def rotate_feedback(self, msg):
        self.get_logger().info(
            f"Turned: {msg.feedback.turned:.2f} rad"
        )

    def send_goal_and_wait(self, client, goal, feedback_callback):
        client.wait_for_server()

        send_future = client.send_goal_async(
            goal,
            feedback_callback=feedback_callback
        )

        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False, None

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(self, result_future)
        response = result_future.result()

        if response is None:
            return False, None

        success = response.status == GoalStatus.STATUS_SUCCEEDED

        return success, response.result

    def stop_robot(self):
        self.cmd_vel_publisher.publish(Twist())

    def run_square(self):
        side_length = self.get_parameter('side_length').value

        for side in range(1, 5):

            self.get_logger().info(
                f'========== SIDE {side} =========='
            )

            # Drive forward
            drive_goal = DriveDistance.Goal()
            drive_goal.distance = side_length

            success, result = self.send_goal_and_wait(
                self.drive_client,
                drive_goal,
                self.drive_feedback
            )

            if not success or not result.reached:
                self.get_logger().error(
                    f'Driving failed on side {side}'
                )
                self.stop_robot()
                return

            # Rotate 90 degrees left
            rotate_goal = Rotate.Goal()
            rotate_goal.angle = math.pi / 2

            success, result = self.send_goal_and_wait(
                self.rotate_client,
                rotate_goal,
                self.rotate_feedback
            )

            if not success:
                self.get_logger().error(
                    f'Rotation failed after side {side}'
                )
                self.stop_robot()
                return

        self.stop_robot()
        self.get_logger().info('Square completed successfully!')


def main(args=None):
    rclpy.init(args=args)
    node = SquareDriver()

    try:
        node.run_square()
    except KeyboardInterrupt:
        node.get_logger().info('Square driver interrupted')
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
