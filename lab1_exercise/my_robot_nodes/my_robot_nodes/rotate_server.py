import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from my_robot_interfaces.action import Rotate


def yaw_from_quaternion(q):
    """Extract yaw (rotation around z) from a quaternion."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def angle_difference(target, current):
    """Calculate the wrapped difference between two angles."""
    return math.atan2(
        math.sin(target - current),
        math.cos(target - current)
    )


class RotateServer(Node):

    def __init__(self):
        super().__init__('rotate_server')

        self.declare_parameter('turn_speed', 0.5)

        self.yaw = None
        self.total_turned = 0.0
        self.rotating = False
        self.lock = threading.Lock()

        self.callback_group = ReentrantCallbackGroup()

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
            callback_group=self.callback_group
        )

        self.action_server = ActionServer(
            self,
            Rotate,
            '/rotate',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info('Rotate action server started')

    def odom_callback(self, msg):
        new_yaw = yaw_from_quaternion(
            msg.pose.pose.orientation
        )

        with self.lock:
            if self.yaw is not None and self.rotating:
                # Accumulate small changes in yaw to handle +/- pi crossing
                self.total_turned += angle_difference(
                    new_yaw,
                    self.yaw
                )

            self.yaw = new_yaw

    def goal_callback(self, goal_request):
        if goal_request.angle == 0.0:
            self.get_logger().warning('Rejecting zero-angle goal')
            return GoalResponse.REJECT

        with self.lock:
            if self.yaw is None:
                self.get_logger().warning('Waiting for odometry')
                return GoalResponse.REJECT

            if self.rotating:
                self.get_logger().warning('Another rotation is active')
                return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Rotation cancellation requested')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        target_angle = goal_handle.request.angle

        with self.lock:
            self.total_turned = 0.0
            self.rotating = True

        turn_speed = abs(
            self.get_parameter('turn_speed').value
        )

        direction = 1.0 if target_angle > 0 else -1.0

        twist = Twist()
        twist.angular.z = direction * turn_speed

        feedback = Rotate.Feedback()
        result = Rotate.Result()

        self.get_logger().info(
            f'Rotating through {target_angle:.2f} radians'
        )

        while rclpy.ok():

            # Handle cancellation
            if goal_handle.is_cancel_requested:
                with self.lock:
                    turned = self.total_turned
                    self.rotating = False

                self.cmd_vel_publisher.publish(Twist())

                result.turned = turned
                goal_handle.canceled()

                self.get_logger().info(
                    f'Rotation canceled at {turned:.2f} radians'
                )
                return result

            with self.lock:
                turned = self.total_turned

                # Check whether the requested angle has been reached
                if target_angle > 0:
                    completed = turned >= target_angle
                else:
                    completed = turned <= target_angle

                if completed:
                    self.rotating = False

            if completed:
                self.cmd_vel_publisher.publish(Twist())

                result.turned = turned
                goal_handle.succeed()

                self.get_logger().info(
                    f'Rotation completed: {turned:.2f} radians'
                )
                return result

            # Continue rotating and publish feedback
            self.cmd_vel_publisher.publish(twist)

            feedback.turned = turned
            goal_handle.publish_feedback(feedback)

            time.sleep(0.1)

        # Stop if ROS shuts down while the action is running
        with self.lock:
            turned = self.total_turned
            self.rotating = False

        self.cmd_vel_publisher.publish(Twist())

        result.turned = turned
        goal_handle.abort()
        return result


def main(args=None):
    rclpy.init(args=args)

    node = RotateServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
