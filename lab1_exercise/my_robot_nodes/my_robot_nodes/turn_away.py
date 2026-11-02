import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool

from lab_interfaces.msg import ObstacleInfo
from my_robot_interfaces.msg import RobotStatus
from my_robot_interfaces.srv import SetTurnSpeed


class TurnAway(Node):

    def __init__(self):
        super().__init__('turn_away')

        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('turn_speed', 0.5)
        self.declare_parameter('max_turn_speed', 1.5)

        # The node starts disabled
        self.enabled = False
        self.last_obstacle_distance = float('inf')

        self.subscription = self.create_subscription(
            ObstacleInfo,
            '/obstacle',
            self.obstacle_callback,
            10
        )

        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.status_publisher = self.create_publisher(
            RobotStatus,
            '/robot_status',
            10
        )

        self.enable_service = self.create_service(
            SetBool,
            '/turn_away_enable',
            self.enable_callback
        )

        self.speed_service = self.create_service(
            SetTurnSpeed,
            '/set_turn_speed',
            self.set_turn_speed_callback
        )

        self.get_logger().info('Turn away node started (disabled)')

    def obstacle_callback(self, msg):
        self.last_obstacle_distance = msg.distance

        status = RobotStatus()
        status.obstacle_distance = msg.distance

        # When disabled, report stopped and publish no velocity
        if not self.enabled:
            status.state = 'stopped'
            self.status_publisher.publish(status)
            return

        forward_speed = self.get_parameter('forward_speed').value
        turn_speed = self.get_parameter('turn_speed').value

        velocity = Twist()

        if not msg.too_close:
            velocity.linear.x = forward_speed
            status.state = 'forward'
        else:
            velocity.linear.x = 0.0
            status.state = 'turning'

            if msg.angle > 0:
                velocity.angular.z = -turn_speed
            else:
                velocity.angular.z = turn_speed

        self.publisher.publish(velocity)
        self.status_publisher.publish(status)

    def enable_callback(self, request, response):
        self.enabled = request.data

        if self.enabled:
            response.success = True
            response.message = 'Turn away enabled'
        else:
            # Publish one zero velocity when switched off
            self.publisher.publish(Twist())

            status = RobotStatus()
            status.state = 'stopped'
            status.obstacle_distance = self.last_obstacle_distance
            self.status_publisher.publish(status)

            response.success = True
            response.message = 'Turn away disabled'

        self.get_logger().info(response.message)
        return response

    def set_turn_speed_callback(self, request, response):
        max_speed = self.get_parameter('max_turn_speed').value
        requested_speed = request.turn_speed

        if requested_speed <= 0.0 or requested_speed > max_speed:
            response.success = False
            response.message = (
                f'Turn speed must be above 0 and at most {max_speed} rad/s'
            )
        else:
            self.set_parameters([
                Parameter('turn_speed', value=float(requested_speed))
            ])

            response.success = True
            response.message = f'Turn speed set to {requested_speed} rad/s'

        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TurnAway()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
