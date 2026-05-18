import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String


class TwinStateMonitorNode(Node):
    def __init__(self):
        super().__init__('twin_state_monitor')

        # Subscribe to physical robot battery state
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.on_battery_state,
            10
        )

        # Publish mirrored battery state to DT topic
        self.battery_pub = self.create_publisher(
            BatteryState,
            '/aurafarm/dt_battery_state',
            10
        )

        # Publish human readable system status
        self.status_pub = self.create_publisher(
            String,
            '/aurafarm/dt_system_status',
            10
        )

        self.last_battery_percentage = None
        self.get_logger().info('TwinStateMonitor started')

    def on_battery_state(self, msg: BatteryState):
        # Mirror battery state to DT
        self.battery_pub.publish(msg)

        # Calculate percentage — voltage based for TurtleBot burger
        # TurtleBot burger battery: min ~11.0V, max ~12.6V
        percentage = (msg.voltage - 11.0) / (12.6 - 11.0) * 100.0
        percentage = max(0.0, min(100.0, percentage))

        # Only log when percentage changes significantly
        if (self.last_battery_percentage is None or
                abs(percentage - self.last_battery_percentage) > 1.0):

            self.last_battery_percentage = percentage

            # Determine battery health status
            if percentage > 50.0:
                battery_status = 'OK'
            elif percentage > 20.0:
                battery_status = 'LOW'
            else:
                battery_status = 'CRITICAL'

            # Publish system status string
            status_msg = String()
            status_msg.data = (
                f'battery:{percentage:.1f}%:'
                f'{battery_status}|'
                f'voltage:{msg.voltage:.2f}V'
            )
            self.status_pub.publish(status_msg)

            self.get_logger().info(
                f'DT state update — battery: {percentage:.1f}% '
                f'({battery_status}), voltage: {msg.voltage:.2f}V'
            )


def main(args=None):
    rclpy.init(args=args)
    node = TwinStateMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()