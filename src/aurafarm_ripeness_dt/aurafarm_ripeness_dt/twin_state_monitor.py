import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String


class TwinStateMonitorNode(Node):
    def __init__(self):
        super().__init__('twin_state_monitor')

        self.battery_sub = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.on_battery_state,
            10
        )

        self.battery_pub = self.create_publisher(
            BatteryState,
            '/aurafarm/dt_battery_state',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/aurafarm/dt_system_status',
            10
        )

        self.last_logged_percentage = None
        self.filtered_percentage = None

        # Smaller value = smoother but slower response
        self.smoothing_alpha = 0.1

        self.get_logger().info('TwinStateMonitor started')

    def voltage_to_percentage(self, voltage):
        # TurtleBot burger battery approximation
        min_voltage = 11.0
        max_voltage = 12.6

        percentage = (voltage - min_voltage) / (max_voltage - min_voltage) * 100.0
        return max(0.0, min(100.0, percentage))

    def on_battery_state(self, msg: BatteryState):
        # Mirror original battery state to DT
        self.battery_pub.publish(msg)

        raw_percentage = self.voltage_to_percentage(msg.voltage)

        # Exponential moving average filter
        if self.filtered_percentage is None:
            self.filtered_percentage = raw_percentage
        else:
            self.filtered_percentage = (
                self.smoothing_alpha * raw_percentage +
                (1.0 - self.smoothing_alpha) * self.filtered_percentage
            )

        percentage = self.filtered_percentage

        # Only log when displayed percentage changes significantly
        if (
            self.last_logged_percentage is None or
            abs(percentage - self.last_logged_percentage) >= 1.0
        ):
            self.last_logged_percentage = percentage

            if percentage > 50.0:
                battery_status = 'OK'
            elif percentage > 20.0:
                battery_status = 'LOW'
            else:
                battery_status = 'CRITICAL'

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