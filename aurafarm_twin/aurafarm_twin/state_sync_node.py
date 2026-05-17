"""Mirror battery, sensor health, and operating mode between sides.

Satisfies project requirement #2 (state synchronization).

Design note: instead of pass-through republishing, we cache the latest message
from each physical topic and republish on a fixed timer. That way the twin UI
sees a steady update stream even if the physical publisher is bursty or
temporarily silent, and we have a single place to attach health checks.
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus


class StateSyncNode(Node):
    def __init__(self) -> None:
        super().__init__('aurafarm_state_sync')

        self.declare_parameter('sync_rate_hz', 5.0)
        self.declare_parameter('low_battery_threshold', 0.20)  # 20 %

        # Last-known messages (None until the physical robot publishes).
        self._last_battery: BatteryState | None = None
        self._last_diag: DiagnosticArray | None = None
        self._last_mode: String | None = None

        # --- Publishers (twin side) ---
        self._battery_pub = self.create_publisher(
            BatteryState, '/sim/battery_state', 10)
        self._diag_pub = self.create_publisher(
            DiagnosticArray, '/sim/diagnostics', 10)
        self._mode_pub = self.create_publisher(String, '/sim/mode', 10)

        # --- Subscribers (physical side) ---
        self.create_subscription(
            BatteryState, '/physical/battery_state',
            self._on_battery, 10)
        self.create_subscription(
            DiagnosticArray, '/physical/diagnostics',
            self._on_diag, 10)
        self.create_subscription(
            String, '/physical/mode', self._on_mode, 10)

        rate = float(self.get_parameter('sync_rate_hz').value)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'State sync up at {rate:.1f} Hz '
            '(battery, diagnostics, mode -> /sim).')

    # ----- ingest --------------------------------------------------------

    def _on_battery(self, msg: BatteryState) -> None:
        self._last_battery = msg
        threshold = float(self.get_parameter('low_battery_threshold').value)
        if 0.0 <= msg.percentage < threshold:
            self.get_logger().warn(
                f'Low battery: {msg.percentage * 100.0:.1f}% '
                f'(threshold {threshold * 100.0:.0f}%).')

    def _on_diag(self, msg: DiagnosticArray) -> None:
        self._last_diag = msg
        for status in msg.status:
            if status.level >= DiagnosticStatus.WARN:
                level = 'ERROR' if status.level >= DiagnosticStatus.ERROR \
                    else 'WARN'
                self.get_logger().warn(
                    f'Sensor diagnostic {level}: {status.name} -> '
                    f'{status.message}')

    def _on_mode(self, msg: String) -> None:
        if self._last_mode is None or self._last_mode.data != msg.data:
            self.get_logger().info(f'Mode change: {msg.data}')
        self._last_mode = msg

    # ----- mirror --------------------------------------------------------

    def _tick(self) -> None:
        if self._last_battery is not None:
            self._battery_pub.publish(self._last_battery)
        if self._last_diag is not None:
            self._diag_pub.publish(self._last_diag)
        if self._last_mode is not None:
            self._mode_pub.publish(self._last_mode)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StateSyncNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
