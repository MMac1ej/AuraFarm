import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
import random

# Ripeness levels matching your demo plan
RIPENESS_VALUES = ['green', 'yellow', 'red']

class SimulatedRipenessSensorNode(Node):
    def __init__(self):
        super().__init__('simulated_ripeness_sensor')

        # Publishes ripeness reading when robot arrives at a crop
        self.publisher_ = self.create_publisher(
            String,
            '/aurafarm/ripeness_data',
            10
        )

        # Listens for crop arrival from navigation node
        self.subscription_ = self.create_subscription(
            Int32,
            '/aurafarm/crop_arrival',
            self.on_crop_arrival,
            10
        )

        self.get_logger().info('SimulatedRipenessSensor started')

    def on_crop_arrival(self, msg: Int32):
        crop_id = msg.data

        # Generate random ripeness value
        ripeness = random.choice(RIPENESS_VALUES)

        # Publish as "crop_id:ripeness" string
        data = String()
        data.data = f'{crop_id}:{ripeness}'
        self.publisher_.publish(data)

        self.get_logger().info(
            f'Crop {crop_id + 1} ripeness: {ripeness}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = SimulatedRipenessSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()