import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RipenessDecisionNode(Node):
    def __init__(self):
        super().__init__('ripeness_decision_node')

        # Listens to sensor readings from the DT
        self.subscription_ = self.create_subscription(
            String,
            '/aurafarm/ripeness_data',
            self.on_ripeness_data,
            10
        )

        # Publishes harvest decision back to navigation node
        self.publisher_ = self.create_publisher(
            String,
            '/aurafarm/harvest_decision',
            10
        )

        self.get_logger().info('RipenessDecisionNode started')

    def on_ripeness_data(self, msg: String):
        # Parse "crop_id:ripeness"
        parts = msg.data.split(':')
        if len(parts) != 2:
            return

        crop_id = int(parts[0])
        ripeness = parts[1]

        # Decision logic — only harvest fully ripe (green) crops
        if ripeness == 'green':
            decision = 'HARVEST'
        else:
            decision = 'SKIP'

        # Publish decision
        decision_msg = String()
        decision_msg.data = f'{crop_id}:{decision}'
        self.publisher_.publish(decision_msg)

        self.get_logger().info(
            f'Crop {crop_id + 1} is {ripeness} → {decision}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = RipenessDecisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()