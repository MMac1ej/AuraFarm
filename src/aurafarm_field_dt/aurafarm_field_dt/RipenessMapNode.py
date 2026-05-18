import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Same positions as navigation node
CROP_POSITIONS = [
    (-1.5, -1.0),
    ( 0.5,  1.0),
    ( 0.9, -1.0),
    (-2.0, -3.0),
    (-2.4, -0.5),
    (-1.0, -1.2),
    (-1.0, -3.0),
]

class RipenessMapNode(Node):
    def __init__(self):
        super().__init__('ripeness_map_node')

        # Tracks ripeness state of all crops
        self.crop_states = {i: 'unknown' for i in range(len(CROP_POSITIONS))}

        # Subscribes to sensor readings
        self.subscription_ = self.create_subscription(
            String,
            '/aurafarm/ripeness_data',
            self.on_ripeness_data,
            10
        )

        # Publishes full crop map every second
        self.publisher_ = self.create_publisher(
            String,
            '/aurafarm/crop_map',
            10
        )
        self.timer_ = self.create_timer(1.0, self.publish_crop_map)

        self.get_logger().info('RipenessMapNode started')

    def on_ripeness_data(self, msg: String):
        # Parse "crop_id:ripeness" format
        parts = msg.data.split(':')
        if len(parts) != 2:
            return

        crop_id = int(parts[0])
        ripeness = parts[1]

        # Update crop state in the digital twin
        self.crop_states[crop_id] = ripeness
        self.get_logger().info(
            f'DT updated: crop {crop_id + 1} is {ripeness}'
        )

    def publish_crop_map(self):
        # Publish full map state as comma separated string
        # Format: "0:green,1:yellow,2:unknown,..."
        map_str = ','.join(
            f'{k}:{v}' for k, v in self.crop_states.items()
        )
        msg = String()
        msg.data = map_str
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RipenessMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()