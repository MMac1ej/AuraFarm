import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# Default thresholds — used if farmer just presses Enter
DEFAULT_THRESHOLDS = {
    'A': 0.8,
    'B': 0.9,
}

# Plant type names for display
PLANT_TYPE_NAMES = {
    'A': 'Zone A (Strawberry)',
    'B': 'Zone B (Blueberry)',
}


class FarmerInputNode(Node):
    def __init__(self):
        super().__init__('farmer_input_node')

        self.publisher_ = self.create_publisher(
            String,
            '/aurafarm/farmer_thresholds',
            10
        )

        self.get_logger().info('FarmerInputNode started')

    def prompt_and_publish(self):
        print('\n' + '='*50)
        print('  AuraFarm — Berry Harvesting System')
        print('  Farmer Threshold Configuration')
        print('='*50)
        print('Enter desired ripeness threshold for each plant type.')
        print('Scale: 0.0 (no fruit) → 1.0 (fully ripe)')
        print('Press Enter to use default value.\n')

        thresholds = {}

        for plant_type, name in PLANT_TYPE_NAMES.items():
            default = DEFAULT_THRESHOLDS[plant_type]
            while True:
                try:
                    raw = input(
                        f'  {name} threshold '
                        f'(default {default}): '
                    ).strip()

                    if raw == '':
                        value = default
                    else:
                        value = float(raw)

                    if 0.0 <= value <= 1.0:
                        thresholds[plant_type] = value
                        break
                    else:
                        print('  ⚠ Please enter a value between 0.0 and 1.0')

                except ValueError:
                    print('  ⚠ Invalid input — please enter a number')

        print('\n' + '-'*50)
        print('  Confirmed thresholds:')
        for plant_type, value in thresholds.items():
            print(f'    {PLANT_TYPE_NAMES[plant_type]}: {value}')
        print('-'*50)

        confirm = input('\n  Start harvesting tour? (y/n): ').strip().lower()
        if confirm != 'y':
            print('  Cancelled. Restart to configure again.')
            return False

        # Publish thresholds — format: "A:0.8,B:0.9"
        threshold_str = ','.join(
            f'{k}:{v}' for k, v in thresholds.items()
        )
        msg = String()
        msg.data = threshold_str
        self.publisher_.publish(msg)

        self.get_logger().info(
            f'Thresholds published: {threshold_str}'
        )

        print('\n  ✓ Thresholds sent to digital twin.')
        print('  ✓ Starting harvesting tour...\n')

        return True


def main(args=None):
    rclpy.init(args=args)
    node = FarmerInputNode()

    # Give other nodes time to start up
    import time
    time.sleep(2.0)

    success = node.prompt_and_publish()

    if success:
        # Keep node alive briefly so message is received
        import time
        time.sleep(2.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()