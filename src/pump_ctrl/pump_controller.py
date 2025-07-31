
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool

class PumpController(Node):
    def __init__(self):
        super().__init__('pump_controller')
        self.moist_sub = self.create_subscription(Float32, 'sensor/moisture', self.evaluate, 10)
        self.pump_pub = self.create_publisher(Bool, 'pump_ctrl/water', 10)

    def evaluate(self, msg):
        should_water = msg.data < 30.0
        self.pump_pub.publish(Bool(data=should_water))
        self.get_logger().info(f'Pump ON: {should_water}')

def main(args=None):
    rclpy.init(args=args)
    node = PumpController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
