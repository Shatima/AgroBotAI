
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32

class RecommenderNode(Node):
    def __init__(self):
        super().__init__('recommender_node')
        self.seed_pub = self.create_publisher(String, 'recommend/seed', 10)
        self.fert_pub = self.create_publisher(String, 'recommend/fertilizer', 10)
        self.moist_sub = self.create_subscription(Float32, 'sensor/moisture', self.process, 10)

    def process(self, msg):
        seed = "Maize" if msg.data < 50 else "Rice"
        fertilizer = "Urea" if msg.data < 50 else "NPK 20-10-10"
        self.seed_pub.publish(String(data=seed))
        self.fert_pub.publish(String(data=fertilizer))
        self.get_logger().info(f'Recommended: {seed} & {fertilizer}')

def main(args=None):
    rclpy.init(args=args)
    node = RecommenderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
