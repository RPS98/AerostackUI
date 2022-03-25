import rclpy
from rclpy.node import Node
import threading
from time import sleep

from as2_msgs.msg import PlatformInfo
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

# Create a ROS2 node, subscribe to a topic, and publish the same message to another topic.


class RepublishNode(Node):

    def __init__(self, type_msg, original_topic, new_topic):
        name = 'republish_' + original_topic + '_to_' + new_topic
        # Replace '/' with '_' in name
        name = name.replace('/', '_')

        super().__init__(name)
        self.new_topic = new_topic
        self.subscription = self.create_subscription(
            type_msg, original_topic, self.callback, 10)
        self.publisher = self.create_publisher(type_msg, new_topic, 10)

    def callback(self, msg):
        self.publisher.publish(msg)

    def shutdown(self):
        self.destroy_subscription(self.subscription)
        self.destroy_node()
        self.get_logger().info('Shutting down')


class RepublishManager():
    def __init__(self, topics_list):
        self.executor = rclpy.executors.MultiThreadedExecutor()
        
        self.node_list = []
        for topic in topics_list:
            node = RepublishNode(topic[0], topic[1], topic[2])
            self.node_list.append(node)
            self.executor.add_node(node)

        self.keep_running = True
        self.spin_thread = threading.Thread(target=self.auto_spin, daemon=False)
        self.spin_thread.start()

    def auto_spin(self):
        while rclpy.ok() and self.keep_running:
            # rclpy.spin_once(self)
            self.executor.spin()
            sleep(0.1)
        self.shutdown()

    def shutdown(self):
        for node in self.node_list:
            node.shutdown()
        self.keep_running = False
        self.spin_thread.join()
        rclpy.shutdown()
        print("Clean exit")


if __name__ == '__main__':
    rclpy.init()

    topics_list = [
        [PlatformInfo, '/drone_sim_rafa_0/platform/info',
            '/drone_sim_rafa_1/platform/info'],
        [Odometry,     '/drone_sim_rafa_0/self_localization/odom',
            '/drone_sim_rafa_1/self_localization/odom'],
        [NavSatFix,    '/drone_sim_rafa_0/platform/gps',
            '/drone_sim_rafa_1/platform/gps'],
    ]

    minimal_publisher = RepublishManager(topics_list)
