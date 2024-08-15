import rclpy
from rclpy.node import Node
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray

class AprilTagMovementNode(Node):
    def __init__(self):
        super().__init__('apriltag_movement_node')

        # Create a subscription to the AprilTag detection topic
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            'apriltag_detections',
            self.listener_callback,
            10
        )
        
        # Initialize publishers for robot movement commands
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def listener_callback(self, msg):
        for detection in msg.detections:
            tag_id = detection.id[0]  # Example: Assuming there's only one ID per detection
            # Process the detection information
            self.move_robot_based_on_detection(tag_id)

    def move_robot_based_on_detection(self, tag_id):
        # Example logic: Move the robot based on detected tag ID
        cmd_msg = Twist()
        if tag_id == 1:
            cmd_msg.linear.x = 0.5  # Move forward
            cmd_msg.angular.z = 0.0
        elif tag_id == 2:
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.5  # Turn
        else:
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0  # Stop
        
        self.cmd_publisher.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagMovementNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

