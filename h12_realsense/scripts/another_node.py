import rclpy
from rclpy.node import Node

def main(args=None):
    """
    Main function for the another_node script.
    Initializes rclpy, creates a simple node, logs a message, and shuts down.
    """
    rclpy.init(args=args)
    
    # Create a simple node
    node = Node('another_node')
    
    # Log an informational message
    node.get_logger().info('Hello from another_node!')
    node.get_logger().info('Hello from a print statement added to another_node after building!')

    
    # In a real application that needs to stay running, you would use rclpy.spin(node)
    # For this example, we just clean up and exit.
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()