import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import mmap
import os

class BRAMReaderNode(Node):
    def __init__(self):
        super().__init__('bram_reader_node')
        
        # Declare parameters for BRAM configuration
        self.declare_parameter('bram_device', '/dev/bram0')  # Default BRAM device path
        self.declare_parameter('bram_offset', 128)  # Offset within BRAM to read from
        self.declare_parameter('publish_rate', 1.0)  # Default publish rate (Hz)
        
        # Get parameter values
        bram_device = self.get_parameter('bram_device').value
        bram_offset = self.get_parameter('bram_offset').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Create publisher
        self.publisher_ = self.create_publisher(Int32, 'bram_value', 10)
        
        # Create timer to periodically read and publish BRAM value
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        
        # Open BRAM device
        try:
            self.bram_fd = os.open(bram_device, os.O_RDONLY)
            self.bram_mmap = mmap.mmap(self.bram_fd, 0, access=mmap.ACCESS_READ)
        except Exception as e:
            self.get_logger().error(f'Failed to open BRAM device: {e}')
            raise
    
    def timer_callback(self):
        try:
            # Seek to the specified offset
            self.bram_mmap.seek(self.get_parameter('bram_offset').value)
            
            # Read 4 bytes (32-bit integer)
            bram_value_bytes = self.bram_mmap.read(4)
            
            # Convert bytes to integer (assuming little-endian)
            bram_value = int.from_bytes(bram_value_bytes, byteorder='little')
            
            # Create message and publish
            msg = Int32()
            msg.data = bram_value
            self.publisher_.publish(msg)
            
            # Log the published value
            self.get_logger().info(f'Publishing BRAM value: {bram_value}')
        
        except Exception as e:
            self.get_logger().error(f'Error reading from BRAM: {e}')
    
    def __del__(self):
        # Clean up resources
        if hasattr(self, 'bram_mmap'):
            self.bram_mmap.close()
        if hasattr(self, 'bram_fd'):
            os.close(self.bram_fd)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        bram_reader_node = BRAMReaderNode()
        rclpy.spin(bram_reader_node)
    except Exception as e:
        print(f'Error initializing BRAM reader node: {e}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()