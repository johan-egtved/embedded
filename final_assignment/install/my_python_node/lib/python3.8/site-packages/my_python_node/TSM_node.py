import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
import os
import numpy as np
import math
import mmap

class RouteLookupNode(Node):
    def __init__(self, bram_device='/dev/bram0', bram_offset=128):
        super().__init__('route_lookup_node')
        
        # BRAM configuration
        self.bram_device = bram_device
        self.bram_offset = bram_offset
        self.last_route_number = None  # Initialize a variable to store the last read value
        # The rest of the __init__ code remains unchanged
        # Open BRAM device
        try:
            self.bram_fd = os.open(self.bram_device, os.O_RDONLY)
            self.bram_mmap = mmap.mmap(self.bram_fd, 0, access=mmap.ACCESS_READ)
        except Exception as e:
            self.get_logger().error(f'Failed to open BRAM device: {e}')
            raise
        
        # Create a publisher to publish the route
        self.route_publisher = self.create_publisher(
            PoseArray,
            'retrieved_route',
            10
        )
        
        # Lookup table (initially blank for routes 0-9)
        # X coordinates
        Rx = 1.0
        Lx = -1.0

        # Y coordinates
        Ty = 10.0
        My = 6.5
        By = 3.0

        self.route_lookup_table = {
            0: [[Rx, Ty],[Rx, My],[Rx, By],[Lx, By],[Lx, My],[Lx, Ty],[Rx, Ty]],  
            1: [[Rx, Ty],[Rx, My],[Rx, By]],  # Example: route for number 1
            2: [[Lx, Ty],[Rx, Ty],[Rx, My],[Lx, My],[Lx, By],[Rx, By]],  # Example: route for number 2
            3: [[Lx, Ty],[Rx, Ty],[Rx, My],[Lx, My],[Rx, Ty]],  # Example: route for number 3
            4: [[Lx, Ty],[Lx, My],[Rx, My],[Rx, Ty],[Rx, My],[Rx, By]],  # Example: route for number 4
            5: [[Rx, Ty],[Lx, Ty],[Lx, My],[Rx, My],[Rx, By],[Lx, By]],  # Example: route for number 5
            6: [[Rx, Ty],[Lx, Ty],[Lx, My],[Rx, My],[Rx, By],[Lx, By],[Lx,My]],  # Example: route for number 6
            7: [[Lx, Ty],[Rx, Ty],[Rx, My],[Rx, By]],  # Example: route for number 7
            8: [[Rx, Ty],[Lx, Ty],[Lx, My],[Rx, My],[Rx, Ty],[Rx, My],[Rx, By],[Lx, By],[Lx, My]],  # Example: route for number 8
            9: [[Rx, Ty],[Lx, Ty],[Lx, My],[Rx, My],[Rx, Ty],[Rx, My],[Rx, By],[Lx, By]]   # Example: route for number 9
        }
        
        # Create a timer to periodically check BRAM
        self.timer = self.create_timer(1.0, self.check_bram)
        
        self.get_logger().info(f'Route Lookup Node initialized. Watching BRAM device: {bram_device}')
    
    def check_bram(self):
        try:
            # Seek to the specified offset
            self.bram_mmap.seek(self.bram_offset)
            
            # Read 4 bytes (32-bit integer)
            route_number_bytes = self.bram_mmap.read(4)
            
            # Convert bytes to integer (assuming little-endian)
            route_number = int.from_bytes(route_number_bytes, byteorder='little')
            
            # Validate route number
            if route_number < 0 or route_number > 9:
                self.get_logger().warn(f'Invalid route number from BRAM: {route_number}. Must be between 0 and 9.')
                return
            
            # Check if the value has changed
            if route_number != self.last_route_number:
                self.last_route_number = route_number  # Update the last route number
                self.lookup_and_publish_route(route_number)  # Perform the route calculation
            
        except Exception as e:
            self.get_logger().error(f'Error reading from BRAM: {e}')
    
    def lookup_and_publish_route(self, route_number):
        # Retrieve the route from the lookup table
        route = self.route_lookup_table.get(route_number, [])
        
        # Create a PoseArray message to publish the route
        route_msg = PoseArray()
        route_msg.header.stamp = self.get_clock().now().to_msg()
        route_msg.header.frame_id = 'map'

        self.get_logger().info(f'Published route for number {route_number}')
        interpolated_route = self.interpolate_route(route)
        self.compute_joint_angles(interpolated_route)
        
    def interpolate_route(self,route):
        """
        Interpolates 10 points between each consecutive pair of points in a route.

        Parameters:
            route (list of lists): A list of [x, y] coordinates representing the route.

        Returns:
            list of lists: A new route with interpolated points.
        """
        if len(route) < 2:
            raise ValueError("Route must contain at least two points to interpolate.")

        interpolated_route = []

        for i in range(len(route) - 1):
            x_start, y_start = route[i]
            x_end, y_end = route[i + 1]

            # Create 10 interpolated points between the current point and the next
            x_interp = np.linspace(x_start, x_end, 12)  # 12 includes start and end points
            y_interp = np.linspace(y_start, y_end, 12)

            # Add the interpolated points to the new route
            for j in range(len(x_interp) - 1):  # Exclude the last point to avoid duplication
                interpolated_route.append([x_interp[j], y_interp[j]])

        # Add the last point of the route to the interpolated route
        interpolated_route.append(route[-1])

        return interpolated_route

    def compute_joint_angles(self,positions):
        """
        Compute the joint angles (q1, q2) for a 2D robotic arm with two links.

        Parameters:
        - positions: Array of end-effector positions, each a tuple (x, y).
        - L1: Length of the first link.
        - L2: Length of the second link.

        Returns:
        - Array of joint angles [(q1, q2), ...], where q1 and q2 are in radians.
        """
        L1 = 6.4
        L2 = 7.0
        joint_angles = []
        self.get_logger().info(f'test')
        counter = 0
        for x, y in positions:
            # Calculate q2
            d = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
            if d < -1 or d > 1:
                raise ValueError(f"Position ({x}, {y}) is out of reach for the given arm lengths.")

            q2 = np.arctan2(np.sqrt(1 - d**2), d)  # Positive elbow solution

            # Calculate q1
            k1 = L1 + L2 * np.cos(q2)
            k2 = L2 * np.sin(q2)
            q1 = np.arctan2(y, x) - np.arctan2(k2, k1)
            if counter ==0:
                counter+=1
                posx=L1*np.cos(q1)+L2*np.cos(q1+q2)
                posy=L1*np.sin(q1)+L2*np.sin(q1+q2)
                self.get_logger().info(f'Positions {posx}, {posy}')

            joint_angles.append((math.degrees(q1), math.degrees(q2)))
        self.get_logger().info(f'Angles {joint_angles}')

    def __del__(self):
        # Clean up resources
        if hasattr(self, 'bram_mmap'):
            self.bram_mmap.close()
        if hasattr(self, 'bram_fd'):
            os.close(self.bram_fd)

def main(args=None):
    # Initialize ROS2 communication
    rclpy.init(args=args)
    
    # Create the node with BRAM configuration
    route_lookup_node = RouteLookupNode(
        bram_device='/dev/bram0',  # Adjust the BRAM device path as needed
        bram_offset=0  # Adjust the offset if necessary
    )
    
    try:
        # Spin the node to keep it running and checking BRAM periodically
        rclpy.spin(route_lookup_node)
    except Exception as e:
        route_lookup_node.get_logger().error(f'Error: {e}')
    finally:
        # Shutdown the node
        route_lookup_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()