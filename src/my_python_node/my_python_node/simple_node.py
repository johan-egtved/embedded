import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray, Pose
import numpy as np
import math
import time
import os

class IntegratedNumberDrawer(Node):
    def __init__(self, input_file_path):
        super().__init__('integrated_number_drawer')
        
        # Initialize publisher for joint states
        self.joint_publisher = self.create_publisher(
            JointState,
            'joint_states',
            10
        )
        
        # Input file path
        self.input_file_path = input_file_path
        
        # Robot arm parameters
        self.L1 = 6.4  # Length of first link
        self.L2 = 7.0  # Length of second link
        
        # Initialize route lookup table
        self.initialize_route_table()
        
        # Drawing state
        self.drawing = False
        
        self.get_logger().info(f'Integrated Number Drawer initialized. Watching file: {input_file_path}')
        
    def initialize_route_table(self):
        """Initialize the route lookup table with coordinates for numbers 0-9."""
        # Define standard coordinates
        Rx = 1.0
        Lx = -1.0
        Ty = 10.0
        My = 6.5
        By = 3.0
        
        self.route_lookup_table = {
            0: [[Rx, Ty],[Rx, My],[Rx, By],[Lx, By],[Lx, My],[Lx, Ty],[Rx, Ty]],
            1: [[Rx, Ty],[Rx, My],[Rx, By]],
            2: [[Lx, Ty],[Rx, Ty],[Rx, My],[Lx, My],[Lx, By],[Rx, By]],
            3: [[Lx, Ty],[Rx, Ty],[Rx, My],[Lx, My],[Rx, Ty]],
            4: [[Lx, Ty],[Lx, My],[Rx, My],[Rx, Ty],[Rx, My],[Rx, By]],
            5: [[Rx, Ty],[Lx, Ty],[Lx, My],[Rx, My],[Rx, By],[Lx, By]],
            6: [[Rx, Ty],[Lx, Ty],[Lx, My],[Rx, My],[Rx, By],[Lx, By],[Lx,My]],
            7: [[Lx, Ty],[Rx, Ty],[Rx, My],[Rx, By]],
            8: [[Rx, Ty],[Lx, Ty],[Lx, My],[Rx, My],[Rx, Ty],[Rx, My],[Rx, By],[Lx, By],[Lx, My]],
            9: [[Rx, Ty],[Lx, Ty],[Lx, My],[Rx, My],[Rx, Ty],[Rx, My],[Rx, By],[Lx, By]]
        }

    def check_input_file(self):
        """Read and process the input file containing the number to draw."""
        try:
            if not os.path.exists(self.input_file_path):
                self.get_logger().warn(f'Input file not found: {self.input_file_path}')
                return
            
            with open(self.input_file_path, 'r') as file:
                number_str = file.read().strip()
            
            try:
                number = int(number_str)
                if number < 0 or number > 9:
                    self.get_logger().warn(f'Invalid number: {number}. Must be between 0 and 9.')
                    return

                self.process_and_draw_number(number)


                
            except ValueError:
                self.get_logger().warn(f'Invalid input: {number_str}. Must be a number.')
                
        except Exception as e:
            self.get_logger().error(f'Error processing input file: {e}')

    def interpolate_route(self, route):
        """Interpolate points between waypoints for smoother movement."""
        if len(route) < 2:
            return route

        interpolated_route = []
        
        for i in range(len(route) - 1):
            x_start, y_start = route[i]
            x_end, y_end = route[i + 1]
            
            x_interp = np.linspace(x_start, x_end, 10)
            y_interp = np.linspace(y_start, y_end, 10)
            
            for j in range(len(x_interp) - 1):
                interpolated_route.append([x_interp[j], y_interp[j]])

        interpolated_route.append(route[-1])

        return interpolated_route

    def compute_joint_angles(self, x, y):
        """Compute joint angles for a given position."""

        try:
            # Calculate distance to target
            d = (x**2 + y**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
            if d < -1 or d > 1:
                raise ValueError(f"Position ({x}, {y}) is out of reach")

            # Calculate q2 (elbow angle)
            q2 = np.arctan2(np.sqrt(1 - d**2), d)

            # Calculate q1 (shoulder angle)
            k1 = self.L1 + self.L2 * np.cos(q2)
            k2 = self.L2 * np.sin(q2)
            q1 = np.arctan2(y, x) - np.arctan2(k2, k1)
            q1_point = math.degrees(q1)*(620/180)+200
            q2_point = math.degrees(q2)*(620/180)+200
            return q1_point, q2_point
            
        except Exception as e:
            self.get_logger().error(f'Error computing joint angles: {e}')
            return None

    def publish_joint_state(self, angle1, angle2):
        """Publish joint states using the standard JointState message."""
        try:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['joint1', 'joint2']  # Names of your joints
            msg.position = [math.radians(angle1), math.radians(angle2)]  # Convert to radians
            msg.velocity = []
            msg.effort = []
            
            self.joint_publisher.publish(msg)
            self.get_logger().info(f'Published joint states: ({angle1}, {angle2})')
            
        except Exception as e:
            self.get_logger().error(f'Error publishing joint states: {e}')

    def process_and_draw_number(self, number):
        """Process the route for a number and execute the drawing."""
        try:
            # Get the route for the number
            route = self.route_lookup_table.get(number, [])
            if not route:
                self.get_logger().error(f'No route found for number {number}')
                return

            # Interpolate the route for smoother movement
            interpolated_route = self.interpolate_route(route)
            
            # Execute the drawing
            self.drawing = True
            self.get_logger().info(f'Starting to draw number {number}')
            
            for point in interpolated_route:
                if not self.drawing:  # Check if we should stop
                    break
                    
                # Compute joint angles for this point
                angles = self.compute_joint_angles(point[0], point[1])
                if angles is None:
                    continue
                    
                # Publish joint states
                self.publish_joint_state(angles[0], angles[1])
                
                # Small delay for movement
                time.sleep(0.1)
            
            self.drawing = False
            self.get_logger().info(f'Finished drawing number {number}')
            
        except Exception as e:
            self.get_logger().error(f'Error in drawing process: {e}')
            self.drawing = False

def main(args=None):
    rclpy.init(args=args)
    
    input_file_path = 'int_file.txt'
    node = IntegratedNumberDrawer(input_file_path)
    
    try:
        node.check_input_file()
        rclpy.spin_once(node)  # Process any pending callbacks
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()