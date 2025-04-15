import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time

class PuppetController(Node):
    def __init__(self):
        super().__init__('puppet_controller')
        
        # Initialize joint names and publisher
        self.joint_names = ['head_joint', 'left_arm_joint', 'right_arm_joint', 
                            'left_leg_joint', 'right_leg_joint']
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        
        # Initialize target positions
        self.target_positions = np.zeros(len(self.joint_names))
        
        # Log initialization
        self.get_logger().info('Puppet Controller initialized')
    
    def send_angles(self, angles, duration=2.0): # added duration
        target_angles = np.radians(angles)  # Convert degrees to radians
        start_positions = self.target_positions.copy()
        steps = 50 # Increase steps for smoother movement.
        
        for step in range(steps + 1):
            fraction = step / steps
            interpolated_angles = start_positions + fraction * (target_angles - start_positions)
            
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = self.joint_names
            joint_state_msg.position = interpolated_angles.tolist()
            self.publisher.publish(joint_state_msg)
            time.sleep(duration / steps) # control speed
            
        self.target_positions = target_angles # update target positions
        self.get_logger().info(f"Publishing joint states: angles={angles}")

    def scene_1(self):
        self.get_logger().info("Scene 1: Awakening")
        self.send_angles([0, 0, 0, 0, 0], 3.0)  # Initial position, slow
        self.send_angles([15, 30, -30, -10, 10], 4.0) # head up, stretch arms and legs, slow
        self.send_angles([20, 60, -60, 20, -20], 4.0) # waving hands and wiggling legs, slow
        self.send_angles([25, 90, -90, 0, 0], 4.0) # bend elbows and wiggle fingers, slow
        self.send_angles([30, 0, 0, 30, -30], 4.0) # lift legs playfully, slow

    def scene_2(self):
        self.get_logger().info("Scene 2: The Wobble Walk")
        self.send_angles([180, 10, -10, 10, -10], 4.0) # stumbles, slow
        self.send_angles([180, 20, -20, 20, -20], 4.0) # arms flail, slow
        self.send_angles([180, 0, 0, 0, 0], 3.0) # balances, slow
        self.send_angles([180, 15, -15, 15, -15], 5.0) # walking confidently, slow
        self.send_angles([180, 0, 0, 0, 0], 2.0) # freeze, slow

    def scene_3(self):
        self.get_logger().info("Scene 3: The Mystery Sound")
        self.send_angles([0, 10, -10, 0, 0], 4.0) # looking around cautiously, slow
        self.send_angles([0, 20, -20, 0, 0], 4.0)
        self.send_angles([0, 0, 0, 0, 0], 3.0) # stomach growls, slow
        self.send_angles([10, 30, -30, 0, 0], 4.0) # searching for food (turn head left), slow
        self.send_angles([-10, 30, -30, 0, 0], 4.0) # searching for food (turn head right), slow

    def scene_4(self):
        self.get_logger().info("Scene 4: The Jumping Test")
        self.send_angles([0, 0, 0, 45, -45], 4.0) # bending knees, slow
        self.send_angles([0, 30, -30, 90, -90], 4.0) # swinging arms and jumping, slow
        self.send_angles([0, 0, 0, 0, 0], 4.0) # landing, slow
        self.send_angles([0, 45, -45, 0, 0], 4.0) # superhero pose, slow

    def scene_5(self):
        self.get_logger().info("Scene 5: The Big Goodbye")
        self.send_angles([0, 45, -45, 0, 0], 4.0) # waving hands, slow
        self.send_angles([0, 90, -90, 0, 0], 4.0)
        self.send_angles([0, 0, 0, 0, 0], 4.0) # bow, slow
        self.send_angles([0, 20, -20, 20, -20], 4.0) # happy dance, slow

def main(args=None):
    rclpy.init(args=args)
    puppet_controller = PuppetController()

    try:
        puppet_controller.scene_1()
        puppet_controller.scene_2()
        puppet_controller.scene_3()
        puppet_controller.scene_4()
        puppet_controller.scene_5()
    except KeyboardInterrupt:
        puppet_controller.get_logger().info("Shutting down...")
    finally:
        puppet_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()