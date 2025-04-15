import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
from pygame import mixer
import pygame
import librosa
from threading import Thread, Lock
import time
import os
import random
import signal
import sys
from ament_index_python.packages import get_package_share_directory

class PuppetController(Node):
    def __init__(self):
        # Generate a unique node name to avoid conflicts
        unique_id = random.randint(1000, 9999)
        super().__init__(f'puppet_controller_{unique_id}')
        
        self.sound_dir = os.path.join(get_package_share_directory('my_puppet'), 'puppetSounds')
        self.get_logger().info(f'Sound directory: {self.sound_dir}')
        # Initialize pygame mixer for audio with proper cleanup
        try:
            if mixer.get_init():
                mixer.quit()  # Ensure any previous mixer is cleaned up
            mixer.init()
            self.get_logger().info('Audio mixer initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize audio mixer: {e}')
        
        # Initialize joint names and publisher
        self.joint_names = ['head_joint', 'left_arm_joint', 'right_arm_joint', 
                           'left_leg_joint', 'right_leg_joint']
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        
        # Thread safety for position updates
        self.position_lock = Lock()
        
        # Initialize target positions (neutral pose)
        self.target_positions = np.zeros(len(self.joint_names))
        
        # Create a high-frequency timer for continuous publishing (50Hz for very smooth motion)
        # Higher frequency ensures there are no gaps in publishing
        self.timer = self.create_timer(0.01, self.publish_joint_states)
        
        # Set up proper shutdown handling
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # Scene control
        self.is_scene_running = False
        self.scene_thread = None
        self.auto_start_count = 0
        
        # Create a countdown timer to auto-start the show
        self.create_timer(1.0, self.countdown_to_start)
        
        # Log initialization
        self.get_logger().info('Puppet Controller initialized with ID: ' + str(unique_id))
        self.get_logger().info('Show will start automatically in 5 seconds...')
        
        # Keep track of the last timestamp we published
        self.last_publish_time = self.get_clock().now()
        
        # Flag to indicate we're actively running
        self.running = True

    def countdown_to_start(self):
        """Countdown timer to start the puppet show automatically"""
        if not self.running:
            return
            
        self.auto_start_count += 1
        
        if self.auto_start_count <= 5:
            self.get_logger().info(f"Starting show in {5 - self.auto_start_count + 1} seconds...")
        
        if self.auto_start_count == 5:
            if not self.is_scene_running:
                self.get_logger().info("Starting puppet show now!")
                self.scene_thread = Thread(target=self.run_scenes)
                self.scene_thread.daemon = True
                self.scene_thread.start()

    def signal_handler(self, sig, frame):
        """Handle clean shutdown on SIGINT"""
        self.get_logger().info("Shutdown signal received, cleaning up...")
        self.running = False
        if mixer.get_init():
            mixer.quit()
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    def reset_positions(self):
        """Reset puppet to neutral position"""
        with self.position_lock:
            # Default position in degrees (will be converted to radians when publishing)
            self.target_positions = np.zeros(len(self.joint_names))
        self.get_logger().info("Reset puppet to neutral position")

    def publish_joint_states(self):
        """Publishes the current joint states at a fixed rate"""
        if not self.running:
            return
            
        current_time = self.get_clock().now()
        
        # Create message with thread safety
        with self.position_lock:
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = current_time.to_msg()
            joint_state_msg.name = self.joint_names
            # Convert degrees to radians for publishing
            radians_positions = np.radians(self.target_positions)
            joint_state_msg.position = radians_positions.tolist()
        
        # Publish the message
        self.publisher.publish(joint_state_msg)
        self.last_publish_time = current_time

    def send_angles(self, angles, duration=1.0):
        """Sets target angles in degrees and waits for specified duration"""
        # Thread-safe update of the target positions
        with self.position_lock:
            self.target_positions = np.array(angles, dtype=float)
        
        self.get_logger().info(f"Setting joint angles to: {angles} degrees")
        
        # Wait for the specified duration
        # This is OK in the scene thread as the publish timer continues to run
        start_time = time.time()
        while time.time() - start_time < duration and self.running:
            time.sleep(0.005)  # Short sleep to allow for interrupt

    def play_audio(self, file_name):
        """Plays audio file with proper path resolution"""
        try:
            full_path = os.path.join(self.sound_dir, file_name)
            if not os.path.exists(full_path):
                self.get_logger().error(f"Audio file not found: {full_path}")
                return
                
            sound = mixer.Sound(full_path)
            sound.play()
            self.get_logger().info(f"Playing audio: {full_path}")
        except Exception as e:
            self.get_logger().error(f"Error playing audio: {e}")

    def run_scenes(self):
        """Run scenes in a separate thread to avoid blocking ROS callbacks"""
        if self.is_scene_running:
            self.get_logger().info("Scenes already running!")
            return
            
        self.is_scene_running = True
        
        try:
            # Initial reset to known position
            self.reset_positions()
            time.sleep(1.0)  # Give time to reach starting position
            
            # # Scene 1
            # self.get_logger().info("Running Scene 1")
            # self.scene_1()
            
            # # Pause between scenes
            # time.sleep(2.0)
            
            # # Scene 2
            # self.get_logger().info("Running Scene 2")
            # self.scene_2()

            # # Pause between scenes
            # time.sleep(2.0)
            
            # # Scene 6
            # self.get_logger().info("Running Scene 6")
            # self.scene_6()

            # # Pause between scenes
            # time.sleep(2.0)
            
            # # # Scene 4
            # self.get_logger().info("Running Scene 5")
            # self.scene_4()

            # # Pause between scenes
            # time.sleep(2.0)
            
            # # Scene 5
            # self.get_logger().info("Running Scene 5")
            # self.scene_5()
            # time.sleep(2.0)

            # self.get_logger().info("Mtr type beat")
            # self.dance("mtr_type_beat.mp3")
            # time.sleep(2.0)

            # self.get_logger().info("Fredie v jason")
            # self.dance("FREDIE_V_JASOM_SAMPLE.mp3")
            # time.sleep(2.0)

            # self.get_logger().info("Nefarious")
            # self.dance("nefarious.mp3")
            # time.sleep(2.0)

            # self.get_logger().info("Prabhas peirre")
            # self.dance("prabhas_pierre.mp3")
            # time.sleep(2.0)

            # self.get_logger().info("Sound cloud beat")
            # self.dance("soundcloudbeat_2.wav")
            # time.sleep(2.0)

            self.get_logger().info("Slow")
            self.dance("slow.mp3")
            time.sleep(2.0)
            
            # # Final reset
            time.sleep(1.0)
            self.reset_positions()
            
            self.get_logger().info("All scenes completed")
        except Exception as e:
            self.get_logger().error(f"Error during scene execution: {e}")
        finally:
            self.is_scene_running = False

    def scene_1(self):
        if not self.running:
            return
            
        self.get_logger().info("Scene 1")
        self.send_angles([15, 0, 0, 0, 0])  # Start from neutral
        
        # First movement
        self.send_angles([15, 60, -60, 0, 90])
        self.play_audio("scene1_line1.mp3")
        self.get_logger().info("Whoa.....")
        self.send_angles([15, 60, -60, 0, 90], 1.5)  # Hold position during audio
        
        # More movements
        self.send_angles([15, 0, 0, 0, 0], 0.7)
        self.send_angles([15, 60, -60, 0, 0], 0.7)
        self.send_angles([15, 0, 0, 0, 0], 0.7)
        self.send_angles([15, 60, -60, 0, 0], 0.7)
        self.send_angles([15, 0, 0, 0, 0], 0.7)
        
        # Second audio line
        self.play_audio("scene1_line2.mp3")
        self.get_logger().info("Haha...")
        self.send_angles([15, 0, 0, 0, 0], 1.0)  # Hold position during audio
        
        # Final movements for scene 1
        self.send_angles([15, 0, -60, 0, 0], 0.7)
        self.send_angles([15, 60, 0, 0, 0], 0.7)
        self.send_angles([15, 0, 0, 0, 90], 0.7)
        self.send_angles([15, 0, 0, 0, 0], 0.7)

    def scene_2(self):
        if not self.running:
            return
            
        self.get_logger().info("Scene 2")
        
        # First audio and movement
        self.play_audio("scene2_line1.mp3")
        self.get_logger().info("Okay its time.....")
        self.send_angles([15, 0, 0, 0, 0], 1.0)  # Hold position during audio
        
        # Initial movements
        self.send_angles([15, 0, 0, 90, 0], 0.7)
        self.send_angles([15, 0, 0, 0, 90], 0.7)
        self.send_angles([15, 0, 0, 0, 0], 0.7)
        
        # Second audio
        self.play_audio("scene2_line2.mp3")
        self.get_logger().info("Walking is.....")
        self.send_angles([15, 0, 0, 0, 0], 1.5)  # Hold position during audio
        
        # Pause
        self.send_angles([15, 0, 0, 0, 0], 2.0)
        
        # Third audio
        self.play_audio("scene2_line3.mp3")
        self.get_logger().info("Look at .......")
        self.send_angles([15, 0, 0, 0, 0], 1.0)  # Hold position during audio
        
        # Walking motion
        for _ in range(3):  # Repeat walking cycle 3 times
            if not self.running:
                break
            self.send_angles([15, 0, 0, 90, 0], 0.5)
            self.send_angles([15, 0, 0, 0, 90], 0.5)

    def scene_4(self):
        if not self.running:
            return
            
        self.get_logger().info("Scene 4")
        
        # First audio and movement
        self.play_audio("scene4_line1.mp3")
        self.get_logger().info("Ooh an obstacle.....")
        self.send_angles([15, 0, 0, 0, 0], 2.0)  # Hold position during audio

        # Jump movements
        self.send_angles([15, 150, -150, 90, 90], 0.5)
        self.send_angles([15, 0, 0, 0, 0], 0.5)


        # Second audio
        self.play_audio("scene4_line2.mp3")
        self.get_logger().info("Woohoo.....")
        
        self.send_angles([15, 0, 0, 0, 0], 6.0)  # Hold position during audio

    def scene_5(self):
        if not self.running:
            return
            
        self.get_logger().info("Scene 5")
        
        # First audio and movement
        self.play_audio("scene5_line1.mp3")
        self.get_logger().info("Oh no its.....")
        self.send_angles([15, 0, 0, 0, 0], 4.0)  # Hold position during audio

        
        # Second audio
        self.play_audio("scene5_line2.mp3")
        self.get_logger().info("Okay friends.....")
        
        # Waving motion
        for _ in range(3):  # Repeat waving cycle 3 times
            if not self.running:
                break
            self.send_angles([15, 0, -150, 0, 0], 1.0)
            self.send_angles([15, 0, 0, 0, 0], 1.0)
    
    def scene_6(self):
        if not self.running:
            return
            
        self.get_logger().info("Scene 6")
        
        # First audio and movement
        self.play_audio("scene6_line1.mpeg")
        self.get_logger().info("Fredie v jasom.....")
        self.send_angles([15, 0, 0, 0, 0], 4.0)  # Hold position during audio
        
        # Waving motion
        for _ in range(5): 
            if not self.running:
                break
            self.send_angles([15, 0, -50, 0, 0], 1.0)
            self.send_angles([15, 50, 0, 0, 0], 1.0)
        
        for _ in range(5): 
            if not self.running:
                break
            self.send_angles([15, 0, 0, 90, 0], 1.0)
            self.send_angles([15, 30, 0, 0, 90], 1.0)
        
        for _ in range(5):
            if not self.running:
                break
            self.send_angles([15, 0, -150, 0, 30], 1.0)
            self.send_angles([15, 90, 0, 90, 0], 1.0)
        
        for _ in range(5): 
            if not self.running:
                break
            self.send_angles([15, 0, -150, 90, 0], 1.0)
            self.send_angles([15, 150, 0, 0, 90], 1.0)
        
        for _ in range(2): 
            if not self.running:
                break
            self.send_angles([15, 0, -150, 0, 0], 1.0)
            self.send_angles([15, 0, 0, 90, 0], 1.0)

    def findDelay_music(self, file_name):
        """Calculate delay between dance moves based on music tempo"""
        try:
            audio_path = os.path.join(self.sound_dir, file_name)
            if not os.path.exists(audio_path):
                self.get_logger().error(f"Audio file not found: {audio_path}")
                return 0.5
                
            y, sr = librosa.load(audio_path)
            onset_env = librosa.onset.onset_strength(y=y, sr=sr)
            tempo = librosa.beat.tempo(onset_envelope=onset_env, sr=sr)[0]
            
            return 60.0 / tempo
        except Exception as e:
            self.get_logger().error(f"Error analyzing audio: {e}")
            return 0.5
    
    def dance(self, file_name):
        """Execute dance routine synchronized with music"""
        try:
            delay = self.findDelay_music(file_name)
            
            # Setup music playback
            audio_path = os.path.join(self.sound_dir, file_name)
            pygame.mixer.music.load(audio_path)
            pygame.mixer.music.play()
            
            self.get_logger().info("Dancing to the music...")
            
            # Keep existing dance steps
            dance_steps = [
                lambda: [self.send_angles([15, 60, -180, 0,0], delay), 
                        self.send_angles([15,120, 0, 0, 0], delay)],
                lambda: [self.send_angles([15,0, 0, 90, 0], delay), 
                        self.send_angles([15,120, -120, 0, 90], delay)],
                lambda: [self.send_angles([15,120, 0, 90, 0], delay), 
                        self.send_angles([15,0, -120, 90, 0], delay)],
                lambda: [self.send_angles([15,0, -120, 90, 0], delay), 
                        self.send_angles([15,120, 0, 0, 90], delay)],
                lambda: [self.send_angles([15,0, 0, 0, 90], delay), 
                        self.send_angles([15,0, -120, 0, 0], delay)]
            ]
            
            # Dance while music plays
            while pygame.mixer.music.get_busy() and self.running:
                selected_step = random.choice(dance_steps)
                for _ in range(5):
                    if not self.running or not pygame.mixer.music.get_busy():
                        return
                    selected_step()
                    time.sleep(delay)
                        
        except Exception as e:
            self.get_logger().error(f"Error in dance routine: {e}")
            if pygame.mixer.music.get_busy():
                pygame.mixer.music.stop()


def main(args=None):
    rclpy.init(args=args)
    puppet_controller = PuppetController()

    try:
        rclpy.spin(puppet_controller)  # Keeps node alive and processes timers
    except KeyboardInterrupt:
        puppet_controller.get_logger().info("Shutting down...")
    finally:
        # Cleanup
        puppet_controller.running = False
        if mixer.get_init():
            mixer.quit()
        puppet_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()