#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import curses
import time
import numpy as np

class WheelController:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('wheel_keyboard_controller', anonymous=True)

        # Publishers for wheel velocity commands
        self.wheel_publishers = {
            'front_left': rospy.Publisher('/omni_car/front_left_joint_velocity_controller/command', 
                                          Float64, queue_size=10),
            'front_right': rospy.Publisher('/omni_car/front_right_joint_velocity_controller/command', 
                                           Float64, queue_size=10),
            'rear_left': rospy.Publisher('/omni_car/rear_left_joint_velocity_controller/command', 
                                         Float64, queue_size=10),
            'rear_right': rospy.Publisher('/omni_car/rear_right_joint_velocity_controller/command', 
                                          Float64, queue_size=10)
        }

        # Wheel velocities (rad/s)
        self.wheel_velocities = {
            'front_left': 0.0,
            'front_right': 0.0,
            'rear_left': 0.0,
            'rear_right': 0.0
        }

        # Target velocities (rad/s)
        self.target_velocities = {
            'front_left': 0.0,
            'front_right': 0.0,
            'rear_left': 0.0,
            'rear_right': 0.0
        }

        # Speed settings
        self.linear_speed = 30  # rad/s for forward/backward
        self.lateral_speed = 20  # rad/s for lateral movement
        self.rotation_speed = 20  # rad/s for rotation

        # Acceleration settings
        self.acceleration = 20.0  # rad/s^2 (rate of change of velocity)
        self.deceleration = 20.0  # rad/s^2 (rate of change when stopping)

        # Velocity matrices for different movements
        # Format: [front_left, front_right, rear_left, rear_right]
        self.velocity_matrices = {
            'forward': [0.707, -0.707, 0.707, -0.707],        # Move left (along Y+)
            'backward': [-0.707, 0.707, -0.707, 0.707],       # Move right (along Y-)
            'left': [0.707, 0.707, -0.707, -0.707],      # Move forward (along X+)
            'right': [-0.707, -0.707, 0.707, 0.707],     # Move backward (along X-)
            'rotate_left': [-1.0, 1.0, -1.0, 1.0],         # Rotate left (counter-clockwise)
            'rotate_right': [1.0, -1.0, 1.0, -1.0],        # Rotate right (clockwise)
            'forward_left': [1.0, 0.0, 0.0, -1.0],         # Forward + left (diagonal)
            'forward_right': [0.0, 1.0, -1.0, 0.0],        # Forward + right (diagonal)
            'backward_left': [-1.0, 0.0, 0.0, 1.0],        # Backward + left (diagonal)
            'backward_right': [0.0, -1.0, 1.0, 0.0]        # Backward + right (diagonal)
        }

        # Control variables
        self.running = True
        self.rate = rospy.Rate(20)  # 20 Hz update rate (increased)

        # Wait for publisher connections
        rospy.loginfo("Wheel controller initialized. Waiting for connections...")
        for wheel, pub in self.wheel_publishers.items():
            while pub.get_num_connections() == 0 and not rospy.is_shutdown():
                rospy.sleep(0.1)
        rospy.loginfo("Controller connected! Ready to control wheels.")

    def apply_velocity_matrix(self, movement_type, speed):
        """Set the target velocities based on the velocity matrix and scale by speed"""
        matrix = self.velocity_matrices[movement_type]
        self.target_velocities['front_left'] = matrix[0] * speed
        self.target_velocities['front_right'] = matrix[1] * speed
        self.target_velocities['rear_left'] = matrix[2] * speed
        self.target_velocities['rear_right'] = matrix[3] * speed

    def update_velocities(self, dt):
        """Update current velocities towards target velocities with acceleration"""
        for wheel in self.wheel_velocities:
            current = self.wheel_velocities[wheel]
            target = self.target_velocities[wheel]
            if target == 0:  # Decelerating to stop
                rate = self.deceleration * dt
                if abs(current) <= rate:
                    self.wheel_velocities[wheel] = 0.0
                else:
                    self.wheel_velocities[wheel] -= rate * np.sign(current)
            else:  # Accelerating towards target
                rate = self.acceleration * dt
                if abs(target - current) <= rate:
                    self.wheel_velocities[wheel] = target
                else:
                    self.wheel_velocities[wheel] += rate * np.sign(target - current)

    def send_wheel_commands(self):
        """Send velocity commands to all wheels"""
        for wheel, velocity in self.wheel_velocities.items():
            msg = Float64()
            msg.data = velocity
            self.wheel_publishers[wheel].publish(msg)

    def stop(self):
        """Set target velocities to 0 to stop the robot"""
        self.target_velocities['front_left'] = 0.0
        self.target_velocities['front_right'] = 0.0
        self.target_velocities['rear_left'] = 0.0
        self.target_velocities['rear_right'] = 0.0

    def keyboard_control(self, stdscr):
        """Handle keyboard control with press-to-move, release-to-stop"""
        # Hide cursor and set timeout for key press detection
        curses.curs_set(0)
        stdscr.timeout(50)  # Timeout after 50ms (increased frequency)

        # Display instructions
        stdscr.addstr(0, 0, "Wheel Keyboard Controller:")
        stdscr.addstr(1, 0, "W/S: Move forward/backward (hold to move, release to stop)")
        stdscr.addstr(2, 0, "A/D: Move left/right (hold to move, release to stop)")
        stdscr.addstr(3, 0, "Q/E: Rotate left/right (hold to move, release to stop)")
        stdscr.addstr(4, 0, "I/O: Move forward-left/forward-right (diagonal)")
        stdscr.addstr(5, 0, "K/L: Move backward-left/backward-right (diagonal)")
        stdscr.addstr(6, 0, "Press 'X' to quit")
        stdscr.refresh()

        # Main control loop
        last_time = time.time()
        while self.running and not rospy.is_shutdown():
            try:
                # Calculate time step
                current_time = time.time()
                dt = current_time - last_time
                last_time = current_time

                # Get pressed key (with timeout)
                key = stdscr.getch()

                # If a key is pressed
                if key != -1:  # -1 means no key was pressed (timeout)
                    # Process key commands
                    if key == ord('w'):
                        self.apply_velocity_matrix('forward', self.linear_speed)
                    elif key == ord('s'):
                        self.apply_velocity_matrix('backward', self.linear_speed)
                    elif key == ord('a'):
                        self.apply_velocity_matrix('left', self.lateral_speed)
                    elif key == ord('d'):
                        self.apply_velocity_matrix('right', self.lateral_speed)
                    elif key == ord('q'):
                        self.apply_velocity_matrix('rotate_left', self.rotation_speed)
                    elif key == ord('e'):
                        self.apply_velocity_matrix('rotate_right', self.rotation_speed)
                    elif key == ord('i'):
                        self.apply_velocity_matrix('forward_left', self.linear_speed)
                    elif key == ord('o'):
                        self.apply_velocity_matrix('forward_right', self.linear_speed)
                    elif key == ord('k'):
                        self.apply_velocity_matrix('backward_left', self.linear_speed)
                    elif key == ord('l'):
                        self.apply_velocity_matrix('backward_right', self.linear_speed)
                    elif key == ord('x'):
                        self.stop()  # Stop before quitting
                        self.running = False
                        break
                else:
                    # No key pressed (timeout), stop the robot
                    self.stop()

                # Update velocities with acceleration
                self.update_velocities(dt)

                # Send commands to wheels
                self.send_wheel_commands()

                # Update display
                stdscr.addstr(8, 0, f"Front Left:  {self.wheel_velocities['front_left']:.2f} rad/s")
                stdscr.addstr(9, 0, f"Front Right: {self.wheel_velocities['front_right']:.2f} rad/s")
                stdscr.addstr(10, 0, f"Rear Left:   {self.wheel_velocities['rear_left']:.2f} rad/s")
                stdscr.addstr(11, 0, f"Rear Right:  {self.wheel_velocities['rear_right']:.2f} rad/s")
                stdscr.addstr(13, 0, "Command sent!   ")
                stdscr.refresh()

            except Exception as e:
                rospy.logerr(f"Control error: {e}")
                break

    def run(self):
        """Run the keyboard controller"""
        try:
            curses.wrapper(self.keyboard_control)
        except Exception as e:
            rospy.logerr(f"Error: {e}")

def main():
    try:
        controller = WheelController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")

if __name__ == '__main__':
    main()