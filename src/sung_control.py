#!/usr/bin/env python3

import rospy 
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import curses
import time

class ArmController:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('arm_keyboard_controller', anonymous=True)
        
        # Publisher for joint trajectory commands
        self.control_publisher = rospy.Publisher('/omni_car/arm_controller/command', 
                                                JointTrajectory, queue_size=10)
        
        # Joint positions
        self.joint_positions = [0.0, 0.0]  # [joint_1_link, joint_2_link]
        
        # Joint limits from URDF
        self.joint_limits = {
            'joint_1_link': {'min': -3.14, 'max': 3.14},  # rotation
            'joint_2_link': {'min': -1.57, 'max': 0.0}    # up/down
        }
        
        # Movement increments
        self.rotation_increment = 0.1  # radians
        self.vertical_increment = 0.05  # radians
        
        # Control variables
        self.running = True
        self.rate = rospy.Rate(10)  # 10 Hz update rate
        
        rospy.loginfo("Arm controller initialized. Waiting for connections...")
        # Wait for publisher connection
        while self.control_publisher.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo("Controller connected! Ready to control joints.")

    def send_joint_command(self):
        """Send the current joint positions to the robot"""
        msg = JointTrajectory()
        msg.header.stamp = rospy.Time.now()
        msg.joint_names = ['joint_1_link', 'joint_2_link']
        
        point = JointTrajectoryPoint()
        point.positions = self.joint_positions
        point.velocities = [0.0, 0.0]
        point.time_from_start = rospy.Duration(0.5)
        
        msg.points.append(point)
        self.control_publisher.publish(msg)

    def rotate_joint_1(self, direction):
        """Rotate joint_1_link (positive or negative direction)"""
        increment = self.rotation_increment * direction
        new_value = self.joint_positions[0] + increment
        
        # Apply joint limits
        if new_value <= self.joint_limits['joint_1_link']['max'] and new_value >= self.joint_limits['joint_1_link']['min']:
            self.joint_positions[0] = new_value
            return True
        return False

    def move_joint_2(self, direction):
        """Move joint_2_link up/down (positive or negative direction)"""
        increment = self.vertical_increment * direction
        new_value = self.joint_positions[1] + increment
        
        # Apply joint limits
        if new_value <= self.joint_limits['joint_2_link']['max'] and new_value >= self.joint_limits['joint_2_link']['min']:
            self.joint_positions[1] = new_value
            return True
        return False

    def keyboard_control(self, stdscr):
        """Handle keyboard control"""
        # Hide cursor and set keyboard to non-blocking
        curses.curs_set(0)
        stdscr.nodelay(True)
        
        # Display instructions
        stdscr.addstr(0, 0, "Arm Keyboard Controller:")
        stdscr.addstr(1, 0, "A/D: Rotate joint_1_link left/right")
        stdscr.addstr(2, 0, "W/S: Move joint_2_link up/down")
        stdscr.addstr(3, 0, "R: Reset to home position")
        stdscr.addstr(4, 0, "Press 'Q' to quit")
        stdscr.refresh()
        
        # Main control loop
        while self.running and not rospy.is_shutdown():
            try:
                # Get pressed key
                key = stdscr.getch()
                
                if key != -1:
                    changed = False
                    
                    # Process key commands
                    if key == ord('a'):
                        changed = self.rotate_joint_1(1)  # Rotate left
                    elif key == ord('d'):
                        changed = self.rotate_joint_1(-1)  # Rotate right
                    elif key == ord('w'):
                        changed = self.move_joint_2(-1)  # Up
                    elif key == ord('s'):
                        changed = self.move_joint_2(1)  # Down
                    elif key == ord('r'):
                        # Reset to home position
                        self.joint_positions = [0.0, -0.5]  # Middle rotation, middle vertical
                        changed = True
                    elif key == ord('q'):
                        self.running = False
                        break
                    
                    # Update display
                    stdscr.addstr(6, 0, f"joint_1_link (rotation): {self.joint_positions[0]:.2f} rad")
                    stdscr.addstr(7, 0, f"joint_2_link (vertical): {self.joint_positions[1]:.2f} rad")
                    
                    # If positions changed, send command
                    if changed:
                        self.send_joint_command()
                        stdscr.addstr(9, 0, "Command sent!   ")
                    
                    stdscr.refresh()
                
                # Sleep briefly to reduce CPU load
                rospy.sleep(0.02)
                
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
        controller = ArmController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")

if __name__ == '__main__':
    main()