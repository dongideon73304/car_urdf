#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64

def move_robot_arm():
    # Initialize ROS node
    rospy.init_node('robot_arm_controller', anonymous=True)
    
    # Create publishers for each joint
    joint_1_pub = rospy.Publisher('/rr_bot_urdf/joint_1_position_controller/command', Float64, queue_size=10)
    joint_2_pub = rospy.Publisher('/rr_bot_urdf/joint_2_position_controller/command', Float64, queue_size=10)
    
    # Wait for publishers to be ready
    rospy.sleep(1)
    
    # Create Float64 message with 45 degree position (convert to radians)
    angle = Float64(3.14159 / 4)  # 45 degrees in radians
    
    # Publish commands to both joints
    joint_1_pub.publish(angle)
    joint_2_pub.publish(angle)
    
    rospy.loginfo("Moved joints to 45 degrees")

if __name__ == '__main__':
    try:
        move_robot_arm()
    except rospy.ROSInterruptException:
        pass
