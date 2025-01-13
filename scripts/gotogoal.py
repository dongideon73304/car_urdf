import rospy
import math
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64MultiArray

# Waypoints to follow
WAYPOINTS = [(1, 1), (2, 0), (3, 1), (4, 0), (1, 1)]

# Tolerance for reaching a waypoint
WAYPOINT_TOLERANCE = 0.1

# Global variables for odometry
current_position = None
current_orientation = None
current_waypoint_index = 0


def quaternion_to_yaw(x, y, z, w):
    """
    Convert a quaternion to a yaw angle.
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def model_states_callback(data):
    """
    Callback function to update the car's position and orientation.
    """
    global current_position, current_orientation

    try:
        index = data.name.index("car_urdf")
        pose = data.pose[index]
        current_position = (pose.position.x, pose.position.y)
        current_orientation = quaternion_to_yaw(
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        )
    except ValueError:
        rospy.logwarn("car_urdf not found in /gazebo/model_states")


def distance_to_goal(current, goal):
    """
    Compute the Euclidean distance between the current position and the goal.
    """
    return math.sqrt((current[0] - goal[0])**2 + (current[1] - goal[1])**2)


def compute_steering_angle(current, goal, yaw):
    """
    Compute the desired steering angle to reach the goal.
    """
    goal_angle = math.atan2(goal[1] - current[1], goal[0] - current[0])
    return goal_angle - yaw


def go_to_goal():
    """
    Main control loop for the Go-To-Goal controller.
    """
    global current_waypoint_index

    # Publishers for wheel and steering controllers
    wheel_pub = rospy.Publisher('/wheel_controller/command', Float64MultiArray, queue_size=10)
    steering_pub = rospy.Publisher('/steering_controller/command', Float64MultiArray, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        if current_position is None or current_orientation is None:
            rospy.loginfo("Waiting for odometry data...")
            rate.sleep()
            continue

        # Current goal waypoint
        goal = WAYPOINTS[current_waypoint_index]
        # print(current_position, "im position")
        
        # print(goal, "current goal")

        # Distance to the goal
        dist = distance_to_goal(current_position, goal)

        if dist < WAYPOINT_TOLERANCE:
            rospy.loginfo(f"Reached waypoint {current_waypoint_index + 1}: {goal}")
            current_waypoint_index = (current_waypoint_index + 1) % len(WAYPOINTS)
            continue

        # Compute control commands
        print(current_orientation, "current orientation")

        steering_angle = compute_steering_angle(current_position, goal, current_orientation) *0.04
        print(steering_angle , 'steering')
        steering_angle = max(min(steering_angle, 0.5), -0.5)  # Limit steering angle
        # print(steering_angle , 'steering after')

        # Publish steering commands
        steering_msg = Float64MultiArray()
        steering_msg.data = [steering_angle, steering_angle]
        steering_pub.publish(steering_msg)

        # Publish wheel velocity commands
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [5.0, 5.0, 5.0, 5.0]  # Constant forward velocity
        wheel_pub.publish(wheel_msg)

        rate.sleep()


if __name__ == "__main__":
    try:
        rospy.init_node('go_to_goal_controller', anonymous=True)

        # Subscribe to /gazebo/model_states for odometry
        rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_callback)

        go_to_goal()
    except rospy.ROSInterruptException:
        pass
