#!/usr/bin/env python3
import rospy
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener
import rospkg
from std_msgs.msg import String

class NodeB_navigation:
    def __init__(self):
        rospy.init_node("node_b_navigation", anonymous=True)

        # Action client to send goals to move_base
        self.feedback_publisher = rospy.Publisher("/node_b/feedback", String, queue_size=10)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base action server.")

        # Action client for head trajectory controller
        self.head_client = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.head_client.wait_for_server()
        rospy.loginfo("Connected to head trajectory controller action server.")

        # TF Buffer to get the current robot pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        rospy.loginfo("TF Listener initialized.")

    def publish_feedback(self, message):
        feedback = String()
        feedback.data = message
        self.feedback_publisher.publish(feedback)

    def tilt_camera(self, pitch_radians, duration=2.0):
        self.publish_feedback(f"Robot tilting camera to pitch angle {math.degrees(pitch_radians)} degrees...")

        # Clamp pitch to safe limits
        pitch_radians = max(min(pitch_radians, 1.0), -1.0)

        # Create a trajectory goal
        head_goal = FollowJointTrajectoryGoal()
        head_goal.trajectory.joint_names = ["head_1_joint", "head_2_joint"]

        # Create a trajectory point
        point = JointTrajectoryPoint()
        point.positions = [0.0, pitch_radians]  # head_1_joint remains fixed; head_2_joint controls tilt
        point.time_from_start = rospy.Duration(duration)

        # Add the trajectory point to the goal
        head_goal.trajectory.points.append(point)

        # Set the goal header
        head_goal.trajectory.header.stamp = rospy.Time.now()

        # Send the goal
        self.head_client.send_goal(head_goal)
        self.head_client.wait_for_result()

        # Debug action state
        state = self.head_client.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            self.publish_feedback("Robot camera tilt complete.")
        else:
            self.publish_feedback(f"Robot camera tilt failed with state: {state}")

    def shift_orientation(self, angle_degrees):
        self.publish_feedback(f"Robot shifting orientation by {angle_degrees} degrees...")

        # Fetch the current robot position and orientation using the transform buffer
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
            current_position = transform.transform.translation
            current_orientation = transform.transform.rotation
        except Exception as e:
            self.publish_feedback(f"Failed to fetch robots current position: {e}")
            return

        # Get the current yaw and calculate the target yaw
        current_yaw = math.atan2(2.0 * (current_orientation.w * current_orientation.z + current_orientation.x * current_orientation.y),
                                1.0 - 2.0 * (current_orientation.y**2 + current_orientation.z**2))
        target_yaw = current_yaw + math.radians(angle_degrees)
        q = quaternion_from_euler(0, 0, target_yaw)

        # Create a goal to adjust orientation while keeping the current position
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = current_position.x
        goal.target_pose.pose.position.y = current_position.y
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        # Send the goal to move_base
        self.client.send_goal(goal)
        self.client.wait_for_result()

        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            self.publish_feedback(f"Robot orientation shifted by {angle_degrees} degrees.")
        else:
            self.publish_feedback(f"Failed to shift robot orientation by {angle_degrees} degrees.")

    def set_orientation(self, target_angle_degrees):
        self.publish_feedback(f"Robot setting orientation to {target_angle_degrees} degrees...")

        # Fetch the current robot position using the transform buffer
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
            current_position = transform.transform.translation
        except Exception as e:
            self.publish_feedback(f"Failed to fetch robots current position: {e}")
            return

        # Convert the target angle to radians and calculate the quaternion
        target_yaw = math.radians(target_angle_degrees)
        q = quaternion_from_euler(0, 0, target_yaw)

        # Create a goal to set orientation while keeping the current position
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = current_position.x
        goal.target_pose.pose.position.y = current_position.y
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        # Send the goal to move_base
        self.client.send_goal(goal)
        self.client.wait_for_result()

        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            self.publish_feedback(f"Robot orientation set to {target_angle_degrees} degrees.")
        else:
            self.publish_feedback(f"Failed to set robot orientation to {target_angle_degrees} degrees.")

    def slow_spin_360(self, step_angle_degrees=10):
        self.publish_feedback("Robot starting a slow 360-degree spin...")

        # Fetch the current robot position and orientation using the transform buffer
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
            current_position = transform.transform.translation
            current_orientation = transform.transform.rotation
        except Exception as e:
            self.publish_feedback(f"Robot failed to fetch current position: {e}")
            return

        # Get the current yaw angle
        current_yaw = math.atan2(2.0 * (current_orientation.w * current_orientation.z + current_orientation.x * current_orientation.y),
                                1.0 - 2.0 * (current_orientation.y**2 + current_orientation.z**2))

        # Step size for incremental angle adjustments (in degrees)
        step_angle_radians = math.radians(step_angle_degrees)

        # Perform the spin incrementally
        for i in range(0, 360, step_angle_degrees):
            # Calculate the target yaw for this step
            target_yaw = current_yaw + math.radians(i)
            q = quaternion_from_euler(0, 0, target_yaw)

            # Create a goal for this incremental spin
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = current_position.x
            goal.target_pose.pose.position.y = current_position.y
            goal.target_pose.pose.orientation.x = q[0]
            goal.target_pose.pose.orientation.y = q[1]
            goal.target_pose.pose.orientation.z = q[2]
            goal.target_pose.pose.orientation.w = q[3]

            # Send the goal to move_base
            self.client.send_goal(goal)
            self.client.wait_for_result()

            # Check if the step succeeded
            if self.client.get_state() != actionlib.GoalStatus.SUCCEEDED:
                self.publish_feedback(f"Robot failed to complete step {i} of the spin.")
                break

        self.publish_feedback("Robot slow 360-degree spin complete.")

    def move_to_goal(self, x, y, orientation_degrees):
        self.publish_feedback(f"Robot moving to position ({x}, {y}) with orientation {orientation_degrees} degrees...")

        # Convert orientation from degrees to quaternion
        orientation_radians = math.radians(orientation_degrees)
        q = quaternion_from_euler(0, 0, orientation_radians)

        # Create a MoveBaseGoal to send to the navigation stack
        goal = MoveBaseGoal()

        # Set the goal frame to "map"
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # Set the goal position and orientation
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Set the orientation using the quaternion
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        # Send the goal to move_base
        self.client.send_goal(goal)

        # Wait for the result
        self.client.wait_for_result()
        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            self.publish_feedback("Robot reached the goal reached successfully!")
        else:
            self.publish_feedback("Robot failed to reach the goal.")

    def read_command_file(self, file_path):
        # Initialize a list to store the parsed commands
        parsed_commands = []

        # Open the file and process each line
        with open(file_path, 'r') as file:
            for line in file:
                # Strip any leading/trailing whitespace (including newlines)
                line = line.strip()
                
                # Skip empty lines
                if not line:
                    continue
                
                # Split the line into parts: the command and its parameters
                parts = line.split()
                command = parts[0]  # First part is the command
                params = list(map(float, parts[1:]))  # Remaining parts are parameters
                
                # Append the parsed command and parameters to the list
                if params:
                    parsed_commands.append([command, params])
                else:
                    parsed_commands.append([command])
        
        return parsed_commands

    def run(self):
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('ir2425_group_26')
        filename = f"{package_path}/scripts/move_commands.txt"

        try:
            command_list = self.read_command_file(filename)

            for command in command_list:
                command_type = command[0]

                if command_type == "move_to":
                    command_params = command[1]
                    self.move_to_goal(command_params[0], command_params[1], command_params[2])
                elif command_type == "spin":
                    self.slow_spin_360(step_angle_degrees=30)
                elif command_type == "set_angle":
                    command_params = command[1]
                    self.set_orientation(command_params[0])
                elif command_type == "shift_angle":
                    command_params = command[1]
                    self.shift_orientation(command_params[0])
                elif command_type == "camera_tilt":
                    command_params = command[1]
                    self.tilt_camera(math.radians(command_params[0]))

        except rospy.ROSInterruptException:
            rospy.loginfo("Navigation script stopped.")
        finally:
            self.publish_feedback("Robot stopped.")


if __name__ == "__main__":
    NodeB_navigation().run()
