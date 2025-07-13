#!/usr/bin/env python3
import rospy
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener

class SpinAndMoveNode:
    def __init__(self):
        rospy.init_node("spin_and_move_node", anonymous=True)

        # Action client to send goals to move_base
        rospy.loginfo("Waiting for move_base action server...")
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base action server.")

        # Action client for head trajectory controller
        rospy.loginfo("Waiting for head trajectory controller action server...")
        self.head_client = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.head_client.wait_for_server()
        rospy.loginfo("Connected to head trajectory controller action server.")

        # TF Buffer to get the current robot pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        rospy.loginfo("TF Listener initialized.")


    def tilt_camera(self, pitch_radians, duration=2.0):
        rospy.loginfo(f"Tilting camera to pitch angle {math.degrees(pitch_radians)} degrees...")

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
            rospy.loginfo("Camera tilt complete.")
        else:
            rospy.logwarn(f"Camera tilt failed with state: {state}")


    def spin(self, angle_degrees):
        rospy.loginfo(f"Starting {angle_degrees}-degree spin...")

        # Fetch the current robot position and orientation using the transform buffer
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
            current_position = transform.transform.translation
            current_orientation = transform.transform.rotation

            #rospy.loginfo(f"Current position: ({current_position.x}, {current_position.y}), orientation: {current_orientation}")
        except Exception as e:
            rospy.logwarn(f"Failed to fetch current position: {e}")
            return

        # Convert the desired spin angle to a quaternion
        current_yaw = math.atan2(2.0 * (current_orientation.w * current_orientation.z + current_orientation.x * current_orientation.y),
                                1.0 - 2.0 * (current_orientation.y**2 + current_orientation.z**2))
        target_yaw = current_yaw + math.radians(angle_degrees)
        q = quaternion_from_euler(0, 0, target_yaw)

        # Create a goal to adjust orientation while keeping the current position
        spin_goal = MoveBaseGoal()

        # Set the goal frame to "map"
        spin_goal.target_pose.header.frame_id = "map"
        spin_goal.target_pose.header.stamp = rospy.Time.now()

        # Use the current position for the goal
        spin_goal.target_pose.pose.position.x = current_position.x
        spin_goal.target_pose.pose.position.y = current_position.y

        # Set the target orientation
        spin_goal.target_pose.pose.orientation.x = q[0]
        spin_goal.target_pose.pose.orientation.y = q[1]
        spin_goal.target_pose.pose.orientation.z = q[2]
        spin_goal.target_pose.pose.orientation.w = q[3]

        # Send the spin goal to move_base
        self.client.send_goal(spin_goal)
        self.client.wait_for_result()

        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"{angle_degrees}-degree spin complete.")
        else:
            rospy.logwarn(f"Failed to spin {angle_degrees} degrees.")


    def move_to_goal(self, x, y, orientation_degrees):
        rospy.loginfo(f"Sending goal to position ({x}, {y}) with orientation {orientation_degrees} degrees...")

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
            rospy.loginfo("Goal reached successfully!")
        else:
            rospy.logwarn("Failed to reach the goal.")

    def run(self):
        try:
            # Tilt the camera up, spin, and move to a goal
            #self.tilt_camera(math.radians(30))  # Tilt camera up by 30 degrees
            #self.tilt_camera(math.radians(-40))  # Tilt camera down by 10 degrees
            
            #self.spin(270)  # Spin by 360 degrees
            self.tilt_camera(math.radians(0))
            self.move_to_goal(0, 0, 0) 

        except rospy.ROSInterruptException:
            rospy.loginfo("Spin, move, and tilt interrupted.")
        finally:
            rospy.loginfo("Robot stopped.")

if __name__ == "__main__":
    SpinAndMoveNode().run()
