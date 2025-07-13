#!/usr/bin/env python

import sys
import math
import rospy
import actionlib
import tf
import tf2_ros
from geometry_msgs.msg import PoseStamped, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import moveit_commander
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from gazebo_ros_link_attacher.srv import Attach
from std_msgs.msg import Float32MultiArray, String
from apriltag_ros.msg import AprilTagDetectionArray
from tf2_geometry_msgs import do_transform_pose

from tf.transformations import euler_from_quaternion #Used for converting from quaternion to gripper position
import time
import json

#x1, y1, yaw1 = 0.0, 0.0, 0.0

z1, yaw_offset = 1.015, 0

class NodeC:
    def _init_(self):

        #Default variables 
        self.AVAILABLE_TAG_IDS = {1, 2, 3, 4, 5, 6, 7, 8, 9}
        self.detected_tag_ids = set()
        self.x1 = 0.0
        self.y1 = 0.0
        self.yaw1 = 0.0
        self.z1 = 1.015


        rospy.init_node("node_c", anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize navigation and manipulation interfaces
        self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base action server.")

        self.head_client = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.head_client.wait_for_server()
        rospy.loginfo("Connected to head trajectory controller action server.")

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_group = moveit_commander.MoveGroupCommander("arm_torso")
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")

        # Set planner and reference frame
        self.arm_group.set_planner_id("SBLkConfigDefault")
        self.arm_group.set_pose_reference_frame("base_footprint")
        self.arm_group.set_max_velocity_scaling_factor(1.0)
        self.arm_group.set_start_state_to_current_state()
        self.arm_group.set_planning_time(5.0)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.listener = tf.TransformListener()

        # Services for virtual attach/detach
        rospy.wait_for_service('/link_attacher_node/attach')
        rospy.wait_for_service('/link_attacher_node/detach')
        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)


        # Initialize placement line coefficients
        self.line_coefficients = None
        rospy.Subscriber("/placing_line_info", Float32MultiArray, self.line_info_callback)

        # Subscribe to apriltag processing from Node B
        rospy.Subscriber('/apriltag_data_json', String, self.tag_callback)

        self.TARGET_TAG_ID = "6" #The tag id BLOCK to pick

        # Initial navigation to pick docking position
        self.initial_move()

        # Execute the pick-and-place task
        self.pick_and_place()

    def initial_move(self):
        rospy.loginfo("Moving to the start position...")
        #self.tilt_camera(math.radians(-30))  # Tilt camera down
        self.move_to(9, -1, -90)
        self.move_to(9, -4, 180)
        self.arm_reset()
        self.move_to(7.9, -4, 180)
        self.move_to(7.9, -4, 90)
        self.move_to(7.9, -3.6, 90)
        self.tilt_camera(math.radians(-50))

    def move_to(self, x, y, orientation_deg):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        quat = tf.transformations.quaternion_from_euler(0, 0, math.radians(orientation_deg))
        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]

        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()

        if self.move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"Successfully reached ({x}, {y}) with orientation {orientation_deg}°")
        else:
            rospy.logerr(f"Failed to reach ({x}, {y}) with orientation {orientation_deg}°")

    def tilt_camera(self, pitch_radians, duration=2.0):
        pitch_radians = max(min(pitch_radians, 1.0), -1.0)
        head_goal = FollowJointTrajectoryGoal()
        head_goal.trajectory.joint_names = ["head_1_joint", "head_2_joint"]
        point = JointTrajectoryPoint()
        point.positions = [0.0, pitch_radians]
        point.time_from_start = rospy.Duration(duration)
        head_goal.trajectory.points.append(point)
        self.head_client.send_goal(head_goal)
        self.head_client.wait_for_result()

    def line_info_callback(self, msg):
        self.line_coefficients = msg.data

    
    def tag_callback(self, msg):
        
        
        #global x1, y1, yaw1

        try:
            tag_data = json.loads(msg.data)

            # Only proceed if the target tagid is in the data
            
            
            for tag_id, data in tag_data.items():
                pos = data["position"]
                rpy = data["orientation"]

                #rospy.loginfo(f"Tag ID: {tag_id}")
                #rospy.loginfo(f"  Position -> x: {pos['x']:.2f}, y: {pos['y']:.2f}, z: {pos['z']:.2f}")
                #rospy.loginfo(f"  Orientation -> roll: {rpy['roll']:.2f}, pitch: {rpy['pitch']:.2f}, yaw: {rpy['yaw']:.2f}")
                #rospy.loginfo("")

            if self.TARGET_TAG_ID in tag_data and self.TARGET_TAG_ID not in self.detected_tag_ids:
                rospy.loginfo(f"Target tag id: {self.TARGET_TAG_ID}")
                
                
                data = tag_data[self.TARGET_TAG_ID]
                pos = data["position"]
                rpy = data["orientation"]

                self.x1 = pos['x']
                self.y1 = pos['y']
                self.yaw1 = rpy['yaw']
                self.detected_tag_ids.add(tag_id)

                rospy.loginfo(f"[Target Tag {self.TARGET_TAG_ID}] Position: x={self.x1:.2f}, y={self.y1:.2f}")
                rospy.loginfo(f"[Target Tag {self.TARGET_TAG_ID}] Yaw: {self.yaw1:.2f}")
            else:
                rospy.logwarn(f"Target tag ID {self.TARGET_TAG_ID} not found in the message.")

        except json.JSONDecodeError as e:
            rospy.logwarn(f"Failed to decode tag data JSON: {e}")

    def pick_and_place(self):
        # Initial move has already been called, assuming docking position

        # Block 6
        self.TARGET_TAG_ID = "6"
        self.execute_pick()
        self.tilt_camera(math.radians(0))
        #moving to placement dock
        self.move_to(8, -3.9, 0)
        self.move_to(9, -4, 0)
        self.move_to(9, -4, 90)
        self.move_to(9, -2, 90)
        self.move_to(9, -2, 180)
        self.move_to(8.6, -2, 180)

        self.tilt_camera(math.radians(-50))

        self.TARGET_TAG_ID = "10" #Change filter target to placement table apriltag
        self.compute_place_point()
        self.execute_place()

        self.tilt_camera(math.radians(0))

        # Block 4
        self.TARGET_TAG_ID = "4"

        #moving to pick dock
        self.move_to(9, -2, -90)
        self.move_to(9, -4, -90)
        self.move_to(9, -4, 180)
        self.move_to(7.8, -4, 180)
        self.move_to(7.8, -4, 90)
        self.move_to(7.8, -3.9, 90)
        self.tilt_camera(math.radians(-50))
        self.execute_pick()
        self.tilt_camera(math.radians(0))
        #moving to placement dock
        self.move_to(7.8, -3.9, 0)
        self.move_to(9, -4, 0)
        self.move_to(9, -4, 90)
        self.move_to(9, -2, 90)
        self.move_to(9, -2, 180)


        self.tilt_camera(math.radians(-50))
        self.TARGET_TAG_ID = "10" #Change filter target to placement table apriltag
        self.compute_place_point()

        self.execute_place()
        self.tilt_camera(math.radians(0))
        # Block 5
        self.TARGET_TAG_ID = "5"

        #moving to pick dock
        self.move_to(9, -2, -90)
        self.move_to(9, -3, -90)
        self.move_to(9, -3, 180)

        self.execute_pick()

        #moving to placement dock
        self.move_to(9, -3, 90)
        self.move_to(9, -2, 90)
        self.move_to(9, -2, 180)

        self.TARGET_TAG_ID = "10" #Change filter target to placement table apriltag
        self.compute_place_point()
        self.execute_place()
    
    def close_gripper(self, value):
        """
        Close or open the gripper by setting finger joints to 'value'.

        Args:
            value (float): Joint position value for both fingers (0.0 = closed).
        """

        joint_goal = {
            'gripper_left_finger_joint': value,
            'gripper_right_finger_joint': value
        }

        self.gripper_group.set_joint_value_target(joint_goal)
        self.gripper_group.go(wait=True)
        self.gripper_group.stop()

        rospy.loginfo(f"Gripper set to position {value}.")
        rospy.sleep(1.0)  # Added wait after gripper closes

    def move_arm_to_pose(self, x, y, z, roll, pitch, yaw):
        rospy.loginfo(f"Planning to pose: position=({x:.2f}, {y:.2f}, {z:.2f}), rpy=({roll:.2f}, {pitch:.2f}, {yaw:.2f})")

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "base_footprint"
        goal_pose.header.stamp = rospy.Time.now()

        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = z

        quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        goal_pose.pose.orientation.x = quat[0]
        goal_pose.pose.orientation.y = quat[1]
        goal_pose.pose.orientation.z = quat[2]
        goal_pose.pose.orientation.w = quat[3]

        self.arm_group.set_pose_target(goal_pose)

        plan_result = self.arm_group.plan()
        if isinstance(plan_result, tuple):
            plan = plan_result[1]  # Extract actual plan
        else:
            plan = plan_result

        if not plan or not hasattr(plan, "joint_trajectory") or len(plan.joint_trajectory.points) == 0:
            rospy.logerr(f"No valid plan found for the pose.")
            return False

        rospy.loginfo(f"Executing plan...")
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

        if success:
            rospy.loginfo(f"Pose executed successfully.")
        else:
            rospy.logerr(f"Motion execution failed.")
        return success
    
    def compute_place_point(self):
        m = self.line_coefficients[0]
        q = self.line_coefficients[1]

        if m is None or q is None:
            rospy.logwarn("[PlacePoint] Line coefficients missing.")
            return

        if self.x1 is None or self.y1 is None:
            rospy.logwarn("[PlacePoint] AprilTag not detected yet.")
            return

        x_place = self.x1 + 0.1
        x_line = x_place - self.x1
        y_line = 1 / m * (x_line - q)
        #y_line = self.m * x_line + self.q
        y_place = self.y1 - y_line

        rospy.loginfo(f"[PlacePoint] x = {x_place:.3f}, y = {y_place:.3f}")
        self.place_point = (x_place, y_place)
    
    def execute_place(self):
        yaw_offset = 0.0
        z1 = 1.015         

        x1, y1, yaw1 = self.place_point[0], self.place_point[1], self.yaw1 + yaw_offset

        target_poses = [
            (0.4, 0.0, 1.2, 1.57, 1.57, 0.0),
            (x1, 0.0, 1.2, 1.57, 1.57, 0.0),
            (x1, y1, 1.2, 1.57, 1.57, 0.0),
            (x1, y1, 1.2, 1.57, 1.57, yaw1),
            (x1, y1, z1, 1.57, 1.57, yaw1)
        ]
        retract_poses = [
            (x1, y1, 1.2, 1.57, 1.57, yaw1),
            (x1, y1, 1.2, 1.57, 1.57, 0.0),
            (x1, 0.0, 1.2, 1.57, 1.57, 0.0),
            (0.4, 0.0, 1.2, 1.57, 1.57, 0.0)
        ]

        for pose in target_poses:
            self.move_arm_to_pose(*pose)

        self.close_gripper(0.04)
        rospy.sleep(1.0)

        for pose in retract_poses:
            self.move_arm_to_pose(*pose)


    def execute_pick(self):
        
        x1 = self.x1
        y1 = self.y1
        yaw1 = self.yaw1
        rospy.loginfo(f" x1: {x1} y1: {y1} yaw1: {yaw1}")
        yaw_offset = 0.0
        z1 = 1.05         

        #x1, y1, yaw1 = self.place_point[0], self.place_point[1], self.yaw1 + yaw_offset

        #global x1, y1, yaw1

        target_poses = [
            (0.4, 0.0, 1.2, 1.57, 1.57, 0.0),
            (x1, 0.0, 1.2, 1.57, 1.57, 0.0),
            (x1, y1, 1.2, 1.57, 1.57, 0.0),
            (x1, y1, 1.2, 1.57, 1.57, yaw1),
            (x1, y1, z1, 1.57, 1.57, yaw1)
        ]
        retract_poses = [
            (x1, y1, 1.2, 1.57, 1.57, yaw1),
            (x1, y1, 1.2, 1.57, 1.57, 0.0),
            (x1, 0.0, 1.2, 1.57, 1.57, 0.0),
            (0.4, 0.0, 1.2, 1.57, 1.57, 0.0)
        ]

        self.close_gripper(0.04)

        for pose in target_poses:
            self.move_arm_to_pose(*pose)

        self.close_gripper(0.001)
        rospy.sleep(1.0)

        for pose in retract_poses:
            self.move_arm_to_pose(*pose)

    
    def arm_reset(self):
        # Enter the coordinates of the april tag heren (x, y, z, roll)
        x = 0.4 
        y = 0.0
        z = 1.2
        roll = 1.57
        pitch = 1.57  # pointing down
        yaw = 0
        
        self.move_arm_to_pose(x, y, z, roll, pitch, yaw)


if _name_ == "_main_":
    try:
        NodeC()
    except rospy.ROSInterruptException:
        pass