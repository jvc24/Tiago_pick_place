#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray, PoseStamped
import tf2_ros
import tf2_geometry_msgs
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection


class NodeB_april_tag_detection:
    def __init__(self):
        rospy.init_node("node_b_april_tag_detection", anonymous=True)
        self.feedback_publisher = rospy.Publisher("/node_b/feedback", String, queue_size=10)
        self.positions_publisher = rospy.Publisher("/node_b/cube_positions", String, queue_size=10)
        self.target_ids_subscriber = rospy.Subscriber("/node_a/target_ids", String, self.target_ids_callback)
        self.april_tag_subscriber = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.april_tag_callback)
        self.target_ids = []  # List of IDs to detect
        self.detected_cubes = PoseArray()  # Stores poses of detected cubes
        self.tag_positions = []  # List to store detected tag positions
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.logged_ids = []
        self.detected_tags = set()  # Initialize detected_tags here
        self.finalized = False  # Flag to ensure finalize_task is only called once

    def target_ids_callback(self, msg):
        self.target_ids = list(map(int, msg.data.split(",")))
        rospy.loginfo(f"Node B received target IDs: {self.target_ids}")

    def april_tag_callback(self, msg):
        new_detections = [detection for detection in msg.detections if detection.id[0] not in self.logged_ids]
        for detection in new_detections:
            self.process_april_tag_detection(detection)

        # If all target tags have been detected and the task hasn't been finalized yet, finalize the task
        if set(self.target_ids).issubset(self.detected_tags) and not self.finalized:
            self.finalize_task()

    def process_april_tag_detection(self, detection: AprilTagDetection):
        tag_id = detection.id[0]

        if tag_id not in self.target_ids:
            if tag_id not in self.logged_ids:
                self.feedback_publisher.publish(f"Robot detected AprilTag {tag_id}, not in targeted list.")
                self.logged_ids.append(tag_id)
            return

        # Log when a tag from the targeted list is found (but don't log position yet)
        if tag_id not in self.detected_tags:
            self.feedback_publisher.publish(f"Robot detected AprilTag {tag_id}, in targeted list.")
            self.detected_tags.add(tag_id)

        # Only add the position once if the tag hasn't been added yet
        if tag_id not in [tag[0] for tag in self.tag_positions]:
            self.process_target_tag(detection)

    def process_target_tag(self, detection: AprilTagDetection):
        tag_id = detection.id[0]
        detected_pose_camera_frame = PoseStamped()
        detected_pose_camera_frame.header = detection.pose.header
        detected_pose_camera_frame.pose = detection.pose.pose.pose  # Access the `pose` inside `PoseWithCovariance`

        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                detected_pose_camera_frame.header.frame_id,
                rospy.Time(0),
                rospy.Duration(1.0),
            )
            pose_in_map_frame = tf2_geometry_msgs.do_transform_pose(detected_pose_camera_frame, transform)
            pose_in_map_frame.pose.position.x += 0.2
            pose_in_map_frame.pose.position.y += 0.1

            # Store the tag ID and its adjusted pose only if not already in the list
            if tag_id not in [tag[0] for tag in self.tag_positions]:
                self.tag_positions.append((tag_id, pose_in_map_frame.pose))

        except tf2_ros.LookupException as e:
            rospy.logerr(f"TF Error: {e}")
        except tf2_ros.TransformException as e:
            rospy.logerr(f"TF Error: {e}")


    def finalize_task(self):

        # Create a formatted string to send all results at once
        results = ""  # Add a header for better readability
        for tag_id, pose in self.tag_positions:
            # Format the position clearly with the tag ID and the position
            results += f"\n---\nTag {tag_id}:\n Position (x: {pose.position.x:.2f}, y: {pose.position.y:.2f}, z: {pose.position.z:.2f})\n---\n"

        # Publish the results as a single string
        self.positions_publisher.publish(results)

        self.finalized = True  # Set the flag to True after finalizing



    def run(self):
        rospy.spin()


if __name__ == "__main__":
    NodeB_april_tag_detection().run()

