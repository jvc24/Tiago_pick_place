#!/usr/bin/env python

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from moveit_commander import PlanningSceneInterface, RobotCommander
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

import tf2_ros
import tf2_geometry_msgs
import tf
import json

# Available and detected tag IDs
AVAILABLE_TAG_IDS = {1, 2, 3, 4, 5, 6, 7, 8, 9}
detected_tag_ids = set()

# Global components
planning_scene = None
robot = None
tag_data_pub = None

# Store tag information in a dictionary
tag_data = {}

def apriltag_callback(data):
    global detected_tag_ids, tag_data_pub, tag_data

    if not data.detections:
        rospy.loginfo("No AprilTags detected.")
        return

    for detection in data.detections:
        tag_id = detection.id[0]

        if tag_id in AVAILABLE_TAG_IDS and tag_id not in detected_tag_ids:
            try:
                raw_pose = detection.pose.pose.pose
                pose_in_camera = PoseStamped()
                pose_in_camera.header.frame_id = detection.pose.header.frame_id
                pose_in_camera.header.stamp = rospy.Time(0)
                pose_in_camera.pose = raw_pose

                transform = tf_buffer.lookup_transform(
                    "base_footprint",  #
                    pose_in_camera.header.frame_id,
                    rospy.Time(0),
                    rospy.Duration(1.0)
                )
                pose_in_base = tf2_geometry_msgs.do_transform_pose(pose_in_camera, transform)

                # Extract position
                pos = pose_in_base.pose.position
                # Extract orientation and convert to roll, pitch, yaw
                quat = pose_in_base.pose.orientation
                quaternion = [quat.x, quat.y, quat.z, quat.w]
                roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)

                rospy.loginfo(f"Detected new AprilTag ID: {tag_id}")
                rospy.loginfo(f"Position: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}")
                rospy.loginfo(f"Orientation: roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}")

                # Store tag data in dictionary
                tag_data[tag_id] = {
                    "position": {
                        "x": round(pos.x, 4),
                        "y": round(pos.y, 4),
                        "z": round(pos.z, 4)
                    },
                    "orientation": {
                        "roll": round(roll, 4),
                        "pitch": round(pitch, 4),
                        "yaw": round(yaw, 4)
                    }
                }

                # Publish the entire dictionary as a JSON-style string
                tag_data_pub.publish(String(data=json.dumps(tag_data)))

                # Mark this tag as processed
                #detected_tag_ids.add(tag_id)

            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException) as e:
                rospy.logwarn(f"TF2 transform error: {e}")



def main():
    global planning_scene, robot, tf_buffer, tf_listener, tag_data_pub

    rospy.init_node("apriltag_pose_publisher", anonymous=True)
    rospy.loginfo("Starting AprilTag pose publisher node...")

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    robot = RobotCommander()
    planning_scene = PlanningSceneInterface()

    tag_data_pub = rospy.Publisher('/apriltag_data_json', String, queue_size=10, latch=True)

    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, apriltag_callback)

    rospy.spin()


if _name_ == "_main_":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("AprilTag pose publisher node terminated.")