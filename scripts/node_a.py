#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerRequest
from geometry_msgs.msg import PoseArray  # For receiving poses of cubes
from tiago_iaslab_simulation.srv import Objs


class NodeA:
    def __init__(self):
        rospy.init_node("node_a", anonymous=True)
        rospy.loginfo("NodeA initialized")
        self.ids_srv = "/apriltag_ids_srv"
        self.ids_publisher = rospy.Publisher("/node_a/target_ids", String, queue_size=10, latch=True)
        self.feedback_subscriber = rospy.Subscriber("/node_b/feedback", String, self.feedback_callback)
        self.positions_subscriber = rospy.Subscriber("/node_b/cube_positions", String, self.positions_callback)

    def request_ids(self):
        rospy.loginfo("Starting request_ids")
        try:
            rospy.wait_for_service(self.ids_srv)
            ids_service = rospy.ServiceProxy(self.ids_srv, Objs)
            response = ids_service(ready=True)
            ids = response.ids  # Extract the list of IDs
            rospy.loginfo(f"Node A received target IDs: {ids}")
            self.ids_publisher.publish(",".join(map(str, ids)))  # Publish with latch enabled
            rospy.loginfo("Published target IDs")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        except Exception as e:
            rospy.logerr(f"Unexpected error: {e}")

    def feedback_callback(self, msg):
        rospy.loginfo(f"Feedback from Node B: {msg.data}")

    def positions_callback(self, msg):
        # Print the received message with better formatting
        formatted_msg = msg.data.strip()  # Strip any leading/trailing whitespace
        rospy.loginfo("Final positions of cubes: \n" + formatted_msg)
        #rospy.loginfo(formatted_msg)

    def run(self):
        rospy.loginfo("NodeA running...")
        self.request_ids()
        rospy.spin()


if __name__ == "__main__":
    try:
        NodeA().run()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception")
