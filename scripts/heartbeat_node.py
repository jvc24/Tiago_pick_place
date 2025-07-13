#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import time


class HeartbeatNode:
    def __init__(self):
        rospy.init_node("heartbeat_node", anonymous=True)
        self.publisher = rospy.Publisher("/heartbeat", String, queue_size=10)
        self.rate = rospy.Rate(1)  # 1 Hz (1 message per second)

    def run(self):
        while not rospy.is_shutdown():
            message = f"System is running: {time.strftime('%Y-%m-%d %H:%M:%S')}"
            rospy.loginfo(message)
            self.publisher.publish(message)
            self.rate.sleep()


if __name__ == "__main__":
    try:
        HeartbeatNode().run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Heartbeat node shutting down.")
