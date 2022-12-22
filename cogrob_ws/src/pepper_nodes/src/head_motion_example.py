#!/usr/bin/python3

import rospy
from std_msgs.msg import Float32MultiArray

class Handler:
    '''
    The constructor subscribes the node to head_rotation topics
    '''
    def __init__(self):
        self.head_motion_pitch_pub = rospy.Publisher("/head_rotation/pitch", Float32MultiArray, queue_size=10)
        self.head_motion_yaw_pub = rospy.Publisher("/head_rotation/yaw", Float32MultiArray, queue_size=10)

    '''
    This method publishes the desired head position relative to yaw angle
    @param angle: Target angle in radians
    @param velocity: Movement speed
    '''
    def move_head_yaw(self, angle, velocity=0.2):
        msg = Float32MultiArray()
        msg.data = [angle, velocity]
        rospy.loginfo(f"Moving head relative to yaw of {angle} radians")
        while self.head_motion_yaw_pub.get_num_connections() <= 0:
            rospy.sleep(0.2)
        self.head_motion_yaw_pub.publish(msg)

    '''
    This method publishes the desired head position relative to pitch
    @param angle: Target angle in radians
    @velocity: Movement speed
    '''
    def move_head_pitch(self, angle, velocity=0.2):
        msg = Float32MultiArray()
        msg.data = [angle, velocity]
        rospy.loginfo(f"Moving head relative to yaw of {angle} radians")
        while self.head_motion_pitch_pub.get_num_connections() <= 0:
            rospy.sleep(0.2)
        self.head_motion_pitch_pub.publish(msg)

if __name__ == "__main__":
    NODE_NAME = "tts_node_example"
    SLEEP_TIME = 3.0 #s
    VELOCITY = 0.15
    rospy.init_node(NODE_NAME)
    handler = Handler()
    handler.move_head_yaw(0.5, VELOCITY)
    rospy.sleep(SLEEP_TIME)
    handler.move_head_yaw(0.0, VELOCITY)
    rospy.sleep(SLEEP_TIME)
    handler.move_head_pitch(-0.35, VELOCITY)
    rospy.sleep(SLEEP_TIME)
    handler.move_head_pitch(0.0, VELOCITY)
