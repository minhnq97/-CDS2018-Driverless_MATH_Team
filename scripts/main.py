#!/usr/bin/env python
import rospy
import sys
import cv2
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import CompressedImage
# our code import
import detectlane 
import lanefilter
import carcontrol

class image:
    def __init__(self):
        self.speed_pub = rospy.Publisher("Team1_speed",Float32,queue_size=10)
        self.steerAngle_pub = rospy.Publisher("Team1_steerAngle",Float32,queue_size=10)
        self.subscriber = rospy.Subscriber("Team1_image/compressed",CompressedImage, self.callback,  queue_size = 1)

    def callback(self, ros_data):
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # CODE START FROM HERE
        lane = detectlane.Lane()
        lane.imgProcess(image_np)
        
        # leftLane = lane.getLeftLane()
        # rightLane = lane.getRightLane()

        # car = carcontrol.Car(leftLane, rightLane)
        # e,v = car.carControl()
        # self.speed_pub.publish(v)
        # self.steerAngle_pub.publish(e)

        cv2.waitKey(2)
        
def main(args):
    img = image()
    rospy.init_node('image_reciver', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

sys.dont_write_bytecode = True