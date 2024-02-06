#!/usr/bin/env python3
import rospy
from fs_sub.msg import DoubleWrench

class WrenchPrinter:
    def __init__(self):
        rospy.init_node("wrench_printer", anonymous=True)
        self.left_wrench = None
        self.right_wrench = None
        rospy.Subscriber("wrench", DoubleWrench, callback=self.wrench_subscriber_callback)
        
    def wrench_subscriber_callback(self, data):
        self.left_wrench = data.left_wrench
        self.right_wrench = data.right_wrench
        self.try_print_wrenches()

    def try_print_wrenches(self):
        if self.left_wrench is not None and self.right_wrench is not None:
            print(f"Left force: {self.left_wrench.force.x}, {self.left_wrench.force.y}, {self.left_wrench.force.z}; \t" +
                  f"Right force: {self.right_wrench.force.x}, {self.right_wrench.force.y}, {self.right_wrench.force.z}")
            self.left_wrench = None
            self.right_wrench = None
            
        elif self.left_wrench is not None:
            rospy.logwarn("Right force missing.")
        elif self.right_wrench is not None:
            rospy.logwarn("Left force missing.")
        else: 
            rospy.logerr("Both forces missing.")
            pass

if __name__ == '__main__':
    wp = WrenchPrinter()
    rospy.spin()