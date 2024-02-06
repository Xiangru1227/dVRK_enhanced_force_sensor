#!/usr/bin/env python3
import rospy
import serial
from fs_sub.msg import DoubleWrench

class SerialWrenchPublisher:
    def __init__(self):
        rospy.init_node('serial_listener', anonymous=True)
        self.serial_port = serial.Serial(port="/dev/ttyACM0", baudrate=57600)
        self.wp = rospy.Publisher("wrench", DoubleWrench, queue_size=10)

    def wrench_parser(self, data):
        double_wrench = DoubleWrench()
        data_str = data.decode('utf-8').strip()
        
        str_parts = [float(part) for part in data_str.split(',')]
    
        attrs = ['x', 'y', 'z']
        for i, attr in enumerate(attrs, start=0):
            setattr(double_wrench.left_wrench.force, attr, str_parts[i])
            setattr(double_wrench.right_wrench.force, attr, str_parts[i + 3])
        
        self.wp.publish(double_wrench)

    def run(self):
        try:
            while not rospy.is_shutdown():
                if self.serial_port.in_waiting > 0:
                    self.wrench_parser(self.serial_port.readline())
        finally:
            self.serial_port.close()

if __name__ == '__main__':
    serial_wrench_publisher = SerialWrenchPublisher()
    serial_wrench_publisher.run()