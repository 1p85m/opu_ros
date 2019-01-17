#! /usr/bin/env python3
import rospy
import os, sys, time ,datetime
import serial
import threading

from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Int32


class encorder(object):
    def __init__(self):

        self.pub_az = rospy.Publisher("encorder_az", Float64, queue_size=1)
        self.pub_el = rospy.Publisher("encorder_el", Float64, queue_size=1)
        self.pub_az_error = rospy.Publisher("encorder_az_error", String, queue_size=1)
        self.pub_el_error = rospy.Publisher("encorder_el_error", String, queue_size=1)

        self.ser_az = serial.Serial("/dev/ttyUSB0",38400,timeout=1,parity="E",bytesize=7,stopbits=2)
        self.ser_el = serial.Serial("/dev/ttyUSB1",38400,timeout=1,parity="E",bytesize=7,stopbits=2)

    def read_az(self):
        while not rospy.is_shutdown():
            self.ser_az.write(b"\x1BA0200\r")
            ret = self.ser_az.readline()
            if not ret:
                continue
            else:
                az = ret[1:11]
                msg　= Float64()
                msg.data = float(az)
                self.pub_az.publish(msg)


    def read_el(self):
        while not rospy.is_shutdown():

            self.ser_el.write(b"\x1BA0200\r")
            ret = self.ser_el.readline()
            if not ret:
                continue
            else:
                el = ret[1:11]
                msg = Float64()
                msg.data = float(el)
                self.pub_el.publish(msg)

    def which_encorder(self):
        self.ser_az = serial.Serial("/dev/ttyUSB0",38400,timeout=1,parity="E",bytesize=7,stopbits=2)
        self.ser_el = serial.Serial("/dev/ttyUSB1",38400,timeout=1,parity="E",bytesize=7,stopbits=2)


if __name__ == "__main__" :
    rospy.init_node("encorder")
    enc = encorder()
    thread_az = threading.Thread(target=enc.read_az)
    thread_el = threading.Thread(target=enc.read_el)
    thread_az.start()
    thread_el.start()
    enc.read_az()
    enc.read_el()


#2019
#written by H.Kondo
