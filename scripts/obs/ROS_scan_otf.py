#! /usr/bin/env python3

import rospy

from std_msgs.msg import Float64
from std_msgs.msg import Float32
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray

import time
import threading
import sys
sys.path.append("/home/exito/ros/src/opust/lib/")
sys.path.append("/home/opust/ros/src/opust/lib/")
import calc_offset

node_name = "worldcoordinate_otf"

class worldcoord(object):

    msg = ""

    def __init__(self):

        rospy.Subscriber("otf_x", Float64, self._receive_x , queue_size=1)
        rospy.Subscriber("otf_y", Float64, self._receive_y , queue_size=1)
        rospy.Subscriber("otf_coord_sys", Float64, self._receive_coord_sys , queue_size=1)
        rospy.Subscriber("otf_dx", Float64, self._receive_dx , queue_size=1)
        rospy.Subscriber("otf_dy", Float64, self._receive_dy , queue_size=1)
        rospy.Subscriber("otf_dt", Float64, self._receive_dt , queue_size=1)
        rospy.Subscriber("otf_num", Float64, self._receive_num , queue_size=1)
        rospy.Subscriber("otf_rampt", Float64, self._receive_rampt , queue_size=1)
        rospy.Subscriber("otf_delay", Float64, self._receive_delay , queue_size=1)
        rospy.Subscriber("otf_off_x", Float64, self._receive_off_x , queue_size=1)
        rospy.Subscriber("otf_off_y", Float64, self._receive_off_y , queue_size=1)
        rospy.Subscriber("otf_offcoord", Float64, self._receive_offcoord , queue_size=1)
        rospy.Subscriber("otf_dcos", Float64, self._receive_dcos , queue_size=1)
        rospy.Subscriber("otf_hosei", Float64, self._receive_hosei , queue_size=1)
        rospy.Subscriber("otf_lamda", Float64, self._receive_lamda , queue_size=1)
        rospy.Subscriber("otf_limit", Float64, self._receive_limit , queue_size=1)
        rospy.Subscriber("otf_from_node", Float64, self._receive_from_node , queue_size=1)
        rospy.Subscriber("otf_timestamp", Float64, self._receive_timestamp , queue_size=1)


        self.pub_x_list = rospy.Publisher("wc_x_list", Float64MultiArray, queue_size=1)
        self.pub_y_list = rospy.Publisher("wc_y_list", Float64MultiArray, queue_size=1)
        self.pub_time_list = rospy.Publisher("wc_time_list", Float64MultiArray, queue_size=1)
        self.pub_coord = rospy.Publisher("wc_coord", String, queue_size=1)
        self.pub_off_az = rospy.Publisher("wc_off_az", Float32, queue_size=1)
        self.pub_off_el = rospy.Publisher("wc_off_el", Float32, queue_size=1)
        self.pub_hosei = rospy.Publisher("wc_hosei", String, queue_size=1)
        self.pub_lamda = rospy.Publisher("wc_lamda", Float64, queue_size=1)
        self.pub_limit = rospy.Publisher("wc_limit", Bool, queue_size=1)
        self.pub_timestamp = rospy.Publisher("wc_timestamp", Float64, queue_size=1)

        self.thread_start = threading.Thread(target=self.create_list)
        pass


    def _receive_x(self, q):
        self.x = q.data

    def _receive_y(self, q):
        self.y = q.data

    def _receive_coord_sys(self, q):
        self.coord_sys = q.data

    def _receive_dx(self, q):
        self.dx = q.data

    def _receive_dy(self, q):
        self.dy = q.data

    def _receive_dt(self, q):
        self.dx = q.data

    def _receive_num(self, q):
        self.num = q.data

    def _receive_delay(self, q):
        self.delay = q.data

    def _receive_rampt(self, q):
        self.rampt = q.data

    def _receive_off_x(self, q):
        self.off_az = q.data

    def _receive_off_y(self, q):
        self.off_el = q.data

    def _receive_offcoord(self, q):
        self.offcoord = q.data

    def _receive_dcos(self, q):
        self.dcos = q.data

    def _receive_hosei(self, q):
        self.hosei = q.data

    def _receive_lamda(self, q):
        self.lamda = q.data

    def _receive_limit(self, q):
        self.limit = q.data

    def _receive_from_node(self, q):
        self.hosei = q.data

    def _receive_timestamp(self, q):
        self.timestamp= q.data

    def create_list(self):
        self.from_node = node_name
        while not rospy.is_shutdown():
            x = 1
            y = self.y
            coord_sys = self.coord_sys
            dx = self.dx
            dy = self.dy
            dt = self.dt
            num = self.num
            rampt = self.rampt
            delay = self.delay
            off_x = self.off_x
            off_y = self.off_y
            dcos = self.dcos
            hosei = self.hosei
            lamda = self.lamda
            limit = self.limit
            timestamp = self.timestamp

            self.x= ""
            self.y= ""
            self.coord_sys= ""
            self.dx= ""
            self.dy= ""
            self.dt= ""
            self.num= ""
            self.rampt= ""
            self.delay= ""
            self.off_x= ""
            self.off_y= ""
            self.dcos= ""
            self.hosei= ""
            self.lamda= ""
            self.limit= ""
            self.timestamp= ""

            if timestamp:
                print("start_create_list")
                #ret = calc_offset.calc_offset(command.x, command.y, command.coord,
                #                              command.off_x, command.off_y, command.offcoord,
                #                              command.dcos)

                start_x = off_x-float(dx)/2.-float(dx)/float(dt)*rampt
                start_y = off_y-float(dy)/2.-float(dy)/float(dt)*rampt
                total_t = rampt + dt * num
                end_x = off_x + dx * (num - 0.5)
                end_y = off_y + dy * (num - 0.5)
                print(start_x, end_x, x)
                #off_dx_vel = (end_x - start_x) / total_t #(obs_end - obs_start)
                #off_dy_vel = (end_y - start_y) / total_t #(obs_end - obs_start)
                #x_list = [command.x+(start_x+off_dx_vel*i*0.1)/3600. for i in range(int(round(total_t/command.dt))*10)]
                #y_list = [command.y+(start_y+off_dy_vel*i*0.1)/3600. for i in range(int(round(total_t/command.dt))*10)]

                msg = Float64MultiArray()
                msg.data = [x*3600.+start_x, x*3600.+end_x]
                self.pub_x_list.publish(msg)

                msg.data = [y*3600.+start_y, y*3600.+end_y]
                self.pub_y_list.publish(msg)

                current_time = time.time()

                msg.data = [timestamp+delay, timestamp+delay+total_t]
                self.pub_time_list.publish(msg)

                msg = String()
                msg.data = coord_sys
                self.pub_coord.publish(msg)

                msg.data = hosei
                self.pub_hosei.publish(msg)

                msg = Float32()
                msg.data = 0
                self.pub_off_az.publish(msg)

                msg.data = 0
                self.pub_off_el.publish(msg)

                msg = Bool()

                msg.data = limit
                self.pub_limit.publish(msg)

                msg = Float64()
                msg.data = current_time
                self.pub_timestamp.publish(msg)

                msg.data = lamda
                self.pub_lamda.publish(msg)

                print("publish status!!\n")
                print("end_create_list\n")
            else:
                pass
            time.sleep(0.1)
        return

if __name__ == "__main__":
    rospy.init_node(node_name)
    wc = worldcoord()
    list_thread = threading.Thread(target=wc.create_list)
    list_thread.start()
    print("start calculation")
