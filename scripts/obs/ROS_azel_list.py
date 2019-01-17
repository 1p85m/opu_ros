#! /usr/bin/env python3

import rospy

from std_msgs.msg import Float64
from std_msgs.msg import Float32
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray

from datetime import datetime
from astropy.time import Time
import time
import sys
sys.path.append("/home/amigos/ros/src/necst/lib/")
sys.path.append("/home/necst/ros/src/necst/lib/")
import calc_coord


node_name = "azel_list"


class azel_list(object):

    stop_flag = False

    def __init__(self):
        self.start_time = time.time()
        rospy.Subscriber("wc_x_list", Float64MultiArray, self._receive_x_list, queue_size=1)
        rospy.Subscriber("wc_y_list", Float64MultiArray, self._receive_y_list, queue_size=1)
        rospy.Subscriber("wc_time_list", Float64MultiArray, self._receive_time_list, queue_size=1)
        rospy.Subscriber("wc_coord", String, self._receive_coord, queue_size=1)
        rospy.Subscriber("wc_off_az", Float32, self._receive_off_az, queue_size=1)
        rospy.Subscriber("wc_off_el", Float32, self._receive_off_el, queue_size=1)
        rospy.Subscriber("wc_hosei", String, self._receive_hosei, queue_size=1)
        rospy.Subscriber("wc_lamda", Float64, self._receive_lamda, queue_size=1)
        rospy.Subscriber("wc_limit", Bool, self._receive_limit, queue_size=1)
        rospy.Subscriber("wc_rotation", Bool, self._receive_rotation, queue_size=1)
        rospy.Subscriber("wc_from_node", String, self._receive_from_node, queue_size=1)
        rospy.Subscriber("wc_timestamp", Float64, self._receive_timestamp, queue_size=1)
        rospy.Subscriber("stop_cmd", Bool, self._stop, queue_size=1)

        self.pub_x_list = rospy.Publisher("azel_x_list", Float64MultiArray, queue_size=1000)
        self.pub_y_list = rospy.Publisher("azel_y_list", Float64MultiArray, queue_size=1000)
        self.pub_time_list = rospy.Publisher("azel_time_list", Float64MultiArray, queue_size=1000)
        self.pub_coord = rospy.Publisher("azel_coord", String, queue_size=1000)
        self.pub_from_node = rospy.Publisher("azel_from_node", String, queue_size=1000)
        self.pub_timestamp = rospy.Publisher("azel_timestamp", Float64, queue_size=1000)

        self.calc = calc_coord.azel_calc()
        pass

    def _receive_x_list(self, q):
        self.x_list = q.data

    def _receive_y_list(self, q):
        self.y_list = q.data

    def _receive_coord(self, q):
        self.coord = q.data

    def _receive_off_az(self, q):
        self.off_az = q.data

    def _receive_off_el(self, q):
        self.off_el = q.data

    def _receive_hosei(self, q):
        self.hosei = q.data

    def _receive_lamda(self, q):
        self.lamda = q.data

    def _receive_limit(self, q):
        self.limit = q.data

    def _receive_rotation(self, q):
        self.rotation = q.data

    def _receive_from_node(self, q):
        self.node = q.data

    def _receive_timestamp(self, q):
        self.timestamp= q.data

    def _receive_time_list(self, q):
        if q.data < self.start_time:
            print("receive_old_list...")
        else:
            self.stop_flag = False
            self.time_list = q.data
            pass
        return

    def _stop(self, q):
        if q.data:
            self.stop_flag = q.data
        else: pass
        return

    def create_azel_list(self):
        print("wait comming list...")
        while (self.timelist =="") and (not rospy.is_shutdown()) :
            time.sleep(0.1)
            continue
        print("start_calclation!!")
        loop = 0
        check = 0
        while not rospy.is_shutdown():
            if not self.timelist:
                time.sleep(1.)
                continue

            else:
                loop = 0
                check = 0
                x_list = self.x_list
                y_list = self.y_list
                time_list = self.time_list
                coord = self.coord
                off_az = self.off_az
                off_el = self.off_el
                hosei = self.hosei
                lamda = self.lamda
                limit = self.limit
                rotation = self.rotation

            if self.stop_flag == False:
                if len(x_list) > 2:
                    dt = 0.1

                    # linear fitting
                    len_x = x_list[loop+1] - x_list[loop]
                    len_y = y_list[loop+1] - y_list[loop]
                    len_t = time_list[loop+1] - time_list[loop]

                    dx = len_x/(len_t*10)#[arcsec/100ms]
                    dy = len_y/(len_t*10)#[arcsec/100ms]
                    dt = 0.1

                    x_list2 = [x_list[loop] + dx*(i+check*10) for i in range(10)]
                    y_list2 = [y_list[loop] + dy*(i+check*10) for i in range(10)]
                    time_list2 = [time_list[loop]+dt*(i+check*10) for i in range(10)]
                    loop_count = 0
                    check_count = 1
                    for i in range(10):
                        if time_list[-1]< time_list2[-1]:
                            del x_list2[-1]
                            del y_list2[-1]
                            del time_list2[-1]
                            self.stop_flag = True
                        elif time_list[loop+1] <= time_list2[-1]:
                            del x_list2[-1]
                            del y_list2[-1]
                            del time_list2[-1]
                            loop_count = 1
                            check = 0
                            check_count = 0
                        else:
                            break
                    loop += loop_count
                    if loop == len(time_list)-1:
                        self.stop_flag = True
                    check +=  check_count
                else:
                    len_x = x_list[1] - x_list[0]
                    len_y = y_list[1] - y_list[0]
                    len_t = time_list[1] - time_list[0]

                    dx = len_x/(len_t*10)#[arcsec/100ms]
                    dy = len_y/(len_t*10)#[arcsec/100ms]
                    dt = 0.1

                    x_list2 = [x_list[0] + dx*(i+loop*10) for i in range(10)]
                    y_list2 = [y_list[0] + dy*(i+loop*10) for i in range(10)]
                    time_list2 = [time_list[0]+dt*(i+loop*10) for i in range(10)]
                    loop += 1

                    for i in range(10):
                        if time_list[-1]< time_list2[-1]:
                            del x_list2[-1]
                            del y_list2[-1]
                            del time_list2[-1]
                            self.stop_flag = True
                        else:
                            break

                if time_list2 != []:
                    time_list3 = [datetime.fromtimestamp(time_list2[i]) for i in range(len(time_list2))]
                    astro_time = Time(time_list3)

                    ret = self.calc.coordinate_calc(x_list2, y_list2, astro_time,
                                                    coord, off_az, off_el,
                                                    hosei, lamda, limit, rotation)
                    if rotation:
                        ret[0] = self.negative_change(ret[0])

                else:
                    limit_flag = True

                """limit check"""
                for i in range(len(time_list2)):
                    if not -240*3600<ret[0][i]<240*3600 or not 0.<=ret[1][i]<90*3600.:
                        print("reaching soft limit : ", )
                        print("az : ", ret[0][i]/3600., "[deg]")
                        print("el : ", ret[1][i]/3600., "[deg]")
                        self.stop_flag = True
                        limit_flag = True
                        break
                    else:
                        pass
                    limit_flag = False

                if not limit_flag:
                    array = Float64MultiArray()
                    array.data = ret[0]
                    self.pub_x_list.publish(array)

                    array.data = ret[1]
                    self.pub_y_list.publish(array)

                    array.data = time_list2
                    self.pub_time_list.publish(array)

                    msg = String()
                    msg.data = self.coord
                    self.pub_coord.publish(msg)

                    msg.data = self.node
                    self.pub_from_node.publish(msg)

                    msg = Float64()
                    msg.data = time.time()
                    self.pub_timestamp.publish(msg)

                    print("publish ok")
                else:
                    limit_flag = False
                    move_flag = False
            else:
                loop = 0
                x_list = ""
                y_list = ""
                time_list = ""
                coord = ""
                off_az = ""
                off_el = ""
                hosei = ""
                lamda = ""
                limit = ""
                rotation = ""
                pass
            time.sleep(0.1)
        return

#cable check
    def negative_change(self, az_list):
        print(az_list)

        if all((-240*3600<i< 240*3600. for i in az_list)):
            pass
        elif all((i<-110*3600. for i in az_list)):
            az_list = [i+360*3600. for i in az_list]
        elif all((i>110*3600. for i in az_list)):
            az_list = [i-360*3600. for i in az_list]
        elif all((-270*3600<i< 270*3600. for i in az_list)):
            pass
        elif any((i>=340*3600. for i in az_list)) and any((i<=20*3600. for i in az_list)):
            az_list = [i-360*3600. if i>=340*3600 else i for i in az_list]
        else:
            print("Az limit error.")
        return az_list



if __name__ == "__main__":
    rospy.init_node(node_name)
    azel = azel_list()
    azel.create_azel_list()
