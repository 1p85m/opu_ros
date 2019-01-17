#!/usr/bin/env python3

name = "antenna_move"

import rospy

from std_msgs.msg import Float64
from std_msgs.msg import Float32
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray

import time
import numpy


class Antenna(object):

    parameters = {
        'az_list':[],
        'el_list':[],
        'start_time_list':[],
        }

    lock_az = False
    lock_el = False

    def __init__(self):
        self.start_time = time.time()

        rospy.Subscriber("azel_x_list", Float64MultiArray, self._receive_x_list, queue_size=1000)
        rospy.Subscriber("azel_y_list", Float64MultiArray, self._receive_y_list, queue_size=1000)
        rospy.Subscriber("azel_time_list", Float64MultiArray, self.set_parameter, queue_size=1000)
        rospy.Subscriber("azel_coord", String, self._receive_coord, queue_size=1000)

        rospy.Subscriber('stop_cmd', Bool, self.stop_move, queue_size=1)

        rospy.Subscriber("/antenna/az_lock", Bool, self.set_flag_az, queue_size = 1)
        rospy.Subscriber("/antenna/el_lock", Bool, self.set_flag_el, queue_size = 1)
    
        self.topic_az = rospy.Publisher("/antenna/az_cmd", Float64, queue_size = 1)
        self.topic_el = rospy.Publisher("/antenna/el_cmd", Float64, queue_size = 1)

        pass


    def _receive_x_list(self,q):
        self.x_list = q.data

    def _receive_y_list(self,q):
        self.y_list = q.data

    def _receive_coord(self,q):
        self.coord = q.data

    def set_flag_az(self, q):
        self.lock_az = q.data
        return

    def set_flag_el(self, q):
        self.lock_el = q.data
        return

    def set_parameter(self, q):
        if not q.data:
            return

        x_list = self.x_list
        y_list = self.y_list

        if self.start_time < q.data[0]:
            if self.parameters['start_time_list'] != []:
                time_len = len(self.parameters['start_time_list'])
                for i in range(time_len):
                    __tmp = self.parameters
                    if q.data[0]< __tmp['start_time_list'][-1]:
                        del __tmp['az_list'][-1]
                        del __tmp['el_list'][-1]
                        del __tmp['start_time_list'][-1]
                        self.parameters = __tmp
                    else:
                        break
            else:
                pass

            tmp_ = self.parameters
            tmp_['az_list'].extend(x_list)
            tmp_['el_list'].extend(y_list)
            tmp_['start_time_list'].extend(q.data)
            self.parameters = tmp_
        else:
            pass
        return

    def time_check(self, st):
        ct = time.time()

        if ct - st[-1] >= 0:
            rospy.loginfo('!!!azel_list is end!!!')
            return
        return ct

    def comp(self):
        """
        DESCRIPTION
        ===========
        This function determine target Az and El from azel_list
        """
        param = self.parameters
        st = param['start_time_list']
        if st == []:
            return

        ct = self.time_check(st)
        if ct == None or param['az_list'] == []:
            return

        else:
            try:
                num = numpy.where(numpy.array(st) > ct)[0][0]
                if len(param['az_list']) > num:
                    if num == 0:
                        az_1 = az_2 =  param['az_list'][num]
                        el_1 = el_2 =  param['el_list'][num]
                        st2 = st[num]
                    else:
                        az_1 = param['az_list'][num-1]
                        az_2 = param['az_list'][num]
                        el_1 = param['el_list'][num-1]
                        el_2 = param['el_list'][num]
                        st2 = st[num-1]
                else:
                    return
                return (az_1,az_2,el_1,el_2,st2)
            except Exception as e:
                rospy.logerr(e)
                return

    def act_azel(self):
        while True:
            if self.lock_az or self.lock_el:
                time.sleep(0.1)
                continue

            ret = self.comp()
            if ret == None:
                time.sleep(0.1)
                continue
            else:
                hensa_az = ret[1] - ret[0]
                hensa_el = ret[3] - ret[2]
                current_time = time.time()
                start_time = ret[4]
                tar_az = ret[0] + hensa_az*(current_time-start_time)*10
                tar_el = ret[2] + hensa_el*(current_time-start_time)*10

                self.topic_az.publish(tar_az/3600.)
                self.topic_el.publish(tar_el/3600.)
        return

    def stop_move(self, q):
        if time.time() - self.start_time <1:
            return
        if q.data:
            self.parameters['az_list'] = []
            self.parameters['el_list'] = []
            self.parameters['start_time_list'] = []
        return


if __name__ == "__main__":
    rospy.init_node(name)
    ant = Antenna()
    ant.act_azel()
