#!/usr/bin/env python3

name = 'controller'

# ----
import time

import rospy
import std_msgs.msg


class controller(object):

    def __init__(self):
        rospy.init_node(name)
        self.ps = PS()

        # ----
        self.antenna = ANTENNA()
        self.hot = HOT()
        self.spectrometer = SPECTROMETER()
        pass

    def display_publisher(self):
        [print(k) for k in self.ps.pub]
        return


class PS(object):
    pub = {
            #"topic_name":rospy.Publisher(name, data_class, queue_size, latch)
            }

    def __init__(self):
        pass

    def publish(self, topic_name, msg):
        self.pub[topic_name].publish(msg)
        return

    def set_publisher(self, topic_name, data_class, queue_size, latch=True):
        if topic_name not in self.pub:
            self.pub[topic_name] = rospy.Publisher(
                                            name = topic_name,
                                            data_class = data_class,
                                            queue_size = queue_size,
                                            latch = latch,
                                        )
            time.sleep(0.01)
        else: pass
        return


class ANTENNA(object):

    def __init__(self):
        self.ps = PS()
        pass

    """
    def _az_move(self, command): # deg
        name = "/antenna/az_cmd"

        self.ps.set_publisher(
                topic_name = name,
                data_class = std_msgs.msg.Float64,
                queue_size = 1,
                latch = False
            )

        self.ps.publish(topic_name=name, msg=command)
        return

    def _el_move(self, command): # deg
        name = "/antenna/el_cmd"

        self.ps.set_publisher(
                topic_name = name,
                data_class = std_msgs.msg.Float64,
                queue_size = 1,
                latch = False
            )

        self.ps.publish(topic_name=name, msg=command)
        return
    """

    def onepoint_move(self, ps_x, ps_y, ps_coord="altaz", ps_planet="", ps_off_x=0, ps_off_y=0, ps_offcoord="altaz", ps_hosei="hosei_230.txt", ps_lamda=2600, ps_dcos=0, ps_limit=True):
        target = {
                "ps_x": [ps_x, std_msgs.msg.Float64],
                "ps_y": [ps_y, std_msgs.msg.Float64],
                "ps_coord": [ps_coord, std_msgs.msg.String],
                "ps_planet": [ps_planet, std_msgs.msg.String],
                "ps_off_x": [ps_off_x, std_msgs.msg.Float64],
                "ps_off_y": [ps_off_y, std_msgs.msg.Float64],
                "ps_offcoord": [ps_offcoord, std_msgs.msg.String],
                "ps_hosei": [ps_hosei, std_msgs.msg.String],
                "ps_lamda": [ps_lamda, std_msgs.msg.Float64],
                "ps_dcos": [ps_dcos, std_msgs.msg.Float64],
                "ps_limit": [ps_limit, std_msgs.msg.Bool],
                "ps_timestamp": [time.time(), std_msgs.msg.Float64],
                }

        [self.ps.set_publisher(
                topic_name = "/obs/{}".format(tar),
                data_class = target[tar][1],
                queue_size = 1,
                latch = True
            ) for tar in target]

        [self.ps.publish(topic_name="/obs/{}".format(tar), msg=target[tar][0]) for tar in target]
        return

    def otf_scan(self, otf_x, otf_y, otf_coord_sys="altaz", otf_dx=1, otf_dy=1, otf_dt=1, otf_num=5, otf_rampt=3, otf_delay=0, otf_off_x=0, otf_off_y=0, otf_offcoord="j2000", otf_dcos=0, otf_hosei="hosei_230.txt", otf_lamda=2600, otf_limit=True):
        target = {
                "otf_x": [otf_x, std_msgs.msg.Float64],
                "otf_y": [otf_y, std_msgs.msg.Float64],
                "otf_dx": [otf_dx, std_msgs.msg.Float64],
                "otf_dy": [otf_dy, std_msgs.msg.Float64],
                "otf_dt": [otf_dt, std_msgs.msg.Float64],
                "otf_num": [otf_num, std_msgs.msg.Float64],
                "otf_rampt": [otf_rampt, std_msgs.msg.Float64],
                "otf_delay": [otf_delay, std_msgs.msg.Float64],
                "otf_coord_sys": [otf_coord_sys, std_msgs.msg.String],
                "otf_off_x": [otf_off_x, std_msgs.msg.Float64],
                "otf_off_y": [otf_off_y, std_msgs.msg.Float64],
                "otf_offcoord": [otf_offcoord, std_msgs.msg.String],
                "otf_hosei": [otf_hosei, std_msgs.msg.String],
                "otf_lamda": [otf_lamda, std_msgs.msg.Float64],
                "otf_dcos": [otf_dcos, std_msgs.msg.Float64],
                "otf_limit": [otf_limit, std_msgs.msg.Bool],
                "otf_timestamp": [time.time(), std_msgs.msg.Float64],
                }

        [self.ps.set_publisher(
                topic_name = "/obs/{}".format(tar),
                data_class = target[tar][1],
                queue_size = 1,
                latch = True
            ) for tar in target]

        [self.ps.publish(topic_name="/obs/{}".format(tar), msg=target[tar][0]) for tar in target]
        return

    def stop(self):
        name = "/obs/stop_cmd"

        self.ps.set_publisher(
                topic_name = name,
                data_class = std_msgs.msg.Bool,
                queue_size = 1,
                latch = True
            )

        self.ps.publish(topic_name=name, msg=True)
        return

class HOT(object):

    def __init__(self):
        self.ps = PS()
        pass

    def position(self, command):
        name = "/hot/position_cmd"

        self.ps.set_publisher(
                topic_name = name,
                data_class = std_msgs.msg.String,
                queue_size = 1,
                latch = True
            )

        self.ps.publish(topic_name=name, msg=command)
        return

class SPECTROMETER(object):

    def __init__(self):
        self.ps = PS()
        pass

    def oneshot(self, exposure):
        name = "/spectrometer/oneshot_cmd"

        self.ps.set_publisher(
                topic_name = name,
                data_class = std_msgs.msg.Float32,
                queue_size = 1,
                latch = True
            )

        self.ps.publish(topic_name=name, msg=exposure)
        return
