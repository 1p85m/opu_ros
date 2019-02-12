#!/usr/bin/env python3

name = 'controller'

# ----
import time

import rospy
import std_msgs.msg
import necst.msg


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
                "ps_y": [ps_y, std_msgs.mgs.Float64],
                "ps_coord": [ps_coord, std_msgs.mgs.String],
                "ps_planet": [ps_planet, std_msgs.mgs.String],
                "ps_off_x": [ps_off_x, std_msgs.mgs.Float64],
                "ps_off_y": [ps_off_y, std_msgs.mgs.Float64],
                "ps_offcoord": [ps_offcoord, std_msgs.mgs.String],
                "ps_hosei": [ps_hosei, std_msgs.mgs.String],
                "ps_lamda": [ps_lamda, std_msgs.mgs.Float64],
                "ps_dcos": [ps_dcos, std_msgs.mgs.Float64],
                "ps_limit": [ps_limit, std_msgs.mgs.Bool],
                }

        [self.ps.set_publisher(
                topic_name = "/obs/{}".format(tar),
                data_class = target[tar][1],
                queue_size = 1,
                latch = True
            ) for tar in target]

        [self.ps.publish(topic_name="/obs/{}".format(tar), msg=target[tar][0]) for tar in target]
        return

    def otf_scan(self, otf_x, otf_y, otf_coord_sys, otf_dx, otf_dy, otf_dt, num, rampt, delay, current_time, off_x=0, off_y=0, offcoord="j2000", dcos=0, hosei="hosei_230.txt", lamda=2600, limit=True):
        target = {
                "ps_x": [ps_x, std_msgs.msg.Float64],
                "ps_y": [ps_y, std_msgs.mgs.Float64],
                "ps_coord": [ps_coord, std_msgs.mgs.String],
                "ps_planet": [ps_planet, std_msgs.mgs.String],
                "ps_off_x": [ps_off_x, std_msgs.mgs.Float64],
                "ps_off_y": [ps_off_y, std_msgs.mgs.Float64],
                "ps_offcoord": [ps_offcoord, std_msgs.mgs.String],
                "ps_hosei": [ps_hosei, std_msgs.mgs.String],
                "ps_lamda": [ps_lamda, std_msgs.mgs.Float64],
                "ps_dcos": [ps_dcos, std_msgs.mgs.Float64],
                "ps_limit": [ps_limit, std_msgs.mgs.Bool],
                }

        self.ps.set_publisher(
                topic_name = name,
                data_class = necst.msg.Otf_mode_msg,
                queue_size = 1,
                latch = True
            )

        self.ps.publish(topic_name=name, msg=command)
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

