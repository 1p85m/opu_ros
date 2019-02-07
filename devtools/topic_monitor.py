#! /usr/bin/env python3

name = 'topic_monitor'

# ----
import time
import threading
import rospy
import std_msgs.msg


class topic_monitor(object):
    values = {}
    refreshing = False
    
    def __init__(self):
        def new(name, data_class):
            rospy.Subscriber(
                name = name,
                data_class = data_class,
                callback = self.callback,
                callback_args = {'name': name},
                queue_size = 1,
            )
            return            
            

        # Antenna
        # -------
        new('az', std_msgs.msg.Float64)
        new('/antenna/az_cmd', std_msgs.msg.Float64)
        new('/antenna/az_soft_limit', std_msgs.msg.Bool)
        new('az_speed', std_msgs.msg.Float64)
        new('el', std_msgs.msg.Float64)
        new('/antenna/el_cmd', std_msgs.msg.Float64)
        new('/antenna/el_soft_limit', std_msgs.msg.Bool)
        new('el_speed', std_msgs.msg.Float64)

        new('/opuctrl/cpz7415v_rsw0_x_speed_cmd', std_msgs.msg.Int64)
        new('/opuctrl/cpz7415v_rsw0_y_speed_cmd', std_msgs.msg.Int64)
        """ 
        new('/cpz7415v_rsw0_x_speed_cmd', std_msgs.msg.Int64)
        new('/cpz7415v_rsw0_y_speed_cmd', std_msgs.msg.Int64)

        # Encoder
        # -------
        new('encoder_az', std_msgs.msg.Float64)
        new('encoder_el', std_msgs.msg.Float64)
        """

        # Hot
        # ---
        new('/hot/position', std_msgs.msg.String)
        new('/hot/position_cmd', std_msgs.msg.String)
        new('/hot/position_lock', std_msgs.msg.Bool)
        
        """
        new('/cpz7415v_rsw0_z_step_cmd', std_msgs.msg.Int64)
        new('/cpz7415v_rsw0_z_step', std_msgs.msg.Int64)
        """
        # Spectrometer
        # ------------
        new('/spectrometer/data1_reader', std_msgs.msg.Float64MultiArray)
        new('/spectrometer/data2_reader', std_msgs.msg.Float64MultiArray)

        pass
    
    def callback(self, msg, args):
        self.values[args['name']] = msg.data
        self.refresh()
        return

    def refresh(self):
        while self.refreshing == True:
            time.sleep(0.1)
            continue
        
        self.refreshing = True
        maxlen = max([len(_k) for _k in self.values.keys()])
        print('----')
        for key in sorted(self.values):
            print(('{0:<'+str(maxlen)+'} {1}').format(key, self.values[key]))
            continue
        self.refreshing = False
        return
    


if __name__=='__main__':
    rospy.init_node(name)
    tm = topic_monitor()
    rospy.spin()
