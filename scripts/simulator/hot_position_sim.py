#!/usr/bin/env python3

name = "hot_position_sim"

import time
import threading
import rospy
import std_msgs.msg


class hot_position_sim(object):
<<<<<<< HEAD
    
    cmd = ''
    #pos_status = 5000
=======


>>>>>>> c26eefd6d9ae55c2af10855bb2a4ccf85f4e68f9

    def __init__(self):
        self.pos_status =  ""

        self.topic_to = rospy.Publisher(
                    name = "/opuctrl/cpz7415v_rsw0_z_step",
                    data_class = std_msgs.msg.Int64,
                    queue_size = 1,
                    latch = True
                )

        self.topic_from = rospy.Subscriber(
                    name = '/opuctrl/cpz7415v_rsw0_z_step_cmd',
                    data_class = std_msgs.msg.Int64,
                    callback = self.update_bit_status,
                    queue_size = 1,
                )

<<<<<<< HEAD
        self.cmd = 5000

        pass

    def update_bit_status(self, command):
            self.cmd = "NASCO"
        else:
            self.cmd = "SMART"
        return

   # def update_bit_status(self, command):
        #self.pos_status = command.data
        #return

    def publish_hot(self):
=======

    def update_bit_status(self, command):
        self.pos_status = command.data

>>>>>>> c26eefd6d9ae55c2af10855bb2a4ccf85f4e68f9

    def move(self):
        while not rospy.is_shutdown():
            pos = self.pos_status

            if pos == "" :
                self.topic_to.publish(0)
                pos2 = 0

            if pos == 0 and pos2 == 5000 :
                for i in range(5):
                    pos2 = 5000-1000*(i+1)
                    self.topic_to.publish(pos2)

            if pos == 5000 and pos2 ==0 :
                for i in range(5):
                    pos2 = 0+1000*(i+1)
                    self.topic_to.publish(pos2)
                    time.sleep(1)

            else:
                pass

            time.sleep(0.1)

            continue


if __name__ == "__main__":
    rospy.init_node(name)
    hot_sim = hot_position_sim()

    pub_thread = threading.Thread(
            target = hot_sim.move(),
            daemon = True
        )
    pub_thread.start()

    rospy.spin()
