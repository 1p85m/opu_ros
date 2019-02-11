#!/usr/bin/env python3

name = "hot_position_sim"

import time
import threading
import rospy
import std_msgs.msg


class hot_position_sim(object):

    pos_status = ""

    def __init__(self):

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

        pass

    def update_bit_status(self, command):
        self.pos_status = command.data


    def move(self):

        while not rospy.is_shutdown():
            pos = self.pos_status
            if pos == "" :
                self.topic_to.piblish(0)

            if pos == 0 :
                self.topic_to.publish(2500)
                self.topic_to.publish(0)
                for i in range(4)
                    pos = 5000-1000*(i+1)
                    self.topic_to.publish(pos)
                pos = ""

            if pos == 5000 :
                for i in range(4)
                    pos = 0+1000*(i+1)
                    self.topic_to.publish(pos)
                pos = ""

            else:
                pass
            time.sleep(0.1)


        return


if __name__ == "__main__":
    rospy.init_node(name)
    hot_sim = hot_position_sim()

    pub_thread = threading.Thread(
            target = hot_sim.publish_hot(),
            daemon = True
        )
    pub_thread.start()

    rospy.spin()
