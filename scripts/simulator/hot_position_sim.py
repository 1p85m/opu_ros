#!/usr/bin/env python3

name = "hot_position_sim"

import time
import threading
import rospy
import std_msgs.msg


class hot_position_sim(object):

    pos_status = 5000

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
        return

    def publish_hot(self):
        while not rospy.is_shutdown():
            pos = self.pos_status
            if pos == 5000 :
                self.topic_to.publish(2500)
                time.sleep(5)
                self.topic_to.publish(5000)
                #time.sleep(5)
                #break
            elif pos == 0  :
                self.topic_to.publish(2500)
                time.sleep(5)
                self.topic_to.publish(0)
                #time.sleep(5)
                #break
            else:
                pass
        #self.pos_status = 5000
            time.sleep(0.1)
            #continue

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
