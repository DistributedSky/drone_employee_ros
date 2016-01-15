#!/usr/bin/env python

from rospy import init_node, spin, Subscriber, Publisher
from mavros_msgs.msg import State
from std_msgs.msg import Bool

def main():
    init_node('state_publisher')
    # Create publishers
    armed_pub = Publisher('armed', Bool, queue_size=1)
    
    # TODO: Battery charger monitor thread
    #ready_pub = Publisher('/dron_employee/battery_full', Bool, queue_size=1)

    # State handler
    def dron_sate(msg):
        armed = Bool()
        armed.data = msg.armed
        armed_pub.publish(armed)
    
    Subscriber('mavros/state', State, dron_sate)
    spin()

if __name__ == '__main__':
    main()
