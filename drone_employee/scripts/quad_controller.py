#!/usr/bin/env python

from mission_wrapper import waypointWrap
from quad_commander import *

from rospy import init_node, spin, sleep, Subscriber, Publisher
from small_atc_msgs.msg import *
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool
from threading import Thread
from Queue import Queue

def drop_all(queue):
    while not queue.empty():
        queue.get()

def take_current(queue):
    drop_all(queue)
    return queue.get(True, None)

def quad_controller(target_queue, position_queue, arming_queue):
    target = target_queue.get(True, None)
    print('Received #'+str(target.id)+' mission')
    if not target.valid:
        print('Route invalid!')
        return
    # Write a mission
    push_mission(waypointWrap(target.route))
    print('Mission created')
    # Set manual mode
    set_mode('ACRO')
    # Enable motors 
    arming()
    # Set autopilot mode
    set_mode('AUTO')
    print('Flight!')
    # Wainting for arming
    sleep(5)
    drop_all(arming_queue)
    while arming_queue.get().data:
        pass
    print('Mission complete')

def main():
    ''' The main routine '''
    init_node('dron_employee_ros')
    # Create queues
    target_queue = Queue()
    position_queue = Queue()
    arming_queue = Queue()
    # Create controller thread
    controller = Thread(target=quad_controller,
                        args=(target_queue, position_queue, arming_queue))
    
    # Target message handler
    def quad_target(msg):
        if not controller.isAlive():
            controller.start()
            target_queue.put(msg)
        else:
            print('Target already exist!')
    
    Subscriber('mavros/global_position/global', NavSatFix, position_queue.put) 
    Subscriber('armed', Bool, arming_queue.put)
    Subscriber('target', RouteResponse, quad_target)
    spin()
            
if __name__ == '__main__':
    main()
