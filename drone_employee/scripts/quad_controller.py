#!/usr/bin/env python

from mission_wrapper import waypointWrap
from quad_commander import *

from rospy import init_node, is_shutdown, spin, sleep, logerr, loginfo
from rospy import Subscriber, Publisher
from small_atc_msgs.msg import *
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, UInt32
from threading import Thread
from Queue import Queue

def drop_all(queue):
    while not queue.empty():
        queue.get()

def take_current(queue):
    drop_all(queue)
    return queue.get(True, None)

def quad_controller(route_queue, position_queue, arming_queue):
    flight_done_pub = Publisher('remove', UInt32, queue_size=1) 
    while not is_shutdown():
        # Take a route
        route = route_queue.get(True, None)
        loginfo('Received route with id #'+str(route.id)+'.')
        if not route.valid:
            logerr('Route invalid!')
            return
        # Push a mission into flight controller
        push_mission(waypointWrap(route.route))
        loginfo('Mission loaded to flight controller.')
        # Set manual mode
        set_mode('ACRO')
        # Enable motors 
        arming()
        # Set autopilot mode
        set_mode('AUTO')
        loginfo('Flight!')

        # Wainting for arming
        # TODO: More elegant case for mission finish check
        sleep(5)
        drop_all(arming_queue)
        while arming_queue.get().data:
            pass

        loginfo('Mission #'+str(route.id)+' complete!')
        flight_done_pub.publish(route.id)
        

def main():
    ''' The main routine '''
    init_node('dron_employee_ros')
    # Create queues
    route_queue = Queue()
    position_queue = Queue()
    arming_queue = Queue()
    # Create controller thread
    controller = Thread(target=quad_controller,
                        args=(route_queue, position_queue, arming_queue))
    
    # Route message handler
    def quad_route(msg):
        if not controller.isAlive():
            controller.start()
        route_queue.put(msg)
    
    Subscriber('mavros/global_position/global', NavSatFix, position_queue.put) 
    Subscriber('armed', Bool, arming_queue.put)
    Subscriber('route', RouteResponse, quad_route)
    spin()
            
if __name__ == '__main__':
    main()
