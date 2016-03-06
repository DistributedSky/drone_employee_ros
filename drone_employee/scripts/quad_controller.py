#!/usr/bin/env python

from mission_wrapper import waypointWrap
from quad_commander import *
from geo_math import len_diff_coord

from rospy import init_node, is_shutdown, spin, sleep, logerr, loginfo, logwarn
from rospy import Subscriber, Publisher
from small_atc_msgs.msg import *
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, UInt32, Int64
from mavros_msgs.msg import WaypointList
from threading import Thread
from Queue import Queue, Empty

def drop_all(queue):
    while not queue.empty():
        queue.get()

def take_current(queue):
    drop_all(queue)
    return queue.get(True, None)

def find_waypoint_id(route, target_pos):
    for i in range(0, len(route.route)):
        rt = route.route[i]
        if rt.latitude == target_pos[0] and rt.longitude == target_pos[1]:
            return i

def find_cur_waypoint_id(route):
    for i in range(0, len(route)):
        if route[i].is_current:
            return i

def drop_cargo():
    logwarn("DROP!!!")

def quad_controller(route_queue, position_queue, arming_queue,
                    trgt_queue, waypoints_queue):
    flight_done_pub = Publisher('remove', UInt32, queue_size=1)
    while not is_shutdown():
        # Take a route
        route = route_queue.get(True, None)
        loginfo('Received route with id #'+str(route.id)+'.')
        if not route.valid:
            logerr('Route invalid!')
            return
        target_pos = trgt_queue.get(True, None)
        need_waypoint_id = None
        
        # Determine the point where you want to dump the load
        need_waypoint_id = find_waypoint_id(route, (target_pos.latitude, target_pos.longitude))
        loginfo('need_waypoint_id: ' + str(need_waypoint_id))

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
        sleep(5)
        drop_all(arming_queue)

        # If you do not have a goal to clear cargo,
        # we believe that already dropped
        droped = (need_waypoint_id == None)
        while arming_queue.get().data and not droped:
            waypoints = None
            # To not accumulate elements in arming_queue
            try:
                waypoints = waypoints_queue.get_nowait().waypoints
            except Empty:
                continue
            current_waypoint_id = find_cur_waypoint_id(waypoints)
            if current_waypoint_id > need_waypoint_id:
                droped = True
                drop_cargo()

        # TODO: More elegant case for mission finish check
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
    trgt_queue = Queue()
    waypoints_queue = Queue()
    # Create controller thread
    controller = Thread(target=quad_controller,
                        args=(route_queue, position_queue, arming_queue,
                              trgt_queue, waypoints_queue))

    # Route message handler
    def quad_route(msg):
        if not controller.isAlive():
            controller.start()
        route_queue.put(msg)

    Subscriber('mavros/global_position/global', NavSatFix, position_queue.put)
    Subscriber('armed', Bool, arming_queue.put)
    # TODO: combine next two topic in one
    Subscriber('target_request', NavSatFix, trgt_queue.put)
    Subscriber('mavros/mission/waypoints', WaypointList, waypoints_queue.put)
    Subscriber('route', RouteResponse, quad_route)
    spin()

if __name__ == '__main__':
    main()
