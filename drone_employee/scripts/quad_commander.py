from mavros_msgs.srv import *
import rospy

def push_mission(waypoints):
    rospy.wait_for_service('mavros/mission/push')
    try:
        service = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
        service(waypoints)
    except rospy.ServiceException, e:
        print('Service call failed: {0}'.format(e))

def set_mode(mode):
    rospy.wait_for_service('mavros/set_mode')
    try:
        service = rospy.ServiceProxy('mavros/set_mode', SetMode)
        service(0, mode)
    except rospy.ServiceException, e:
        print('Service call failed: {0}'.format(e))

def arming():
    rospy.wait_for_service('mavros/cmd/arming')
    try:
        service = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
        service(True)
    except rospy.ServiceException, e:
        print('Service call failed: {0}'.format(e))

def takeoff():
    rospy.wait_for_service('mavros/cmd/takeoff')
    try:
        service = rospy.ServiceProxy('mavros/cmd/takeoff', CommandBool)
        service(True)
    except rospy.ServiceException, e:
        print('Service call failed: {0}'.format(e))
