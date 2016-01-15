from mavros_msgs.msg import Waypoint, CommandCode

def waypoint(lat, lon, alt, delay):
    w = Waypoint()
    w.frame = Waypoint.FRAME_GLOBAL
    w.command = CommandCode.NAV_WAYPOINT
    w.is_current = False
    w.autocontinue = True
    w.param1 = delay # Hold time in mession
    w.param2 = 2     # Position trashold in meters
    w.x_lat = lat
    w.y_long = lon
    w.z_alt = alt
    return w

def rtl():
    w = Waypoint()
    w.command = CommandCode.NAV_RETURN_TO_LAUNCH
    return w

def waypointWrap(pointList):
    wps = [waypoint(p.latitude, p.longitude, p.altitude, 10) for p in pointList]
    wps.append(rtl())
    return wps
