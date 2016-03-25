from mavros_msgs.msg import Waypoint, CommandCode

def waypoint(lat, lon, alt, delay):
    w = Waypoint()
    w.frame = 3 # Currently is relative to HOME frame TODO: set global frame 
    w.command = CommandCode.NAV_WAYPOINT
    w.is_current = False
    w.autocontinue = True
    w.param1 = delay # Hold time in mession
    w.param2 = 2     # Position trashold in meters
    w.x_lat = lat
    w.y_long = lon
    w.z_alt = alt
    return w

def waypointWrap(pointList):
    wps = [waypoint(p.latitude, p.longitude, p.altitude, 10) for p in pointList]
    # Set current waypoint to first
    wps[0].is_current = True
    # Set first/last waypoint to takeoff/land
    wps[0].command = CommandCode.NAV_TAKEOFF
    wps[-1].command = CommandCode.NAV_LAND
    return wps
