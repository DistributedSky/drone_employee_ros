import math

def len_diff_coord(lat1, lon1, lat2, lon2):
     R = 6378.137 # Radius of earth in KM
     dLat = (lat2 - lat1) * math.pi / 180
     dLon = (lon2 - lon1) * math.pi / 180;
     a = (math.sin(dLat/2) * math.sin(dLat/2) + 
          math.cos(lat1 * math.pi / 180) * math.cos(lat2 * math.pi / 180) *
          math.sin(dLon/2) * math.sin(dLon/2))
     c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
     d = R * c
     return d * 1000 # meters
