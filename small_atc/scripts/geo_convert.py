#!/usr/bin/python2

from geo_helper import satFix2Point, point2SatFix
from geometry_msgs.msg import Point
from small_atc_msgs.msg import SatFix
from sys import argv

DESCRIPTION = '''DESCRIPTION:\n\tThe script for simple convert from SatFix to Point and from Point to SatFix'''
USAGE = '''USAGE:\n\t{0} -p x y z\n\t{0} -s lat lon alt'''.format(argv[0])

def main():

    def wrong_usage():
        print(USAGE)
        exit(1)

    def convert_init(cls, members_name, init_mass):
        res = cls()
        for i in zip(members_name, map(float, init_mass)):
            res.__setattr__(*i)
        return res

    if argv[1] == '-h':
        print DESCRIPTION
        print USAGE
        exit(0)

    if len(argv) != 5:
        wrong_usage()

    if argv[1] == '-p':
        print 'convert to SatFix'
        try:
            point = convert_init(Point, ['x', 'y', 'z'], argv[2:])
        except ValueError:
            wrong_usage()
        print point2SatFix(point)

    elif argv[1] == '-s':
        print 'convert to Point'
        try:
            satfix = convert_init(SatFix, ['latitude', 'longitude', 'altitude'], argv[2:])
        except ValueError:
            wrong_usage()
        print satFix2Point(satfix)

    else:
        wrong_usage()

if __name__ == '__main__':
    main()
