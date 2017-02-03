#!/usr/bin/env python
# -*- coding: utf-8 -*- 
from sqlalchemy import create_engine, Column, Integer, String, Sequence, ForeignKey
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm.session import sessionmaker
from conf import DB_CONN_STRING 
from quad_commander import *
import rosservice
import rospy

## Declarative database synonym
Base = declarative_base()

class DroneMove(Base):
    __tablename__ = 'drone_move'
    id    = Column(Integer, Sequence('drone_move_id_seq'), primary_key=True)
    point = Column(String)

def spawn_db():
    db = create_engine(DB_CONN_STRING, client_encoding='utf8')
    return sessionmaker(bind=db)()

if __name__ == '__main__':
    db = spawn_db()
    rospy.init_node('drone_hire')
    rospy.wait_for_service('mavros/mission/set_current')
    mission = rospy.ServiceProxy('mavros/mission/set_current', WaypointSetCurrent)
    rospy.loginfo('mission service found')

    while len(db.query(DroneMove).all()) == 0:
        rospy.sleep(1)

    rospy.loginfo("command received, activating...")
    arming()
    rospy.sleep(5)
    set_mode('AUTO')
    rospy.loginfo("AUTO mode activated")

    while not rospy.is_shutdown():
        if len(db.query(DroneMove).all()) > 0:
            act = db.query(DroneMove).first()
            rospy.loginfo('received command: ' + act.point)
            db.query(DroneMove).delete()
            db.commit()
            if act.point == 'A':
                mission(2)
            elif act.point == 'B':
                mission(3)
            elif act.point == 'C':
                mission(4)
            elif act.point == 'Home':
                set_mode('RTL')
                rospy.loginfo('drone fly home, mission done')
                break
        rospy.sleep(1)
