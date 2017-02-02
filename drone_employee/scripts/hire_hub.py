#!/usr/bin/env python
# -*- coding: utf-8 -*- 
from sqlalchemy import Column, Integer, String, Sequence, ForeignKey
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import relationship
from conf import DB_CONN_STRING 
from quad_commander import *
import rosservice
import rospy

## Declarative database synonym
Base = declarative_base()

class DroneMove(Base):
    __tablename__ = 'drone_move'
    id   = Column(Integer, Sequence('drone_move_id_seq'), primary_key=True)
    move = Column(String)

def spawn_db():
    db = create_engine(DB_CONN_STRING, client_encoding='utf8')
    return sessionmaker(bind=db)

if __name__ == '__main__':
    db = spawn_db()
    rospy.init_node('drone_hire')
    rospy.wait_for_service('mission/set_current')
    mission = rospy.ServiceProxy('mission/set_current', WaypointSetCurrent)

    while len(db.query(DroneMove).all()) == 0:
        rospy.sleep(1)

    takeoff()
    rospy.sleep(5)
    set_mode('AUTO')

    while not rospy.is_shutdown():
        if len(db.query(DroneMove).all()) > 0:
            act = db.query(DroneMove).first()
            db.query(DroneMove).all().delete()
            rospy.loginfo('Received command: ' + act.move)
            if act.move == 'A':
                mission(2)
            elif act.move == 'B':
                mission(3)
            elif act.move == 'C':
                mission(4)
            elif act.move == 'Home':
                set_mode('RTL')
        else:
            rospy.sleep(1)
