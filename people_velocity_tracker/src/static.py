#!/usr/bin/python

import roslib; roslib.load_manifest('people_velocity_tracker')
import rospy
import sys
from people_velocity_tracker.msg import PersonPositionAndVelocity

class VelocityTracker:
    def __init__(self):
        self.ppub = rospy.Publisher('/people', PersonPositionAndVelocity)

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            pv = PersonPositionAndVelocity()
            pv.header.stamp = rospy.Time.now()
            pv.header.frame_id = '/base_link'
            pv.position.x = float(sys.argv[1])
            pv.position.y = float(sys.argv[2])
            pv.position.z = .5
            pv.velocity.x = float(sys.argv[3])
            pv.velocity.y = float(sys.argv[4])
            pv.id = 'asdf'
            pv.reliability = .90       
            self.ppub.publish(pv)
            rate.sleep()

    def publish(self):        
        gen.counter = 0
        for p in self.people.values():
            p.publish_markers(self.mpub)
            self.ppub.publish(p.get_object())

rospy.init_node("people_velocity_tracker")
vt = VelocityTracker()
vt.spin()
