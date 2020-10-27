#!/usr/bin/env python

import rospy
from std_msgs.msg import String


class PlanLibrary:

    def __init__(self):
        nodename = rospy.get_name()
        rospy.loginfo("%s started" % nodename)

        rospy.Subscriber("rosplan_planner_interface/planner_output", String, self.planner_callback)

    def planner_callback(self, data):
        self.latest_plan = data.data
        self.latest_plan_time = rospy.Time.now()

        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


if __name__ == "__main__":

    try:
        rospy.init_node("plan_library")
        PlanLib = PlanLibrary()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
