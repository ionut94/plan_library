#!/usr/bin/env python

import rospy
from std_msgs.msg import String

class PlanLibrary:

    def __init__(self):
        nodename = rospy.get_name()
        rospy.loginfo("KCL: (%s) Ready to receive some PLANZ" % nodename)

        self.plans_path = str(rospy.get_param("~stored_plans_path"))
        # self.plan_dictionary = self.load_planlib()

        rospy.Subscriber("rosplan_problem_interface/problem_instance", String, self.problem_callback)
        rospy.Subscriber("rosplan_planner_interface/planner_output", String, self.planner_callback)


    def planner_callback(self, data):
        self.latest_plan = data.data
        self.latest_plan_time = rospy.Time.now()

    def problem_callback(self, data):
        self.latest_problem = data.data
        self.latest_problem_time = rospy.Time.now()

        print(data.data)

    def trim_plan_problem(self):


    def save_planlib(self, line):
        with open(self.plans_path, 'a+') as myplans:
            myplans.write(line)

    def load_planlib(self):
        # print(self.plans_path)
        with open(self.plans_path) as myplans:
            for line in myplans:
                print (line)

        return {}



if __name__ == "__main__":

    try:
        rospy.init_node("plan_library")
        PlanLib = PlanLibrary()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
