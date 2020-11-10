#!/usr/bin/env python

import rospy
from std_msgs.msg import String

import yaml
import pprint
pp = pprint.PrettyPrinter(indent=1)


testProblem = """(define (problem task)
(:domain turtlebot)
(:objects
    wp0 wp1 wp2 wp3 wp4 - waypoint
    kenny - robot
)
(:init
    (robot_at kenny wp0)



    (docked kenny)


    (dock_at wp0)

    (= (charge kenny) 0)

)
(:goal (and
    (visited wp0)
    (visited wp1)
    (visited wp2)
    (visited wp3)
    (visited wp4)
    (docked kenny)
    (>  (charge kenny) 0)
))
)
"""

class PlanLibrary:

    def __init__(self):
        nodename = rospy.get_name()
        rospy.loginfo("KCL: (%s) Plan Library initialising..." % nodename)

        self.plans_path = str(rospy.get_param("~stored_plans_path"))
        self.plan_dictionary = self.load_planlib()

        self.planner_input_pub = rospy.Publisher("~"+rospy.get_param("~planner_input_topic"), String, queue_size=1)
        self.plan_lib_output_pub = rospy.Publisher("~"+rospy.get_param("~plan_from_lib_topic"), String, queue_size=1)

        rospy.Subscriber(rospy.get_param("~problem_instance"), String, self.problem_callback)
        rospy.Subscriber(rospy.get_param("~planner_output"), String, self.planner_callback)


    # Recieve plan and save it into the planLib object
    def planner_callback(self, data):
        self.latest_plan = data.data
        self.latest_plan_time = rospy.Time.now()
        print(self.latest_plan)

        self.plan_lib_output_pub.publish(self.latest_plan)

    # Recieve problem and save it into the planLib object
    def problem_callback(self, data):
        self.latest_problem = data.data
        self.latest_problem_time = rospy.Time.now()

        print(self.latest_problem)
        # print(self.trim_plan_and_problem(self.latest_problem))

        self.planner_input_pub.publish(self.latest_problem)


    # take the problem file and parse it into the planLib object format needed
    def trim_plan_and_problem(self, string_to_trim):
        trimmed = string_to_trim.replace("\n", "")

        return trimmed

    # Take the current plans from the planLib object and parse them back into the yaml format.
    def save_planlib(self, line):
        with open(self.plans_path, 'a+') as myplans:
            myplans.write(line)

    # Open the yaml file and load the plans into the planLib object
    def load_planlib(self):
        rospy.loginfo("Loading the plans from %s" %self.plans_path)

        with open(self.plans_path) as myplans:
            loaded_plan_lib = yaml.load(myplans, Loader=yaml.FullLoader)
            pp.pprint(type(loaded_plan_lib))

        return loaded_plan_lib


if __name__ == "__main__":

    try:
        rospy.init_node("plan_library")
        PlanLib = PlanLibrary()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
