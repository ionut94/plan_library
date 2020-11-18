#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import yaml
# import pprint
# pp = pprint.PrettyPrinter(indent=1)


class PlanLibrary:

    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("KCL: (%s) Plan Library initialising..." % self.node_name)

        self.plans_path = str(rospy.get_param("~stored_plans_path"))
        self.plan_dictionary = self.load_plan_library()
        self.problem_dictionary = {}

        self.planner_input_pub = rospy.Publisher("~" + rospy.get_param("~planner_input_topic"), String, queue_size=1)
        self.plan_lib_output_pub = rospy.Publisher("~" + rospy.get_param("~plan_from_lib_topic"), String, queue_size=1)

        rospy.Subscriber(rospy.get_param("~problem_instance"), String, self.problem_callback)
        rospy.Subscriber(rospy.get_param("~planner_output"), String, self.planner_callback)

    # Receive plan and save it into the planLib object
    def planner_callback(self, data):
        self.latest_plan = data.data
        self.latest_plan_time = rospy.Time.now()

        self.problem_dictionary["test"].update({"plan": self.latest_plan})
        self.plan_dictionary["test"] = self.problem_dictionary["test"]

        self.save_plan_library()

        self.plan_lib_output_pub.publish(self.latest_plan)

    # Receive problem and save it into the planLib object
    def problem_callback(self, data):
        self.latest_problem = data.data
        self.latest_problem_time = rospy.Time.now()

        self.problem_dictionary["test"] = self.parse_problem_to_dict(self.latest_problem)

        check, plans = self.check_for_plan_in_library()
        if check:
            # self.latest_problem
            rospy.loginfo("KCL: (%s) There is a plan for this problem in the plan library" % self.node_name)
            self.plan_lib_output_pub.publish(plans[0])
        else:
            rospy.loginfo("KCL: (%s) Problem not in the plan library" % self.node_name)
            self.planner_input_pub.publish(self.latest_problem)

    # Check if the latest problem is already in our plan library
    # TODO Implement a more robust version of searching for a plan in the PlanLib
    def check_for_plan_in_library(self):
        count = 0
        plans = []

        for problem, data in self.plan_dictionary.iteritems():
            if data['problem'] in self.latest_problem.replace("\n", ""):
                count += 1
                plans.append(data['plan'])

        if count > 0:
            return True, plans
        else:
            return False, plans

    # take the problem file and parse it into the planLib object format needed
    def parse_problem_to_dict(self, string_to_trim):
        status = [False, False, False]
        object_dict = {}
        init_dict = {}
        goal_dict = {}

        for line in string_to_trim.split("\n"):
            if ":objects" in line:
                status = [True, False, False]
            elif ":init" in line:
                status = [False, True, False]
            elif ":goal" in line:
                status = [False, False, True]
            elif (status[0] or status[1] or status[2]) and len(line.replace("(", "").replace(")", "")) > 0:

                if status[0]:
                    object_dict[line.split("-")[1]] = filter(None, line.split("-")[0].split(" "))
                elif status[1]:
                    self.process_line_for_parsing(init_dict, line)
                elif status[2]:
                    self.process_line_for_parsing(goal_dict, line)

        problem_dict = {"problem": string_to_trim.replace("\n", ""), "object": object_dict, "init": init_dict,
                        "goal": goal_dict}

        return problem_dict

    # Parse each line from init and goal of the problem
    @staticmethod
    def process_line_for_parsing(line_dict, line):
        line_split = filter(None, line.replace("(", "").replace(")", "").split(" "))
        if line_split[0] in "><=":
            if line_split[1] in line_dict.keys():
                line_dict[line_split[1]].append([line_split[0]] + line_split[2:])
            else:
                line_dict[line_split[1]] = [[line_split[0]] + line_split[2:]]

        else:
            if line_split[0] in line_dict.keys():
                line_dict[line_split[0]].append(line_split[1:])
            else:
                line_dict[line_split[0]] = [line_split[1:]]

    # Take the current plans from the planLib object and parse them back into the yaml format.
    def save_plan_library(self):
        stream = file(self.plans_path, 'w')
        yaml.dump(self.plan_dictionary, stream, default_flow_style=False)

    # Open the yaml file and load the plans into the planLib object
    def load_plan_library(self):
        rospy.loginfo("KCL: (%s) Loading plans from %s" % (self.node_name, self.plans_path))

        with open(self.plans_path) as plan_lib:
            loaded_plan_lib = yaml.load(plan_lib, Loader=yaml.FullLoader)

        return loaded_plan_lib


if __name__ == "__main__":
    try:
        rospy.init_node("plan_library")
        PlanLib = PlanLibrary()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
