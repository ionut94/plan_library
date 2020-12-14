#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse, Empty
from rosplan_dispatch_msgs.srv import DispatchService, DispatchServiceResponse
from rosplan_knowledge_msgs.srv import GetDomainNameService, GetDomainTypeService, GetDomainOperatorService, \
    GetDomainAttributeService
import yaml
import uuid
import pprint
import time
import pandas as pd
import os

pp = pprint.PrettyPrinter(indent=1)


class PlanLibrary:

    def __init__(self):
        self.node_name = rospy.get_name()
        self.use_library = rospy.get_param("~use_library")
        self.problem_name = str(rospy.get_param("~problem_path")).split("/")[-1]
        self.action_probability = str(rospy.get_param("~action_probability"))

        if self.use_library:
            rospy.loginfo("KCL: (%s) Plan Library initialising..." % self.node_name)

            # Initialisation
            self.latest_plan = ""
            self.latest_plan_time = 0
            self.latest_problem = ""
            self.latest_problem_time = 0
            self.plans_path = str(rospy.get_param("~stored_plans_path"))
            self.problem_dictionary = {}

            # Get the domain from plan lib or initialise it from KB
            # if self.plan_dictionary is None:
            #     self.plan_dictionary = {"domain": self.get_domain_from_KB()}
            # self.domain = self.plan_dictionary["domain"]

            # Get the location of the results with plan lib
            self.results_path = str(rospy.get_param("~results_path")) + "results_with_plan_lib.csv"
        else:
            # Get the location of the results with plan lib
            rospy.loginfo("KCL: (%s) Plan Library not in use" % self.node_name)
            self.results_path = str(rospy.get_param("~results_path")) + "results_no_plan_lib.csv"

        # Load results and check if empty.
        if os.path.getsize(self.results_path) == 0:
            self.results_df = pd.DataFrame(
                columns=["name", "action probability", "planned", "used planner", "used plan lib", "time planning",
                         "time checking plan lib", "total time", "plan lib size"], index=None)
        else:
            self.results_df = pd.read_csv(self.results_path)

        # Variables for synchronisation
        self.plan_in_lib = False
        self._plan_lib_checked = False
        self._problem_sent = False

        # Variables needed for evaluation
        self.replan = 0
        self.used_planner = 0
        self.time_planning = []
        self.time_checking_plan_library = []

        # TODO make service that calls all the services needed to run plan lib until successes
        self._execute_current_problem = rospy.Service("~execute_current_problem", Trigger, self.run_problem)

        # The rosplan service calls for executing the problem
        self._problem_gen = rospy.ServiceProxy("/rosplan_problem_interface/problem_generation_server", Empty)
        self._planner = rospy.ServiceProxy("/rosplan_planner_interface/planning_server", Empty)
        self._parse_plan = rospy.ServiceProxy("/rosplan_parsing_interface/parse_plan", Empty)
        self._dispatch_plan = rospy.ServiceProxy("/rosplan_plan_dispatcher/dispatch_plan", DispatchService)

        # Create publishers to planner and plan_parser
        self.planner_input_pub = rospy.Publisher("~" + rospy.get_param("~planner_input_topic"), String, queue_size=1)
        self.plan_lib_output_pub = rospy.Publisher("~" + rospy.get_param("~plan_from_lib_topic"), String, queue_size=1)

        # Create subscribers to prob_generation and planner
        rospy.Subscriber(rospy.get_param("~problem_instance"), String, self.problem_callback)
        rospy.Subscriber(rospy.get_param("~planner_output"), String, self.planner_callback)

    def run_problem(self, srv):
        res = DispatchServiceResponse()

        self.replan = 0
        self.used_planner = 0
        self.time_planning = []
        self.time_checking_plan_library = []

        if self.use_library:
            self.plan_dictionary = self.load_plan_library()
            self.problem_dictionary = {}

            # Get the domain from plan lib or initialise it from KB
            if self.plan_dictionary is None:
                self.plan_dictionary = {"domain": self.get_domain_from_KB()}
            self.domain = self.plan_dictionary["domain"]

        while not res.goal_achieved:
            try:
                self._problem_gen.call()
                self.replan += 1

                while not self._plan_lib_checked:
                    rospy.sleep(1)
                self._plan_lib_checked = False

                if not self.plan_in_lib:
                    self.used_planner += 1
                    start = time.time()
                    self._planner.call()
                    end = time.time()
                    self.time_planning.append(end - start)

                while not self._problem_sent:
                    rospy.sleep(1)
                self._problem_sent = False

                self._parse_plan.call()
                res = self._dispatch_plan.call()
            except rospy.ServiceException as e:
                rospy.logerr(rospy.get_name() + ": Service call failed: %s" % e)

        self.write_results()

        rospy.loginfo("KCL: (%s) Had to plan %s times, using the planner for %s times and the plan library for the "
                      "other %s times" % (self.node_name, self.replan, self.used_planner, self.replan -
                                          self.used_planner))
        if self.replan > 0:
            rospy.loginfo("KCL: (%s) Total time spent planning is %s, with an average of %s" % (
                self.node_name, sum(self.time_planning), sum(self.time_planning) / len(self.time_planning)))

        if self.use_library:
            rospy.loginfo("KCL: (%s) Total time spent checking the plan library is %s, with an average of %s" % (
                self.node_name, sum(self.time_checking_plan_library),
                sum(self.time_checking_plan_library) / len(self.time_checking_plan_library)))
            self.save_plan_library()
            rospy.loginfo("KCL: (%s) Plan Library saved" % self.node_name)

        rospy.signal_shutdown()
        return TriggerResponse(True, 'The goal has been reached')

    def write_results(self):
        # HEADERS: "name", "action probability", "planned", "used planner", "used plan lib", "time planning",
        # "time checking plan lib", "total time"

        if self.use_library:
            self.results_df = self.results_df.append({"name": self.problem_name,
                                                      "action probability": self.action_probability,
                                                      "planned": self.replan,
                                                      "used planner": self.used_planner,
                                                      "used plan lib": self.replan - self.used_planner,
                                                      "time planning": sum(self.time_planning),
                                                      "time checking plan lib": sum(self.time_checking_plan_library),
                                                      "total time": sum(self.time_planning) +
                                                                    sum(self.time_checking_plan_library),
                                                      "plan lib size": len(self.plan_dictionary)
                                                      }, ignore_index=True)
        else:
            self.results_df = self.results_df.append({"name": self.problem_name,
                                                      "action probability": self.action_probability,
                                                      "planned": self.replan,
                                                      "used planner": self.used_planner,
                                                      "used plan lib": 0,
                                                      "time planning": sum(self.time_planning),
                                                      "time checking plan lib": 0,
                                                      "total time": sum(self.time_planning),
                                                      "plan lib size": 0
                                                      }, ignore_index=True)
        self.results_df.to_csv(self.results_path, index=False)

    # Receive plan and save it into the planLib object
    def planner_callback(self, data):
        self.latest_plan = data.data

        if self.use_library:
            # Add new plan to library
            self.problem_dictionary.update({"plan": self.latest_plan})
            self.plan_dictionary[str(uuid.uuid1())] = self.problem_dictionary

        self.plan_lib_output_pub.publish(self.latest_plan)
        self._problem_sent = True

    # Receive problem and save it into the planLib object
    def problem_callback(self, data):
        self.latest_problem = data.data

        self.problem_dictionary = self.parse_problem_to_dict(self.latest_problem)

        if self.use_library:
            start = time.time()
            check, plans = self.check_for_plan_in_library()
            end = time.time()
            self.time_checking_plan_library.append(end - start)

            if check:
                rospy.loginfo("KCL: (%s) There is a plan for this problem in the plan library" % self.node_name)
                self.plan_lib_output_pub.publish(plans[0])
                self.plan_in_lib = True
                self._problem_sent = True
            else:
                rospy.loginfo("KCL: (%s) Problem not in the plan library" % self.node_name)
                self.planner_input_pub.publish(self.latest_problem)
                self.plan_in_lib = False
        else:
            self.planner_input_pub.publish(self.latest_problem)
        self._plan_lib_checked = True

    # Check if the latest problem is already in our plan library
    # TODO Implement a more robust version of searching for a plan in the PlanLib
    def check_for_plan_in_library(self):
        count = 0
        plans = []

        for key, plan_lib_elem in self.plan_dictionary.iteritems():
            if key not in "domain":
                if str(plan_lib_elem['problem']) in self.latest_problem.replace("\n", ""):
                    count += 1
                    plans.append(plan_lib_elem['plan'])
                    break
                else:
                    # TODO Check for goals
                    pass

                    # if all(elem in plan_lib_elem["goal"].keys() for elem in self.problem_dictionary["goal"].keys()):
                    #     goals_match = True
                    #     for goal_key, goal_predicate in self.problem_dictionary["goal"].iteritems():
                    #         for goals in goal_predicate:
                    #             if goals not in plan_lib_elem["goal"][str(goal_key)]:
                    #                 goals_match = False
                    #                 break
                    #             # if all(items in problem_data[])

        if count > 0:
            return True, plans
        else:
            return False, plans

    # take the problem file and parse it into the planLib object format needed
    @staticmethod
    def parse_problem_to_dict(string_to_trim):
        # TODO: add the rest of the plan selection elements in the dict
        # status = [False, False, False]
        object_dict = {}
        init_dict = {}
        goal_dict = {}

        # for line in string_to_trim.split("\n"):
        #     if ":objects" in line:
        #         status = [True, False, False]
        #     elif ":init" in line:
        #         status = [False, True, False]
        #     elif ":goal" in line:
        #         status = [False, False, True]
        #     elif (status[0] or status[1] or status[2]) and len(line.replace("(", "").replace(")", "")) > 0:
        #         if status[0]:
        #             object_dict[line.split("-")[1]] = filter(None, line.split("-")[0].split(" "))
        #         elif status[1]:
        #             self.process_line_for_planlib(init_dict, line)
        #         elif status[2]:
        #             self.process_line_for_planlib(goal_dict, line)

        problem_dict = {"problem": string_to_trim.replace("\n", ""), "object": object_dict, "init": init_dict,
                        "goal": goal_dict}

        return problem_dict

    # Parse each line from init and goal of the problem
    @staticmethod
    def process_line_for_planlib(param_dict, line):
        split_line = filter(None, line.replace("(", "").replace(")", "").split(" "))

        # TODO: change this to see if functions are in the domain
        if split_line[0] in "><=":
            if split_line[1] in param_dict.keys():
                param_dict[split_line[1]].append([split_line[0]] + split_line[2:])
            else:
                param_dict[split_line[1]] = [[split_line[0]] + split_line[2:]]

        else:
            if split_line[0] in param_dict.keys():
                param_dict[split_line[0]].append(split_line[1:])
            else:
                param_dict[split_line[0]] = [split_line[1:]]

    # Get domain from the KB via service calls
    def get_domain_from_KB(self):
        rospy.loginfo("KCL: (%s) Getting domain info from KB" % self.node_name)

        # The service names
        domain_service_name = '/rosplan_knowledge_base/domain/name'
        type_service_name = '/rosplan_knowledge_base/domain/types'
        operators_service_name = '/rosplan_knowledge_base/domain/operators'
        predicate_service_name = '/rosplan_knowledge_base/domain/predicates'
        functions_service_name = '/rosplan_knowledge_base/domain/functions'

        domain_dict = {"has": []}

        try:
            rospy.wait_for_service(predicate_service_name)
            domain_predicates_srv = rospy.ServiceProxy(predicate_service_name, GetDomainAttributeService)
            predicate = {}
            for item in domain_predicates_srv().items:
                predicate[item.name] = [x.value for x in item.typed_parameters]
            domain_dict["predicate"] = predicate
            domain_dict["has"].append("predicates")
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call (predicates) failed: %s" % (e,))

        # Getting the functions from KB
        try:
            rospy.wait_for_service(functions_service_name)
            domain_function_srv = rospy.ServiceProxy(functions_service_name, GetDomainAttributeService)
            functions = {}
            for item in domain_function_srv().items:
                functions[item.name] = [x.value for x in item.typed_parameters]
            domain_dict["function"] = functions
            domain_dict["has"].append("functions")
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call (functions) failed: %s" % (e,))

        # Getting the domain name from KB
        try:
            rospy.wait_for_service(domain_service_name)
            domain_name_srv = rospy.ServiceProxy(domain_service_name, GetDomainNameService)

            domain_dict["name"] = domain_name_srv().domain_name
            domain_dict["has"].append("name")
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call (name) failed: %s" % (e,))

        # Getting the types from KB
        try:
            rospy.wait_for_service(type_service_name)
            domain_type_srv = rospy.ServiceProxy(type_service_name, GetDomainTypeService)
            domain_dict["types"] = domain_type_srv().types
            domain_dict["has"].append("types")
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call (types) failed: %s" % (e,))

        # Getting the operators from KB
        try:
            rospy.wait_for_service(operators_service_name)
            domain_operators_srv = rospy.ServiceProxy(operators_service_name, GetDomainOperatorService)
            operators = {}
            for operator in domain_operators_srv().operators:
                operators[operator.name] = [x.value for x in operator.typed_parameters]
            domain_dict["operators"] = operators
            domain_dict["has"].append("operators")
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call (operators) failed: %s" % (e,))

        return domain_dict

    # Take the current plans from the planLib object and parse them back into the yaml format.
    def save_plan_library(self):
        stream = file(self.plans_path, 'w')
        yaml.dump(self.plan_dictionary, stream, default_flow_style=False)

    # Open the yaml file and load the plans into the planLib object
    def load_plan_library(self):
        rospy.loginfo("KCL: (%s) Loading plans from %s" % (self.node_name, self.plans_path))

        with open(self.plans_path) as plan_lib:
            loaded_plan_lib = yaml.load(plan_lib, Loader=yaml.FullLoader)

        rospy.loginfo("KCL: (%s) Plan Library loaded" % self.node_name)

        return loaded_plan_lib


if __name__ == "__main__":
    try:
        rospy.init_node("plan_library")
        PlanLib = PlanLibrary()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
