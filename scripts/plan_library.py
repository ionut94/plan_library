#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from rosplan_knowledge_msgs.srv import GetDomainNameService, GetDomainTypeService, GetDomainOperatorService, \
    GetDomainAttributeService
import yaml
import uuid
import pprint

pp = pprint.PrettyPrinter(indent=1)


class PlanLibrary:

    def __init__(self):
        self.node_name = rospy.get_name()
        self.use_library = rospy.get_param("~use_library")

        if self.use_library:
            rospy.loginfo("KCL: (%s) Plan Library initialising..." % self.node_name)

            # Initialisation
            self.latest_plan = ""
            self.latest_plan_time = 0
            self.latest_problem = ""
            self.latest_problem_time = 0
            self.plans_path = str(rospy.get_param("~stored_plans_path"))
            self.plan_dictionary = self.load_plan_library()
            self.problem_dictionary = {}

            # Get the domain from plan lib or initialise it from KB
            if self.plan_dictionary is None:
                self.plan_dictionary = {"domain": self.get_domain_from_KB()}
            self.domain = self.plan_dictionary["domain"]
        else:
            rospy.loginfo("KCL: (%s) Plan Library not in use" % self.node_name)

        # Create publishers to planner and plan_parser
        self.planner_input_pub = rospy.Publisher("~" + rospy.get_param("~planner_input_topic"), String, queue_size=1)
        self.plan_lib_output_pub = rospy.Publisher("~" + rospy.get_param("~plan_from_lib_topic"), String, queue_size=1)

        # Create subscribers to prob_generation and planner
        rospy.Subscriber(rospy.get_param("~problem_instance"), String, self.problem_callback)
        rospy.Subscriber(rospy.get_param("~planner_output"), String, self.planner_callback)

    # Receive plan and save it into the planLib object
    def planner_callback(self, data):
        self.latest_plan = data.data
        self.latest_plan_time = rospy.Time.now()

        if self.use_library:

            # Add new plan to library
            self.problem_dictionary.update({"plan": self.latest_plan})
            self.plan_dictionary[str(uuid.uuid1().node)] = self.problem_dictionary
            self.save_plan_library()

        self.plan_lib_output_pub.publish(self.latest_plan)

    # Receive problem and save it into the planLib object
    def problem_callback(self, data):
        self.latest_problem = data.data
        self.latest_problem_time = rospy.Time.now()
        self.problem_dictionary = self.parse_problem_to_dict(self.latest_problem)

        if self.use_library:
            check, plans = self.check_for_plan_in_library()

            if check:
                # self.latest_problem
                rospy.loginfo("KCL: (%s) There is a plan for this problem in the plan library" % self.node_name)
                self.plan_lib_output_pub.publish(plans[0])
            else:
                rospy.loginfo("KCL: (%s) Problem not in the plan library" % self.node_name)
                self.planner_input_pub.publish(self.latest_problem)
        else:
            self.planner_input_pub.publish(self.latest_problem)

    # Check if the latest problem is already in our plan library
    # TODO Implement a more robust version of searching for a plan in the PlanLib
    def check_for_plan_in_library(self):
        count = 0
        plans = []

        for key, plan_lib_elem in self.plan_dictionary.iteritems():
            if key not in "domain":
                if plan_lib_elem['problem'] in self.latest_problem.replace("\n", ""):
                    count += 1
                    plans.append(plan_lib_elem['plan'])
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
                    self.process_line_for_planlib(init_dict, line)
                elif status[2]:
                    self.process_line_for_planlib(goal_dict, line)

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

        return loaded_plan_lib


if __name__ == "__main__":
    try:
        rospy.init_node("plan_library")
        PlanLib = PlanLibrary()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
