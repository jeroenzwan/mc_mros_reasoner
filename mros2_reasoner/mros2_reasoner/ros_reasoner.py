import rclpy

from rclpy.action import ActionServer, ActionClient
from rclpy.action import CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node as ROS2Node
from rclpy.parameter import Parameter

from system_modes_msgs.srv import ChangeMode
from std_msgs.msg import Empty
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import KeyValue
from unique_identifier_msgs.msg import UUID

from mros2_reasoner.reasoner import Reasoner

from mros2_msgs.action import ControlQos
from mros2_msgs.msg import QoS
from mros2_msgs.srv import MetacontrolFD

from plansys2_msgs.msg import Param, Node, Tree
from plansys2_msgs.srv import AffectParam, AffectNode, AddProblemGoal, GetStates, GetPlan
from plansys2_msgs.srv import GetPlan, GetDomain, GetProblem
from mros2_msgs.action import ExecutePlan

class RosReasoner(ROS2Node, Reasoner):

    def __init__(self):

        ROS2Node.__init__(self, 'mros2_reasoner_node')

        self.declare_parameter('model_file', Parameter.Type.STRING)
        self.declare_parameter('tomasys_file', Parameter.Type.STRING_ARRAY)

        # Get ontology and tomasys file paths from parameters
        Reasoner.__init__(
            self,
            self.get_parameter('tomasys_file').value,
            self.get_parameter('model_file').value
        )

        self.declare_parameter('desired_configuration', Parameter.Type.STRING)
        self.declare_parameter('reasoning_period', 2)
        self.declare_parameter('use_reconfigure_srv', True)

        # Whether or not to use system modes reconfiguration
        #  Used mainly for testing
        self.use_reconfiguration_srv = self.get_parameter(
            'use_reconfigure_srv').value

        self.instance_cli = self.create_client(AffectParam,
                '/problem_expert/add_problem_instance')
        while not self.instance_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('instance service not available, waiting again...')

        self.predicate_cli = self.create_client(AffectNode,
                '/problem_expert/add_problem_predicate')
        while not self.predicate_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('predicate service not available, waiting again...')

        self.get_predicates_cli = self.create_client(GetStates,
                '/problem_expert/get_problem_predicates')
        while not self.get_predicates_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('get predicates service not available, waiting again...')

        self.function_cli = self.create_client(AffectNode,
                '/problem_expert/add_problem_function')
        while not self.function_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('function service not available, waiting again...')

        self.goal_cli = self.create_client(AddProblemGoal,
                '/problem_expert/add_problem_goal')
        while not self.goal_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('goal service not available, waiting again...')

        self.remove_predicate_cli = self.create_client(AffectNode,
                '/problem_expert/remove_problem_predicate')
        while not self.remove_predicate_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('remove predicate service not available, waiting again...')

        self.plan_cli = self.create_client(GetPlan,
                '/planner/get_plan')
        while not self.plan_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('plan service not available, waiting again...')

        self.domain_cli = self.create_client(GetDomain,
                '/domain_expert/get_domain')
        while not self.domain_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('plan service not available, waiting again...')

        self.problem_cli = self.create_client(GetProblem,
                '/problem_expert/get_problem')
        while not self.problem_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('plan service not available, waiting again...')

        self.plan_handler_action_cli = ActionClient(
            self,
            ExecutePlan,
            'plan_handler')

        # Use execute_ros instead of Reasoner.execute
        if self.use_reconfiguration_srv:
            self.execute = self.execute_ros

        self.is_initialized = False

        self.cb_group = ReentrantCallbackGroup()

        # Start interfaces
        # subscriptions for different message types (named, pins, angle)
        # Node's default callback group is mutually exclusive. This would
        # prevent the change mode requests' response from being processed until
        # the timer callback finished. The callback_group should solve this.
        self.diganostic_sub = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diagnostics_callback,
            1,
            callback_group=self.cb_group)

        # Create action server
        self.objective_action_server = ActionServer(
            self,
            ControlQos,
            '/mros/objective',
            self.objective_action_callback,
            callback_group=self.cb_group,
            cancel_callback=self.objective_cancel_goal_callback)

        # Get desired_configuration_name from parameters
        self.set_initial_fd(self.get_parameter('desired_configuration').value)

        timer_period = self.get_parameter('reasoning_period').value

        self.feedback_rate = self.create_rate(1/timer_period)
        self.metacontrol_loop_timer = self.create_timer(
            timer_period,
            self.metacontrol_loop_callback,
            callback_group=self.cb_group)

        self.logger = self.get_logger()

        self.old_available_fds_filtered = None
        self.plan_executing = False
        self.plan_handle = None
        self.functions_set = False

        # Reasoner initialization completed
        self.is_initialized = True
        self.logger.info('[RosReasoner] -- Reasoner Initialization Ok')

    def set_initial_fd(self, initial_fd):
        if initial_fd != '':
            self.grounded_configuration = initial_fd
            self.logger.info('grounded_configuration initialized to: ' +
                             str(self.grounded_configuration))
        else:
            self.logger.info('grounded_configuration set to None')
            self.grounded_configuration = None

    def set_initial_instances_pddl(self):

        # self.call_instance_service('f_mock','function')

        # self.call_instance_service('fd_mock1','functiondesign')
        # self.call_instance_service('fd_mock2','functiondesign')
        # self.call_instance_service('fd_mock3','functiondesign')

        # self.call_instance_service('f_fake','function')

        # self.call_instance_service('fd_fake1','functiondesign')
        # self.call_instance_service('fd_fake2','functiondesign')
        # self.call_instance_service('fd_fake3','functiondesign')

        self.call_instance_service('bluerov', 'uuv')
        self.call_instance_service('pl1', 'pipeline')

        self.call_instance_service('recharge', 'action')
        self.call_instance_service('search', 'action')
        self.call_instance_service('follow', 'action')

        self.call_instance_service('f_maintain_motion', 'function')
        self.call_instance_service('fd_set_speed_high', 'functiondesign')
        self.call_instance_service('fd_set_speed_medium', 'functiondesign')
        self.call_instance_service('fd_set_speed_low', 'functiondesign')

        self.call_instance_service('f_recharge_wp', 'function')
        self.call_instance_service('fd_generate_recharge_wp', 'functiondesign')

        self.call_instance_service('f_search_pipeline_wp', 'function')
        self.call_instance_service('fd_spiral_low', 'functiondesign')
        self.call_instance_service('fd_spiral_medium', 'functiondesign')
        self.call_instance_service('fd_spiral_high', 'functiondesign')
        
        self.call_instance_service('f_follow_pipeline_wp', 'function')
        self.call_instance_service('fd_generate_follow_wp', 'functiondesign')

        self.call_predicate_service('pipeline_not_found', [['pl1','pipeline']])
        self.call_predicate_service('pipeline_not_inspected', [['pl1','pipeline']])

        self.call_predicate_service('search_a', [['search','action']])
        self.call_predicate_service('follow_a', [['follow','action']])
        self.call_predicate_service('recharge_a', [['recharge','action']])

        self.call_predicate_service('a_req_f',
                    [['search','action'],['f_maintain_motion','function']])
        self.call_predicate_service('a_req_f',
                    [['search','action'],['f_search_pipeline_wp','function']])
                    
        self.call_predicate_service('a_req_f',
                    [['follow','action'],['f_maintain_motion','function']])
        self.call_predicate_service('a_req_f',
                    [['follow','action'],['f_follow_pipeline_wp','function']])
                    
        self.call_predicate_service('a_req_f',
                    [['recharge','action'],['f_maintain_motion','function']])
        self.call_predicate_service('a_req_f',
                    [['recharge','action'],['f_recharge_wp','function']])

        self.call_predicate_service('charged', [['bluerov','uuv']])

        self.call_function_service('time',
                [['fd_set_speed_high','functiondesign']], 20.)

        self.call_function_service('time',
                [['fd_set_speed_medium','functiondesign']], 30.)

        self.call_function_service('time',
                [['fd_set_speed_low','functiondesign']], 40.)

        self.call_function_service('time',
                [['fd_generate_recharge_wp','functiondesign']], 5.)

        self.call_function_service('time',
                [['fd_spiral_low','functiondesign']], 20.)

        self.call_function_service('time',
                [['fd_spiral_medium','functiondesign']], 10.)

        self.call_function_service('time',
                [['fd_spiral_high','functiondesign']], 5.)

        self.call_function_service('time',
                [['fd_generate_follow_wp','functiondesign']], 5.)

        # self.call_function_service('battery_usage',
        #         [['fd_set_speed_high','functiondesign']], 30.)

        # self.call_function_service('battery_usage',
        #         [['fd_set_speed_medium','functiondesign']], 20.)

        # self.call_function_service('battery_usage',
        #         [['fd_set_speed_low','functiondesign']], 10.)
                
        # self.call_function_service('battery_usage',
        #         [['fd_recover','functiondesign']], 5.)

        # self.call_function_service('battery_usage',
        #         [['fd_spiral_low','functiondesign']], 10.)

        # self.call_function_service('battery_usage',
        #         [['fd_spiral_medium','functiondesign']], 15.)

        # self.call_function_service('battery_usage',
        #         [['fd_spiral_high','functiondesign']], 20.)

        # self.call_function_service('battery_usage',
        #         [['fd_generate_follow_wp','functiondesign']], 5.)

        # self.call_function_service('battery_level',
        #         [['bluerov','uuv']], 100.)

        self.call_goal_service('pipeline_inspected', ['pl1', 'pipeline'], 5)

    def objective_cancel_goal_callback(self, cancel_request):
        self.logger.info('Cancel action callback!')
        # Stop reasoning

        if self.use_reconfiguration_srv:
            function_name = self.get_function_name_from_objective_id(
                cancel_request.request.qos_expected.objective_id)
            reconfiguration_result = self.request_configuration(
                'fd_unground',
                function_name)

            if reconfiguration_result is None \
               or reconfiguration_result.success is False:
                self.logger.info('Objective {} cancel req failed'.format(
                    cancel_request.request.qos_expected.objective_id))
                return CancelResponse.REJECT

        if self.remove_objective(
                cancel_request.request.qos_expected.objective_id):
            self.logger.info('Objective {} cancelled'.format(
                cancel_request.request.qos_expected.objective_id))
            return CancelResponse.ACCEPT
        else:
            self.logger.info('Objective {} not found'.format(
                cancel_request.request.qos_expected.objective_id))
            return CancelResponse.REJECT

    def objective_action_callback(self, objective_handle):

        self.logger.info('Objective Action Callback!')

        obj_created = self.create_objective(objective_handle.request)
        if obj_created:
            send_feedback = True
            while send_feedback:
                feedback_msg = ControlQos.Feedback()
                objective = self.get_objective_from_objective_id(
                    objective_handle.request.qos_expected.objective_id)

                if objective is not None:
                    feedback_msg.qos_status.objective_id = objective.name
                    if objective.o_status is None:
                        feedback_msg.qos_status.objective_status = str(
                            'IN_PROGRESS')
                    else:
                        feedback_msg.qos_status.objective_status = str(
                            objective.o_status)

                    feedback_msg.qos_status.objective_type = \
                        str(objective.typeF.name)

                    self.request_configuration(
                            objective_handle.request.qos_expected.selected_mode, objective.typeF.name)
                    self.set_new_grounding(
                            objective_handle.request.qos_expected.selected_mode, objective)

                    fg_instance = self.onto.search_one(solvesO=objective)
                    if fg_instance is not None:
                        feedback_msg.qos_status.selected_mode = \
                            fg_instance.typeFD.name

                        for qa in fg_instance.hasQAvalue:
                            QAValue = KeyValue()
                            QAValue.key = str(qa.isQAtype.name)
                            QAValue.value = str(qa.hasValue)
                            feedback_msg.qos_status.qos.append(QAValue)
                    objective_handle.publish_feedback(feedback_msg)
                else:
                    send_feedback = False

                self.feedback_rate.sleep()
            objective_handle.succeed()
        else:
            objective_handle.fail()

        return ControlQos.Result()

    def create_objective(self, goal_request):
        new_objective = self.get_new_tomasys_objective(
            goal_request.qos_expected.objective_id,
            "*" + goal_request.qos_expected.objective_type)
        self.logger.info('Creating Objective {0}'.format(new_objective))
        for nfr_key in goal_request.qos_expected.qos:
            nfr_id = \
                goal_request.qos_expected.objective_id + '_nfr_' + nfr_key.key
            new_nfr = self.get_new_tomasys_nfr(
                 nfr_id, nfr_key.key, float(nfr_key.value))
            self.logger.info('Adding NFRs {}'.format(new_nfr))
            new_objective.hasNFR.append(new_nfr)

        # TODO: this is not working
        if not goal_request.qos_expected.selected_mode:
            self.set_initial_fd(None)
        else:
            self.set_initial_fd(goal_request.qos_expected.selected_mode)

        # TODO: shouldn't this be a swrl rule instead of hardcoded?
        new_objective.o_status = 'UNGROUNDED'

        return True

    # MVP: callback for diagnostic msg received from QA Observer
    def diagnostics_callback(self, msg):
        if self.onto is not None and self.has_objective() is True:
            for diagnostic_status in msg.status:
                if diagnostic_status.message == 'binding error':
                    self.logger.info('binding error received')
                    up_binding = self.update_binding(diagnostic_status)
                    if up_binding == -1:
                        self.logger.warning(
                            'Unkown Function Grounding: %s' +
                            diagnostic_status.name)
                    elif up_binding == 0:
                        self.logger.warning(
                            'Diagnostics message received for %s' +
                            ' with level %d, nothing done about it.' %
                            (diagnostic_status.name, diagnostic_status.level))

                # Component error
                elif diagnostic_status.message == "Component status":
                    up_cs = self.update_component_status(
                        diagnostic_status)
                    if up_cs == -1:
                        self.logger.warning(
                            'CS message refers to a FG not found in the KB, ' +
                            ' we asume it refers to the current ' +
                            'grounded_configuration (1st fg found in the KB)')
                    elif up_cs == 1:
                        self.logger.info(
                            '\n\nCS Message received!' +
                            '\tTYPE: {0}\tVALUE: {1}'.format(
                                diagnostic_status.values[0].key,
                                diagnostic_status.values[0].value))
                    else:
                        self.logger.warning(
                            'Unsupported CS Message received: %s ' + str(
                                diagnostic_status.values[0].key))

                elif diagnostic_status.message == "QA status":
                    up_qa = self.update_qa(diagnostic_status)
                    self.logger.info(
                        '\nCS Message received!' +
                        '\tTYPE: {0}\tVALUE: {1}'.format(
                            diagnostic_status.values[0].key,
                            diagnostic_status.values[0].value))
                    if not up_qa:
                        self.logger.warning(
                            'Unsupported QA TYPE received: %s ' + str(
                                diagnostic_status.values[0].key))

    def request_configuration(self, desired_configuration, function_name):
        self.logger.warning(
            'New Configuration for function_name {0} requested: {1}'.format(
                function_name, desired_configuration))

        mode_change_cli = self.create_client(
                MetacontrolFD,
                '/mros/request_configuration',
                callback_group=self.cb_group)

        while not mode_change_cli.wait_for_service(timeout_sec=1.0):
            self.logger().warn(
                'Mode change service ' +
                '/mros/request_configuration not available, waiting...')

        try:
            req = MetacontrolFD.Request()
            req.required_function_name = function_name
            req.required_fd_name = desired_configuration
            mode_change_response = mode_change_cli.call(req)
        except Exception as e:
            self.logger.info('Request creation failed {}'.format(e))
            return None
        else:
            return mode_change_response

    def execute_ros(self, desired_configurations):
        if self.has_objective() is False or desired_configurations == dict():
            return

        self.logger.info('  >> Started MAPE-K ** EXECUTION **')
        self.logger.info(
                'desired_configurations are: {}'.format(
                    desired_configurations))
        for objective in desired_configurations:
            reconfiguration_result = self.request_configuration(
                desired_configurations[objective], str(objective.typeF.name))

            if reconfiguration_result is not None \
               and reconfiguration_result.success is True:
                self.logger.info(
                    'Got Reconfiguration result {}'.format(
                        reconfiguration_result.success))

                # Process adaptation feedback to update KB:
                self.set_new_grounding(
                    desired_configurations[objective], objective)
            else:
                self.logger.error('= RECONFIGURATION FAILED =')
                return

    def call_instance_service(self, _name, _type):
        msg = Param()
        msg.name = _name
        msg.type = _type

        req = AffectParam.Request()
        req.param = msg

        try:
            instance_call_response = self.instance_cli.call(req)
        except Exception as e:
            self.logger().info('Request creation failed %r' % (e,))
            return None
        else:
            return instance_call_response

    def call_predicate_service(self, _name, _params):
        msg = Node()
        msg.node_type = 5
        msg.name = _name

        param_list = []
        for _param in _params:
            param = Param()
            param.name = _param[0]
            param.type = _param[1]
            param_list.append(param)

        msg.parameters = param_list

        req = AffectNode.Request()
        req.node = msg

        try:
            predicate_call_response = self.predicate_cli.call(req)
        except Exception as e:
            self.logger().info('Request creation failed %r' % (e,))
            return None
        else:
            return predicate_call_response

        # future = self.predicate_cli.call_async(req)
        # rclpy.spin_until_future_complete(self, future)
        # return future

    def call_goal_service(self, _pred, _param, _type):
        msg = Tree()
        node = Node()
        node.node_type = _type
        node.name = _pred

        param = Param()
        param.name = _param[0]
        param.type = _param[1]

        node.parameters = [param]

        msg.nodes = [node]

        req = AddProblemGoal.Request()
        req.tree = msg

        try:
            goal_call_response = self.goal_cli.call(req)
        except Exception as e:
            self.logger().info('Request creation failed %r' % (e,))
            return None
        else:
            return goal_call_response

        # future = self.goal_cli.call_async(req)
        # rclpy.spin_until_future_complete(self, future)
        # return future

    def get_predicates_pddl(self):
        req = GetStates.Request()
        req.request = Empty()

        try:
            get_predicates_call_response = self.get_predicates_cli.call(req)
        except Exception as e:
            self.logger().info('Request creation failed %r' % (e,))
            return None
        else:
            return get_predicates_call_response

    def remove_predicates_pddl(self):
        predicates = self.get_predicates_pddl()
        for predicate in predicates.states:
            if predicate.name == 'fd_available':
                self.logger.info('removed predicate name is {}'.format(
                                    predicate.name))
                self.call_remove_predicate_service(predicate)

    def call_remove_predicate_service(self, predicate):
        # self.logger.info(str(_name))
        # self.logger.info(str(_params))
        # msg = Node()
        # msg.node_type = 5
        # msg.name = _name

        # msg.parameters = _params

        req = AffectNode.Request()
        req.node = predicate

        try:
            predicate_call_response = self.remove_predicate_cli.call(req)
        except Exception as e:
            self.logger().info('Request creation failed %r' % (e,))
            return None
        else:
            return predicate_call_response

    def call_function_service(self, _name, _params, _value):
        msg = Node()
        msg.node_type = 6
        msg.name = _name
        msg.value = _value

        param_list = []
        for _param in _params:
            param = Param()
            param.name = _param[0]
            param.type = _param[1]
            param_list.append(param)

        msg.parameters = param_list

        req = AffectNode.Request()
        req.node = msg

        try:
            function_call_response = self.function_cli.call(req)
        except Exception as e:
            self.logger().info('Request creation failed %r' % (e,))
            return None
        else:
            return function_call_response

        # future = self.function_cli.call_async(req)
        # rclpy.spin_until_future_complete(self, future)
        # return future
    
    def call_plan_service(self, domain, problem):
        req = GetPlan.Request()
        req.domain = domain
        req.problem = problem

        try:
            plan_call_response = self.plan_cli.call(req)
        except Exception as e:
            self.logger().info('Request creation failed %r' % (e,))
            return None
        else:
            return plan_call_response

    def call_domain_service(self):
        req = GetDomain.Request()

        try:
            domain_call_response = self.domain_cli.call(req)
        except Exception as e:
            self.logger().info('Request creation failed %r' % (e,))
            return None
        else:
            return domain_call_response

    def call_problem_service(self):
        req = GetProblem.Request()

        try:
            problem_call_response = self.problem_cli.call(req)
        except Exception as e:
            self.logger().info('Request creation failed %r' % (e,))
            return None
        else:
            return problem_call_response

    def plan_action_callback(self, replan):
        goal_msg = ExecutePlan.Goal()
        goal_msg.change_plan = replan

        self.plan_handler_action_cli.wait_for_server()

        plan_handle = self.plan_handler_action_cli.send_goal(goal_msg)
        return plan_handle

    def plan_pddl(self, available_fds_filtered):
        self.logger.info('  >> Started MAPE-K ** PLAN adaptation PDDL  **')

        self.set_initial_instances_pddl()

        
        if available_fds_filtered is not []:
            if self.functions_set:
                self.remove_predicates_pddl()
            self.logger.info('available_configurations are {}'.format(
                                available_fds_filtered))
            
            for available_fd in available_fds_filtered:
                self.call_predicate_service('fd_available',
                        [[available_fd[0].name,'functiondesign']])
                self.call_predicate_service('fd_solve_f',
                        [[available_fd[0].name,'functiondesign'],
                            [available_fd[0].solvesF.name,'function']])

        if self.plan_executing: 
            self.logger.info("Plan is being executed, canceling and starting new plan")
            goal_uuid = UUID()
            goal_handle = rclpy.action.client.ClientGoalHandle(self.plan_handler_action_cli, goal_uuid, self.plan_handle)
            # self.plan_handle = self.plan_handler_action_cli._cancel_goal(goal_handle)
            goal_handle.cancel_goal()
            self.plan_action_callback(True)
        else:
            self.logger.info("Plan is not being executed, starting first plan")
            self.plan_handle = self.plan_action_callback(False)
            self.logger.info("--------------------------------")
            self.plan_executing = True


        self.functions_set = True
        success = True           
        return success

    # main metacontrol loop
    def metacontrol_loop_callback(self):

        if self.is_initialized is not True:
            self.logger.info(
                'Waiting to initialize Reasoner -  Nothing else will be done')
            return

        # Analyze
        # objectives_in_error = self.analyze()

        objectives_in_error, available_fds_filtered = self.analyze_pddl()
        
        # Plan
        # desired_configuration = self.plan(objectives_in_error)

        # self.set_initial_instances_pddl()

        if (objectives_in_error != []) or (available_fds_filtered != self.old_available_fds_filtered):
            self.logger.info("objectives in error bool "+str(objectives_in_error!=[]))
            self.logger.info("new available_fds_filtered "+str(available_fds_filtered!=self.old_available_fds_filtered))
            self.plan_pddl(available_fds_filtered)

        # if available_configurations is not None:
            # self.logger.info('available_configurations is not None')
            # self.logger.info('available_configurations is {}'.format(
                                # available_configurations))
            # self.remove_predicates_pddl()
            
            # for available_configuration in available_configurations:
                # self.call_predicate_service('available_fd',
                        # [[available_configuration[0].name,'functiondesign']])

        # if current_fd is not None:
            # self.call_predicate_service('fd_selected',
                    # [[current_fd.name, 'functiondesign']])

        # Execute
        # self.execute(desired_configuration)
        self.old_available_fds_filtered = available_fds_filtered

        self.logger.info(
            'Exited metacontrol_loop_callback')

