import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.callback_groups import ReentrantCallbackGroup
from system_modes.srv import ChangeMode
from diagnostic_msgs.msg import DiagnosticArray

from mros2_reasoner.reasoner import Reasoner
from mros2_reasoner.tomasys import obtainBestFunctionDesign, print_ontology_status, evaluateObjectives, resetKBstatuses



class RosReasoner(Node, Reasoner):
    """docstring for RosComponents."""

    def __init__(self):
        super().__init__('mros2_reasoner_node')
        # This is really not very elegant, but what can we do?
        Reasoner.__init__(self)

        self.initialized = False
        self.mode_change_srv_call_future = None
        self.wait_response_timer = None
        self.reconfuration_response = None

        self.declare_parameter("model_file")
        self.declare_parameter("tomasys_file")
        self.declare_parameter("desired_configuration")
        self.declare_parameter("nfr_energy")
        self.declare_parameter("nfr_safety")
        self.declare_parameter("reasoning_rate")
        self.declare_parameter("node_name")


        sleep_rate = self.create_rate(2.0)
        #### Read ROS parameters
        # Get ontology and tomasys file paths from parameters
        model_file = self.check_and_read_parameter('model_file')
        tomasys_file =  self.check_and_read_parameter('tomasys_file')
        # Get desired_configuration_name from parameters
        self.grounded_configuration = self.check_and_read_parameter('desired_configuration')
        node_name = self.check_and_read_parameter('node_name', 'pilot')

        #Start interfaces
        # subscriptions for different message types (named, pins, angle)
        self.diganostic_sub = self.create_subscription(
                            DiagnosticArray,
                            '/diagnostics',
                            self.callbackDiagnostics,
                            10)
        #sub_diagnostics = rospy.Subscriber('/diagnostics', DiagnosticArray, self.callbackDiagnostics)
        # Node's default callback group is mutually exclusive. This would prevent the client response
        # from being processed until the timer callback finished, but the timer callback in this
        # example is waiting for the client response

        cb_group = ReentrantCallbackGroup()

        self.system_modes_cli = self.create_client(ChangeMode,
            '/' + node_name + '/change_mode',
            callback_group=cb_group)

        # rosgraph_manipulator_client.wait_for_server()

        # load tomasys
        if tomasys_file is not None:
            self.load_tomasys_from_file(tomasys_file)
            if self.tomasys is not None:
                self.get_logger().info("Loaded tomasys: " + str(tomasys_file))
            else:
                self.get_logger().error("Failed to load tomasys from: " + str(tomasys_file))
                return
        else:
            self.get_logger().warning("No tomasys file provided!")
            return

        #sleep_rate.sleep() # Wait for subscribers (only for the test_1_level_functional_architecture)

        # load ontology
        if model_file is not None:
            self.load_onto_from_file(model_file)
            if self.onto is not None:
                self.get_logger().info("Loaded ontology: " + str(model_file))
            else:
                self.get_logger().error("Failed to load ontology from: " + str(model_file))
                return
        else:
            self.get_logger().warning("No ontology file provided!")
            return


        if self.grounded_configuration is not None:
            self.get_logger().info('grounded_configuration initialized to: ' + str(self.grounded_configuration))
        else:
            self.get_logger().warning('grounded_configuration not found in the param server')


#        self.start_reasoning_timer()
        timer_rate = float(self.check_and_read_parameter('reasoning_rate', 2.0))
        self.timer = self.create_timer(timer_rate, self.timer_cb, callback_group=cb_group)

        self.initialized = True
        # Reasoner initialization completed
        self.get_logger().info("[RosReasoner] -- Reasoner Initialization Ok")


    def start_wait_response_timer(self):
        if self.wait_response_timer is None:
            self.wait_response_timer = self.create_timer(0.5, self.wait_response_timer_cb)
        else:
            self.get_logger().warning('wait_response_timer already active')


    def stop_wait_response_timer(self):
        if self.wait_response_timer is not None:
            # Stop the timer
            self.wait_response_timer.destroy()
            # Delete the variable
            self.wait_response_timer = None


    def check_and_read_parameter(self, param_name, default_value=None):
        """ Checks if a parameter exists and returns its value
            Args:
                    param_name (string): The name of the parameter.
            Returns:
                    The parameter value if it exists, None otherwise.
        """
        # Helper function to return value of a parameter
        if self.has_parameter(param_name):
            param_desc = self.get_parameter(param_name)
            if param_desc.type_== Parameter.Type.NOT_SET:
                ret = default_value
            else:
                ret = param_desc.value
        else:
            self.get_logger().warning('Fetch of parameter that does not exist: ' + param_name + ' - Returning ' + str(default_value))
            ret = default_value

        return ret



    # NOTE REFACTORING: This KB initialization is completely mixed with ROS interfaces, probably libraty should not have an initKB method, but utility methods to update the abox according to incoming information
    # Initializes the KB according to 2 cases:
    # - If there is an Objective individual in the ontology file, the KB is initialized only using the OWL file
    # - If there is no Objective individual, a navigation Objective is create in the KB, with associated NFRs that are read frmo rosparam
    def initKB(self):

        self.get_logger().info('KB initialization:\n \t - Supported QAs: \n \t \t - for Function f_navigate: /nfr_energy, /nfr_safety \n \t - If an Objective instance is not found in the owl file, a default o_navigate is created.' )

        objectives = self.search_objectives()

        # if no objectives in the OWL file, standard navigation objective is assumed
        if objectives == []:
            self.get_logger().info('Creating default Objective o_navigateA with default NFRs')

            obj_navigate = self.get_new_tomasys_objective("o_navigateA", "*f_navigate")

            # Get ontology and tomasys file paths from parameters
            nfr_energy_value = float(self.check_and_read_parameter('nfr_energy', 0.5))
            nfr_safety_value = float(self.check_and_read_parameter('nfr_safety', 0.8))

            # Load NFRs in the KB
            nfr_energy = self.get_new_tomasys_nrf("nfr_energy", "*energy", nfr_energy_value)
            nfr_safety = self.get_new_tomasys_nrf("nfr_safety", "*safety", nfr_safety_value)

            # Link NFRs to objective
            obj_navigate.hasNFR.append(nfr_energy)
            obj_navigate.hasNFR.append(nfr_safety)

            # # Function Groundings and Objectives
            self.set_new_grounding(self.grounded_configuration, obj_navigate)
            self.request_configuration(self.grounded_configuration)

        elif len(objectives) == 1:
            o = objectives[0]
            fd = obtainBestFunctionDesign(o, self.tomasys)
            self.get_logger().warning('Objective, NFRs and initial FG are generated from the OWL file')
            ## Make sure we are on the initial configuration
            if fd.name != self.grounded_configuration:
                self.get_logger().info('Requesting initial configuration: ' + str(fd.name))
                self.request_configuration(fd.name)
        else:
            self.get_logger().error('Metacontrol cannot handle more than one Objective in the OWL file (the Root Objective)')

        # For debugging InConsistent ontology errors, save the ontology before reasoning
        self.onto.save(file="tmp_debug.owl", format="rdfxml")


    # MVP: callback for diagnostic msg received from QA Observer
    def callbackDiagnostics(self, msg):
        if self.onto is not None:
            for diagnostic_status in msg.status:
                # 2 types of diagnostics considered: about bindings in error (TODO not implemented yet) or about QAs
                if diagnostic_status.message == "binding error":
                    self.get_logger().info("binding error received")
                    up_binding = self.updateBinding(diagnostic_status.name, diagnostic_status.level)
                    if up_binding == -1:
                        self.get_logger().warning("Unkown Function Grounding: %s", diagnostic_name)
                    elif up_binding == 0:
                        self.get_logger().warning("Diagnostics message received for %s with level %d, nothing done about it." % (diagnostic_name, diagnostic_level))

                if diagnostic_status.message == "QA status":
                    self.get_logger().warning("QA value received for\t{0} \tTYPE: {1}\tVALUE: {2}".format(diagnostic_status.name, diagnostic_status.values[0].key, diagnostic_status.values[0].value))
                    up_qa = self.updateQA(diagnostic_status)
                    if up_qa == -1:
                        self.get_logger().warning("QA message refers to a FG not found in the KB, we asume it refers to the current grounded_configuration (1st fg found in the KB)")
                    elif up_qa == 1:
                        self.get_logger().info("QA value received!\tTYPE: {0}\tVALUE: {1}".format(diagnostic_status.values[0].key, diagnostic_status.values[0].value))
                    else:
                        self.get_logger().warning("Unsupported QA TYPE received: %s ", str(diagnostic_status.values[0].key))

    # for MVP with QAs - request the FD.name to reconfigure to
    def request_configuration(self, new_configuration):
        self.get_logger().warning('New Configuration requested: {}'.format(new_configuration))

        self.get_logger().info('MAPE: E xecute: --change mode to-- '+ new_configuration)

        while not self.system_modes_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Mode change service not available, waiting again...')

        self.req = ChangeMode.Request()

        self.req.mode_name = new_configuration

        # node_name field is necessary in system_modes feature/rules branch but incompatible with master
        self.req.node_name = self.node_name
        
        # async call, but the follow up while loop BLOCKS execution till there is a response
        mode_change_srv_call_future = self.system_modes_cli.call_async(self.req)

        rclpy.spin_until_future_complete(self, mode_change_srv_call_future)
        if mode_change_srv_call_future.result() is not None:
            self.get_logger().info('Mode change completed - Result ' + str(mode_change_srv_call_future.result().success))
            return 1
        else:
            self.get_logger().error('Exception while calling service: %r' % mode_change_srv_call_future.exception())
            return
        # Activate another timer to wait for the response.
        #self.start_wait_response_timer()

        return

    ## Loop to wait for an answer to service call
    def wait_response_timer_cb(self):
        if self.mode_change_srv_call_future.done():
            try:
                response = self.mode_change_srv_call_future.result()
            except Exception as e:
                self.get_logger().info(
                'Service call failed %r' % (e,))

            else:
                self.get_logger().info(
                'Result of change node to mode %s is %s' %
                (self.req.mode_name, response.success))
                self.get_logger().warning("= RECONFIGURATION SUCCEEDED =") # for DEBUGGING in csv
                # updates the ontology according to the result of the adaptation action - destroy fg for Obj and create the newly grounded one
                self.grounded_configuration = self.set_new_grounding(fd.name, o) # Set new grounded_configuration
                resetKBstatuses(self.tomasys)
            finally:
                self.stop_wait_response_timer()



    ## main metacontrol loop
    def timer_cb(self):

        self.get_logger().info('Entered timer_cb for metacontrol reasoning')
        # If we're waiting for a response from the reconfiguration, nothing should be done
        if self.wait_response_timer is not None:
            self.get_logger().info('Waiting for response of reconfiguration -  Nothing else will be done')
            return

        self.get_logger().info('  >> Started MAPE-K ** Analysis (ontological reasoning) **')

        # EXEC REASONING to update ontology with inferences
        if self.perform_reasoning():
            self.get_logger().info('     >> Finished ontological reasoning)')
        else:
            self.get_logger().error("Reasoning error")

        # PRINT system status
        print_ontology_status(self.tomasys)

        # EVALUATE functional hierarchy (objectives statuses) (MAPE - Analysis)
        objectives_internal_error = evaluateObjectives(self.tomasys)
        if not objectives_internal_error:
            self.get_logger().info("No Objectives in status ERROR: no adaptation is needed")
            self.get_logger().info('  >> Finished MAPE-K ** ANALYSIS **')
            self.get_logger().info('Exited timer_cb for metacontrol reasoning')
            return
        elif len(objectives_internal_error) > 1 :
            self.get_logger().error("- More than 1 objectives in error, case not supported yet.")
            self.get_logger().info('  >> Finished MAPE-K ** ANALYSIS **')
            self.get_logger().info('Exited timer_cb for metacontrol reasoning')
            return
        else:
            self.get_logger().warning("Objectives in status ERROR: {}".format([o.name for o in objectives_internal_error]) )
            self.get_logger().info('  >> Finished MAPE-K ** ANALYSIS **')

        # ADAPT MAPE -Plan & Execute
        self.get_logger().info('  >> Started MAPE-K ** PLAN adaptation **')


        o = objectives_internal_error[0]
        self.get_logger().info("=> Reasoner searches FD for objective: {}".format(o.name) )
        fd = obtainBestFunctionDesign(o, self.tomasys)
        if not fd:
            self.get_logger().error(
                "No FD found to solve Objective {}".format(o.name)) # for DEBUGGING in csv
            self.get_logger().info('Exited timer_cb for metacontrol reasoning')
            return
        self.get_logger().info('  >> Finished MAPE-K ** Plan adaptation **')

        # request new configuration
        self.get_logger().info('  >> Started MAPE-K ** EXECUTION **')
        result = self.request_configuration(fd.name)
        self.get_logger().info('  >> Finished MAPE-K ** EXECUTION **')
        # Process adaptation feedback to update KB:
        if result == 1: # reconfiguration executed ok
            self.get_logger().warning("= RECONFIGURATION SUCCEEDED =") # for DEBUGGING in csv
            # updates the ontology according to the result of the adaptation action - destroy fg for Obj and create the newly grounded one
            self.grounded_configuration = self.set_new_grounding(fd.name, o) # Set new grounded_configuration
            resetKBstatuses(self.tomasys)
        else:
            self.get_logger().error("= RECONFIGURATION FAILED =") # for DEBUGGING in csv

        self.get_logger().info('Exited timer_cb for metacontrol reasoning')
