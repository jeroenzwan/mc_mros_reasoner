#!/usr/bin/env python
###########################################
#
# authors:    M.A.GarzonOviedo@tudelft.nl
#             c.h.corbato@tudelft.nl
##########################################

import signal
import sys
from threading import Lock

from mros2_reasoner.tomasys import get_objectives_in_error
from mros2_reasoner.tomasys import ground_fd
from mros2_reasoner.tomasys import read_ontology_file
from mros2_reasoner.tomasys import remove_objective_grounding
from mros2_reasoner.tomasys import reset_fd_realisability
from mros2_reasoner.tomasys import reset_objective_status
from mros2_reasoner.tomasys import update_qa_value

from owlready2 import destroy_entity
from owlready2 import sync_reasoner_pellet

import logging


class Reasoner:
    """docstring for Reasoner."""

    def __init__(self, tomasys_file, model_file):

        # owl model with the tomasys ontology
        self.tomasys = read_ontology_file(tomasys_file)
        # application model as individuals of tomasys classes
        self.onto = read_ontology_file(model_file)

        # name of the current system configuration, as stored in KB
        self.grounded_configuration = None
        # TODO move to RosReasoner or remove: there can be multiple
        # configurations grounded (for multiple objectives)
        # This Lock is used to ensure safety of tQAvalues
        self.ontology_lock = Lock()

        signal.signal(signal.SIGINT, self.save_ontology_exit)

    def remove_objective(self, objective_id):
        # Checks if there are previously defined objectives.
        old_objective = self.onto.search_one(iri="*{}".format(objective_id))
        if old_objective:
            old_fg_instance = self.onto.search_one(solvesO=old_objective)
            with self.ontology_lock:
                destroy_entity(old_fg_instance)
                destroy_entity(old_objective)
            return True
        else:
            return False

    def search_objectives(self):
        # Root objectives
        objectives = self.onto.search(type=self.tomasys.Objective)
        return objectives

    def has_objective(self):
        objectives = self.search_objectives()
        has_objective = False
        if objectives == []:
            self.get_logger().info(
                'No objectives found, waiting for new Objective')
        else:
            has_objective = True
            for objective in objectives:
                self.get_logger().info(
                    'Objective {} found'.format(
                        objective.name))
        return has_objective

    def get_objectives_in_error(self):
        return get_objectives_in_error(self.search_objectives())

    def get_new_tomasys_objective(self, objective_name, iri_seed):
        """ Creates Objective individual in the KB given a desired name and a
        string seed for the Function name
        """
        objective = self.tomasys.Objective(
            str(objective_name),
            namespace=self.onto,
            typeF=self.onto.search_one(
                iri=str(iri_seed)))
        return objective

    def get_new_tomasys_nrf(self, qa_value_name, iri_seed, nfr_value):
        """Creates QAvalue individual in the KB given a desired name and a
        string seed for the QAtype name and the value
        """
        new_nfr = self.tomasys.QAvalue(
            str(qa_value_name),
            namespace=self.onto,
            isQAtype=self.onto.search_one(
                iri=str(iri_seed)),
            hasValue=nfr_value)
        return new_nfr

    def set_new_grounding(self, fd_name, objective):
        """Given a string fd_name with the name of a FunctionDesign and an
        objective, removes the previous fg for the objective and ground a new
        fg of typeF fd
        """
        remove_objective_grounding(objective, self.tomasys, self.onto)
        fd = self.onto.search_one(
            iri="*{}".format(fd_name),
            is_a=self.tomasys.FunctionDesign)
        if fd:
            with self.ontology_lock:
                ground_fd(fd, objective, self.tomasys, self.onto)
                reset_objective_status(objective)
            return str(fd.name)
        else:
            return None

    # the DiagnosticStatus message process contains, per field
    # - message: "binding_error"
    # - name: name of the fg reported, as named in the OWL file
    # - level: values 0 and 1 are mapped to nothing, values 2 or 3 are mapped
    # to fg.status="INTERNAL_ERROR"
    def update_binding(self, diagnostic_status):
        fg = self.onto.search_one(iri="*{}".format(diagnostic_status.name))
        if fg is None:
            return -1
        if diagnostic_status.level > 1:
            fg.fg_status = "INTERNAL_ERROR"
            return 1
        else:
            return 0

    # the DiagnosticStatus message process contains, per field
    # - message: "Component status"
    # - key: name of the Component Reported, as named in the OWL file
    # - value: (True, False ) Meaning wheter or not the component is working OK
    def update_component_status(self, diagnostic_status):
        # Find the Component with the same name that the one in the Component
        # Status message (in diagnostic_status.key)
        component_type = self.onto.search_one(
            iri="*{}".format(diagnostic_status.values[0].key))
        if component_type is not None:
            value = diagnostic_status.values[0].value
            with self.ontology_lock:
                reset_fd_realisability(
                    self.tomasys,
                    self.onto,
                    diagnostic_status.values[0].key)
                component_type.c_status = value
            return_value = 1
        else:
            return_value = 0
        return return_value

    # update QA value based on incoming diagnostic
    def update_qa(self, diagnostic_status):
        # Find the FG with the same name that the one in the QA message (in
        # diagnostic_status.name)
        fg = next((fg for fg in self.tomasys.FunctionGrounding.instances()
                  if fg.name == diagnostic_status.name), None)
        if fg is None:
            fg = self.tomasys.FunctionGrounding.instances()[0]
            return_value = -1
        qa_type = self.onto.search_one(
            iri="*{}".format(diagnostic_status.values[0].key))
        if qa_type is not None:
            value = float(diagnostic_status.values[0].value)
            with self.ontology_lock:
                update_qa_value(fg, qa_type, value, self.tomasys, self.onto)
            return_value = 1
        else:
            return_value = 0
        return return_value

    # EXEC REASONING to update ontology with inferences
    # TODO CHECK: update reasoner facts, evaluate, retrieve action, publish
    # update reasoner facts
    def perform_reasoning(self):
        return_value = False
        with self.ontology_lock:
            with self.onto:
                try:
                    sync_reasoner_pellet(
                        infer_property_values=True,
                        infer_data_property_values=True)
                    return_value = True
                except Exception as err:
                    logging.exception("{0}".format(err))
                    return False
                    # raise err

        return return_value

    # For debugging purposes: saves state of the KB in an ontology file
    # TODO move to library
    # TODO save file in a temp location
    def save_ontology_exit(self, signal, frame):
        self.onto.save(file="error.owl", format="rdfxml")
        sys.exit(0)

    def handle_updatable_objectives(self, objective_list):
        for obj_in_error in objective_list:
            if obj_in_error.o_status == "UPDATABLE":
                self.get_logger().info(
                    ">> UPDATABLE objective - Try to clear Components status")
                for comp_inst in list(
                        self.tomasys.ComponentState.instances()):
                    if comp_inst.c_status == "RECOVERED":
                        self.get_logger().info(
                            "Component {0} Status {1} - Set to None".format(
                                comp_inst.name, comp_inst.c_status))
                        comp_inst.c_status = None
