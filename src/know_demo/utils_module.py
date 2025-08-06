#!/usr/bin/env python3
# coding=utf-8
# Author: Alberto Olivares Alarcos <aolivares@iri.upc.edu>, Institut de Robòtica i Informàtica Industrial, CSIC-UPC


import rospy
import rospkg
import pandas as pd
import numpy as np

from rosprolog_client import PrologException, Prolog

class generalUtils:
    def __init__(self):
        self.main_path = rospkg.RosPack().get_path('know_demo')
        self.csv_file_path = self.main_path + "/csv"
        self.ontology_entity_to_compare = "PlanID" #it must match the name in the csv (e.g. 'PlanID')

    def create_dict_from_csv_file(self, csv_file_path, csv_file_name, separator):
        """
        Creates a dictionary from the data stored in a csv file.

        Args:
            csv_file_path: a path to a csv file (without the last '/', it is added here)
            csv_file_name: a csv file name (including the .csv)

        Returns:
            A dictionary with the first row of the csv file as keys and the rest of the values 
            as a list (for each key).
        """

        out_dict = dict()

        data = pd.read_csv(csv_file_path + "/" + csv_file_name, sep=separator) #data frame
        out_dict = data.to_dict(orient='list') # dict of lists (first row as 'k')
        #print(out_dict)
        
        return out_dict
    
    def create_csv_file_from_dict(self, dict_in, csv_file_path, csv_file_name):
        """
        Creates a dictionary from the data stored in a csv file.

        Args:
            dict_in: a dictionary to be stored as CSV file
            csv_file_path: a path to a csv file (without the last '/', it is added here)
            csv_file_name: a csv file name (including the .csv)

        Returns:
            None.
        """

        out_dict = dict()

        df = pd.DataFrame.from_dict(dict_in, orient='columns')
        df.to_csv(csv_file_path + '/' + csv_file_name, index=False) # data frame
    
   
class rosprologUtils:
    def __init__(self):
        self.main_path = rospkg.RosPack().get_path('know_demo')
        self.csv_file_path = self.main_path + "/csv"
        self.ontology_entity_to_compare = "PlanID" #it must match the name in the csv (e.g. 'PlanID')

        # Define varibles
        self.client_rosprolog_ = Prolog()
        ##self.current_plan_kb_uri = ""

        if (rospy.has_param('~semantic_map_namespace')):
            self.semantic_map_namespace = rospy.get_param('~semantic_map_namespace')
        else:
            self.semantic_map_namespace = "ocra_common"

        self.inverse_ontology_relations_dict = self.get_ontology_property_and_inverse_dict()

    def plan_qualities_dict_to_triples_list(self, plan_dict):
        """
        Creates a list of triples to assert to the knowledge base using the set of plans' qualities
        from the input dictionary. 

        Args:
            plan_dict: a dictionary containing the qualities of plans (keys are qualities IDs and values are lists)

        Returns:
            triples_list: a list of triples of the plans' qualities knowledge ready to be asserted
        """

        rospy.loginfo(rospy.get_name() + ": Formatting the plan sequence details as a set of triples to assert them to the ontology KB")

        triples_list = list()
        for i in range(0, len(plan_dict["PlanID"])):
            plan_id = plan_dict["PlanID"][i] #+ "_" + str(datetime.utcnow()).replace(" ", "_") + "-UTC"
            plan_kb_uri = self.semantic_map_namespace + ":'" + plan_id + "'"
            triples_list.append([plan_kb_uri, "rdf:'type'", "dul:'Plan'"])

            ##self.current_plan_kb_uri = plan_kb_uri

            # knowledge about plan properties
            triples_list.append([plan_kb_uri, "ocra_common:'hasExpectedMakespan'", \
                                self.semantic_map_namespace + ":'" + plan_id + "_makespan'"])
            triples_list.append([self.semantic_map_namespace + ":'" + plan_id + "_makespan'", \
                                "dul:'hasDataValue'", str(plan_dict["Makespan"][i])])
            triples_list.append([self.semantic_map_namespace + ":'" + plan_id + "_makespan'", \
                                "rdf:'type'", "dul:'Quality'"])
            
            triples_list.append([plan_kb_uri, "ocra_common:'hasNumberOfTasks'", \
                                self.semantic_map_namespace + ":'" + plan_id + "_number_of_tasks'"])
            triples_list.append([self.semantic_map_namespace + ":'" + plan_id + "_number_of_tasks'", \
                                "dul:'hasDataValue'", str(plan_dict["NumberOfTasks"][i])])
            triples_list.append([self.semantic_map_namespace + ":'" + plan_id + "_number_of_tasks'", \
                                "rdf:'type'", "dul:'Quality'"])
            
            triples_list.append([plan_kb_uri, "ocra_common:'hasCost'", \
                                self.semantic_map_namespace + ":'" + plan_id + "_cost'"])
            triples_list.append([self.semantic_map_namespace + ":'" + plan_id + "_cost'", \
                                "dul:'hasDataValue'", str(plan_dict["Cost"][i])])
            triples_list.append([self.semantic_map_namespace + ":'" + plan_id + "_cost'", \
                                "rdf:'type'", "dul:'Quality'"])
        
            
        return triples_list
    
    def construct_query_text_for_single_triple_assertion(self, triple_subject, triple_relation, triple_object, add_final_dot, add_inverse_triple):
        """
        Creates the text for an assertion query using multiple triples.  

        Args:
            triple_(subject, relation, object): a string containing the (subject, relation, object) of the triple to assert
            add_final_dot: a boolean indicating whether to add a final dot to the text or not (only after the last triple)
            add_inverse_triple: a boolean indicating whether it is necessary to also assert the inverse triple 

        Returns:
            query_text: the final string of the query
        """

        ## rospy.loginfo(rospy.get_name() + ": Construct query text for single triple assertion")
        query_text = "kb_project(triple(" + triple_subject + ", " + triple_relation + ", " + triple_object +"))"
        if add_inverse_triple:
            tp_relation_name = triple_relation.split("'")[1] # it works if the relation complies with "rdf:'type'" format
            if (tp_relation_name in self.inverse_ontology_relations_dict):
                tp_relation_uri = triple_relation.split("'")[0] # it works if the relation complies with "rdf:'type'" format
                inverted_triple_relation = tp_relation_uri + "'" + self.inverse_ontology_relations_dict[tp_relation_name] + "'"
                query_text = query_text + ", " + "kb_project(triple(" + triple_object + ", " + \
                    inverted_triple_relation + ", " + triple_subject + "))"

                ## print(query_text)
            else: 
                rospy.logwarn(rospy.get_name() + ": The triple property %s does not have inverse.", triple_relation) # for data properties is expected
        else: 
            pass
        if add_final_dot:
            query_text = query_text + "."
        else:
            pass

        ## query_text = query_text.replace('-','_') # ontology does not like '-'

        return query_text
    
    def construct_query_text_for_multiple_triples_assertion(self, triples, add_inverse_triple):
        """
        Creates the text for an assertion query using multiple triples.  

        Args:
            triples: a list of triples to assert in the format of a list of three elements S,P,O
            add_inverse_triple: a boolean indicating whether it is necessary to also assert the inverse triples 

        Returns:
            query_text: the final string of the query
        """

        rospy.loginfo(rospy.get_name() + ": Construct query text for multiple triples assertion")

        query_text = self.construct_query_text_for_single_triple_assertion(triples[0][0], triples[0][1], triples[0][2], False, add_inverse_triple) 

        for i in range(1, len(triples)):
            query_text = query_text + ", " + \
                self.construct_query_text_for_single_triple_assertion(triples[i][0], triples[i][1], triples[i][2], False, add_inverse_triple) 
        
        query_text = query_text + "."

        return query_text

    def rosprolog_assertion_query(self, query_text):
        """
        Makes an assertion query to the knowledge base (no need for waiting for an answer). 

        Args:
            query_text: the final string of the query (it needs to be formated before) 

        Returns:
            None.
        """

        rospy.loginfo(rospy.get_name() + ": New query to assert ontological knowledge")

        # query the knowldge base
        query = self.client_rosprolog_.query(query_text)
        query.finish()
    
    def get_ontology_property_and_inverse_dict(self):
        """
        Creates a dictionary containing all the relationships that exist in the knowledge base and their inverse. 

        Args:
            - 

        Returns:
            ont_property_inverse_dict: a dictionary with all the relationships that exist in the knowledge base as 
            the key and their inverse relation as the value
        """
        ont_property_inverse_dict = dict()

        query = self.client_rosprolog_.query("kb_call(triple(S, owl:'inverseOf', O))")

        for solution in query.solutions():
            subj_ = solution['S'].split('#')[-1] # without the ontology URI
            obj_ = solution['O'].split('#')[-1] # without the ontology URI

            ont_property_inverse_dict[subj_] = obj_
            ont_property_inverse_dict[obj_] = subj_

        query.finish()

        return ont_property_inverse_dict