#!/usr/bin/env python3
# coding=utf-8
# Author: Alberto Olivares Alarcos <aolivares@iri.upc.edu>, Institut de Robòtica i Informàtica Industrial, CSIC-UPC

import sys
import rospy

from know_demo.utils_module import generalUtils, rosprologUtils

if __name__ == "__main__":
    rospy.init_node("memory_generation_from_data_node", sys.argv)
    rospy.loginfo(rospy.get_name() + ": a know-demo node has been initialized.")
    
    general_utils_object = generalUtils()
    rosprolog_utils_object = rosprologUtils()

    save_neem = False
    dataset_csv_file_name = 'plans_general_properties.csv'

    # read the properties of the target plans (demonstrations from data)
    plans_qualities_values_dict = general_utils_object.create_dict_from_csv_file(general_utils_object.csv_file_path,\
                                                                                  dataset_csv_file_name, ',')
    
    print(plans_qualities_values_dict)



    plan_triples_list = rosprolog_utils_object.plan_qualities_dict_to_triples_list(plans_qualities_values_dict)
    """ # for debugging
    for i in range(0, 25):
        print(plan_triples_list[i])
        print("\n")
    """
    #print(plan_triples_list)

    plan_assertion_query_text = rosprolog_utils_object.construct_query_text_for_multiple_triples_assertion(plan_triples_list, True)
    ## print(plan_assertion_query_text)
    rosprolog_utils_object.rosprolog_assertion_query(plan_assertion_query_text)

    # save the NEEM using in the name 'plan_adaptation_case_' and the current time
    """
    if save_neem:
        query_string_foo_ = "ros_package_path('know_demo', P1), \
            atom_concat(P1, '/neem/contrastive_plans/"+ dataset_csv_file_name +"_' , P2), \
            get_time(T), atom_concat(P2, T, P3), mng_dump(roslog, P3)."
        rpcra.rosprolog_wrapper_for_rosplan_cra_.rosprolog_assertion_query(query_string_foo_)
    """

    #rospy.spin()

