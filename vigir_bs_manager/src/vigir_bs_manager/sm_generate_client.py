#!/usr/bin/env python

import rospy

from vigir_bs_msgs.srv import SMGenerate

def sm_generate_client(synthesized_automata, yaml_config):
    '''Client'''

    rospy.wait_for_service('sm_generate')
    
    try:
        sm_generate_srv = rospy.ServiceProxy('sm_generate', SMGenerate)
        yaml_config = "../vigir_repo/catkin_ws/src/vigir_behavior_synthesis/examples/all_modes_pickup/pickup.yaml"
        response = sm_generate_srv(synthesized_automata, yaml_config)
        
        #DEBUG
        for si i response.state_definition:
            print(si)
        
        return response
    
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == "__main__":
    sm_generate_client()
