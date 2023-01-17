#!/usr/bin/env python3

import rospy
from sgpr_ros.srv import Eigenvalues, EigenvaluesRequest, EigenvaluesResponse

def eval_callback(req: EigenvaluesRequest):

    # Run AD and KS test and return the 
    return EigenvaluesResponse([.5,.5])

def evaluation_test_service():
    rospy.init_node('evaluation_service_node')
    rospy.Service('evaluation_service', Eigenvalues, eval_callback)
    print("Eval Service Up")
    rospy.spin()

if __name__ == "__main__":
    evaluation_test_service()
