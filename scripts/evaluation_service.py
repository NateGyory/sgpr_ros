#!/usr/bin/env python3

import rospy
from sgpr_ros.srv import Eigenvalues, EigenvaluesRequest, EigenvaluesResponse

from scipy.stats import ks_2samp, anderson_ksamp

ks_result = False
ad_result = False
updated = False

def eval_callback(req: EigenvaluesRequest):
    global ks_result, ad_result, updated

    result = anderson_ksamp([req.r_eigs, req.q_eigs])
    print("AD Test results: ", result)
    if result[2] > 0.05:
        ad_result = True
    else:
        ad_result = False

    result = ks_2samp(req.r_eigs, req.q_eigs)
    print("AD Test results: ", result)
    if result.pvalue > 0.05:
        ks_result = True
    else:
        ks_result = False
    
    updated = True
    # Run AD and KS test and return the 
    return EigenvaluesResponse([.5,.5])

def evaluation_test_service():
    global updated, ks_result, ad_result
    rospy.init_node('evaluation_service_node')
    rospy.Service('evaluation_service', Eigenvalues, eval_callback)
    print("Eval Service Up")
    #rospy.spin()
    while 1:
        if (updated):
            print("ks: ", ks_result)
            print("ad: ", ad_result)
            updated = False

if __name__ == "__main__":
    evaluation_test_service()
