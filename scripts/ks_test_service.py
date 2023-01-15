#!/usr/bin/env python3

import rospy
from sgpr_ros.srv import Eigenvalues, EigenvaluesRequest, EigenvaluesResponse

def ks_test_callback(req: EigenvaluesRequest):
    # Run AD Test on two arrays
    return EigenvaluesResponse(req.a + req.b)

def ks_test_service():
    rospy.init_node('ks_test_service_node')
    rospy.Service('ks_test_service', Eigenvalues, ks_test_callback)
    print("ready for ks_service")
    rospy.spin()

if __name__ == "__main__":
    ks_test_service()