#!/usr/bin/env python3

import rospy
from sgpr_ros.srv import Eigenvalues, EigenvaluesRequest, EigenvaluesResponse

def ad_test_callback(req: EigenvaluesRequest):
    # Run AD Test on two arrays
    return EigenvaluesResponse(req.a + req.b)

def ad_test_service():
    rospy.init_node('ad_test_service_node')
    rospy.Service('ad_test_service', Eigenvalues, ad_test_callback)
    print("ready for ad_service")
    rospy.spin()

if __name__ == "__main__":
    ad_test_service()
