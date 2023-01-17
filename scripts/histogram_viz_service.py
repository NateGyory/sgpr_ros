#!/usr/bin/env python3

import rospy
from sgpr_ros.srv import Eigenvalues, EigenvaluesRequest, EigenvaluesResponse

def histogram_callback(req: EigenvaluesRequest):
    # Run AD Test on two arrays
    return EigenvaluesResponse([0.0])

def histogram_viz_service():
    rospy.init_node('histogram_viz_service_node')
    rospy.Service('histogram_viz_service', Eigenvalues, histogram_callback)
    print("Histogram Viz Service Up")
    rospy.spin()

if __name__ == "__main__":
    histogram_viz_service()
