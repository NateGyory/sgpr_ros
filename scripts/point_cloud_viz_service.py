#!/usr/bin/env python3

import rospy
from sgpr_ros.srv import PointClouds, PointCloudsRequest, PointCloudsResponse

def viz_callback(req: PointCloudsRequest):
    # Run open3D viz window
    return PointCloudsResponse(True)

def point_clouds_service():
    rospy.init_node('point_cloud_viz_service_node')
    rospy.Service('point_cloud_viz_service', PointClouds, viz_callback)
    print("Point Cloud Viz Service Up")
    rospy.spin()

if __name__ == "__main__":
    point_clouds_service()
