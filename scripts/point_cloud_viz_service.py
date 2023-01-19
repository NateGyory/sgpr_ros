#!/usr/bin/env python3

import rospy
import numpy as np
from sgpr_ros.srv import PointClouds, PointCloudsRequest, PointCloudsResponse
import open3d_conversions

import open3d

#vis1 = open3d.visualization.Visualizer()
#vis2 = open3d.visualization.Visualizer()

#vis1.create_window(width=500,height=500,left=1200,top=50)
#vis1.create_window(width=500,height=500,left=1200,top=1200)

#opt1 = vis1.get_render_option()
#opt1.background_color = np.asarray([0, 0, 0])

#opt2 = vis2.get_render_option()
#opt2.background_color = np.asarray([0, 0, 0])

cloud1 = open3d.geometry.PointCloud()
cloud2 = open3d.geometry.PointCloud()

def viz_callback(req: PointCloudsRequest):
    # Run open3D viz window
    global cloud1, cloud2

    cloud1 = open3d_conversions.from_msg(req.cloud1)
    cloud2 = open3d_conversions.from_msg(req.cloud2)

    open3d.visualization.draw([cloud1],non_blocking_and_return_uid=False)
    #vis1.clear_geometries()
    #vis2.clear_geometries()
    #vis1.add_geometry(cloud1)
    #vis2.add_geometry(cloud2)

    return PointCloudsResponse(True)

def point_clouds_service():
    global vis1, vis2
    rospy.init_node('point_cloud_viz_service_node')
    rospy.Service('point_cloud_viz_service', PointClouds, viz_callback)
    print("Point Cloud Viz Service Up")
    rospy.spin()
    #while 1:
      #vis1.add_geometry(cloud1)
      ##vis2.update_geometry(cloud2)

      #vis1.poll_events()
      ##vis2.poll_events()

      #vis1.update_renderer()
      ##vis2.update_renderer()

    #vis1.destroy_window()
    #vis2.destroy_window()

if __name__ == "__main__":
    point_clouds_service()
