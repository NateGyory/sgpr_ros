import open3d as o3d
import json
import copy
import numpy as np
import time
import os
from numpy.linalg import inv
import argparse

def parse_calibration(calib, seq):
    calib_file = open("/home/nate/Datasets/SemanticKitti/dataset/sequences/" + seq + "/calib.txt")
    
    key_num = 0
    
    for line in calib_file:
        key, content = line.strip().split(":")
        values = [float(v) for v in content.strip().split()]
        pose = np.zeros((4,4))
        
        pose[0, 0:4] = values[0:4]
        pose[1, 0:4] = values[4:8]
        pose[2, 0:4] = values[8:12]
        pose[3, 3] = 1.0
    
        calib[key] = pose
    
    calib_file.close()

def parse_pose(poses, calib, seq):
    pose_file = open("/home/nate/Datasets/SemanticKitti/dataset/sequences/" + seq + "/poses.txt")
    
    Tr = calib["Tr"]
    Tr_inv = inv(Tr)
    
    for line in pose_file:
        values = [float(v) for v in line.strip().split()]
    
        pose = np.zeros((4, 4))
        pose[0, 0:4] = values[0:4]
        pose[1, 0:4] = values[4:8]
        pose[2, 0:4] = values[8:12]
        pose[3, 3] = 1.0
    
        poses.append(np.matmul(Tr_inv, np.matmul(pose, Tr)))
        #poses.append(pose)

def get_file_names(file_names, seq):
    path = "/home/nate/Datasets/SemanticKittiPLY/" + seq
    file_names[:] = os.listdir(path)
    file_names[:] = sorted(file_names, key=lambda x: int(x[:-4]))

def preprocess_point_clouds(file_names, poses, point_clouds, seq):
    path = "/home/nate/Datasets/SemanticKittiPLY/" + seq
    idx = 0

    for file in file_names:
        print(file)
        cloud = o3d.io.read_point_cloud(path + "/" + file)
        cloud.transform(poses[idx])
        down_sampled_cloud = cloud.uniform_down_sample(every_k_points=500)
        colors = np.asarray(down_sampled_cloud.colors)
        non_black_indices = np.where(np.sum(colors, axis=1) != 0)[0]
        final_cloud = down_sampled_cloud.select_by_index(non_black_indices)
        point_clouds.append(final_cloud)
        idx += 1

def get_line_set(vertices, edges, seq):
    path = "/home/nate/Development/semnatickitteval/results/" + seq + "/poses.json"
    data = None
    with open(path, 'r') as f:
        data = json.load(f)
    

    for pose in data["poses"]:
        point = list()
        point.append(pose["x"])
        point.append(pose["y"])
        point.append(pose["z"])
        vertices.append(point)

    prev = 0
    for i in range(1, len(data["poses"])):
        edges.append([prev, i])
        prev = i

def get_loops(loops, threshold, seq, samp, method):
    path = "/home/nate/Development/semnatickitteval/results/" + seq + "/" + samp + "/" + method
    file_list = os.listdir(path)
    
    for file_name in file_list:
    
        f = open(path + "/" + file_name)
        data = json.load(f)
        pred_list = data["pred"]
        loop_candidates = data["poses"]
        for idx, pred in enumerate(pred_list):
            if pred > threshold:
                loops[loop_candidates[idx]["query_id"]] = True

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seq", help="the sequence to process", required=True)
    parser.add_argument("--samp", help="the sequence to process", required=True)
    parser.add_argument("--m", help="the method, 0 for all 1 for best match", required=True)
    parser.add_argument("--t", help="threshold", required=True)
    parser.add_argument("--big", help="big", required=True)
    args = parser.parse_args()

    seq = args.seq
    samp = args.samp
    method = args.m
    threshold = float(args.t)
    big = int(args.big)

    calib = dict()
    poses = list()
    file_names = list()
    point_clouds = list()
    vertices = list()
    edges = list()
    loops = dict()


    parse_calibration(calib, seq)
    parse_pose(poses, calib, seq)
    get_file_names(file_names, seq)
    preprocess_point_clouds(file_names, poses, point_clouds, seq)
    get_line_set(vertices, edges, seq)
    get_loops(loops, threshold, seq, samp, method)

    vis = o3d.visualization.Visualizer()
    vis.create_window(height=925, width=1650)
    vis.get_render_option().point_size = 0.1
    vis.get_render_option().line_width = 20.0
    vis.get_render_option().show_coordinate_frame = True

    line_set = o3d.geometry.LineSet()
    line_set.points=o3d.utility.Vector3dVector(vertices[:2])
    line_set.lines=o3d.utility.Vector2iVector(edges[0:1])
    #line_set.colors=o3d.utility.Vector3dVector([[1,0,0],[1,0,0]])

    cloud = point_clouds[0]
    vis.add_geometry(cloud)
    vis.add_geometry(line_set)

    dt = 0.0001
    idx = 1
    size = len(point_clouds)
    
    previous_t = time.time()
    
    keep_running = True
    while keep_running:
        
        if time.time() - previous_t > dt:
            if idx < size:
                cloud.points.extend(point_clouds[idx].points)
                cloud.colors.extend(point_clouds[idx].colors)
                vis.update_geometry(cloud)

                if idx > 1:
                    line_set.points.extend(vertices[idx:idx+1])
                    line_set.lines.extend(edges[idx - 1:idx])

                    vis.update_geometry(line_set)

                # if point is a loop closure
                if idx in loops:
                    sphere = None
                    if big == 1:
                        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=6.0)
                        sphere.paint_uniform_color([0,0,0])
                    else:
                        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=1.0)
                        sphere.paint_uniform_color([0,1,0])


                    point = np.array(vertices[idx])
                    sphere.translate(point)
                    vis.add_geometry(sphere)

                previous_t = time.time()
                idx += 1
    
        keep_running = vis.poll_events()
        vis.update_renderer()


if __name__ == "__main__":
    main()


# NOTE this is code to remove duplicate points
#colors = np.asarray(pc.colors)
#points = np.asarray(pc.points)
#unique_points, indices = np.unique(points, axis=0, return_index=True)
#unique_colors = colors[indices]

#pc.points = o3d.utility.Vector3dVector(unique_points)
#pc.colors = o3d.utility.Vector3dVector(unique_colors)
