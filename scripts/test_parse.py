import json

# JSON format to parse
#{
#  reference_scans: [{
#    scan_id:
#    objects: [{
#      label:
#      global_id:
#      scene_id
#      ply_color:
#      eigenvalues:
#    }]
#  }],
#  query_scans: [
#    scan_id:
#    reference_scan_id:
#    objects: [{
#      label:
#      global_id:
#      scene_id
#      ply_color:
#      eigenvalues:
#    }]
#  }],
#}

file_path = "/home/nate/Development/catkin_ws/src/sgpr_ros/data/DT_eigs.json"
f = open(file_path)
data = json.load(f)

# parse reference scans
ref_eig_map = dict()
reference_scans = data["reference_scans"]
for scan in reference_scans:
    scan_id = scan["scan_id"]
    obj_vec = scan["objects"]

    obj_dict = dict()
    for obj in obj_vec:
        global_id = obj["global_id"]
        obj_dict[global_id] = obj

    ref_eig_map[scan_id] = obj_dict

# parse query scans
query_eig_map = dict()
query_scans = data["query_scans"]
for scan in query_scans:
    scan_id = scan["scan_id"]
    reference_scan_id = scan["reference_scan_id"]
    obj_vec = scan["objects"]

    obj_dict = dict()
    for obj in obj_vec:
        global_id = obj["global_id"]
        obj_dict[global_id] = obj

    query_eig_map[scan_id] = [reference_scan_id, obj_dict]
