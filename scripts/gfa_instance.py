import json
from numpy import linalg
from sklearn.neighbors import KNeighborsClassifier

f = open('/home/nate/Development/catkin_ws/src/sgpr_ros/results/gfa/results.json')
data = json.load(f)

reference_map = dict()
reference_scans = data["reference_scans"]
scan_idx = 0
prev_scan = ""
for scan in reference_scans:
    scan_id = scan["scan_id"]
    if prev_scan == "": prev_scan = scan_id
    if prev_scan != scan_id: scan_idx+=1
    object_map = dict()

    # populate object map
    for obj in scan:
        if obj == "scan_id": continue
        object_map[obj] = (data["reference_scans"][scan_idx][obj]["label"], data["reference_scans"][scan_idx][obj]["global_id"], data["reference_scans"][scan_idx][obj]["gfa_features"])
    reference_map[scan_id] = object_map

# parse query scans
query_map = dict()
query_scans = data["query_scans"]
scan_idx = 0
prev_scan = ""
for scan in query_scans:
    scan_id = scan["scan_id"]
    if prev_scan == "": prev_scan = scan_id
    if prev_scan != scan_id: scan_idx+=1
    reference_scan_id = scan["reference_scan_id"]
    object_map = dict()

    # populate object map
    for obj in scan:
        if obj == "scan_id" or obj == "reference_scan_id": continue
        object_map[obj] = (data["query_scans"][scan_idx][obj]["label"], data["query_scans"][scan_idx][obj]["global_id"], data["query_scans"][scan_idx][obj]["gfa_features"])
    query_map[scan_id] = (reference_scan_id, object_map)

# TODO get training data from the reference scans
# When training we need to associate the indicie of the training pt with the reference_scan_id
training_list = list()
training_global_id_class_list = list()
reference_info_list = list()
for k, v in reference_map.items():
    reference_scan_id = k
    for obj_k, obj_v in v.items():
        ply_color = obj_k
        label = obj_v[0]
        global_id = obj_v[1]
        gfa_features = obj_v[2][:7]

        training_list.append(gfa_features)
        training_global_id_class_list.append(global_id)
        reference_info_list.append((label, ply_color, reference_scan_id))


# TODO read all the values from the json files into their respective lists
query_gfa = data["q_gfa"]
ref_gfa = data["r_gfa"]
q_global_id = data["q_global_id"]
r_global_id = data["r_global_id"]
q_scene_id = data["q_scene_id"]
r_scene_id = data["r_scene_id"]
truth_labels = data["truth_labels"]

# Create a instance to id map
# Create a id to instance map
instance_id_map = dict()
id_instance_map = dict()

# Populate the map
id_idx = 0
list_idx = list()

for i in range(len(r_global_id)):
    key_tuple = (r_global_id[i], r_scene_id[i])
    instance_id_map[key_tuple] = id_idx
    id_instance_map[id_idx]
    list_idx.append(id_idx)
    id_idx = id_idx + 1

knn = KNeighborsClassifier(n_neighbors=1)

knn.fit(ref_gfa, list_idx)

# Loop through the the q_gfa
for i in range(len(query_gfa)):
    # get the obj gfa
    # get the global id
    # get the 

# Will contain the reference_scan_id followed by a list which contains, correct predictions, incorrect_predictions, total items compared

# TODO Now test query scans
total_correct = 0
total_incorrect = 0
for k, v in query_map.items():
    prediction_scan_map = dict()
    correct_pred_count = 0
    incorrect_pred_count = 0
    total_preds = 0
    query_scan_id = k
    reference_scan_id = v[0]
    print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    print("Query scan ID: " + query_scan_id)
    print("Reference scan ID: " + reference_scan_id)
    for obj_k, obj_v in v[1].items():
        total_preds+=1
        ply_color = obj_k
        label = obj_v[0]
        global_id = obj_v[1]
        gfa_features = obj_v[2][:7]

        distance_list, indicie_list = knn.kneighbors([gfa_features])
        idx = indicie_list[0][0]
        pred_global_id = training_global_id_class_list[idx]
        pred_label = reference_info_list[idx][0]
        pred_ply_color = reference_info_list[idx][1]
        pred_reference_scan = reference_info_list[idx][2]

        # TODO apply hard thresholding for worse results muhahahahah
        query_gfa = gfa_features
        reference_gfa = training_list[idx]

        diff = [ (query_gfa[i] - reference_gfa[i]) for i in range(len(query_gfa)) ]

        norm = linalg.norm(diff)

        if(norm >= 0.01): continue

       # print("!!!!!!!!!!!!!!!!!!!!!!!!!")
       # print('Actual Item, global_id: {}\nlabel: {}\nply_color: {}'.format(global_id, label, ply_color))
       # print('Predicted Item, global_id: {}\nlabel: {}\nply_color: {}'.format(pred_global_id, pred_label, pred_ply_color))

        # What are we testing
        # 1) Match: Test if global_id's match and the label matches
        if(global_id == pred_global_id and label == pred_label):
            correct_pred_count+=1
        # 2) No match: if not above
        else:
            incorrect_pred_count+=1
        # 3) total: keep track of the idx
        # 4) put it in prediction_scan_map
        prediction_scan_map[pred_reference_scan] = (correct_pred_count, incorrect_pred_count, total_preds)

    print('-----------------------------------------')
    print("Prediction Results")
    max_correct_pred_scan = ''
    max_correct_pred = 0
    for pred_k, prod_v in prediction_scan_map.items():
        total_predicted_matches = prod_v[0] + prod_v[1]
        if(total_predicted_matches > max_correct_pred):
            max_correct_pred_scan = pred_k
            max_correct_pred = total_predicted_matches
        print('Reference scan prediction: {}'.format(pred_k))
        print('correct matches: {}\nincorrect matches: {}\ntotal matches: {}'.format(prod_v[0], prod_v[1], prod_v[2]))

    if(max_correct_pred_scan == reference_scan_id):
        print("GOOD MATCH")
        total_correct+=1
    else:
        print("BAD MATCH")
        total_incorrect+=1

print('Correct Predictions: {}\nTotal Predictions: {}\nAccuracy: {}'.format(total_correct, (total_correct + total_incorrect), (total_correct/(total_correct + total_incorrect))))
#print(neigh.predict([[1.1]]))
#print(neigh.predict_proba([[0.9]]))
#print(neigh.predict_proba([[6.5]]))
