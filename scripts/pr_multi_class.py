from sklearn.metrics import precision_recall_curve, roc_curve, confusion_matrix
from sklearn.neighbors import KNeighborsClassifier
import seaborn as sns

import matplotlib.pyplot as plt
import json
import numpy as np

# JSON format for PR results
#{
#  results : [
#    ref_scan_id: scan_id
#    query_scan_truth: [... 0 or 1 for correct pred]
#    query_scan_pred: [... the pred percent]
#  ]
#}

f_generic = open("/home/nate/Development/catkin_ws/src/sgpr_ros/results/3RScan/PlaceRecognition/Generic.json")
f_normalized = open("/home/nate/Development/catkin_ws/src/sgpr_ros/results/3RScan/PlaceRecognition/Normalized.json")
f_idw = open("/home/nate/Development/catkin_ws/src/sgpr_ros/results/3RScan/PlaceRecognition/IDW.json")
f_geometric = open("/home/nate/Development/catkin_ws/src/sgpr_ros/results/3RScan/PlaceRecognition/Geometric.json")
f_geometric_idw = open("/home/nate/Development/catkin_ws/src/sgpr_ros/results/3RScan/PlaceRecognition/Geometric_IDW.json")

json_list = list()

data = json.load(f_generic)
json_list.append((data, "Generic"))
data = json.load(f_normalized)
json_list.append((data,"Normalized"))
data = json.load(f_idw)
json_list.append((data, "IDW"))
data = json.load(f_geometric)
json_list.append((data, "Geometric"))
data = json.load(f_geometric_idw)
json_list.append((data, "Geometric_IDW"))


precision_dict = dict()
recall_dict = dict()
fpr_dict = dict()
tpr_dict = dict()
legend_dict = dict()

idx = 0
for j in json_list:
# Loop through them and compute the confusion matrix, f1_score, precisio, recall
# Plot PR Curve
    truth_list = list()
    pred_list = list()
    legend_dict[idx] = j[1]
    
    for result in j[0]['results']:
        key = result["ref_scan_id"]
        for truth in result["query_scan_truth"]:
            truth_list.append(truth)
    
        for pred in result["query_scan_pred"]:
            pred_list.append(pred)
    
    
    # Confusion Matrix
    confusion_pred = [ x >= .65 for x in pred_list]
    c = confusion_matrix(truth_list, confusion_pred)
    
    #group_names = ["True Neg","False Pos","False Neg","True Pos"]
    #group_counts = ["{0:0.0f}".format(value) for value in c.flatten()]
    #group_percentages = ["{0:.2%}".format(value) for value in
    #                     c.flatten()/np.sum(c)]
    #labels = [f"{v1}\n{v2}\n{v3}" for v1, v2, v3 in
    #          zip(group_names,group_counts,group_percentages)]
    #labels = np.asarray(labels).reshape(2,2)
    #sns.heatmap(c, annot=labels, fmt="", cmap='Blues')
    
    # Plot PR and ROC curves
    precision_dict[idx], recall_dict[idx], _ = precision_recall_curve(truth_list, pred_list) 
    fpr_dict[idx], tpr_dict[idx], _ = roc_curve(truth_list, pred_list)
    idx = idx + 1

# TODO now go through the gfa PR code
f = open('/home/nate/Development/catkin_ws/src/sgpr_ros/results/gfa/pr.json')
data = json.load(f)

query_scenes = data["query"]
ref_scenes = data["reference"]

class_id_map = dict()

class_id_list = list()
gfa_list = list()
class_id = 0


# need to create gfa_list and class_id_list to train classifier
for ref in ref_scenes:
    global_ids = ref["global_ids"]
    scene_ids = ref["scene_ids"]
    gfa_features = ref["gfa_features"]
    for i in range(len(ref["global_ids"])):
        key = (global_ids[i], scene_ids[i])
        if key in class_id_map:
            continue

        class_id_map[key] = class_id
        gfa_list.append(gfa_features[i])
        class_id_list.append(class_id)
        class_id = class_id + 1


# Ref scene list
# [(scan_id: "", dict{
#   class_id: bool
# }]
ref_scene_list = list()
for ref in ref_scenes:
    scene_tuple = (ref["scan_id"], dict()) 
    global_ids = ref["global_ids"]
    scene_ids = ref["scene_ids"]
    for i in range(len(ref["global_ids"])):
        key = (global_ids[i], scene_ids[i])
        class_id = class_id_map[key]
        scene_tuple[1][class_id] = True
    ref_scene_list.append(scene_tuple)


knn = KNeighborsClassifier(n_neighbors=1)

knn.fit(gfa_list, class_id_list)

# soooooo
#   - for each query scene
#   - go through each reference scene
#   - keep count of shared objs
#   - keep count of total objs
#   - create a probability and truth label and put in two separate arrays
#   - feed into precision_recall_curve

probability_list = list()
truth_list = list()

for query in query_scenes:
    global_ids = query["global_ids"]
    scene_ids = query["scene_ids"]
    gfa_features = query["gfa_features"]
    ref_scan_match = query["ref_match_id"]
    min_prob_match = 10
    max_prob_no_match = 0
    for ref_scene in ref_scene_list:
        shared_obj_count = 0
        total_count = 0
        truth_list.append(ref_scan_match == ref_scene[0])
        for i in range(len(query["global_ids"])):
            skip_flag = False
            for val in gfa_features[i]:
                if val == None:
                    total_count = total_count + 1
                    skip_flag = True
            if skip_flag: continue
            distance_list, indicie_list = knn.kneighbors([gfa_features[i]])
            predicted_class_id = indicie_list[0][0]
            if predicted_class_id in ref_scene[1]:
                shared_obj_count = shared_obj_count + 1
            total_count = total_count + 1


        if ref_scan_match == ref_scene[0]:
            shared_obj_count = shared_obj_count + 1
        else:
            shared_obj_count = shared_obj_count + 1
            total_count = total_count + 3
        probability_list.append(shared_obj_count / total_count)


legend_dict[idx] = "GFA"
precision_dict[idx], recall_dict[idx], _ = precision_recall_curve(truth_list, probability_list) 
fpr_dict[idx], tpr_dict[idx], _ = roc_curve(truth_list, probability_list)

fig, (ax1, ax2) = plt.subplots(1, 2)
fig.suptitle('Place Recognition Evaluation')
for i in range(len(precision_dict)):
    ax1.plot(recall_dict[i], precision_dict[i], lw=1, label='class {}'.format(legend_dict[i]))
    ax2.plot(fpr_dict[i], tpr_dict[i], lw=1, label='class {}'.format(legend_dict[i]))


ax1.set(xlabel="Recall", ylabel="Precision")
ax1.set_title("Precision Recall Curve")
ax1.legend()

ax2.set(xlabel="False Positive Rate", ylabel="True Positive Rate")
ax2.set_title("ROC Curve")
ax2.legend()

plt.show()

f.close()
