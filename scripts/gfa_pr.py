import json
from numpy import linalg
from sklearn.neighbors import KNeighborsClassifier
from sklearn.metrics import precision_recall_curve, roc_curve, confusion_matrix
import matplotlib.pyplot as plt

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
    shared_obj_count = 0
    total_count = 0
    for ref_scene in ref_scene_list:
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

        probability_list.append(shared_obj_count / total_count)

            

precision, recall, pr_thresholds = precision_recall_curve(truth_list, probability_list) 
false_positive_rate, true_positive_rate, roc_thresholds = roc_curve(truth_list, probability_list)

fig, (ax1, ax2) = plt.subplots(1, 2)
fig.suptitle('Place Recognition Evaluation')
ax1.plot(recall, precision)
ax2.plot(false_positive_rate, true_positive_rate)


ax1.set(xlabel="Recall", ylabel="Precision")
ax1.set_title("Precision Recall Curve")

ax2.set(xlabel="False Positive Rate", ylabel="True Positive Rate")
ax2.set_title("ROC Curve")

plt.show()

#
#TP = 0
#TN = 0
#FP = 0
#FN = 0
#for i in range(len(query_gfa)):
#    key_tuple = (q_global_id[i], q_scene_id[i])
#    distance_list, indicie_list = knn.kneighbors([query_gfa[i]])
#    predicted_class_id = indicie_list[0][0]
#    actual_class = instance_id_map[key_tuple]
#
#    if actual_class == predicted_class_id:
#        if distance_list[0][0] > 0.05:
#            FN = FN +1
#        else:
#            TP = TP + 1
#    else:
#        if distance_list[0][0] > 0.05:
#            TN = TN + 1
#        else:
#            FP = FP + 1
#
#
#print("TP: ", TP)
#print("TN: ", TN)
#print("FP: ", FP)
#print("FN: ", FN)
#
#precision = TP / (TP + FP)
#recall = TP / (TP + FN)
#print("Precision: ", precision)
#print("Recall: ", recall)
#print("F1-Score: ", (2 * precision * recall)/(precision + recall))
#print("Accuracy: ", (TP + TN) / (TP + TN + FP + FN))
