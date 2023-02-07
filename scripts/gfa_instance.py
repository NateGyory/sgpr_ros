import json
from numpy import linalg
from sklearn.neighbors import KNeighborsClassifier

f = open('/home/nate/Development/catkin_ws/src/sgpr_ros/results/gfa/results.json')
data = json.load(f)

query_gfa = data["q_gfa"]
ref_gfa = data["r_gfa"]
q_global_id = data["q_global_id"]
r_global_id = data["r_global_id"]
q_scene_id = data["q_scene_id"]
r_scene_id = data["r_scene_id"]
truth_labels = data["truth_labels"]

instance_id_map = dict()

class_id = 0

for i in range(len(r_global_id)):
    key_tuple = (r_global_id[i], r_scene_id[i])
    if key_tuple in instance_id_map:
        continue

    instance_id_map[key_tuple] = class_id
    class_id = class_id + 1

class_ids = list()
for i in range(len(r_global_id)):
    key_tuple = (r_global_id[i], r_scene_id[i])
    class_ids.append(instance_id_map[key_tuple])

knn = KNeighborsClassifier(n_neighbors=1)

knn.fit(ref_gfa, class_ids)

TP = 0
TN = 0
FP = 0
FN = 0
for i in range(len(query_gfa)):
    skip_flag = False
    for val in query_gfa[i]:
        if val == None:
            skip_flag = True
    if skip_flag: continue

    key_tuple = (q_global_id[i], q_scene_id[i])
    distance_list, indicie_list = knn.kneighbors([query_gfa[i]])
    predicted_class_id = indicie_list[0][0]
    if not key_tuple in instance_id_map:
        continue
    actual_class = instance_id_map[key_tuple]

    if actual_class == predicted_class_id:
        if distance_list[0][0] > 0.05:
            FN = FN +1
        else:
            TP = TP + 1
    else:
        if distance_list[0][0] > 0.05:
            TN = TN + 1
        else:
            FP = FP + 1


print("TP: ", TP)
print("TN: ", TN)
print("FP: ", FP)
print("FN: ", FN)

precision = TP / (TP + FP)
recall = TP / (TP + FN)
print("Precision: ", precision)
print("Recall: ", recall)
print("F1-Score: ", (2 * precision * recall)/(precision + recall))
print("Accuracy: ", (TP + TN) / (TP + TN + FP + FN))
