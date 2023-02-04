import json
from sklearn.metrics import confusion_matrix
import seaborn as sns

import numpy as np

import matplotlib.pyplot as plt


dataset = "3RScan"
laplacian = "Generic"
file = "0.json"
path = "/home/nate/Development/catkin_ws/src/sgpr_ros/results/{}/{}/{}".format(dataset, laplacian, file)

with open(path, "r") as file:
    data = json.load(file)

instance_accuracy = data["instance_accuracy"]
j_precision = data["precision"]
j_recall = data["recall"]
j_f1_score = data["f1_score"]
pred_labels = data["pred_labels"]
truth_labels = data["truth_labels"]

c = confusion_matrix(truth_labels, pred_labels)

group_names = ["True Neg","False Pos","False Neg","True Pos"]
group_counts = ["{0:0.0f}".format(value) for value in c.flatten()]
group_percentages = ["{0:.2%}".format(value) for value in
                     c.flatten()/np.sum(c)]
labels = [f"{v1}\n{v2}\n{v3}" for v1, v2, v3 in
          zip(group_names,group_counts,group_percentages)]
labels = np.asarray(labels).reshape(2,2)
sns.heatmap(c, annot=labels, fmt="", cmap='Blues')
plt.show()
