from sklearn.metrics import precision_recall_curve, roc_curve, confusion_matrix
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

f = open("/home/nate/Development/catkin_ws/src/sgpr_ros/results/pr.json")
data = json.load(f)

truth_list = list()
pred_list = list()

for result in data['results']:
    key = result["ref_scan_id"]
    for truth in result["query_scan_truth"]:
        truth_list.append(truth)

    for pred in result["query_scan_pred"]:
        pred_list.append(pred)


# Confusion Matrix
confusion_pred = [ x >= .65 for x in pred_list]
c = confusion_matrix(truth_list, confusion_pred)

group_names = ["True Neg","False Pos","False Neg","True Pos"]
group_counts = ["{0:0.0f}".format(value) for value in c.flatten()]
group_percentages = ["{0:.2%}".format(value) for value in
                     c.flatten()/np.sum(c)]
labels = [f"{v1}\n{v2}\n{v3}" for v1, v2, v3 in
          zip(group_names,group_counts,group_percentages)]
labels = np.asarray(labels).reshape(2,2)
sns.heatmap(c, annot=labels, fmt="", cmap='Blues')

# Plot PR and ROC curves
precision, recall, pr_thresholds = precision_recall_curve(truth_list, pred_list) 
false_positive_rate, true_positive_rate, roc_thresholds = roc_curve(truth_list, pred_list)

fig, (ax1, ax2) = plt.subplots(1, 2)
fig.suptitle('Place Recognition Evaluation')
ax1.plot(recall, precision)
ax2.plot(false_positive_rate, true_positive_rate)


ax1.set(xlabel="Recall", ylabel="Precision")
ax1.set_title("Precision Recall Curve")

ax2.set(xlabel="False Positive Rate", ylabel="True Positive Rate")
ax2.set_title("ROC Curve")

plt.show()

f.close()
