from sklearn.metrics import precision_recall_curve, roc_curve, confusion_matrix
import seaborn as sns

import matplotlib.pyplot as plt
import json
import numpy as np

#{
#  results : [
#    ref_scan_id: scan_id
#    query_scan_truth: [... 0 or 1 for correct pred]
#    query_scan_pred: [... the pred percent]
#  ]
#}

# Read in json file

f = open("/home/nate/Development/catkin_ws/src/sgpr_ros/results/pr.json")
#  
data = json.load(f)
#  
precision = dict()
recall = dict()
for i in data['results']:
    key = i["ref_scan_id"]
    precision[key], recall[key], _ = precision_recall_curve(i["query_scan_truth"], i["query_scan_pred"])
    plt.plot(recall[key], precision[key], lw=1)
    #plt.plot(recall[key], precision[key], lw=2, label='class {}'.format(key))

    print(precision[key])
    print(recall[key])

#precision, recall, thresholds = precision_recall_curve(
#    data["results"][0]["query_scan_truth"] ,data["results"][0]["query_scan_pred"])
#
#print(precision)
#print(recall)
#print(thresholds)

#plt.plot(recall, precision, lw=2, label='class {}'.format(1))

plt.xlabel("recall")
plt.ylabel("precision")
plt.legend(loc="best")
plt.title("precision vs. recall curve")
plt.show()
  
# Closing file

#for i in range(n_classes):
#    precision[i], recall[i], _ = precision_recall_curve(y_test[:, i],
#                                                        y_score[:, i])
#    plt.plot(recall[i], precision[i], lw=2, label='class {}'.format(i))
#    
#plt.xlabel("recall")
#plt.ylabel("precision")
#plt.legend(loc="best")
#plt.title("precision vs. recall curve")
#plt.show()

cm = confusion_matrix(y_test, y_pred)

# Creating a dataframe for a array-formatted Confusion matrix,so it will be easy for plotting.
cm_df = pd.DataFrame(cm,
                     index = ['SETOSA','VERSICOLR','VIRGINICA'], 
                     columns = ['SETOSA','VERSICOLR','VIRGINICA'])

#Plotting the confusion matrix
plt.figure(figsize=(5,4))
sns.heatmap(cm_df, annot=True)
plt.title('Confusion Matrix')
plt.ylabel('Actal Values')
plt.xlabel('Predicted Values')
plt.show()



f.close()
