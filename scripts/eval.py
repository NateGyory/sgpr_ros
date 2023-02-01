import json


dataset = "3RScan"
laplacian = "Generic"
file = "0.json"
path = "/home/nate/Development/catkin_ws/src/sgpr_ros/results/{}/{}/{}".format(dataset, laplacian, file)

with open(path, "r") as file:
    data = json.load(file)

accuracy = data["accuracy"]
precision = data["precision"]
recall = data["recall"]
f1_score = data["f1_score"]

# Display PR Curve

# Display ROC Curve

# Display Confusion Matrix
