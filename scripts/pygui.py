import sys
import json

from colorama import Fore, Back, Style

from scipy.stats import ks_2samp, anderson_ksamp
#from sklearn.metrics import mean_squared_error

from PyQt5.QtWidgets import QApplication, QGridLayout, QMainWindow, QPushButton, QWidget, QLineEdit, QFrame, QLabel

global ref_eigenvals, query_eigenvals, ref_eig_map, query_eig_map

##########################################################
# Dict Formats
##########################################################

# JSON format to parse from DT_eigs.json
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

# score_card = {
#   query_scan_id: string
#   reference_scan_id_match: string
#   total_objs: int
#   ref_scan_scores : {
#       <scan_id>: <total_matches>
#       ...
#   }
#}

##########################################################
# Dataset Testing analysis functions here
##########################################################
def compareObjects(ref_obj_map, query_obj_map, type):
    # For all objs in the query obj map try and find a global id match in ref map
    total_matches = 0
    for k, v in query_obj_map.items():
        # search for key in the ref_obj_map
        if (k not in ref_obj_map):
            continue

        ref_obj = ref_obj_map[k]
        if (len(ref_obj["eigenvalues"]) == 0 or len(v["eigenvalues"]) == 0):
            continue

        if type == 0: # KSTest
            result = ks_2samp(ref_obj["eigenvalues"], v["eigenvalues"])

            if result.pvalue > 0.05:
                #print("Match for Label: {} Global ID: {}".format(k, v["label"]))     
                total_matches+=1

        elif type == 1: # ADTest
            result = anderson_ksamp([ref_obj["eigenvalues"], v["eigenvalues"]])

            if result[2] > 0.05:
                #print("Match for Label: {} Global ID: {}".format(k, v["label"]))     
                total_matches+=1

    return total_matches

def dtRunTest(type):
    global ref_eig_map, query_eig_map

    total_scenes = 0
    total_correct = 0

    for q_k, q_v in query_eig_map.items():
        query_scan_id = q_k
        reference_scan_id = q_v[0]

        score_card = dict()
        score_card["query_scan_id"] = query_scan_id
        score_card["reference_scan_id_match"] = reference_scan_id
        score_card["total_objs"] = len(q_v[0])
        score_card["ref_scan_scores"] = dict()
        
        print("######################################")
        print("Query scan ID: " + query_scan_id)
        print("Correct Reference Scan ID: " + reference_scan_id)

        # For the query scan test against every reference scan
        for r_k, r_v in ref_eig_map.items():
            #print("-------------------------------------")
            #print("Testing Reference Scan:\n{}".format(r_k))
            total_matches = compareObjects(r_v, q_v[1], type)
            score_card["ref_scan_scores"][r_k] = total_matches
            #print("-------------------------------------")

        # for each each ref_scan_scores print the one with the highest
        ref_winner = str()
        total_matches = -1
        for k, v in score_card["ref_scan_scores"].items():
            if v > total_matches:
                total_matches = v
                ref_winner = k
        
        total_scenes+=1
        print("\nWinner:{}\nTotal Matches: {}/{}".format(ref_winner, total_matches, len(q_v[1])))

        if reference_scan_id == ref_winner:
            total_correct+=1
            print(Fore.GREEN + "CORRECT!!!")
            print(Style.RESET_ALL)
        else:
            print(Fore.RED + "WRONG!!!")
            print(Style.RESET_ALL)

    print("\n######################################")
    print("Final Results")
    print("Accuracy: {}/{}".format(total_correct, total_scenes))

# TODO Need to change the global_id key to not overwrite other objects that have the same global_id in the scene
def dtParseJSON(file_path):
    global ref_eig_map, query_eig_map

    
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
##########################################################
# Novel Methods Testing analysis functions here
##########################################################

def nmtRunADTest():
    global ref_eigenvals, query_eigenvals

    result = anderson_ksamp([ref_eigenvals, query_eigenvals])
    print("AD Test results: ", result)

    if result[2] > 0.05:
        print("AD Test predicts a match")     

def nmtRunKSTest():
    global ref_eigenvals, query_eigenvals

    result = ks_2samp(ref_eigenvals, query_eigenvals)
    print("KS Test results: ", result)

    if result.pvalue > 0.05:
        print("KS Test predicts a match")     

def nmtParseJSON(file_path):
    global ref_eigenvals, query_eigenvals

    f = open(file_path)
    data = json.load(f)
    
    reference_scans = data["reference_scans"]
    for eigenvalues in reference_scans:
        ref_eigenvals = eigenvalues["eigenvalues"]
    
    query_scans = data["query_scans"]
    for eigenvalues in query_scans:
        query_eigenvals = eigenvalues["eigenvalues"]

##########################################################
# All UI elements here
##########################################################
class QHLine(QFrame):
    def __init__(self):
        super(QHLine, self).__init__()
        self.setFrameShape(QFrame.HLine)
        self.setFrameShadow(QFrame.Sunken)


class QVLine(QFrame):
    def __init__(self):
        super(QVLine, self).__init__()
        self.setFrameShape(QFrame.VLine)
        self.setFrameShadow(QFrame.Sunken)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.nmt_eigs_f = "NMT_eigs.json"
        self.dt_eigs_f = "DT_eigs.json"
        self.eigs_path = "/home/nate/Development/catkin_ws/src/sgpr_ros/data/"
        self.initUI()

    def initUI(self):
        def dt_read_json_file():
            self.dt_eigs_f = self.dt_textbox.text()
            dtParseJSON(self.eigs_path + self.dt_eigs_f)

        def dt_ad_clicked():
            dtRunTest(1)

        def dt_ks_clicked():
            dtRunTest(0)

        def nmt_read_json_file():
            self.dt_eigs_f = self.nmt_textbox.text()
            nmtParseJSON(self.eigs_path + self.nmt_eigs_f)

        def nmt_ad_clicked():
            nmtRunADTest()

        def nmt_ks_clicked():
            nmtRunKSTest()

        self.setWindowTitle("Analysis")
        self.setGeometry(100, 60, 800, 175)

        self.nmt_textbox = QLineEdit(self)
        self.nmt_textbox.setText(self.nmt_eigs_f)

        nmt_button1 = QPushButton('Read JSON file', self)
        nmt_button2 = QPushButton("Anderson-Darling Test", self)
        nmt_button3 = QPushButton("KS Test", self)

        nmt_button1.clicked.connect(nmt_read_json_file)
        nmt_button2.clicked.connect(nmt_ad_clicked)
        nmt_button3.clicked.connect(nmt_ks_clicked)

        self.dt_textbox = QLineEdit(self)
        self.dt_textbox.setText(self.dt_eigs_f)

        dt_button1 = QPushButton('Read JSON file', self)
        dt_button2 = QPushButton("Anderson-Darling Test", self)
        dt_button3 = QPushButton("KS Test", self)

        dt_button1.clicked.connect(dt_read_json_file)
        dt_button2.clicked.connect(dt_ad_clicked)
        dt_button3.clicked.connect(dt_ks_clicked)

        layout = QGridLayout()
        layout.addWidget(QVLine(), 0, 2, 7, 1)
        layout.addWidget(QLabel("Novel Methods Testing"), 0, 0, 1, 2)
        layout.addWidget(QLabel("Dataset Testing"), 0, 3, 1, 2)
        layout.addWidget(QHLine(), 1, 0, 1, 5)
        layout.addWidget(nmt_button1, 2, 0, 1, 1)
        layout.addWidget(self.nmt_textbox, 2, 1, 1, 1)
        layout.addWidget(dt_button1, 2, 3, 1, 1)
        layout.addWidget(self.dt_textbox, 2, 4, 1, 1)
        layout.addWidget(QHLine(), 3, 0, 1, 5)
        layout.addWidget(nmt_button2, 4, 0, 1, 1)
        layout.addWidget(dt_button2, 4, 3, 1, 1)
        layout.addWidget(QHLine(), 5, 0, 1, 5)
        layout.addWidget(nmt_button3, 6, 0, 1, 1)
        layout.addWidget(dt_button3, 6, 3, 1, 1)

        centralWidget = QWidget()
        centralWidget.setLayout(layout)
        self.setCentralWidget(centralWidget)

##########################################################
# Entry point here
##########################################################
app = QApplication(sys.argv)
window = MainWindow()
window.show()
sys.exit(app.exec_())
