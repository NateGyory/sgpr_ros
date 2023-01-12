import sys
import json

from scipy.stats import ks_2samp, anderson_ksamp
#from sklearn.metrics import mean_squared_error

from PyQt5.QtWidgets import QApplication, QGridLayout, QMainWindow, QPushButton, QWidget, QLineEdit, QFrame, QLabel

global ref_eigenvals, query_eigenvals, ref_eig_map, query_eig_map

##########################################################
# Dataset Testing analysis functions here
##########################################################
def dtRunADTest():
    global ref_eig_map, query_eig_map

def dtRunKSTest():
    global ref_eig_map, query_eig_map

def dtParseJSON(file_path):
    global ref_eig_map, query_eig_map

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
            dtRunADTest()

        def dt_ks_clicked():
            dtRunKSTest()

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
