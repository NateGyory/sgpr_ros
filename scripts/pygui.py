import sys
import json

from scipy.stats import ks_2samp, anderson_ksamp
#from sklearn.metrics import mean_squared_error

from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QWidget, QVBoxLayout, QLineEdit

global ref_eigenvals, query_eigenvals

##########################################################
# Data analysis functions here
##########################################################

def runADTest():
    global ref_eigenvals, query_eigenvals

    result = anderson_ksamp([ref_eigenvals, query_eigenvals])
    print("AD Test results: ", result)

    if result[2] > 0.05:
        print("AD Test predicts a match")     

def runKSTest():
    global ref_eigenvals, query_eigenvals

    result = ks_2samp(ref_eigenvals, query_eigenvals)
    print("KS Test results: ", result)

    if result.pvalue > 0.05:
        print("KS Test predicts a match")     

def parseJSON(file_path):
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
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.eigs_f = "eigs.json"
        self.eigs_path = "/home/nate/Development/catkin_ws/src/sgpr_ros/data/"
        self.initUI()

    def initUI(self):
        def read_json_file():
            self.eigs_f = self.textbox.text()
            parseJSON(self.eigs_path + self.eigs_f)

        def ad_clicked():
            runADTest()

        def ks_clicked():
            runKSTest()

        self.textbox = QLineEdit(self)
        self.textbox.setText(self.eigs_f)

        button1 = QPushButton('Read JSON file', self)
        button2 = QPushButton("Anderson-Darling Test", self)
        button3 = QPushButton("KS Test", self)

        button1.clicked.connect(read_json_file)
        button2.clicked.connect(ad_clicked)
        button3.clicked.connect(ks_clicked)

        layout = QVBoxLayout()
        layout.addWidget(self.textbox)
        layout.addWidget(button1)
        layout.addWidget(button2)
        layout.addWidget(button3)

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
