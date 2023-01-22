#!/usr/bin/env python3

import sys
import rospy
import numpy as np
from sgpr_ros.srv import Eigenvalues, EigenvaluesRequest, EigenvaluesResponse

from scipy.stats import ks_2samp, anderson_ksamp
from PyQt5.QtWidgets import QApplication, QMainWindow, QLineEdit, QLabel
from PyQt5 import QtCore

current_req = EigenvaluesRequest()
update = False

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        # ROS callback service
        rospy.init_node('evaluation_service_node')
        rospy.Service('evaluation_service', Eigenvalues, self.eval_callback)

        # Sleep timer so callback can update the GUI bool values
        self._update_timer = QtCore.QTimer()
        self._update_timer.timeout.connect(self.update_labels)
        self._update_timer.start(100) # milliseconds

        # Set up the GUI
        self.initUI()


    def js_div_test(self, q_eigs, r_eigs):    
        q = np.asarray(q_eigs)
        r = np.asarray(r_eigs)

        # normalize
        q /= q.sum()
        r /= r.sum()
        m = (q + r) / 2
        return (self.kl_div_test(q, m) + self.kl_div_test(r, m)) / 2

    def kl_div_test(self, q_eigs, r_eigs):    
        q = np.asarray(q_eigs)
        r = np.asarray(r_eigs)

        div = q * np.log(q/ r)
        return np.sum(div)

    def ks_test(self, q_eigs, r_eigs):
        result = ks_2samp(q_eigs, r_eigs)
        return result.pvalue > 0.05

    def ad_test(self, q_eigs, r_eigs):
        result = anderson_ksamp([q_eigs, r_eigs])
        return result[2] > 0.05

    def update_labels(self):
        global current_req, update

        if (update):
            # KS Test
            ret = self.ks_test(current_req.q_eigs, current_req.r_eigs)
            self.ksTextbox.setText(str(ret))
            if ret:
                self.ksTextbox.setStyleSheet("color: green;")
            else:
                self.ksTextbox.setStyleSheet("color: red;")

            # AD Test
            ret = self.ad_test(current_req.q_eigs, current_req.r_eigs)
            self.adTextbox.setText(str(ret))
            if ret:
                self.adTextbox.setStyleSheet("color: green;")
            else:
                self.adTextbox.setStyleSheet("color: red;")

            # KL Div Test
            if len(current_req.q_eigs) != len(current_req.r_eigs):
                self.klTextbox.setText("r != q size")
            else:
                ret = self.kl_div_test(current_req.q_eigs, current_req.r_eigs)
                self.klTextbox.setText(str(ret))

            # JS Div Test
            if len(current_req.q_eigs) != len(current_req.r_eigs):
                self.jsTextbox.setText("r != q size")
            else:
                ret = self.js_div_test(current_req.q_eigs, current_req.r_eigs)
                self.jsTextbox.setText(str(ret))
            
            update = False



    def eval_callback(self, req: EigenvaluesRequest):
        global current_req, update

        current_req = req
        update = True
        return EigenvaluesResponse([.5,.5])

    def initUI(self):

        self.setWindowTitle("Analysis")
        self.setGeometry(1532, 0, 388, 250)

        self.ksLabel = QLabel(self)
        self.ksLabel.setText('KS Test:')
        self.ksTextbox = QLineEdit(self)

        self.ksLabel.move(20, 20)
        self.ksTextbox.move(130, 20)
        self.ksTextbox.resize(200, 32)

        self.adLabel = QLabel(self)
        self.adLabel.setText('AD Test:')
        self.adTextbox = QLineEdit(self)

        self.adLabel.move(20, 75)
        self.adTextbox.move(130, 75)
        self.adTextbox.resize(200, 32)

        self.klLabel = QLabel(self)
        self.klLabel.setText('KL Divergence:')
        self.klTextbox = QLineEdit(self)

        self.klLabel.move(20, 125)
        self.klTextbox.move(130, 125)
        self.klTextbox.resize(200, 32)

        self.jsLabel = QLabel(self)
        self.jsLabel.setText('JS Divergence:')
        self.jsTextbox = QLineEdit(self)

        self.jsLabel.move(20, 175)
        self.jsTextbox.move(130, 175)
        self.jsTextbox.resize(200, 32)

##########################################################
# Entry point here
##########################################################
app = QApplication(sys.argv)
window = MainWindow()
window.show()
sys.exit(app.exec_())
