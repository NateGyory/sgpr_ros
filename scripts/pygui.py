import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QWidget, QVBoxLayout

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.initUI()

    def initUI(self):
        def ad_clicked():
            print("AD clicked!")

        def ks_clicked():
            print("KS clicked!")

        button1 = QPushButton("Anderson-Darling Test", self)
        button1.clicked.connect(ad_clicked)
        button2 = QPushButton("KS Test", self)
        button2.clicked.connect(ks_clicked)

        layout = QVBoxLayout()
        layout.addWidget(button1)
        layout.addWidget(button2)

        centralWidget = QWidget()
        centralWidget.setLayout(layout)
        self.setCentralWidget(centralWidget)

app = QApplication(sys.argv)
window = MainWindow()
window.show()
sys.exit(app.exec_())
