from __future__ import annotations
from PyQt5 import QtCore, QtGui, QtWidgets
import rospy
from std_msgs.msg import Bool
from std_srvs.srv import EmptyRequest
from geometry_msgs.msg import Point, Pose2D
from sprinter_srvs.srv import SetPointArrRequest, SetPose2DRequest


class Ui_MainWindow(object):
    def __init__(self, fake_gps_publisher : rospy.Publisher, initialize_service : rospy.ServiceProxy,
                 safety_stop_service : rospy.ServiceProxy, reset_service : rospy.ServiceProxy,
                 start_printing_service : rospy.ServiceProxy, go_to_pose_service : rospy.ServiceProxy) -> None:
        
        self.fake_gps_publisher = fake_gps_publisher
        self.initialize_service = initialize_service
        self.safety_stop_service = safety_stop_service
        self.reset_service = reset_service
        self.start_printing_service = start_printing_service
        self.go_to_pose_service = go_to_pose_service
        self.printing_points : list[Point] = []
    
    def setupUi(self, MainWindow : QtWidgets.QMainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 600)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(590, 60, 131, 141))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.lineEdit = QtWidgets.QLineEdit(self.verticalLayoutWidget)
        self.lineEdit.setObjectName("lineEdit")
        self.verticalLayout.addWidget(self.lineEdit)
        self.lineEdit_2 = QtWidgets.QLineEdit(self.verticalLayoutWidget)
        self.lineEdit_2.setObjectName("lineEdit_2")
        self.verticalLayout.addWidget(self.lineEdit_2)
        self.lineEdit_3 = QtWidgets.QLineEdit(self.verticalLayoutWidget)
        self.lineEdit_3.setObjectName("lineEdit_3")
        self.verticalLayout.addWidget(self.lineEdit_3)
        self.addPointButton = QtWidgets.QPushButton(self.centralwidget)
        self.addPointButton.setGeometry(QtCore.QRect(570, 230, 171, 81))
        self.addPointButton.setObjectName("addPointButton")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(590, 330, 151, 61))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.label.setFont(font)
        self.label.setObjectName("label")
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_2.setGeometry(QtCore.QRect(550, 0, 221, 61))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.label_2.setFont(font)
        self.label_2.setObjectName("label_2")
        self.initButton = QtWidgets.QPushButton(self.centralwidget)
        self.initButton.setGeometry(QtCore.QRect(40, 50, 211, 91))
        self.initButton.setObjectName("initButton")
        self.fakeGPSButton = QtWidgets.QPushButton(self.centralwidget)
        self.fakeGPSButton.setGeometry(QtCore.QRect(300, 70, 181, 51))
        self.fakeGPSButton.setObjectName("fakeGPSButton")
        self.startPrintingButton = QtWidgets.QPushButton(self.centralwidget)
        self.startPrintingButton.setGeometry(QtCore.QRect(40, 160, 211, 91))
        self.startPrintingButton.setObjectName("startPrintingButton")
        self.goToPoseButton = QtWidgets.QPushButton(self.centralwidget)
        self.goToPoseButton.setGeometry(QtCore.QRect(40, 270, 211, 91))
        self.goToPoseButton.setObjectName("goToPoseButton")
        self.resetOdomButton = QtWidgets.QPushButton(self.centralwidget)
        self.resetOdomButton.setGeometry(QtCore.QRect(40, 390, 211, 91))
        self.resetOdomButton.setObjectName("resetOdomButton")
        self.safetyStopButton = QtWidgets.QPushButton(self.centralwidget)
        self.safetyStopButton.setGeometry(QtCore.QRect(330, 210, 121, 211))
        self.safetyStopButton.setStyleSheet("background-color: rgb(204, 0, 0)")
        self.safetyStopButton.setObjectName("safetyStopButton")
        self.verticalLayoutWidget_2 = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget_2.setGeometry(QtCore.QRect(590, 390, 131, 141))
        self.verticalLayoutWidget_2.setObjectName("verticalLayoutWidget_2")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_2)
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.lineEdit_4 = QtWidgets.QLineEdit(self.verticalLayoutWidget_2)
        self.lineEdit_4.setObjectName("lineEdit_4")
        self.verticalLayout_2.addWidget(self.lineEdit_4)
        self.lineEdit_5 = QtWidgets.QLineEdit(self.verticalLayoutWidget_2)
        self.lineEdit_5.setObjectName("lineEdit_5")
        self.verticalLayout_2.addWidget(self.lineEdit_5)
        self.lineEdit_6 = QtWidgets.QLineEdit(self.verticalLayoutWidget_2)
        self.lineEdit_6.setObjectName("lineEdit_6")
        self.verticalLayout_2.addWidget(self.lineEdit_6)
        self.line = QtWidgets.QFrame(self.centralwidget)
        self.line.setGeometry(QtCore.QRect(510, 10, 20, 551))
        self.line.setFrameShape(QtWidgets.QFrame.Shape.VLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Shadow.Sunken)
        self.line.setObjectName("line")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

        self._bind_slots_for_buttons()        

    def retranslateUi(self, MainWindow : QtWidgets.QMainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "sPrinter - HMI"))
        self.addPointButton.setText(_translate("MainWindow", "ADD PRINTING POINT"))
        self.label.setText(_translate("MainWindow", "Target Pose"))
        self.label_2.setText(_translate("MainWindow", "Target Printing Point"))
        self.initButton.setText(_translate("MainWindow", "Initialize sPrinter"))
        self.fakeGPSButton.setText(_translate("MainWindow", "Fake GPS Response"))
        self.startPrintingButton.setText(_translate("MainWindow", "Start Printing"))
        self.goToPoseButton.setText(_translate("MainWindow", "Go to Pose"))
        self.resetOdomButton.setText(_translate("MainWindow", "Reset Odometry"))
        self.safetyStopButton.setText(_translate("MainWindow", "SAFETY STOP"))

    def _bind_slots_for_buttons(self) -> None:
        self.addPointButton.clicked.connect(self._add_point_slot)
        self.initButton.clicked.connect(self._init_slot)
        self.fakeGPSButton.clicked.connect(self._fake_gps_slot)
        self.startPrintingButton.clicked.connect(self._start_printing_slot)
        self.goToPoseButton.clicked.connect(self._go_to_pose_slot)
        self.resetOdomButton.clicked.connect(self._reset_odom_slot)
        self.safetyStopButton.clicked.connect(self._safety_stop_slot)

    def _add_point_slot(self):
        x = self.lineEdit.text()
        y = self.lineEdit_2.text()
        z = self.lineEdit_3.text()
        
        self.printing_points.append(Point(x = float(x),
                                          y = float(y),
                                          z = float(z)))
        self._clear_printer_point_form()
    
    def _init_slot(self):
        request = EmptyRequest()
        self.initialize_service(request) 

    def _fake_gps_slot(self):
        msg = Bool()
        msg.data = True
        self.fake_gps_publisher.publish(msg)
        
    def _clear_printer_point_form(self) -> None:
        self.lineEdit.clear()
        self.lineEdit_2.clear()
        self.lineEdit_3.clear()
    
    def _clear_pose_form(self) -> None:
        self.lineEdit_4.clear()
        self.lineEdit_5.clear()
        self.lineEdit_6.clear()
        
    
    def _start_printing_slot(self):
        request = SetPointArrRequest()
        request.points = self.printing_points
        self.start_printing_service(request)
        self.printing_points.clear()
    
    def _go_to_pose_slot(self):
        x = self.lineEdit_4.text()
        y = self.lineEdit_5.text()
        theta = self.lineEdit_6.text()
        
        pose = Pose2D(x = float(x), y = float(y), theta = float(theta))    
          
        request = SetPose2DRequest()
        request.pose = pose
        self.go_to_pose_service(request)
        self._clear_pose_form()
    
    def _reset_odom_slot(self):
        request = EmptyRequest()
        self.reset_service(request)
    
    def _safety_stop_slot(self):
        request = EmptyRequest()
        self.safety_stop_service(request)
    